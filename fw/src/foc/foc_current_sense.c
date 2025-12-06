/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "foc_current_sense.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(foc_current_sense, LOG_LEVEL_INF);

/* ADC resolution - typically 12-bit for STM32G4 */
#define ADC_RESOLUTION 12
#define ADC_MAX_VALUE ((1 << ADC_RESOLUTION) - 1)

int current_sense_init(struct current_sense_dev *dev,
                       const struct device *adc_dev,
                       uint8_t channel_a,
                       uint8_t channel_b,
                       const struct current_sense_amp_config *amp_config)
{
	int ret;

	if (!dev || !adc_dev || !amp_config) {
		return -EINVAL;
	}

	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device not ready");
		return -ENODEV;
	}

	/* Store device and channel configuration */
	dev->adc_dev = adc_dev;
	dev->channel_a = channel_a;
	dev->channel_b = channel_b;

	/* Copy amplifier configuration */
	memcpy(&dev->amp_config, amp_config, sizeof(struct current_sense_amp_config));

	/* Configure ADC channel A */
	dev->channel_cfg_a = (struct adc_channel_cfg){
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = channel_a,
		.differential = 0,
	};

	ret = adc_channel_setup(adc_dev, &dev->channel_cfg_a);
	if (ret < 0) {
		LOG_ERR("Failed to setup ADC channel A (%d): %d", channel_a, ret);
		return ret;
	}

	/* Configure ADC channel B */
	dev->channel_cfg_b = (struct adc_channel_cfg){
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = channel_b,
		.differential = 0,
	};

	ret = adc_channel_setup(adc_dev, &dev->channel_cfg_b);
	if (ret < 0) {
		LOG_ERR("Failed to setup ADC channel B (%d): %d", channel_b, ret);
		return ret;
	}

	/* Configure multi-channel ADC sequence for DMA
	 * Read both channels simultaneously in one DMA transfer
	 */
	dev->sequence = (struct adc_sequence){
		.channels = BIT(channel_a) | BIT(channel_b),
		.buffer = dev->adc_buffer,
		.buffer_size = sizeof(dev->adc_buffer),
		.resolution = ADC_RESOLUTION,
	};

	/* Initialize state */
	dev->offset_a = 0.0f;
	dev->offset_b = 0.0f;
	dev->calibrated = false;
	dev->current_a = 0.0f;
	dev->current_b = 0.0f;
	dev->current_c = 0.0f;
	dev->current_d = 0.0f;
	dev->current_q = 0.0f;

	LOG_INF("Current sense initialized (DMA multi-channel): gain=%.1f, shunt=%.4f Ohm",
	        amp_config->gain, amp_config->shunt_resistance);
	LOG_INF("ADC channels: A=%d, B=%d (simultaneous read)", channel_a, channel_b);
	LOG_INF("Phase C current calculated from Ia + Ib + Ic = 0");

	return 0;
}

int current_sense_calibrate(struct current_sense_dev *dev, uint16_t num_samples)
{
	int ret;
	int32_t sum_a = 0, sum_b = 0, sum_c = 0;
	int16_t raw_a, raw_b, raw_c;

	if (!dev) {
		return -EINVAL;
	}

	if (num_samples == 0) {
		num_samples = 100;  /* Default to 100 samples */
	}

	LOG_INF("Starting current sense calibration with %d samples...", num_samples);

	/* Accumulate samples */
	for (uint16_t i = 0; i < num_samples; i++) {
		ret = current_sense_read_raw(dev, &raw_a, &raw_b, &raw_c);
		if (ret < 0) {
			LOG_ERR("Calibration read failed: %d", ret);
			return ret;
		}

		sum_a += raw_a;
		sum_b += raw_b;
		sum_c += raw_c;

		k_sleep(K_USEC(100));  /* Small delay between samples */
	}

	/* Calculate average offsets */
	dev->offset_a = (float)sum_a / (float)num_samples;
	dev->offset_b = (float)sum_b / (float)num_samples;
	dev->calibrated = true;

	LOG_INF("Current sense calibration complete:");
	LOG_INF("  Offset A: %.1f (raw)", (double)dev->offset_a);
	LOG_INF("  Offset B: %.1f (raw)", (double)dev->offset_b);

	return 0;
}

int current_sense_read_raw(struct current_sense_dev *dev,
                           int16_t *raw_a, int16_t *raw_b, int16_t *raw_c)
{
	int ret;

	if (!dev || !raw_a || !raw_b || !raw_c) {
		return -EINVAL;
	}

	/* Read both channels simultaneously with DMA
	 * This performs a single ADC conversion sequence for both channels,
	 * ensuring they are sampled at the same time
	 */
	ret = adc_read(dev->adc_dev, &dev->sequence);
	if (ret < 0) {
		LOG_ERR("ADC DMA read failed: %d", ret);
		return ret;
	}

	/* Extract values from DMA buffer
	 * Buffer order depends on channel numbering - lower channel number comes first
	 */
	if (dev->channel_a < dev->channel_b) {
		*raw_a = dev->adc_buffer[0];
		*raw_b = dev->adc_buffer[1];
	} else {
		*raw_a = dev->adc_buffer[1];
		*raw_b = dev->adc_buffer[0];
	}

	/* Phase C is not measured, set to zero (will be calculated later) */
	*raw_c = 0;

	return 0;
}

int current_sense_read_abc(struct current_sense_dev *dev, struct abc_frame *currents)
{
	int16_t raw_a, raw_b, raw_c;
	int ret;

	if (!dev || !currents) {
		return -EINVAL;
	}

	/* Read raw ADC values */
	ret = current_sense_read_raw(dev, &raw_a, &raw_b, &raw_c);
	if (ret < 0) {
		return ret;
	}

	/* Convert to currents in Amps */
	dev->current_a = current_sense_raw_to_amps(dev, raw_a, dev->offset_a);
	dev->current_b = current_sense_raw_to_amps(dev, raw_b, dev->offset_b);

	/* Calculate phase C from Kirchhoff's law: Ia + Ib + Ic = 0
	 * Therefore: Ic = -(Ia + Ib)
	 */
	dev->current_c = -(dev->current_a + dev->current_b);

	/* Store in output structure */
	currents->a = dev->current_a;
	currents->b = dev->current_b;
	currents->c = dev->current_c;

	return 0;
}

int current_sense_read_dq(struct current_sense_dev *dev, float angle_el,
                          struct dq_frame *currents)
{
	struct abc_frame abc;
	struct ab_frame ab;
	int ret;

	if (!dev || !currents) {
		return -EINVAL;
	}

	/* Read ABC currents */
	ret = current_sense_read_abc(dev, &abc);
	if (ret < 0) {
		return ret;
	}

	/* Apply Clarke transform: ABC -> Alpha-Beta */
	foc_clarke_transform(&abc, &ab);

	/* Apply Park transform: Alpha-Beta -> DQ */
	foc_park_transform(&ab, angle_el, currents);

	/* Store in device structure */
	dev->current_d = currents->d;
	dev->current_q = currents->q;

	return 0;
}

float current_sense_raw_to_amps(const struct current_sense_dev *dev,
                                int16_t raw_value, float offset)
{
	float voltage, current;

	if (!dev) {
		return 0.0f;
	}

	/* Convert raw ADC to voltage */
	voltage = ((float)raw_value / (float)ADC_MAX_VALUE) * dev->amp_config.vref;

	/* Subtract offset voltage (calculated from raw offset) */
	float offset_voltage = (offset / (float)ADC_MAX_VALUE) * dev->amp_config.vref;
	voltage -= offset_voltage;

	/* Convert voltage to current using amplifier gain and shunt resistance
	 * I = V_sense / (Gain * R_shunt)
	 */
	current = voltage / (dev->amp_config.gain * dev->amp_config.shunt_resistance);

	return current;
}

void current_sense_get_abc(const struct current_sense_dev *dev, struct abc_frame *currents)
{
	if (!dev || !currents) {
		return;
	}

	currents->a = dev->current_a;
	currents->b = dev->current_b;
	currents->c = dev->current_c;
}

void current_sense_get_dq(const struct current_sense_dev *dev, struct dq_frame *currents)
{
	if (!dev || !currents) {
		return;
	}

	currents->d = dev->current_d;
	currents->q = dev->current_q;
}
