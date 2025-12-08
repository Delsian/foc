/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adc_dma.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <string.h>

LOG_MODULE_REGISTER(adc_dma, LOG_LEVEL_INF);

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc2));

#define ADC_RESOLUTION		12
#define ADC_GAIN		ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 248)
#define ADC_VREF_MV		3300
#define NUM_CHANNELS		CONFIG_FOC_ADC_NUM_CHANNELS
#define DMA_BUFFER_SIZE		(NUM_CHANNELS * sizeof(uint16_t))

static uint16_t dma_buffer[NUM_CHANNELS] __aligned(4);
static volatile bool data_ready = false;
static volatile bool sampling_active = false;

static struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = 0,
};

static enum adc_action adc_dma_scan_callback(const struct device *dev,
					     const struct adc_sequence *sequence,
					     uint16_t sampling_index)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sequence);
	ARG_UNUSED(sampling_index);

	data_ready = true;

	if (sampling_active) {
		return ADC_ACTION_REPEAT;
	}
	return ADC_ACTION_FINISH;
}

static struct adc_sequence_options sequence_opts = {
	.callback = adc_dma_scan_callback,
	.extra_samplings = 0,
};

static struct adc_sequence sequence = {
	.options = &sequence_opts,
	.channels = 0,
	.buffer = dma_buffer,
	.buffer_size = DMA_BUFFER_SIZE,
	.resolution = ADC_RESOLUTION,
	.oversampling = 0,
	.calibrate = false,
};

static int adc_channels_init(void)
{
	int ret;
	uint32_t channel_mask = 0;

	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device not ready");
		return -ENODEV;
	}

	for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
		channel_cfg.channel_id = i + 1;
		ret = adc_channel_setup(adc_dev, &channel_cfg);
		if (ret < 0) {
			LOG_ERR("Failed to setup ADC channel %d: %d", i + 1, ret);
			return ret;
		}
		channel_mask |= BIT(i + 1);
	}

	sequence.channels = channel_mask;

	LOG_INF("ADC initialized: %d channels, mask=0x%02x",
		NUM_CHANNELS, channel_mask);

	return 0;
}

int adc_dma_init(void)
{
	int ret = adc_channels_init();
	if (ret < 0) {
		return ret;
	}

	data_ready = false;
	sampling_active = false;
	memset(dma_buffer, 0, sizeof(dma_buffer));

	return 0;
}

void adc_dma_start(void)
{
	data_ready = false;
	sampling_active = true;

	int ret = adc_read(adc_dev, &sequence);
	if (ret < 0) {
		LOG_ERR("Failed to start ADC DMA sampling: %d", ret);
		sampling_active = false;
		return;
	}

	LOG_INF("ADC DMA sampling started");
}

void adc_dma_stop(void)
{
	sampling_active = false;
	LOG_INF("ADC DMA sampling stopped");
}

int adc_dma_get_channel(uint8_t channel, uint16_t *value)
{
	if (!value || channel >= NUM_CHANNELS) {
		return -EINVAL;
	}

	if (!data_ready) {
		return -ENODATA;
	}

	*value = dma_buffer[channel];
	return 0;
}

int adc_dma_get_all_channels(uint16_t *values, uint8_t num_channels)
{
	if (!values || num_channels > NUM_CHANNELS) {
		return -EINVAL;
	}

	if (!data_ready) {
		return -ENODATA;
	}

	for (uint8_t i = 0; i < num_channels; i++) {
		values[i] = dma_buffer[i];
	}

	return 0;
}

uint32_t adc_dma_raw_to_mv(uint16_t raw_value)
{
	return ((uint32_t)raw_value * ADC_VREF_MV) / 4096;
}

static int adc_dma_sys_init(void)
{
	int ret = adc_dma_init();
	if (ret < 0) {
		LOG_ERR("ADC DMA initialization failed: %d", ret);
		return ret;
	}

	adc_dma_start();
	return 0;
}

SYS_INIT(adc_dma_sys_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
