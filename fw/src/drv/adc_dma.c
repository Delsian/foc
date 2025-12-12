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

/* STM32 register offsets and bit definitions for ADC/TIM sync */
#define ADC_CFGR_OFFSET 0x0C
#define ADC_CR_OFFSET 0x08
#define TIM_CR2_OFFSET 0x04

/* ADC_CFGR register bits */
#define ADC_CFGR_EXTEN_RISING (1UL << 10)  /* External trigger on rising edge */
#define ADC_CFGR_EXTSEL_TIM2_TRGO (0xBUL << 5)  /* TIM2_TRGO event (bits 9:5 = 0b01011) */

/* TIM_CR2 register bits */
#define TIM_CR2_MMS_UPDATE (2UL << 4)  /* Master mode: Update event as TRGO */

#define ADC_RESOLUTION		12
#define ADC_GAIN		ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 48)  /* 47.5 cycles */
#define ADC_VREF_MV		3300
#define NUM_CHANNELS		CONFIG_FOC_ADC_NUM_CHANNELS
#define DMA_BUFFER_SIZE		(NUM_CHANNELS * sizeof(uint16_t))

static uint16_t dma_buffer[NUM_CHANNELS] __aligned(4);
static volatile bool data_ready = false;

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

	/* Return REPEAT to keep ADC running continuously for hardware triggers */
	return ADC_ACTION_REPEAT;
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

static void configure_tim2_adc_trigger(void)
{
	volatile uint32_t *tim2_cr2 = (volatile uint32_t *)(TIM2_BASE + TIM_CR2_OFFSET);

	/* Configure TIM2 master mode to output update event on TRGO */
	uint32_t cr2_val = *tim2_cr2;
	cr2_val &= ~(0x7UL << 4);  /* Clear MMS bits */
	cr2_val |= TIM_CR2_MMS_UPDATE;  /* Set MMS to update event */
	*tim2_cr2 = cr2_val;

	LOG_INF("TIM2 configured to trigger ADC on update events");
}

static void configure_adc_hw_trigger(void)
{
	volatile uint32_t *adc2_cfgr = (volatile uint32_t *)(ADC2_BASE + ADC_CFGR_OFFSET);

	/* Configure ADC2 to trigger on TIM2 TRGO rising edge */
	uint32_t cfgr_val = *adc2_cfgr;
	cfgr_val &= ~(0x1FUL << 5);  /* Clear EXTSEL bits */
	cfgr_val &= ~(0x3UL << 10);  /* Clear EXTEN bits */
	cfgr_val |= ADC_CFGR_EXTSEL_TIM2_TRGO;  /* Select TIM2_TRGO */
	cfgr_val |= ADC_CFGR_EXTEN_RISING;  /* Enable rising edge trigger */
	*adc2_cfgr = cfgr_val;

	LOG_INF("ADC2 configured for hardware trigger from TIM2 TRGO");
}

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

	data_ready = false;

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
	int ret = adc_channels_init();
	if (ret < 0) {
		return ret;
	}

	data_ready = false;
	memset(dma_buffer, 0, sizeof(dma_buffer));

	/* Configure TIM2 to trigger ADC conversions BEFORE starting ADC */
	configure_tim2_adc_trigger();

	/* Configure ADC2 to use TIM2 TRGO as external trigger BEFORE adc_read */
	configure_adc_hw_trigger();

	/* Start ADC - it will now wait for TIM2 TRGO triggers */
	ret = adc_read(adc_dev, &sequence);
	if (ret < 0) {
		LOG_ERR("Failed to start ADC DMA sampling: %d", ret);
		return ret;
	}

	LOG_INF("ADC sampling synchronized with TIM2 PWM (20kHz)");

	return 0;
}

SYS_INIT(adc_dma_sys_init, APPLICATION, 92);
