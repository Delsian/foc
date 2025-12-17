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
#include <zephyr/irq.h>
#include <string.h>

LOG_MODULE_REGISTER(adc_dma, LOG_LEVEL_INF);

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc2));

static struct k_poll_signal adc_signal = K_POLL_SIGNAL_INITIALIZER(adc_signal);
static struct k_poll_event adc_event = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
								 K_POLL_MODE_NOTIFY_ONLY,
								 &adc_signal);

#define ADC_RESOLUTION		12
#define ADC_GAIN		ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	DT_PROP(DT_CHILD(DT_NODELABEL(adc2), channel_1), zephyr_acquisition_time)
#define ADC_VREF_MV		3300
#define NUM_CHANNELS		CONFIG_FOC_ADC_NUM_CHANNELS
#define ADC_NUM_SAMPLES		5
#define DMA_BUFFER_SIZE		(NUM_CHANNELS * ADC_NUM_SAMPLES * sizeof(uint16_t))

static uint16_t dma_buffer[NUM_CHANNELS * ADC_NUM_SAMPLES] __aligned(4);

#define ADC_THREAD_STACK_SIZE	1024
#define ADC_THREAD_PRIORITY	5

static void adc_thread(void *arg1, void *arg2, void *arg3);
static void tim2_isr(const void *arg);

K_THREAD_DEFINE(adc_thread_id, ADC_THREAD_STACK_SIZE,
		adc_thread, NULL, NULL, NULL,
		ADC_THREAD_PRIORITY, 0, 0);

static struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = 0,
};

static struct adc_sequence_options sequence_options = {
	.extra_samplings = ADC_NUM_SAMPLES - 1,
	.interval_us = 0,
};

static struct adc_sequence sequence = {
	.options = &sequence_options,
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

int adc_dma_get_channel(uint8_t channel, uint16_t *value)
{
	if (!value || channel >= NUM_CHANNELS) {
		return -EINVAL;
	}

	*value = dma_buffer[channel];
	return 0;
}

int adc_dma_get_all_channels(uint16_t *values, uint8_t num_channels)
{
	if (!values || num_channels > NUM_CHANNELS) {
		return -EINVAL;
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

static void adc_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	uint32_t count = 0;
	uint32_t accum[NUM_CHANNELS] = {0};
	uint16_t avg_values[NUM_CHANNELS];
	int ret = adc_channels_init();
	if (ret < 0) {
		LOG_ERR("ADC channels initialization failed: %d", ret);
		return;
	}

	memset(dma_buffer, 0, sizeof(dma_buffer));

	/* Connect TIM2 interrupt */
	IRQ_CONNECT(TIM2_IRQn, 0, tim2_isr, NULL, 0);
	irq_enable(TIM2_IRQn);

	/* Enable TIM2 update interrupt */
	TIM2->DIER |= TIM_DIER_UIE;

	/* Wait for ADC conversions triggered by interrupt */
	while (1) {
		/* Wait for ADC conversion completion (5 samples per channel) */
		ret = k_poll(&adc_event, 1, K_FOREVER);
		if (ret < 0) {
			continue;
		}

		/* Reset signal for next conversion */
		k_poll_signal_reset(&adc_signal);

		/* Accumulate 5 samples per channel from buffer */
		for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
			accum[i] = 0;
			for (uint8_t sample = 0; sample < ADC_NUM_SAMPLES; sample++) {
				accum[i] += dma_buffer[i * ADC_NUM_SAMPLES + sample];
			}
			avg_values[i] = accum[i] / ADC_NUM_SAMPLES;
		}

		count++;

		if (count >= 400) {  /* Print every 400 averaged conversions (~5 Hz) */
			printk("ADC values:");
			for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
				printk("  CH%d: %u mV",
					i, adc_dma_raw_to_mv(avg_values[i]));
			}
			printk("\n");
			count = 0;
		}
	}
}

/**
 * @brief TIM2 update interrupt handler
 */
static void tim2_isr(const void *arg)
{
	ARG_UNUSED(arg);

	/* Check if update interrupt flag is set */
	if (TIM2->SR & TIM_SR_UIF) {
		/* Clear the update interrupt flag */
		TIM2->SR &= ~TIM_SR_UIF;

		/* Trigger ADC read */
		int ret = adc_read_async(adc_dev, &sequence, &adc_signal);
		if (ret < 0) {
			/* ADC read failed - may already be in progress */
		}
	}
}