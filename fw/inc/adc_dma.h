/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ADC_DMA_H
#define ADC_DMA_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdint.h>

/**
 * @brief ADC DMA driver for periodic analog channel sampling
 *
 * This driver uses DMA to periodically read ADC channels at a configured
 * frequency and store the results in a circular buffer. The application
 * can read the latest values at any time without blocking.
 */

/**
 * @brief Initialize ADC DMA driver
 *
 * Sets up ADC with DMA for continuous sampling at the configured frequency.
 * Channels are read in sequence and stored in the buffer.
 *
 * @return 0 on success, negative errno on failure
 */
int adc_dma_init(void);

/**
 * @brief Start ADC DMA sampling
 *
 * Begins continuous ADC sampling using DMA. Samples are stored in
 * internal buffer and can be read using adc_dma_get_channel().
 */
void adc_dma_start(void);

/**
 * @brief Stop ADC DMA sampling
 *
 * Stops continuous ADC sampling. Buffer retains last sampled values.
 */
void adc_dma_stop(void);

/**
 * @brief Get latest ADC value for a channel
 *
 * Returns the most recently sampled ADC value for the specified channel.
 * This is a non-blocking read from the DMA buffer.
 *
 * @param channel Channel number (0-based, must be < FOC_ADC_NUM_CHANNELS)
 * @param value Pointer to store the 12-bit ADC value (0-4095)
 *
 * @return 0 on success, -EINVAL if channel invalid, -ENODATA if no data yet
 */
int adc_dma_get_channel(uint8_t channel, uint16_t *value);

/**
 * @brief Get all ADC channel values
 *
 * Copies all current ADC channel values to the provided buffer.
 * This is a non-blocking read from the DMA buffer.
 *
 * @param values Buffer to store ADC values (must hold FOC_ADC_NUM_CHANNELS values)
 * @param num_channels Number of channels to read
 *
 * @return 0 on success, -EINVAL if parameters invalid, -ENODATA if no data yet
 */
int adc_dma_get_all_channels(uint16_t *values, uint8_t num_channels);

/**
 * @brief Convert ADC raw value to millivolts
 *
 * Converts a 12-bit ADC value to millivolts based on the reference voltage.
 *
 * @param raw_value Raw ADC value (0-4095)
 *
 * @return Voltage in millivolts
 */
uint32_t adc_dma_raw_to_mv(uint16_t raw_value);

#endif /* ADC_DMA_H */
