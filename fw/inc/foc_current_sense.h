/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FOC_CURRENT_SENSE_H
#define FOC_CURRENT_SENSE_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include "foc_math.h"

/**
 * @brief Current sense amplifier configuration
 */
struct current_sense_amp_config {
	float gain;              /* Amplifier gain (V/V) */
	float shunt_resistance;  /* Shunt resistor value (Ohms) */
	float offset_voltage;    /* Amplifier offset voltage (V) */
	float vref;              /* ADC reference voltage (V) */
};

/**
 * @brief Current sense device structure (DMA-based multi-channel)
 */
struct current_sense_dev {
	/* ADC device */
	const struct device *adc_dev;

	/* ADC channel IDs */
	uint8_t channel_a;
	uint8_t channel_b;

	/* Channel configurations */
	struct adc_channel_cfg channel_cfg_a;
	struct adc_channel_cfg channel_cfg_b;

	/* DMA buffer for simultaneous reads (2 channels) */
	int16_t adc_buffer[2];

	/* ADC sequence for multi-channel DMA read */
	struct adc_sequence sequence;

	/* Amplifier configuration */
	struct current_sense_amp_config amp_config;

	/* Calibration data */
	float offset_a;  /* Phase A ADC offset */
	float offset_b;  /* Phase B ADC offset */
	bool calibrated;

	/* Current readings (Amps) */
	float current_a;
	float current_b;
	float current_c;  /* Calculated from Kirchhoff's law: Ic = -(Ia + Ib) */

	/* DQ frame currents */
	float current_d;
	float current_q;
};

/**
 * @brief Initialize current sense module
 *
 * Initializes 2-channel current sensing. Phase C current is automatically
 * calculated using Kirchhoff's law: Ia + Ib + Ic = 0, therefore Ic = -(Ia + Ib)
 *
 * @param dev Pointer to current sense device structure
 * @param adc_dev Pointer to ADC device
 * @param channel_a ADC channel for phase A
 * @param channel_b ADC channel for phase B
 * @param amp_config Amplifier configuration
 * @return 0 on success, negative errno on failure
 */
int current_sense_init(struct current_sense_dev *dev,
                       const struct device *adc_dev,
                       uint8_t channel_a,
                       uint8_t channel_b,
                       const struct current_sense_amp_config *amp_config);

/**
 * @brief Calibrate current sense offsets
 *
 * This should be called when the motor is not powered to measure
 * the zero-current ADC offset values.
 *
 * @param dev Pointer to current sense device structure
 * @param num_samples Number of samples to average for calibration
 * @return 0 on success, negative errno on failure
 */
int current_sense_calibrate(struct current_sense_dev *dev, uint16_t num_samples);

/**
 * @brief Read raw ADC values from all three phases
 *
 * @param dev Pointer to current sense device structure
 * @param raw_a Pointer to store raw ADC value for phase A
 * @param raw_b Pointer to store raw ADC value for phase B
 * @param raw_c Pointer to store raw ADC value for phase C
 * @return 0 on success, negative errno on failure
 */
int current_sense_read_raw(struct current_sense_dev *dev,
                           int16_t *raw_a, int16_t *raw_b, int16_t *raw_c);

/**
 * @brief Read three-phase currents in ABC frame
 *
 * Reads ADC values and converts to current in Amps, applying calibration
 * offsets. Updates dev->current_a, dev->current_b, dev->current_c.
 *
 * @param dev Pointer to current sense device structure
 * @param currents Pointer to ABC frame structure to store currents
 * @return 0 on success, negative errno on failure
 */
int current_sense_read_abc(struct current_sense_dev *dev, struct abc_frame *currents);

/**
 * @brief Read currents in DQ frame
 *
 * Reads ABC currents, applies Clarke and Park transforms to get DQ currents.
 * Updates dev->current_d and dev->current_q.
 *
 * @param dev Pointer to current sense device structure
 * @param angle_el Electrical angle in radians
 * @param currents Pointer to DQ frame structure to store currents
 * @return 0 on success, negative errno on failure
 */
int current_sense_read_dq(struct current_sense_dev *dev, float angle_el,
                          struct dq_frame *currents);

/**
 * @brief Convert raw ADC value to current in Amps
 *
 * Formula: I = (V_adc - V_offset) / (Gain * R_shunt)
 * where V_adc = (raw / ADC_max) * V_ref
 *
 * @param dev Pointer to current sense device structure
 * @param raw_value Raw ADC value
 * @param offset Calibrated offset value
 * @return Current in Amps
 */
float current_sense_raw_to_amps(const struct current_sense_dev *dev,
                                int16_t raw_value, float offset);

/**
 * @brief Get the last measured ABC currents
 *
 * @param dev Pointer to current sense device structure
 * @param currents Pointer to ABC frame structure to store currents
 */
void current_sense_get_abc(const struct current_sense_dev *dev, struct abc_frame *currents);

/**
 * @brief Get the last measured DQ currents
 *
 * @param dev Pointer to current sense device structure
 * @param currents Pointer to DQ frame structure to store currents
 */
void current_sense_get_dq(const struct current_sense_dev *dev, struct dq_frame *currents);

/**
 * @brief Check if current sense is calibrated
 *
 * @param dev Pointer to current sense device structure
 * @return true if calibrated, false otherwise
 */
static inline bool current_sense_is_calibrated(const struct current_sense_dev *dev)
{
	return dev ? dev->calibrated : false;
}

#endif /* FOC_CURRENT_SENSE_H */
