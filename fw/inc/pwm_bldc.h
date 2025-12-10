/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PWM_BLDC_H
#define PWM_BLDC_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

/**
 * @brief PWM BLDC motor driver
 *
 * This driver provides 3-phase PWM control for BLDC motors
 * using a PWM device specified via device tree alias.
 */

/**
 * @brief PWM BLDC device configuration (from DTS)
 */
struct pwm_bldc_config {
	struct pwm_dt_spec pwm_ch1;  /* Phase A */
	struct pwm_dt_spec pwm_ch2;  /* Phase B */
	struct pwm_dt_spec pwm_ch3;  /* Phase C */
	uint32_t pwm_period_ns;      /* PWM period in nanoseconds */
};

/**
 * @brief PWM BLDC device runtime data
 */
struct pwm_bldc_data {
	const struct device *dev;
	bool initialized;
	float phase;  /* Current phase angle in degrees */
	float duty;   /* Current duty cycle in percentage */
};

/**
 * @brief Set duty cycle for all three phases with independent control
 *
 * @param dev Pointer to PWM BLDC device
 * @param duty_a Duty cycle for phase A (0-100%)
 * @param duty_b Duty cycle for phase B (0-100%)
 * @param duty_c Duty cycle for phase C (0-100%)
 * @return 0 on success, negative errno on failure
 */
int pwm_bldc_set_duty(const struct device *dev, float duty_a, float duty_b, float duty_c);

/**
 * @brief Set duty cycle for a single phase
 *
 * @param dev Pointer to PWM BLDC device
 * @param phase Phase number (0=A, 1=B, 2=C)
 * @param duty Duty cycle (0-100%)
 * @return 0 on success, negative errno on failure
 */
int pwm_bldc_set_phase_duty(const struct device *dev, uint8_t phase, float duty);

/**
 * @brief Set three-phase sinusoidal PWM with 120-degree spacing
 *
 * @param dev Pointer to PWM BLDC device
 * @param angle_deg Electrical angle in degrees (0-360)
 * @param amplitude Amplitude/magnitude (0-100%)
 * @return 0 on success, negative errno on failure
 */
int pwm_bldc_set_vector(const struct device *dev, float angle_deg, float amplitude);

/**
 * @brief Disable all PWM outputs (set to 0%)
 *
 * @param dev Pointer to PWM BLDC device
 * @return 0 on success, negative errno on failure
 */
int pwm_bldc_disable(const struct device *dev);

/**
 * @brief Get PWM device by alias
 *
 * @param alias DTS alias name (e.g., "pwm-motor0")
 * @return Pointer to device or NULL if not found
 */
const struct device *pwm_bldc_get_device(const char *alias);

/**
 * @brief Update OLED display with current PWM values
 *
 * Displays phase and duty cycle for both motors on the OLED.
 * Motor 0 on line 0, Motor 1 on line 1.
 */
void pwm_bldc_update_oled(void);

#endif /* PWM_BLDC_H */
