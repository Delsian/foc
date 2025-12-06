/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include "foc_motor.h"
#include "mt6701.h"
#include "foc_current_sense.h"

/**
 * @brief Motor control context for a single motor
 */
struct motor_control_ctx {
	/* Motor identifier */
	uint8_t motor_id;

	/* FOC motor structure */
	struct foc_motor motor;

	/* Encoder device */
	struct mt6701_dev encoder;

	/* Current sensing device */
	struct current_sense_dev current_sense;

	/* Hardware configuration */
	const struct device *i2c_dev;
	const struct device *pwm_dev;
	const struct device *adc_dev;

	/* Encoder I2C address */
	uint8_t encoder_i2c_addr;

	/* PWM channels (1-indexed) */
	uint8_t pwm_channel_a;
	uint8_t pwm_channel_b;
	uint8_t pwm_channel_c;

	/* ADC channels */
	uint8_t adc_channel_a;
	uint8_t adc_channel_b;

	/* Control thread */
	k_tid_t thread_id;
	bool thread_running;

	/* Motor parameters */
	struct foc_motor_params params;

	/* Current sense configuration */
	struct current_sense_amp_config amp_config;
};

/**
 * @brief Motor control configuration
 */
struct motor_control_config {
	/* Motor parameters */
	uint8_t pole_pairs;
	float phase_resistance;      /* Ohms */
	float phase_inductance;      /* Henries */
	float voltage_supply;        /* Volts */
	float voltage_limit;         /* Volts */
	float velocity_limit;        /* rad/s */
	float current_limit;         /* Amps */

	/* PWM configuration */
	uint32_t pwm_period_ns;

	/* Current sense amplifier configuration */
	float current_sense_gain;
	float current_sense_shunt;
	float current_sense_vref;
	float current_sense_offset;

	/* Control loop frequency */
	uint32_t control_loop_us;    /* Control loop period in microseconds */
};

/**
 * @brief Initialize a motor control context
 *
 * @param ctx Pointer to motor control context
 * @param motor_id Motor identifier (0 or 1)
 * @param i2c_dev I2C device for encoder
 * @param encoder_addr Encoder I2C address
 * @param pwm_dev PWM device
 * @param pwm_ch_a PWM channel for phase A
 * @param pwm_ch_b PWM channel for phase B
 * @param pwm_ch_c PWM channel for phase C
 * @param adc_dev ADC device (or NULL if no current sensing)
 * @param adc_ch_a ADC channel for phase A current
 * @param adc_ch_b ADC channel for phase B current
 * @param config Motor configuration
 * @return 0 on success, negative errno on failure
 */
int motor_control_init(struct motor_control_ctx *ctx,
                       uint8_t motor_id,
                       const struct device *i2c_dev,
                       uint8_t encoder_addr,
                       const struct device *pwm_dev,
                       uint8_t pwm_ch_a,
                       uint8_t pwm_ch_b,
                       uint8_t pwm_ch_c,
                       const struct device *adc_dev,
                       uint8_t adc_ch_a,
                       uint8_t adc_ch_b,
                       const struct motor_control_config *config);

/**
 * @brief Start motor control thread
 *
 * @param ctx Pointer to motor control context
 * @param stack_size Thread stack size in bytes
 * @param priority Thread priority
 * @return 0 on success, negative errno on failure
 */
int motor_control_start_thread(struct motor_control_ctx *ctx,
                               size_t stack_size,
                               int priority);

/**
 * @brief Stop motor control thread
 *
 * @param ctx Pointer to motor control context
 */
void motor_control_stop_thread(struct motor_control_ctx *ctx);

/**
 * @brief Set motor control mode
 *
 * @param ctx Pointer to motor control context
 * @param mode Control mode (voltage, velocity, position, torque)
 */
void motor_control_set_mode(struct motor_control_ctx *ctx, enum foc_control_mode mode);

/**
 * @brief Set motor velocity target
 *
 * @param ctx Pointer to motor control context
 * @param velocity Target velocity in rad/s
 */
void motor_control_set_velocity(struct motor_control_ctx *ctx, float velocity);

/**
 * @brief Set motor position target
 *
 * @param ctx Pointer to motor control context
 * @param position Target position in radians
 */
void motor_control_set_position(struct motor_control_ctx *ctx, float position);

/**
 * @brief Set motor current target (torque control)
 *
 * @param ctx Pointer to motor control context
 * @param iq Q-axis current target in Amps
 * @param id D-axis current target in Amps
 */
void motor_control_set_current(struct motor_control_ctx *ctx, float iq, float id);

/**
 * @brief Get motor velocity
 *
 * @param ctx Pointer to motor control context
 * @return Current velocity in rad/s
 */
float motor_control_get_velocity(struct motor_control_ctx *ctx);

/**
 * @brief Get motor position
 *
 * @param ctx Pointer to motor control context
 * @return Current position in radians
 */
float motor_control_get_position(struct motor_control_ctx *ctx);

/**
 * @brief Get motor state
 *
 * @param ctx Pointer to motor control context
 * @return Current motor state
 */
enum foc_motor_state motor_control_get_state(struct motor_control_ctx *ctx);

/**
 * @brief Enable motor
 *
 * @param ctx Pointer to motor control context
 * @return 0 on success, negative errno on failure
 */
int motor_control_enable(struct motor_control_ctx *ctx);

/**
 * @brief Disable motor
 *
 * @param ctx Pointer to motor control context
 */
void motor_control_disable(struct motor_control_ctx *ctx);

#endif /* MOTOR_CONTROL_H */
