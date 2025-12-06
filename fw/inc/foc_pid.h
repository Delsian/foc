/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FOC_PID_H
#define FOC_PID_H

#include <zephyr/kernel.h>

/**
 * @brief PID controller structure
 */
struct pid_controller {
	/* Tuning parameters */
	float kp;           /* Proportional gain */
	float ki;           /* Integral gain */
	float kd;           /* Derivative gain (not typically used in FOC) */

	/* Controller state */
	float integral;     /* Integral accumulator */
	float prev_error;   /* Previous error for derivative */
	float prev_output;  /* Previous output value */

	/* Output limits */
	float output_min;   /* Minimum output value */
	float output_max;   /* Maximum output value */

	/* Anti-windup */
	float integral_min; /* Minimum integral value */
	float integral_max; /* Maximum integral value */

	/* Time tracking */
	int64_t prev_time_us; /* Previous update time in microseconds */
};

/**
 * @brief Initialize PID controller
 *
 * @param pid Pointer to PID controller structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param output_min Minimum output value
 * @param output_max Maximum output value
 */
void pid_init(struct pid_controller *pid, float kp, float ki, float kd,
              float output_min, float output_max);

/**
 * @brief Reset PID controller state
 *
 * Clears integral accumulator and error history
 *
 * @param pid Pointer to PID controller structure
 */
void pid_reset(struct pid_controller *pid);

/**
 * @brief Update PID controller
 *
 * Computes PID output based on error between setpoint and measurement
 *
 * @param pid Pointer to PID controller structure
 * @param setpoint Desired value
 * @param measurement Current measured value
 * @param dt Time step in seconds (if 0, auto-calculated from system time)
 * @return Control output
 */
float pid_update(struct pid_controller *pid, float setpoint, float measurement, float dt);

/**
 * @brief Set PID gains
 *
 * @param pid Pointer to PID controller structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void pid_set_gains(struct pid_controller *pid, float kp, float ki, float kd);

/**
 * @brief Set output limits
 *
 * @param pid Pointer to PID controller structure
 * @param min Minimum output value
 * @param max Maximum output value
 */
void pid_set_output_limits(struct pid_controller *pid, float min, float max);

/**
 * @brief Set integral limits (anti-windup)
 *
 * @param pid Pointer to PID controller structure
 * @param min Minimum integral value
 * @param max Maximum integral value
 */
void pid_set_integral_limits(struct pid_controller *pid, float min, float max);

#endif /* FOC_PID_H */
