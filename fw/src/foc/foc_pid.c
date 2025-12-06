/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "foc_pid.h"
#include "foc_math.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(foc_pid, LOG_LEVEL_DBG);

void pid_init(struct pid_controller *pid, float kp, float ki, float kd,
              float output_min, float output_max)
{
	if (!pid) {
		return;
	}

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
	pid->prev_output = 0.0f;

	pid->output_min = output_min;
	pid->output_max = output_max;

	/* Default integral limits to output limits */
	pid->integral_min = output_min;
	pid->integral_max = output_max;

	pid->prev_time_us = 0;
}

void pid_reset(struct pid_controller *pid)
{
	if (!pid) {
		return;
	}

	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
	pid->prev_output = 0.0f;
	pid->prev_time_us = 0;
}

float pid_update(struct pid_controller *pid, float setpoint, float measurement, float dt)
{
	float error, derivative, output;
	int64_t current_time_us;

	if (!pid) {
		return 0.0f;
	}

	/* Calculate error */
	error = setpoint - measurement;

	/* Auto-calculate dt if not provided */
	if (dt <= 0.0f) {
		current_time_us = k_cyc_to_us_floor64(k_cycle_get_64());
		if (pid->prev_time_us > 0) {
			dt = (float)(current_time_us - pid->prev_time_us) / 1000000.0f;
		} else {
			/* First call, assume small dt */
			dt = 0.001f;
		}
		pid->prev_time_us = current_time_us;
	}

	/* Proportional term */
	float p_term = pid->kp * error;

	/* Integral term with anti-windup */
	pid->integral += error * dt;
	pid->integral = foc_constrain(pid->integral, pid->integral_min, pid->integral_max);
	float i_term = pid->ki * pid->integral;

	/* Derivative term */
	derivative = (error - pid->prev_error) / dt;
	float d_term = pid->kd * derivative;

	/* Calculate output */
	output = p_term + i_term + d_term;

	/* Apply output limits */
	output = foc_constrain(output, pid->output_min, pid->output_max);

	/* Anti-windup: back-calculate integral if output is saturated */
	if (pid->ki != 0.0f) {
		float output_unsaturated = p_term + i_term + d_term;
		if ((output_unsaturated > pid->output_max && error > 0) ||
		    (output_unsaturated < pid->output_min && error < 0)) {
			/* Output is saturated, reduce integral */
			pid->integral = (output - p_term - d_term) / pid->ki;
		}
	}

	/* Store values for next iteration */
	pid->prev_error = error;
	pid->prev_output = output;

	return output;
}

void pid_set_gains(struct pid_controller *pid, float kp, float ki, float kd)
{
	if (!pid) {
		return;
	}

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

void pid_set_output_limits(struct pid_controller *pid, float min, float max)
{
	if (!pid) {
		return;
	}

	pid->output_min = min;
	pid->output_max = max;
}

void pid_set_integral_limits(struct pid_controller *pid, float min, float max)
{
	if (!pid) {
		return;
	}

	pid->integral_min = min;
	pid->integral_max = max;
}
