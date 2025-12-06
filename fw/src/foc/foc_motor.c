/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "foc_motor.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(foc_motor, LOG_LEVEL_INF);

/* Low-pass filter coefficient for velocity calculation */
#define VELOCITY_LPF_ALPHA 0.2f

static int foc_motor_set_pwm(struct foc_motor *motor, const struct abc_frame *voltages);
static int foc_motor_read_sensor(struct foc_motor *motor);
static void foc_motor_calculate_velocity(struct foc_motor *motor, float dt);

int foc_motor_init(struct foc_motor *motor,
                   const struct foc_motor_params *params,
                   const struct mt6701_dev *sensor,
                   const struct foc_pwm_output *pwm_output)
{
	if (!motor || !params || !sensor || !pwm_output) {
		return -EINVAL;
	}

	/* Copy parameters */
	memcpy(&motor->params, params, sizeof(struct foc_motor_params));
	motor->sensor = sensor;
	memcpy(&motor->pwm, pwm_output, sizeof(struct foc_pwm_output));

	/* Check PWM device */
	if (!device_is_ready(motor->pwm.pwm_dev)) {
		LOG_ERR("PWM device not ready");
		return -ENODEV;
	}

	/* Initialize PIDs with default gains */
	/* Velocity PID (outputs voltage q) */
	pid_init(&motor->pid_velocity, 0.5f, 5.0f, 0.0f,
	         -motor->params.voltage_limit, motor->params.voltage_limit);

	/* Position PID (outputs velocity target) */
	pid_init(&motor->pid_position, 20.0f, 0.0f, 0.1f,
	         -motor->params.velocity_limit, motor->params.velocity_limit);

	/* D-axis PID (flux control, typically set to zero) */
	pid_init(&motor->pid_d, 1.0f, 10.0f, 0.0f,
	         -motor->params.voltage_limit, motor->params.voltage_limit);

	/* Q-axis PID (torque control) */
	pid_init(&motor->pid_q, 1.0f, 10.0f, 0.0f,
	         -motor->params.voltage_limit, motor->params.voltage_limit);

	/* Initialize state */
	motor->mode = FOC_MODE_VELOCITY;
	motor->state = FOC_STATE_DISABLED;
	motor->angle_mechanical = 0.0f;
	motor->angle_electrical = 0.0f;
	motor->velocity = 0.0f;
	motor->position_target = 0.0f;
	motor->velocity_target = 0.0f;
	motor->voltage_q = 0.0f;
	motor->voltage_d = 0.0f;
	motor->current_q_target = 0.0f;
	motor->current_d_target = 0.0f;
	motor->current_q = 0.0f;
	motor->current_d = 0.0f;
	motor->sensor_offset = 0.0f;
	motor->sensor_calibrated = false;
	motor->current_sense = NULL;
	motor->current_sense_enabled = false;
	motor->prev_time_us = 0;

	LOG_INF("FOC motor initialized: %d pole pairs, %.1fV limit",
	        params->pole_pairs, params->voltage_limit);

	return 0;
}

int foc_motor_enable(struct foc_motor *motor)
{
	if (!motor) {
		return -EINVAL;
	}

	if (motor->state == FOC_STATE_ERROR) {
		LOG_ERR("Cannot enable motor in error state");
		return -EFAULT;
	}

	/* Reset PIDs */
	pid_reset(&motor->pid_velocity);
	pid_reset(&motor->pid_position);
	pid_reset(&motor->pid_d);
	pid_reset(&motor->pid_q);

	/* Reset time tracking */
	motor->prev_time_us = k_cyc_to_us_floor64(k_cycle_get_64());

	motor->state = FOC_STATE_IDLE;
	LOG_INF("Motor enabled");

	return 0;
}

int foc_motor_disable(struct foc_motor *motor)
{
	struct abc_frame zero_voltages = { 0.0f, 0.0f, 0.0f };

	if (!motor) {
		return -EINVAL;
	}

	/* Set all PWM outputs to zero */
	foc_motor_set_pwm(motor, &zero_voltages);

	motor->state = FOC_STATE_DISABLED;
	motor->voltage_q = 0.0f;
	motor->voltage_d = 0.0f;

	LOG_INF("Motor disabled");

	return 0;
}

int foc_motor_set_mode(struct foc_motor *motor, enum foc_control_mode mode)
{
	if (!motor) {
		return -EINVAL;
	}

	/* Check if torque mode requires current sensing */
	if (mode == FOC_MODE_TORQUE && !motor->current_sense_enabled) {
		LOG_ERR("Torque mode requires current sensing - link current sense first");
		return -EINVAL;
	}

	motor->mode = mode;
	LOG_INF("Motor mode set to %d", mode);

	return 0;
}

int foc_motor_link_current_sense(struct foc_motor *motor,
                                  struct current_sense_dev *current_sense)
{
	if (!motor || !current_sense) {
		return -EINVAL;
	}

	motor->current_sense = current_sense;
	motor->current_sense_enabled = true;

	/* Update PID output limits to current instead of voltage */
	pid_set_output_limits(&motor->pid_q,
	                       -motor->params.current_limit,
	                       motor->params.current_limit);
	pid_set_output_limits(&motor->pid_d,
	                       -motor->params.current_limit,
	                       motor->params.current_limit);

	LOG_INF("Current sensing linked to motor");

	return 0;
}

void foc_motor_set_velocity_pid(struct foc_motor *motor, float kp, float ki, float kd)
{
	if (!motor) {
		return;
	}

	pid_set_gains(&motor->pid_velocity, kp, ki, kd);
	LOG_INF("Velocity PID: Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
}

void foc_motor_set_position_pid(struct foc_motor *motor, float kp, float ki, float kd)
{
	if (!motor) {
		return;
	}

	pid_set_gains(&motor->pid_position, kp, ki, kd);
	LOG_INF("Position PID: Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
}

void foc_motor_set_velocity(struct foc_motor *motor, float velocity)
{
	if (!motor) {
		return;
	}

	motor->velocity_target = foc_constrain(velocity,
	                                        -motor->params.velocity_limit,
	                                        motor->params.velocity_limit);
}

void foc_motor_set_position(struct foc_motor *motor, float position)
{
	if (!motor) {
		return;
	}

	motor->position_target = position;
}

void foc_motor_set_voltage(struct foc_motor *motor, float uq, float ud)
{
	if (!motor) {
		return;
	}

	motor->voltage_q = foc_constrain(uq,
	                                  -motor->params.voltage_limit,
	                                  motor->params.voltage_limit);
	motor->voltage_d = foc_constrain(ud,
	                                  -motor->params.voltage_limit,
	                                  motor->params.voltage_limit);
}

void foc_motor_set_current(struct foc_motor *motor, float iq, float id)
{
	if (!motor) {
		return;
	}

	motor->current_q_target = foc_constrain(iq,
	                                         -motor->params.current_limit,
	                                         motor->params.current_limit);
	motor->current_d_target = foc_constrain(id,
	                                         -motor->params.current_limit,
	                                         motor->params.current_limit);
}

int foc_motor_loop(struct foc_motor *motor)
{
	struct abc_frame voltages;
	int64_t current_time_us;
	float dt;
	int ret;

	if (!motor) {
		return -EINVAL;
	}

	if (motor->state == FOC_STATE_DISABLED || motor->state == FOC_STATE_ERROR) {
		return 0;
	}

	/* Calculate time step */
	current_time_us = k_cyc_to_us_floor64(k_cycle_get_64());
	if (motor->prev_time_us > 0) {
		dt = (float)(current_time_us - motor->prev_time_us) / 1000000.0f;
	} else {
		dt = 0.001f; /* 1ms default */
	}
	motor->prev_time_us = current_time_us;

	/* Read sensor position */
	ret = foc_motor_read_sensor(motor);
	if (ret < 0) {
		LOG_ERR("Sensor read failed: %d", ret);
		motor->state = FOC_STATE_ERROR;
		return ret;
	}

	/* Calculate velocity */
	foc_motor_calculate_velocity(motor, dt);

	/* Read currents if current sensing is enabled */
	if (motor->current_sense_enabled) {
		struct dq_frame currents;
		ret = current_sense_read_dq(motor->current_sense,
		                             motor->angle_electrical,
		                             &currents);
		if (ret == 0) {
			motor->current_d = currents.d;
			motor->current_q = currents.q;
		}
	}

	/* Run control algorithm based on mode */
	switch (motor->mode) {
	case FOC_MODE_VOLTAGE:
		/* Direct voltage control - voltage_q and voltage_d already set */
		motor->state = FOC_STATE_RUNNING;
		break;

	case FOC_MODE_VELOCITY:
		/* Velocity control: PID(velocity) -> voltage_q */
		if (motor->current_sense_enabled) {
			/* With current sensing: velocity PID -> current target -> voltage */
			motor->current_q_target = pid_update(&motor->pid_velocity,
			                                      motor->velocity_target,
			                                      motor->velocity,
			                                      dt);
			motor->current_d_target = 0.0f;
			/* Current PIDs output voltage */
			motor->voltage_q = pid_update(&motor->pid_q,
			                               motor->current_q_target,
			                               motor->current_q,
			                               dt);
			motor->voltage_d = pid_update(&motor->pid_d,
			                               motor->current_d_target,
			                               motor->current_d,
			                               dt);
		} else {
			/* Without current sensing: velocity PID -> voltage */
			motor->voltage_q = pid_update(&motor->pid_velocity,
			                               motor->velocity_target,
			                               motor->velocity,
			                               dt);
			motor->voltage_d = 0.0f;
		}
		motor->state = FOC_STATE_RUNNING;
		break;

	case FOC_MODE_POSITION:
		/* Position control: PID(position) -> velocity_target -> voltage_q */
		motor->velocity_target = pid_update(&motor->pid_position,
		                                     motor->position_target,
		                                     motor->angle_mechanical,
		                                     dt);
		if (motor->current_sense_enabled) {
			/* With current sensing */
			motor->current_q_target = pid_update(&motor->pid_velocity,
			                                      motor->velocity_target,
			                                      motor->velocity,
			                                      dt);
			motor->current_d_target = 0.0f;
			motor->voltage_q = pid_update(&motor->pid_q,
			                               motor->current_q_target,
			                               motor->current_q,
			                               dt);
			motor->voltage_d = pid_update(&motor->pid_d,
			                               motor->current_d_target,
			                               motor->current_d,
			                               dt);
		} else {
			/* Without current sensing */
			motor->voltage_q = pid_update(&motor->pid_velocity,
			                               motor->velocity_target,
			                               motor->velocity,
			                               dt);
			motor->voltage_d = 0.0f;
		}
		motor->state = FOC_STATE_RUNNING;
		break;

	case FOC_MODE_TORQUE:
		/* Torque control: direct current control (requires current sensing) */
		if (!motor->current_sense_enabled) {
			LOG_ERR("Torque mode requires current sensing");
			motor->state = FOC_STATE_ERROR;
			return -EINVAL;
		}
		/* Current PIDs: target current -> voltage */
		motor->voltage_q = pid_update(&motor->pid_q,
		                               motor->current_q_target,
		                               motor->current_q,
		                               dt);
		motor->voltage_d = pid_update(&motor->pid_d,
		                               motor->current_d_target,
		                               motor->current_d,
		                               dt);
		motor->state = FOC_STATE_RUNNING;
		break;

	default:
		LOG_ERR("Unknown control mode: %d", motor->mode);
		motor->state = FOC_STATE_ERROR;
		return -EINVAL;
	}

	/* Apply FOC algorithm: convert Uq, Ud to three-phase voltages */
	foc_svm(motor->voltage_q, motor->voltage_d,
	        motor->angle_electrical,
	        motor->params.voltage_limit,
	        &voltages);

	/* Set PWM outputs */
	ret = foc_motor_set_pwm(motor, &voltages);
	if (ret < 0) {
		LOG_ERR("PWM set failed: %d", ret);
		motor->state = FOC_STATE_ERROR;
		return ret;
	}

	return 0;
}

int foc_motor_calibrate_sensor(struct foc_motor *motor)
{
	/* Simple calibration: read current sensor position as zero offset
	 * More sophisticated calibration would involve rotating the motor
	 * and finding the electrical zero position
	 */
	float angle_raw;
	int ret;

	if (!motor) {
		return -EINVAL;
	}

	motor->state = FOC_STATE_CALIBRATING;
	LOG_INF("Starting sensor calibration...");

	ret = mt6701_read_angle_rad(motor->sensor, &angle_raw);
	if (ret < 0) {
		LOG_ERR("Sensor read failed during calibration: %d", ret);
		motor->state = FOC_STATE_ERROR;
		return ret;
	}

	motor->sensor_offset = angle_raw;
	motor->sensor_calibrated = true;
	motor->state = FOC_STATE_IDLE;

	LOG_INF("Sensor calibration complete: offset = %.3f rad", motor->sensor_offset);

	return 0;
}

enum foc_motor_state foc_motor_get_state(const struct foc_motor *motor)
{
	return motor ? motor->state : FOC_STATE_ERROR;
}

float foc_motor_get_velocity(const struct foc_motor *motor)
{
	return motor ? motor->velocity : 0.0f;
}

float foc_motor_get_position(const struct foc_motor *motor)
{
	return motor ? motor->angle_mechanical : 0.0f;
}

/* Private helper functions */

static int foc_motor_read_sensor(struct foc_motor *motor)
{
	float angle_raw;
	int ret;

	ret = mt6701_read_angle_rad(motor->sensor, &angle_raw);
	if (ret < 0) {
		return ret;
	}

	/* Apply sensor offset if calibrated */
	if (motor->sensor_calibrated) {
		motor->angle_mechanical = foc_normalize_angle(angle_raw - motor->sensor_offset);
	} else {
		motor->angle_mechanical = angle_raw;
	}

	/* Calculate electrical angle */
	motor->angle_electrical = foc_electrical_angle(motor->angle_mechanical,
	                                                motor->params.pole_pairs);

	return 0;
}

static void foc_motor_calculate_velocity(struct foc_motor *motor, float dt)
{
	static float prev_angle = 0.0f;
	float angle_diff;
	float velocity_raw;

	/* Calculate angular difference, handling wraparound */
	angle_diff = motor->angle_mechanical - prev_angle;

	/* Handle angle wraparound (crossing 0/2π boundary) */
	if (angle_diff > M_PI) {
		angle_diff -= M_2PI;
	} else if (angle_diff < -M_PI) {
		angle_diff += M_2PI;
	}

	/* Calculate raw velocity */
	if (dt > 0.0f) {
		velocity_raw = angle_diff / dt;
	} else {
		velocity_raw = 0.0f;
	}

	/* Apply low-pass filter to reduce noise */
	motor->velocity = VELOCITY_LPF_ALPHA * velocity_raw +
	                  (1.0f - VELOCITY_LPF_ALPHA) * motor->velocity;

	prev_angle = motor->angle_mechanical;
}

static int foc_motor_set_pwm(struct foc_motor *motor, const struct abc_frame *voltages)
{
	uint32_t duty_a, duty_b, duty_c;
	float voltage_range;
	int ret;

	if (!motor || !voltages) {
		return -EINVAL;
	}

	/* Convert voltages to duty cycle (0-100%)
	 * Voltages are in range [-voltage_limit, +voltage_limit]
	 * Center at 50% duty cycle
	 */
	voltage_range = motor->params.voltage_supply;

	/* Normalize to 0-1 range and convert to PWM period */
	duty_a = (uint32_t)((0.5f + voltages->a / (2.0f * voltage_range)) *
	                     motor->pwm.pwm_period_ns);
	duty_b = (uint32_t)((0.5f + voltages->b / (2.0f * voltage_range)) *
	                     motor->pwm.pwm_period_ns);
	duty_c = (uint32_t)((0.5f + voltages->c / (2.0f * voltage_range)) *
	                     motor->pwm.pwm_period_ns);

	/* Clamp to valid range */
	duty_a = CLAMP(duty_a, 0, motor->pwm.pwm_period_ns);
	duty_b = CLAMP(duty_b, 0, motor->pwm.pwm_period_ns);
	duty_c = CLAMP(duty_c, 0, motor->pwm.pwm_period_ns);

	/* Set PWM outputs */
	ret = pwm_set(motor->pwm.pwm_dev, motor->pwm.channel_a,
	              motor->pwm.pwm_period_ns, duty_a, 0);
	if (ret < 0) {
		return ret;
	}

	ret = pwm_set(motor->pwm.pwm_dev, motor->pwm.channel_b,
	              motor->pwm.pwm_period_ns, duty_b, 0);
	if (ret < 0) {
		return ret;
	}

	ret = pwm_set(motor->pwm.pwm_dev, motor->pwm.channel_c,
	              motor->pwm.pwm_period_ns, duty_c, 0);
	if (ret < 0) {
		return ret;
	}

	return 0;
}
