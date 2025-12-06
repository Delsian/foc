/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FOC_MOTOR_H
#define FOC_MOTOR_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include "foc_math.h"
#include "foc_pid.h"
#include "foc_current_sense.h"
#include "mt6701.h"

/**
 * @brief Motor control modes
 */
enum foc_control_mode {
	FOC_MODE_VOLTAGE,      /* Direct voltage control (open loop) */
	FOC_MODE_VELOCITY,     /* Velocity control with PID */
	FOC_MODE_POSITION,     /* Position control with PID */
	FOC_MODE_TORQUE,       /* Torque control (requires current sensing) */
};

/**
 * @brief Motor state
 */
enum foc_motor_state {
	FOC_STATE_DISABLED,    /* Motor disabled */
	FOC_STATE_INIT,        /* Initializing */
	FOC_STATE_CALIBRATING, /* Calibrating sensor alignment */
	FOC_STATE_IDLE,        /* Enabled but not moving */
	FOC_STATE_RUNNING,     /* Running control loop */
	FOC_STATE_ERROR,       /* Error state */
};

/**
 * @brief Motor parameters
 */
struct foc_motor_params {
	int pole_pairs;           /* Number of motor pole pairs */
	float phase_resistance;   /* Phase resistance in Ohms */
	float phase_inductance;   /* Phase inductance in Henries */
	float voltage_supply;     /* Supply voltage */
	float voltage_limit;      /* Maximum voltage to apply to motor */
	float velocity_limit;     /* Maximum velocity in rad/s */
	float current_limit;      /* Maximum current in Amps (if current sensing) */
};

/**
 * @brief PWM output structure
 */
struct foc_pwm_output {
	const struct device *pwm_dev;
	uint32_t channel_a;
	uint32_t channel_b;
	uint32_t channel_c;
	uint32_t pwm_period_ns;  /* PWM period in nanoseconds */
};

/**
 * @brief Motor control structure
 */
struct foc_motor {
	/* Motor parameters */
	struct foc_motor_params params;

	/* Sensor */
	const struct mt6701_dev *sensor;

	/* Current sensing (optional) */
	struct current_sense_dev *current_sense;
	bool current_sense_enabled;

	/* PWM outputs */
	struct foc_pwm_output pwm;

	/* Control mode and state */
	enum foc_control_mode mode;
	enum foc_motor_state state;

	/* PID controllers */
	struct pid_controller pid_velocity;
	struct pid_controller pid_position;
	struct pid_controller pid_d;  /* Direct axis current/voltage */
	struct pid_controller pid_q;  /* Quadrature axis current/voltage */

	/* Position and velocity measurements */
	float angle_mechanical;   /* Mechanical shaft angle (rad) */
	float angle_electrical;   /* Electrical angle (rad) */
	float velocity;           /* Angular velocity (rad/s) */
	float position_target;    /* Target position (rad) */
	float velocity_target;    /* Target velocity (rad/s) */

	/* Voltage commands */
	float voltage_q;          /* Quadrature voltage command */
	float voltage_d;          /* Direct voltage command */

	/* Current targets and measurements (when current sensing enabled) */
	float current_q_target;   /* Target Q-axis current */
	float current_d_target;   /* Target D-axis current */
	float current_q;          /* Measured Q-axis current */
	float current_d;          /* Measured D-axis current */

	/* Calibration data */
	float sensor_offset;      /* Sensor zero offset */
	bool sensor_calibrated;   /* Sensor calibration status */

	/* Time tracking */
	int64_t prev_time_us;
};

/**
 * @brief Initialize FOC motor controller
 *
 * @param motor Pointer to motor structure
 * @param params Motor parameters
 * @param sensor Pointer to position sensor
 * @param pwm_output PWM output configuration
 * @return 0 on success, negative errno on failure
 */
int foc_motor_init(struct foc_motor *motor,
                   const struct foc_motor_params *params,
                   const struct mt6701_dev *sensor,
                   const struct foc_pwm_output *pwm_output);

/**
 * @brief Enable motor controller
 *
 * @param motor Pointer to motor structure
 * @return 0 on success, negative errno on failure
 */
int foc_motor_enable(struct foc_motor *motor);

/**
 * @brief Disable motor controller
 *
 * Sets all PWM outputs to zero and disables control
 *
 * @param motor Pointer to motor structure
 * @return 0 on success, negative errno on failure
 */
int foc_motor_disable(struct foc_motor *motor);

/**
 * @brief Set motor control mode
 *
 * @param motor Pointer to motor structure
 * @param mode Control mode
 * @return 0 on success, negative errno on failure
 */
int foc_motor_set_mode(struct foc_motor *motor, enum foc_control_mode mode);

/**
 * @brief Link current sense module to motor
 *
 * Enables current-based control modes (torque control)
 *
 * @param motor Pointer to motor structure
 * @param current_sense Pointer to current sense device
 * @return 0 on success, negative errno on failure
 */
int foc_motor_link_current_sense(struct foc_motor *motor,
                                  struct current_sense_dev *current_sense);

/**
 * @brief Set velocity PID gains
 *
 * @param motor Pointer to motor structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void foc_motor_set_velocity_pid(struct foc_motor *motor, float kp, float ki, float kd);

/**
 * @brief Set position PID gains
 *
 * @param motor Pointer to motor structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void foc_motor_set_position_pid(struct foc_motor *motor, float kp, float ki, float kd);

/**
 * @brief Set target velocity
 *
 * @param motor Pointer to motor structure
 * @param velocity Target velocity in rad/s
 */
void foc_motor_set_velocity(struct foc_motor *motor, float velocity);

/**
 * @brief Set target position
 *
 * @param motor Pointer to motor structure
 * @param position Target position in radians
 */
void foc_motor_set_position(struct foc_motor *motor, float position);

/**
 * @brief Set direct voltage (Uq, Ud)
 *
 * For voltage control mode
 *
 * @param motor Pointer to motor structure
 * @param uq Quadrature voltage
 * @param ud Direct voltage
 */
void foc_motor_set_voltage(struct foc_motor *motor, float uq, float ud);

/**
 * @brief Set target current (Iq, Id)
 *
 * For torque control mode (requires current sensing)
 *
 * @param motor Pointer to motor structure
 * @param iq Quadrature current (torque producing)
 * @param id Direct current (flux producing, typically 0)
 */
void foc_motor_set_current(struct foc_motor *motor, float iq, float id);

/**
 * @brief Run FOC control loop (call this regularly, e.g., 1-10 kHz)
 *
 * Reads sensor, updates control algorithms, and sets PWM outputs
 *
 * @param motor Pointer to motor structure
 * @return 0 on success, negative errno on failure
 */
int foc_motor_loop(struct foc_motor *motor);

/**
 * @brief Calibrate sensor alignment
 *
 * Rotates motor to find sensor zero position
 *
 * @param motor Pointer to motor structure
 * @return 0 on success, negative errno on failure
 */
int foc_motor_calibrate_sensor(struct foc_motor *motor);

/**
 * @brief Get current motor state
 *
 * @param motor Pointer to motor structure
 * @return Current motor state
 */
enum foc_motor_state foc_motor_get_state(const struct foc_motor *motor);

/**
 * @brief Get current velocity
 *
 * @param motor Pointer to motor structure
 * @return Current velocity in rad/s
 */
float foc_motor_get_velocity(const struct foc_motor *motor);

/**
 * @brief Get current position
 *
 * @param motor Pointer to motor structure
 * @return Current position in radians
 */
float foc_motor_get_position(const struct foc_motor *motor);

#endif /* FOC_MOTOR_H */
