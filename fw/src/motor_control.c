/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_control.h"
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_INF);

/* Forward declaration of thread entry function */
static void motor_control_thread_entry(void *arg1, void *arg2, void *arg3);

/* Thread stack definitions - will be allocated dynamically per motor */
#define MOTOR_THREAD_STACK_SIZE 2048

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
                       const struct motor_control_config *config)
{
	int ret;

	if (!ctx || !i2c_dev || !pwm_dev || !config) {
		return -EINVAL;
	}

	/* Initialize context */
	memset(ctx, 0, sizeof(struct motor_control_ctx));
	ctx->motor_id = motor_id;
	ctx->i2c_dev = i2c_dev;
	ctx->pwm_dev = pwm_dev;
	ctx->adc_dev = adc_dev;
	ctx->encoder_i2c_addr = encoder_addr;
	ctx->pwm_channel_a = pwm_ch_a;
	ctx->pwm_channel_b = pwm_ch_b;
	ctx->pwm_channel_c = pwm_ch_c;
	ctx->adc_channel_a = adc_ch_a;
	ctx->adc_channel_b = adc_ch_b;
	ctx->thread_running = false;

	LOG_INF("Initializing motor %d", motor_id);

	/* Check I2C device */
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("Motor %d: I2C device not ready", motor_id);
		return -ENODEV;
	}

	/* Initialize MT6701 encoder */
	ret = mt6701_init(&ctx->encoder, i2c_dev, encoder_addr);
	if (ret < 0) {
		LOG_ERR("Motor %d: Failed to initialize MT6701: %d", motor_id, ret);
		return ret;
	}
	LOG_INF("Motor %d: Encoder initialized", motor_id);

	/* Check PWM device */
	if (!device_is_ready(pwm_dev)) {
		LOG_ERR("Motor %d: PWM device not ready", motor_id);
		return -ENODEV;
	}

	/* Setup motor parameters */
	ctx->params.pole_pairs = config->pole_pairs;
	ctx->params.phase_resistance = config->phase_resistance;
	ctx->params.phase_inductance = config->phase_inductance;
	ctx->params.voltage_supply = config->voltage_supply;
	ctx->params.voltage_limit = config->voltage_limit;
	ctx->params.velocity_limit = config->velocity_limit;
	ctx->params.current_limit = config->current_limit;

	/* Setup PWM configuration */
	struct foc_pwm_output pwm_config = {
		.pwm_dev = pwm_dev,
		.channel_a = pwm_ch_a,
		.channel_b = pwm_ch_b,
		.channel_c = pwm_ch_c,
		.pwm_period_ns = config->pwm_period_ns,
	};

	/* Initialize FOC motor */
	ret = foc_motor_init(&ctx->motor, &ctx->params, &ctx->encoder, &pwm_config);
	if (ret < 0) {
		LOG_ERR("Motor %d: Failed to initialize FOC: %d", motor_id, ret);
		return ret;
	}
	LOG_INF("Motor %d: FOC motor initialized", motor_id);

	/* Initialize current sensing if ADC is provided */
	if (adc_dev != NULL && device_is_ready(adc_dev)) {
		/* Setup amplifier configuration */
		ctx->amp_config.gain = config->current_sense_gain;
		ctx->amp_config.shunt_resistance = config->current_sense_shunt;
		ctx->amp_config.offset_voltage = config->current_sense_offset;
		ctx->amp_config.vref = config->current_sense_vref;

		/* Initialize current sensing */
		ret = current_sense_init(&ctx->current_sense, adc_dev,
		                         adc_ch_a, adc_ch_b,
		                         &ctx->amp_config);
		if (ret < 0) {
			LOG_WRN("Motor %d: Current sense init failed: %d", motor_id, ret);
		} else {
			/* Calibrate current sensing */
			ret = current_sense_calibrate(&ctx->current_sense, 100);
			if (ret < 0) {
				LOG_WRN("Motor %d: Current sense calibration failed: %d",
				        motor_id, ret);
			} else {
				/* Link current sensing to motor */
				foc_motor_link_current_sense(&ctx->motor, &ctx->current_sense);
				LOG_INF("Motor %d: Current sensing enabled", motor_id);
			}
		}
	} else {
		LOG_INF("Motor %d: Current sensing disabled (no ADC)", motor_id);
	}

	LOG_INF("Motor %d: Initialization complete", motor_id);
	return 0;
}

/* Motor control thread function */
static void motor_control_thread_entry(void *arg1, void *arg2, void *arg3)
{
	struct motor_control_ctx *ctx = (struct motor_control_ctx *)arg1;
	int ret;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_INF("Motor %d: Control thread started", ctx->motor_id);

	/* Wait for system to stabilize */
	k_sleep(K_MSEC(100));

	/* Enable motor */
	ret = foc_motor_enable(&ctx->motor);
	if (ret < 0) {
		LOG_ERR("Motor %d: Failed to enable: %d", ctx->motor_id, ret);
		ctx->thread_running = false;
		return;
	}

	/* Optional: Calibrate sensor alignment */
	ret = foc_motor_calibrate_sensor(&ctx->motor);
	if (ret < 0) {
		LOG_WRN("Motor %d: Sensor calibration failed: %d", ctx->motor_id, ret);
	}

	/* Set default control mode */
	foc_motor_set_mode(&ctx->motor, FOC_MODE_VELOCITY);
	foc_motor_set_velocity(&ctx->motor, 0.0f); /* Start at zero velocity */

	LOG_INF("Motor %d: Entering control loop", ctx->motor_id);

	/* Control loop */
	while (ctx->thread_running) {
		ret = foc_motor_loop(&ctx->motor);
		if (ret < 0) {
			LOG_ERR("Motor %d: FOC loop error: %d", ctx->motor_id, ret);
			k_sleep(K_MSEC(100));
			continue;
		}

		/* Sleep for control loop period (default 1ms = 1kHz) */
		k_sleep(K_USEC(1000));
	}

	/* Disable motor before exiting */
	foc_motor_disable(&ctx->motor);
	LOG_INF("Motor %d: Control thread stopped", ctx->motor_id);
}

/* Thread stacks - statically allocated for both motors */
K_THREAD_STACK_DEFINE(motor0_stack, MOTOR_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(motor1_stack, MOTOR_THREAD_STACK_SIZE);

static struct k_thread motor0_thread_data;
static struct k_thread motor1_thread_data;

int motor_control_start_thread(struct motor_control_ctx *ctx,
                               size_t stack_size,
                               int priority)
{
	if (!ctx) {
		return -EINVAL;
	}

	if (ctx->thread_running) {
		LOG_WRN("Motor %d: Thread already running", ctx->motor_id);
		return -EALREADY;
	}

	/* Select appropriate stack based on motor ID */
	k_thread_stack_t *stack;
	struct k_thread *thread_data;

	if (ctx->motor_id == 0) {
		stack = motor0_stack;
		thread_data = &motor0_thread_data;
	} else if (ctx->motor_id == 1) {
		stack = motor1_stack;
		thread_data = &motor1_thread_data;
	} else {
		LOG_ERR("Motor %d: Invalid motor ID (must be 0 or 1)", ctx->motor_id);
		return -EINVAL;
	}

	/* Use provided stack size or default */
	if (stack_size == 0) {
		stack_size = MOTOR_THREAD_STACK_SIZE;
	}

	ctx->thread_running = true;

	/* Create thread */
	ctx->thread_id = k_thread_create(thread_data, stack, stack_size,
	                                 motor_control_thread_entry,
	                                 ctx, NULL, NULL,
	                                 priority, 0, K_NO_WAIT);

	if (ctx->thread_id == NULL) {
		LOG_ERR("Motor %d: Failed to create thread", ctx->motor_id);
		ctx->thread_running = false;
		return -ENOMEM;
	}

	/* Set thread name */
	char thread_name[16];
	snprintk(thread_name, sizeof(thread_name), "motor_%d", ctx->motor_id);
	k_thread_name_set(ctx->thread_id, thread_name);

	LOG_INF("Motor %d: Thread started with priority %d", ctx->motor_id, priority);
	return 0;
}

void motor_control_stop_thread(struct motor_control_ctx *ctx)
{
	if (!ctx || !ctx->thread_running) {
		return;
	}

	LOG_INF("Motor %d: Stopping thread", ctx->motor_id);
	ctx->thread_running = false;

	/* Wait for thread to finish */
	k_thread_join(ctx->thread_id, K_FOREVER);
}

void motor_control_set_mode(struct motor_control_ctx *ctx, enum foc_control_mode mode)
{
	if (ctx) {
		foc_motor_set_mode(&ctx->motor, mode);
	}
}

void motor_control_set_velocity(struct motor_control_ctx *ctx, float velocity)
{
	if (ctx) {
		foc_motor_set_velocity(&ctx->motor, velocity);
	}
}

void motor_control_set_position(struct motor_control_ctx *ctx, float position)
{
	if (ctx) {
		foc_motor_set_position(&ctx->motor, position);
	}
}

void motor_control_set_current(struct motor_control_ctx *ctx, float iq, float id)
{
	if (ctx) {
		foc_motor_set_current(&ctx->motor, iq, id);
	}
}

float motor_control_get_velocity(struct motor_control_ctx *ctx)
{
	return ctx ? foc_motor_get_velocity(&ctx->motor) : 0.0f;
}

float motor_control_get_position(struct motor_control_ctx *ctx)
{
	return ctx ? foc_motor_get_position(&ctx->motor) : 0.0f;
}

enum foc_motor_state motor_control_get_state(struct motor_control_ctx *ctx)
{
	return ctx ? foc_motor_get_state(&ctx->motor) : FOC_STATE_DISABLED;
}

int motor_control_enable(struct motor_control_ctx *ctx)
{
	if (!ctx) {
		return -EINVAL;
	}
	return foc_motor_enable(&ctx->motor);
}

void motor_control_disable(struct motor_control_ctx *ctx)
{
	if (ctx) {
		foc_motor_disable(&ctx->motor);
	}
}
