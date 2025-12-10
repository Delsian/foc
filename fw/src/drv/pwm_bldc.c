/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pwm_bldc.h"
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <math.h>

LOG_MODULE_REGISTER(pwm_bldc, LOG_LEVEL_INF);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Device instances - Motor 0 (TIM2) */
/* TIM2 channels: CH1=1, CH2=2, CH3=3 */
static const struct pwm_bldc_config pwm_bldc_motor0_config = {
	.pwm_ch1 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor0_ch1)),  /* CH1 - Phase A */
	.pwm_ch2 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor0_ch2)),  /* CH2 - Phase B */
	.pwm_ch3 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor0_ch3)),  /* CH3 - Phase C */
	.pwm_period_ns = 50000,  /* 20 kHz PWM frequency (50us period) */
};

static struct pwm_bldc_data pwm_bldc_motor0_data = {
	.initialized = false,
};

/* Device instances - Motor 1 (TIM3) */
/* TIM3 channels: CH2=2, CH3=3, CH4=4 */
static const struct pwm_bldc_config pwm_bldc_motor1_config = {
	.pwm_ch1 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor1_ch1)),  /* CH2 - Phase A */
	.pwm_ch2 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor1_ch2)),  /* CH3 - Phase B */
	.pwm_ch3 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor1_ch3)),  /* CH4 - Phase C */
	.pwm_period_ns = 50000,  /* 20 kHz PWM frequency (50us period) */
};

static struct pwm_bldc_data pwm_bldc_motor1_data = {
	.initialized = false,
};

static int pwm_bldc_init(const struct device *dev)
{
	const struct pwm_bldc_config *config = dev->config;
	struct pwm_bldc_data *data = dev->data;
	int ret;

	/* Check if PWM devices are ready */
	if (!pwm_is_ready_dt(&config->pwm_ch1)) {
		LOG_ERR("%s: PWM channel 1 not ready", dev->name);
		return -ENODEV;
	}

	if (!pwm_is_ready_dt(&config->pwm_ch2)) {
		LOG_ERR("%s: PWM channel 2 not ready", dev->name);
		return -ENODEV;
	}

	if (!pwm_is_ready_dt(&config->pwm_ch3)) {
		LOG_ERR("%s: PWM channel 3 not ready", dev->name);
		return -ENODEV;
	}

	/* Initialize all channels to 0% duty cycle */
	ret = pwm_set_dt(&config->pwm_ch1, config->pwm_period_ns, 0);
	if (ret < 0) {
		LOG_ERR("%s: Failed to init PWM channel 1: %d", dev->name, ret);
		return ret;
	}

	ret = pwm_set_dt(&config->pwm_ch2, config->pwm_period_ns, 0);
	if (ret < 0) {
		LOG_ERR("%s: Failed to init PWM channel 2: %d", dev->name, ret);
		return ret;
	}

	ret = pwm_set_dt(&config->pwm_ch3, config->pwm_period_ns, 0);
	if (ret < 0) {
		LOG_ERR("%s: Failed to init PWM channel 3: %d", dev->name, ret);
		return ret;
	}

	data->initialized = true;
	LOG_INF("%s: PWM BLDC initialized (period: %u ns, freq: %u Hz)",
		dev->name, config->pwm_period_ns,
		(uint32_t)(1000000000ULL / config->pwm_period_ns));

	return 0;
}

int pwm_bldc_set_duty(const struct device *dev, float duty_a, float duty_b, float duty_c)
{
	const struct pwm_bldc_config *config = dev->config;
	struct pwm_bldc_data *data = dev->data;
	uint32_t pulse_a, pulse_b, pulse_c;
	int ret;

	if (!data->initialized) {
		LOG_ERR("%s: Device not initialized", dev->name);
		return -ENODEV;
	}

	/* Clamp duty cycles to 0-100% */
	if (duty_a < 0.0f) duty_a = 0.0f;
	if (duty_a > 100.0f) duty_a = 100.0f;
	if (duty_b < 0.0f) duty_b = 0.0f;
	if (duty_b > 100.0f) duty_b = 100.0f;
	if (duty_c < 0.0f) duty_c = 0.0f;
	if (duty_c > 100.0f) duty_c = 100.0f;

	/* Convert duty cycle percentage to pulse width in nanoseconds */
	pulse_a = (uint32_t)((duty_a / 100.0f) * config->pwm_period_ns);
	pulse_b = (uint32_t)((duty_b / 100.0f) * config->pwm_period_ns);
	pulse_c = (uint32_t)((duty_c / 100.0f) * config->pwm_period_ns);

	/* Set PWM outputs */
	ret = pwm_set_dt(&config->pwm_ch1, config->pwm_period_ns, pulse_a);
	if (ret < 0) {
		LOG_ERR("%s: Failed to set PWM channel 1: %d", dev->name, ret);
		return ret;
	}

	ret = pwm_set_dt(&config->pwm_ch2, config->pwm_period_ns, pulse_b);
	if (ret < 0) {
		LOG_ERR("%s: Failed to set PWM channel 2: %d", dev->name, ret);
		return ret;
	}

	ret = pwm_set_dt(&config->pwm_ch3, config->pwm_period_ns, pulse_c);
	if (ret < 0) {
		LOG_ERR("%s: Failed to set PWM channel 3: %d", dev->name, ret);
		return ret;
	}

	return 0;
}

int pwm_bldc_set_phase_duty(const struct device *dev, uint8_t phase, float duty)
{
	const struct pwm_bldc_config *config = dev->config;
	struct pwm_bldc_data *data = dev->data;
	uint32_t pulse;
	int ret;

	if (!data->initialized) {
		LOG_ERR("%s: Device not initialized", dev->name);
		return -ENODEV;
	}

	if (phase > 2) {
		LOG_ERR("%s: Invalid phase %u (must be 0-2)", dev->name, phase);
		return -EINVAL;
	}

	/* Clamp duty cycle to 0-100% */
	if (duty < 0.0f) duty = 0.0f;
	if (duty > 100.0f) duty = 100.0f;

	/* Convert duty cycle percentage to pulse width in nanoseconds */
	pulse = (uint32_t)((duty / 100.0f) * config->pwm_period_ns);

	/* Set the appropriate PWM channel */
	switch (phase) {
	case 0:  /* Phase A */
		ret = pwm_set_dt(&config->pwm_ch1, config->pwm_period_ns, pulse);
		break;
	case 1:  /* Phase B */
		ret = pwm_set_dt(&config->pwm_ch2, config->pwm_period_ns, pulse);
		break;
	case 2:  /* Phase C */
		ret = pwm_set_dt(&config->pwm_ch3, config->pwm_period_ns, pulse);
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0) {
		LOG_ERR("%s: Failed to set PWM phase %u: %d", dev->name, phase, ret);
		return ret;
	}

	return 0;
}

int pwm_bldc_set_vector(const struct device *dev, float angle_deg, float amplitude)
{
	const struct pwm_bldc_config *config = dev->config;
	struct pwm_bldc_data *data = dev->data;
	float angle_rad;
	float duty_a, duty_b, duty_c;
	float sine_a, sine_b, sine_c;
	uint32_t pulse_a, pulse_b, pulse_c;
	int ret;

	if (!data->initialized) {
		LOG_ERR("%s: Device not initialized", dev->name);
		return -ENODEV;
	}

	/* Clamp amplitude to 0-100% */
	if (amplitude < 0.0f) amplitude = 0.0f;
	if (amplitude > 100.0f) amplitude = 100.0f;

	/* Normalize angle to 0-360 degrees */
	while (angle_deg < 0.0f) angle_deg += 360.0f;
	while (angle_deg >= 360.0f) angle_deg -= 360.0f;

	/* Convert to radians */
	angle_rad = angle_deg * M_PI / 180.0f;

	/* Calculate three-phase sinusoidal values with 120-degree spacing
	 * Phase A: sin(θ)
	 * Phase B: sin(θ - 120°)
	 * Phase C: sin(θ - 240°) = sin(θ + 120°)
	 */
	sine_a = sinf(angle_rad);
	sine_b = sinf(angle_rad - 2.0f * M_PI / 3.0f);  /* -120 degrees */
	sine_c = sinf(angle_rad + 2.0f * M_PI / 3.0f);  /* +120 degrees */

	/* Convert sine values (-1 to +1) to duty cycles (0 to 100%)
	 * Using bipolar modulation: duty = 50% + (sine * amplitude/2)
	 */
	duty_a = 50.0f + (sine_a * amplitude / 2.0f);
	duty_b = 50.0f + (sine_b * amplitude / 2.0f);
	duty_c = 50.0f + (sine_c * amplitude / 2.0f);

	/* Clamp to valid range */
	if (duty_a < 0.0f) duty_a = 0.0f;
	if (duty_a > 100.0f) duty_a = 100.0f;
	if (duty_b < 0.0f) duty_b = 0.0f;
	if (duty_b > 100.0f) duty_b = 100.0f;
	if (duty_c < 0.0f) duty_c = 0.0f;
	if (duty_c > 100.0f) duty_c = 100.0f;

	/* Convert duty cycles to pulse widths */
	pulse_a = (uint32_t)((duty_a / 100.0f) * config->pwm_period_ns);
	pulse_b = (uint32_t)((duty_b / 100.0f) * config->pwm_period_ns);
	pulse_c = (uint32_t)((duty_c / 100.0f) * config->pwm_period_ns);

	/* Set PWM outputs */
	ret = pwm_set_dt(&config->pwm_ch1, config->pwm_period_ns, pulse_a);
	if (ret < 0) {
		LOG_ERR("%s: Failed to set PWM channel 1: %d", dev->name, ret);
		return ret;
	}

	ret = pwm_set_dt(&config->pwm_ch2, config->pwm_period_ns, pulse_b);
	if (ret < 0) {
		LOG_ERR("%s: Failed to set PWM channel 2: %d", dev->name, ret);
		return ret;
	}

	ret = pwm_set_dt(&config->pwm_ch3, config->pwm_period_ns, pulse_c);
	if (ret < 0) {
		LOG_ERR("%s: Failed to set PWM channel 3: %d", dev->name, ret);
		return ret;
	}

	return 0;
}

int pwm_bldc_disable(const struct device *dev)
{
	const struct pwm_bldc_config *config = dev->config;
	struct pwm_bldc_data *data = dev->data;
	int ret;

	if (!data->initialized) {
		LOG_ERR("%s: Device not initialized", dev->name);
		return -ENODEV;
	}

	/* Set all channels to 0% duty cycle */
	ret = pwm_set_dt(&config->pwm_ch1, config->pwm_period_ns, 0);
	if (ret < 0) {
		LOG_ERR("%s: Failed to disable PWM channel 1: %d", dev->name, ret);
		return ret;
	}

	ret = pwm_set_dt(&config->pwm_ch2, config->pwm_period_ns, 0);
	if (ret < 0) {
		LOG_ERR("%s: Failed to disable PWM channel 2: %d", dev->name, ret);
		return ret;
	}

	ret = pwm_set_dt(&config->pwm_ch3, config->pwm_period_ns, 0);
	if (ret < 0) {
		LOG_ERR("%s: Failed to disable PWM channel 3: %d", dev->name, ret);
		return ret;
	}

	LOG_INF("%s: PWM outputs disabled", dev->name);
	return 0;
}

/* Device state structures */
static struct device_state pwm_bldc_motor0_state = {
	.init_res = 0,
	.initialized = true,
};

static struct device_state pwm_bldc_motor1_state = {
	.init_res = 0,
	.initialized = true,
};

/* Static device instances for get_device */
static const struct device pwm_bldc_device_motor0 = {
	.name = "pwm_bldc_motor0",
	.config = &pwm_bldc_motor0_config,
	.data = &pwm_bldc_motor0_data,
	.state = &pwm_bldc_motor0_state,
};

static const struct device pwm_bldc_device_motor1 = {
	.name = "pwm_bldc_motor1",
	.config = &pwm_bldc_motor1_config,
	.data = &pwm_bldc_motor1_data,
	.state = &pwm_bldc_motor1_state,
};

const struct device *pwm_bldc_get_device(const char *alias)
{
	if (strcmp(alias, "pwm-motor0") == 0) {
		return &pwm_bldc_device_motor0;
	} else if (strcmp(alias, "pwm-motor1") == 0) {
		return &pwm_bldc_device_motor1;
	}

	LOG_ERR("Unknown PWM BLDC alias: %s", alias);
	return NULL;
}

/* Auto-initialization at boot */
static int pwm_bldc_motor0_auto_init(void)
{
	return pwm_bldc_init(&pwm_bldc_device_motor0);
}

static int pwm_bldc_motor1_auto_init(void)
{
	return pwm_bldc_init(&pwm_bldc_device_motor1);
}

SYS_INIT(pwm_bldc_motor0_auto_init, APPLICATION, 90);
SYS_INIT(pwm_bldc_motor1_auto_init, APPLICATION, 90);
