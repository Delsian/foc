/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mt6701.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <math.h>

LOG_MODULE_REGISTER(mt6701, LOG_LEVEL_INF);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Device instances */
static const struct mt6701_config mt6701_motor0_config = {
	.i2c = I2C_DT_SPEC_GET(DT_ALIAS(encoder_motor0)),
};

static const struct mt6701_config mt6701_motor1_config = {
	.i2c = I2C_DT_SPEC_GET(DT_ALIAS(encoder_motor1)),
};

static struct mt6701_data mt6701_motor0_data;
static struct mt6701_data mt6701_motor1_data;

static int mt6701_init(const struct device *dev)
{
	const struct mt6701_config *config = dev->config;
	uint8_t test_data;
	int ret;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("%s: I2C bus not ready", dev->name);
		return -ENODEV;
	}

	LOG_INF("%s: MT6701 initialized on I2C addr 0x%02x",
		dev->name, config->i2c.addr);

	/* Try to read a register to verify device is present and responding */
	ret = i2c_reg_read_byte_dt(&config->i2c, MT6701_REG_ANGLE_H, &test_data);
	if (ret < 0) {
		LOG_WRN("%s: Device not responding on I2C (error %d) - check hardware/mode",
			dev->name, ret);
		/* Don't fail init - device might not be connected yet */
	} else {
		LOG_INF("%s: Device responded successfully (test read: 0x%02x)",
			dev->name, test_data);
	}

	return 0;
}

int mt6701_read_angle_raw(const struct device *dev, uint16_t *angle)
{
	const struct mt6701_config *config = dev->config;
	uint8_t data[2];
	int ret;

	if (!dev || !angle) {
		return -EINVAL;
	}

	/* Read angle high byte (register 0x03) */
	ret = i2c_reg_read_byte_dt(&config->i2c, MT6701_REG_ANGLE_H, &data[0]);
	if (ret < 0) {
		LOG_ERR("%s: Failed to read angle high byte: %d", dev->name, ret);
		return ret;
	}

	/* Read angle low byte (register 0x04) */
	ret = i2c_reg_read_byte_dt(&config->i2c, MT6701_REG_ANGLE_L, &data[1]);
	if (ret < 0) {
		LOG_ERR("%s: Failed to read angle low byte: %d", dev->name, ret);
		return ret;
	}

	/* Combine into 14-bit angle value
	 * Register 0x03: Angle[13:6]
	 * Register 0x04: Angle[5:0] in bits [7:2], bits [1:0] are status
	 */
	*angle = ((uint16_t)data[0] << 6) | ((data[1] >> 2) & 0x3F);

	return 0;
}

int mt6701_read_angle_deg(const struct device *dev, float *angle_deg)
{
	uint16_t raw_angle;
	int ret;

	if (!dev || !angle_deg) {
		return -EINVAL;
	}

	ret = mt6701_read_angle_raw(dev, &raw_angle);
	if (ret < 0) {
		return ret;
	}

	/* Convert 14-bit value (0-16383) to degrees (0-360)
	 * Formula from datasheet: angle = (raw * 360) / 16384
	 */
	*angle_deg = ((float)raw_angle * 360.0f) / (float)MT6701_ANGLE_RESOLUTION;

	return 0;
}

int mt6701_read_angle_rad(const struct device *dev, float *angle_rad)
{
	uint16_t raw_angle;
	int ret;

	if (!dev || !angle_rad) {
		return -EINVAL;
	}

	ret = mt6701_read_angle_raw(dev, &raw_angle);
	if (ret < 0) {
		return ret;
	}

	/* Convert 14-bit value (0-16383) to radians (0-2π)
	 * Formula: angle = (raw * 2π) / 16384
	 */
	*angle_rad = ((float)raw_angle * 2.0f * (float)M_PI) / (float)MT6701_ANGLE_RESOLUTION;

	return 0;
}

/* Define devices using DT_ALIAS */
DEVICE_DT_DEFINE(DT_ALIAS(encoder_motor0),
		 mt6701_init,
		 NULL,
		 &mt6701_motor0_data,
		 &mt6701_motor0_config,
		 POST_KERNEL,
		 75, /* Init priority: after I2C (70) */
		 NULL);

DEVICE_DT_DEFINE(DT_ALIAS(encoder_motor1),
		 mt6701_init,
		 NULL,
		 &mt6701_motor1_data,
		 &mt6701_motor1_config,
		 POST_KERNEL,
		 75, /* Init priority: after I2C (70) */
		 NULL);
