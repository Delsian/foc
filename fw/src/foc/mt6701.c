/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mt6701.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(mt6701, LOG_LEVEL_INF);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int mt6701_init(struct mt6701_dev *dev, const struct device *i2c_dev, uint8_t i2c_addr)
{
	if (!dev || !i2c_dev) {
		return -EINVAL;
	}

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	dev->i2c_dev = i2c_dev;
	dev->i2c_addr = i2c_addr;

	LOG_INF("MT6701 initialized on I2C addr 0x%02x", i2c_addr);

	return 0;
}

int mt6701_read_angle_raw(const struct mt6701_dev *dev, uint16_t *angle)
{
	uint8_t reg_addr;
	uint8_t data[2];
	int ret;

	if (!dev || !angle) {
		return -EINVAL;
	}

	/* Read angle high byte (register 0x03) */
	reg_addr = MT6701_REG_ANGLE_H;
	ret = i2c_write_read(dev->i2c_dev, dev->i2c_addr,
			     &reg_addr, sizeof(reg_addr),
			     &data[0], 1);
	if (ret < 0) {
		LOG_ERR("Failed to read angle high byte: %d", ret);
		return ret;
	}

	/* Read angle low byte (register 0x04) */
	reg_addr = MT6701_REG_ANGLE_L;
	ret = i2c_write_read(dev->i2c_dev, dev->i2c_addr,
			     &reg_addr, sizeof(reg_addr),
			     &data[1], 1);
	if (ret < 0) {
		LOG_ERR("Failed to read angle low byte: %d", ret);
		return ret;
	}

	/* Combine into 14-bit angle value
	 * Register 0x03: Angle[13:6]
	 * Register 0x04: Angle[5:0] in bits [7:2], bits [1:0] are status
	 */
	*angle = ((uint16_t)data[0] << 6) | ((data[1] >> 2) & 0x3F);

	return 0;
}

int mt6701_read_angle_deg(const struct mt6701_dev *dev, float *angle_deg)
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

int mt6701_read_angle_rad(const struct mt6701_dev *dev, float *angle_rad)
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
