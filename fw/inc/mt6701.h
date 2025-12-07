/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MT6701_H
#define MT6701_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/**
 * @brief MT6701 magnetic angle encoder driver
 *
 * This driver provides I2C communication with the MT6701 14-bit
 * magnetic angle position encoder sensor.
 */

/* MT6701 register addresses */
#define MT6701_REG_ANGLE_H         0x03  /* Angle[13:6] */
#define MT6701_REG_ANGLE_L         0x04  /* Angle[5:0] + status bits */

/* Angle resolution */
#define MT6701_ANGLE_RESOLUTION    16384  /* 14-bit: 2^14 */

/**
 * @brief MT6701 device configuration (from DTS)
 */
struct mt6701_config {
	struct i2c_dt_spec i2c;
};

/**
 * @brief MT6701 device runtime data
 */
struct mt6701_data {
	const struct device *dev;
};

/**
 * @brief Read raw angle value from MT6701
 *
 * @param dev Pointer to MT6701 device
 * @param angle Pointer to store 14-bit angle value (0-16383)
 * @return 0 on success, negative errno on failure
 */
int mt6701_read_angle_raw(const struct device *dev, uint16_t *angle);

/**
 * @brief Read angle in degrees from MT6701
 *
 * @param dev Pointer to MT6701 device
 * @param angle_deg Pointer to store angle in degrees (0.0-360.0)
 * @return 0 on success, negative errno on failure
 */
int mt6701_read_angle_deg(const struct device *dev, float *angle_deg);

/**
 * @brief Read angle in radians from MT6701
 *
 * @param dev Pointer to MT6701 device
 * @param angle_rad Pointer to store angle in radians (0.0-2π)
 * @return 0 on success, negative errno on failure
 */
int mt6701_read_angle_rad(const struct device *dev, float *angle_rad);

#endif /* MT6701_H */
