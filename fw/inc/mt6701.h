/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MT6701_H
#define MT6701_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>

/**
 * @brief MT6701 magnetic angle encoder driver
 *
 * This driver provides I2C communication with the MT6701 14-bit
 * magnetic angle position encoder sensor.
 */

/* MT6701 I2C slave address (7-bit) */
#define MT6701_I2C_ADDR_DEFAULT    0x06

/* MT6701 register addresses */
#define MT6701_REG_ANGLE_H         0x03  /* Angle[13:6] */
#define MT6701_REG_ANGLE_L         0x04  /* Angle[5:0] + status bits */

/* Angle resolution */
#define MT6701_ANGLE_RESOLUTION    16384  /* 14-bit: 2^14 */

/**
 * @brief MT6701 device structure
 */
struct mt6701_dev {
	const struct device *i2c_dev;
	uint8_t i2c_addr;
};

/**
 * @brief Initialize MT6701 device
 *
 * @param dev Pointer to MT6701 device structure
 * @param i2c_dev Pointer to I2C device
 * @param i2c_addr I2C slave address (7-bit)
 * @return 0 on success, negative errno on failure
 */
int mt6701_init(struct mt6701_dev *dev, const struct device *i2c_dev, uint8_t i2c_addr);

/**
 * @brief Read raw angle value from MT6701
 *
 * @param dev Pointer to MT6701 device structure
 * @param angle Pointer to store 14-bit angle value (0-16383)
 * @return 0 on success, negative errno on failure
 */
int mt6701_read_angle_raw(const struct mt6701_dev *dev, uint16_t *angle);

/**
 * @brief Read angle in degrees from MT6701
 *
 * @param dev Pointer to MT6701 device structure
 * @param angle_deg Pointer to store angle in degrees (0.0-360.0)
 * @return 0 on success, negative errno on failure
 */
int mt6701_read_angle_deg(const struct mt6701_dev *dev, float *angle_deg);

/**
 * @brief Read angle in radians from MT6701
 *
 * @param dev Pointer to MT6701 device structure
 * @param angle_rad Pointer to store angle in radians (0.0-2π)
 * @return 0 on success, negative errno on failure
 */
int mt6701_read_angle_rad(const struct mt6701_dev *dev, float *angle_rad);

#endif /* MT6701_H */
