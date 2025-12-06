#ifndef BOARD_H
#define BOARD_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

// SSD1306 OLED Configuration
#define SSD1306_I2C_ADDR    0x3C    // 7-bit I2C address
#define SSD1306_WIDTH       40      // Display width in pixels
#define SSD1306_HEIGHT      72      // Display height in pixels

// I2C device - will be defined in DTS
#define I2C_DEV_NODE DT_NODELABEL(i2c1)

// Function prototypes from Zephyr I2C driver
// (These mirror the STM32 HAL functions used in oled.c)
// Note: oled.c needs to be updated to use Zephyr I2C API

#endif // BOARD_H
