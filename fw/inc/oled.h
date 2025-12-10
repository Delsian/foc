#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Update the OLED display with current buffer content
 *
 * This function sends the display buffer to the OLED screen.
 */
void oled_update(void);

/**
 * @brief Write a number to the OLED display
 *
 * @param n Number to display (0-999)
 *
 * Displays a number on the OLED screen using large 16x24 font.
 * Numbers greater than 999 are ignored.
 */
void oled_write(uint16_t n);

/**
 * @brief Set cursor position on OLED display
 *
 * @param x X coordinate
 * @param y Y coordinate
 */
void oled_set_pos(uint8_t x, uint8_t y);

/**
 * @brief Set display contrast
 *
 * @param value Contrast value (0-255)
 */
void ssd1306_SetContrast(const uint8_t value);

#endif // OLED_H
