#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Clear the OLED display buffer
 *
 * Clears the internal buffer. Call oled_update() to refresh the display.
 */
void oled_clear(void);

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
 * @param line Line number (vertical position)
 * @param pos Position on the line (horizontal position)
 *
 * Displays a number on the OLED screen using 12x16 font.
 * Numbers greater than 999 are ignored.
 */
void oled_write(uint16_t n, uint8_t line, uint8_t pos);

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
