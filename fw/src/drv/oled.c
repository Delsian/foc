#include "board.h"
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(oled, LOG_LEVEL_INF);

static const struct device *i2c_dev;
static bool oled_initialized = false;
static uint8_t buffer[(SSD1306_WIDTH/8) * SSD1306_HEIGHT];

/**
 * @brief Send data to SSD1306 via I2C
 *
 * @param addr Control byte (0x00 for command, 0x40 for data)
 * @param bytes Data buffer to send
 * @param len Length of data
 * @return 0 on success, negative error code on failure
 */
static int oled_send(uint8_t addr, const uint8_t *bytes, uint8_t len) {
    uint8_t buf[len + 1];
    int ret;

    buf[0] = addr;
    memcpy(&buf[1], bytes, len);

    ret = i2c_write(i2c_dev, buf, len + 1, SSD1306_I2C_ADDR);
    if (ret != 0) {
        LOG_INF("I2C write failed: %d", ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Send a single byte command to SSD1306
 */
static int ssd1306_WriteCommand(uint8_t byte) {
    return oled_send(0x00, &byte, 1);
}

/**
 * @brief Send a sequence of commands to SSD1306
 */
static int ssd1306_WriteCommandSeq(const uint8_t *bytes, size_t len) {
    int ret;

    for(size_t i = 0; i < len; i++) {
        ret = oled_send(0x00, &bytes[i], 1);
        if (ret != 0) {
            return ret;
        }
    }

    return 0;
}

/**
 * @brief Send data to SSD1306 display
 */
static int ssd1306_WriteData(const uint8_t* data, size_t buff_size) {
    return oled_send(0x40, data, buff_size);
}

void ssd1306_SetContrast(const uint8_t value) {
    const uint8_t kSetContrastControlRegister = 0x81;
    ssd1306_WriteCommand(kSetContrastControlRegister);
    ssd1306_WriteCommand(value);
}

static const uint8_t init_seq[] = {
    0xae,  // --turn off oled panel

    0xd5,  // --set display clock divide ratio/oscillator frequency
    0x80,  // --set divide ratio

    0xa8,  // --set multiplex ratio
    0x3F,  // 0x27,//--1/40 duty (using 0x3F for 64 multiplex, 0x27 for 40)

    0xd3,  // -set display offset
    0x00,  // -not offset

    0x20,  // Addressing Setting Command Table
    0x00,  // horizontal addressing mode

    0xad,  // --Internal IREF Setting
    0x30,  // --

    0x8d,  // --set Charge Pump enable/disable
    0x14,  // --set(0x10) disable

    0x68,  // 0x40,//--set start line address

    0xa6,  // --set normal display

    0xa4,  // Disable Entire Display On

    0xa1,  // --set segment re-map 128 to 0

    0xC8,  // --Set COM Output Scan Direction 64 to 0

    0xda,  // --set com pins hardware configuration
    0x12,

    0x81,  // --set contrast control register
    0x7f,

    0xd9,  // --set pre-charge period
    0x22,

    0xdb,  // --set vcomh
    0x40,

    0xaf   // --turn on oled panel
};

static int oled_init(void) {
    int ret;

    // Get I2C device binding
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c_dev)) {
        LOG_INF("I2C device not ready");
        oled_initialized = false;
        return ENODEV;
    }

    // Send initialization sequence
    ret = ssd1306_WriteCommandSeq(init_seq, sizeof(init_seq));
    if (ret != 0) {
        LOG_INF("Failed to initialize SSD1306: %d", ret);
        oled_initialized = false;
        return ret;
    }

    LOG_INF("SSD1306 initialized");
    oled_initialized = true;
    return 0;
}

void oled_update(void)
{
    const uint8_t clr_seq[] = {
        0x21, 28, 99, 0x22, 0, 4
    };

    ssd1306_WriteCommandSeq(clr_seq, sizeof(clr_seq));

    for (int i = 0; i < 72; i++) {
        ssd1306_WriteData(&buffer[5 * i], 5);
    }
}

void oled_set_pos(uint8_t x, uint8_t y) {
    uint8_t pos_cmd[] = {0x21, x, x+1, 0x22, y, y+25};
    ssd1306_WriteCommandSeq(pos_cmd, sizeof(pos_cmd));
}

static const uint8_t font12x16[] = {
    // 0
    0x00, 0xE0, 0xF8, 0x1C, 0x0E, 0x06, 0x06, 0x0E, 0x1C, 0xF8, 0xE0, 0x00,
    0x00, 0x0F, 0x3F, 0x70, 0xE0, 0xC0, 0xC0, 0xE0, 0x70, 0x3F, 0x0F, 0x00,

    // 1
    0x00, 0x00, 0x30, 0x18, 0x1C, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,

    // 2
    0x00, 0x38, 0x1C, 0x0E, 0x06, 0x06, 0x06, 0x0E, 0x9C, 0xF8, 0x70, 0x00,
    0x00, 0xE0, 0xF0, 0xF8, 0xDC, 0xCE, 0xC7, 0xC3, 0xC1, 0xC0, 0xC0, 0x00,

    // 3
    0x00, 0x18, 0x0C, 0x0E, 0x86, 0x86, 0x86, 0xCE, 0xFC, 0x78, 0x00, 0x00,
    0x00, 0x30, 0x60, 0xE0, 0xC1, 0xC1, 0xC1, 0xE3, 0x7F, 0x3E, 0x00, 0x00,

    // 4
    0x00, 0x00, 0x80, 0xC0, 0x60, 0x30, 0x18, 0xFE, 0xFE, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x0F, 0x0C, 0x0C, 0x0C, 0x0C, 0xFF, 0xFF, 0x0C, 0x0C, 0x00,

    // 5
    0x00, 0x00, 0xFE, 0xFE, 0xC6, 0xC6, 0xC6, 0xC6, 0x86, 0x06, 0x00, 0x00,
    0x00, 0x30, 0x61, 0xE1, 0xC1, 0xC1, 0xC1, 0xE3, 0x7F, 0x3E, 0x00, 0x00,

    // 6
    0x00, 0xE0, 0xF8, 0x9C, 0xCE, 0xC6, 0xC6, 0xC6, 0x8E, 0x1C, 0x00, 0x00,
    0x00, 0x0F, 0x3F, 0x71, 0xE1, 0xC1, 0xC1, 0xE1, 0x7F, 0x3F, 0x00, 0x00,

    // 7
    0x00, 0x06, 0x06, 0x06, 0x06, 0x06, 0x86, 0xE6, 0x7E, 0x1E, 0x06, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xC0, 0xF8, 0x3F, 0x07, 0x00, 0x00, 0x00, 0x00,

    // 8
    0x00, 0x78, 0xFC, 0xCE, 0x86, 0x86, 0x86, 0xCE, 0xFC, 0x78, 0x00, 0x00,
    0x00, 0x3E, 0x7F, 0xE3, 0xC1, 0xC1, 0xC1, 0xE3, 0x7F, 0x3E, 0x00, 0x00,

    // 9
    0x00, 0xF8, 0xFC, 0x8E, 0x06, 0x06, 0x06, 0x0E, 0x9C, 0xF8, 0xE0, 0x00,
    0x00, 0x00, 0xE1, 0xC3, 0xC3, 0xC3, 0xE7, 0x7E, 0x3F, 0x0F, 0x01, 0x00,
};

static void copy_buf(const uint8_t* ptr, uint8_t line, uint8_t pos) {
    uint16_t offset = line * SSD1306_HEIGHT + pos;
    for(int i = 0; i < 12; i++) {
        buffer[i + offset] |= ptr[i];
        buffer[i + offset + SSD1306_HEIGHT] |= ptr[i + 12];
    }
}

void oled_clear(void) {
    if (!oled_initialized) {
        return;
    }
    memset(buffer, 0, sizeof(buffer));
}

void oled_write(uint16_t n, uint8_t line, uint8_t pos) {
    bool show = false;

    // Exit early if OLED is not initialized
    if (!oled_initialized) {
        return;
    }

    if (n > 999) {
        return;
    }

    if (n > 99) {
        copy_buf(&font12x16[(n/100) * 24], line, pos);
        n -= (n/100) * 100;
        pos += 12;
        show = true;
    }

    if (n > 9 || show) {
        copy_buf(&font12x16[(n/10) * 24], line, pos);
        n -= (n/10) * 10;
        pos += 12;
    }

    copy_buf(&font12x16[n * 24], line, pos);
}

SYS_INIT(oled_init, APPLICATION, 1);