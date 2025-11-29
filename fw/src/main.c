#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "oled.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

int main(void)
{
    uint8_t val = 0;

    LOG_INF("Starting");

    // Initialize OLED display (failures are handled internally)
    oled_init();

    while (1) {
        oled_write(val++);
        k_sleep(K_MSEC(500));
    }

    return 0;
}