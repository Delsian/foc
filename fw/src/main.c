#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "oled.h"
#include "cdcacm.h"
#include "console.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
    uint8_t val = 0;

    LOG_INF("FOC STM32G431 Starting");

#ifdef CONFIG_ENABLE_USB
    cdcacm_init();
#endif /* CONFIG_ENABLE_USB */

    while (1) {
        oled_write(val++);
        k_sleep(K_MSEC(500));
    }

    return 0;
}
