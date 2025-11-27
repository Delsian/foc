#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "oled.h"
#include "usb_cdc.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

int main(void)
{
    uint8_t val = 0;
    int ret;

    LOG_INF("Starting FOC firmware");


    /* Initialize USB CDC-ACM */
    ret = usb_cdc_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize USB CDC-ACM: %d", ret);
    } else {
        LOG_INF("USB CDC-ACM ready");
    }

    /* Initialize OLED display */
    oled_init();

    while (1) {
        oled_write(val++);
        k_sleep(K_MSEC(500));
    }

    return 0;
}