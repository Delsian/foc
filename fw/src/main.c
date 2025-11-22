#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

int main(void)
{
    int err;

    LOG_INF("Starting");

    err = dk_leds_init();
    if (err) {
        LOG_ERR("LEDs init failed (err %d)", err);
        return err;
    }

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}