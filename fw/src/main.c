#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

int main(void)
{
    LOG_INF("Starting");

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}