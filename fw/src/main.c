#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "oled.h"
#include "cdcacm.h"
#include "console.h"
#include "pwm_bldc.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
    const struct device *pwm_motor0;

    LOG_INF("FOC STM32G431 Starting");

#ifdef CONFIG_ENABLE_USB
    cdcacm_init();
#endif /* CONFIG_ENABLE_USB */

    /* Get PWM BLDC device by alias */
    pwm_motor0 = pwm_bldc_get_device("pwm-motor0");
    if (pwm_motor0 == NULL) {
        LOG_ERR("Failed to get PWM motor0 device");
    } else if (!device_is_ready(pwm_motor0)) {
        LOG_ERR("PWM motor0 device not ready");
    } else {
        LOG_INF("PWM motor0 initialized successfully");

        /* Set initial duty cycle to 0% on all phases */
        pwm_bldc_disable(pwm_motor0);
    }

    while (1) {
        pwm_bldc_update_oled();
        k_sleep(K_MSEC(500));
    }

    return 0;
}
