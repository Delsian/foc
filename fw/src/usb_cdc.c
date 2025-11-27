#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/bos.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include "usb_cdc.h"

LOG_MODULE_REGISTER(usb_cdc, LOG_LEVEL_DBG);

#define FOC_USB_VID    0x2FE3
#define FOC_USB_PID    0x0100

/* USB device context */
USBD_DEVICE_DEFINE(foc_usbd,
                   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
                   FOC_USB_VID, FOC_USB_PID);

/* String descriptors */
USBD_DESC_LANG_DEFINE(foc_lang);
USBD_DESC_MANUFACTURER_DEFINE(foc_mfr, "FOC");
USBD_DESC_PRODUCT_DEFINE(foc_product, "FOC STM32G431");

/* Configuration descriptors */
USBD_DESC_CONFIG_DEFINE(foc_fs_cfg_desc, "FS Configuration");

/* USB configuration */
USBD_CONFIGURATION_DEFINE(foc_fs_config,
                          USB_SCD_SELF_POWERED,
                          100, /* 200mA max power */
                          &foc_fs_cfg_desc);

static struct usbd_context *usbd_ctx = &foc_usbd;
static bool usb_enabled = false;
static K_SEM_DEFINE(dtr_sem, 0, 1);

const struct device *const cdc_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

static void usb_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
    LOG_DBG("USB message: %s", usbd_msg_type_string(msg->type));

    /* Handle VBUS detection if supported */
    if (usbd_can_detect_vbus(ctx)) {
        if (msg->type == USBD_MSG_VBUS_READY) {
            if (usbd_enable(ctx)) {
                LOG_ERR("Failed to enable USB on VBUS ready");
            }
        }

        if (msg->type == USBD_MSG_VBUS_REMOVED) {
            if (usbd_disable(ctx)) {
                LOG_ERR("Failed to disable USB on VBUS removed");
            }
        }
    }

    /* Handle DTR signal from host */
    if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
        uint32_t dtr = 0;

        uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
        if (dtr) {
            LOG_INF("DTR set - host connected");
            k_sem_give(&dtr_sem);
        } else {
            LOG_INF("DTR cleared - host disconnected");
        }
    }

    /* Log baudrate changes */
    if (msg->type == USBD_MSG_CDC_ACM_LINE_CODING) {
        uint32_t baudrate = 0;

        if (uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_BAUD_RATE, &baudrate) == 0) {
            LOG_INF("Line coding changed, baudrate: %u", baudrate);
        }
    }
}

int usb_cdc_init(void)
{
    int err;

    /* Check CDC device */
    if (!device_is_ready(cdc_dev)) {
        LOG_ERR("CDC ACM device not ready");
        return -ENODEV;
    }

    /* Add string descriptors */
    err = usbd_add_descriptor(usbd_ctx, &foc_lang);
    if (err) {
        LOG_ERR("Failed to add language descriptor (%d)", err);
        return err;
    }

    err = usbd_add_descriptor(usbd_ctx, &foc_mfr);
    if (err) {
        LOG_ERR("Failed to add manufacturer descriptor (%d)", err);
        return err;
    }

    err = usbd_add_descriptor(usbd_ctx, &foc_product);
    if (err) {
        LOG_ERR("Failed to add product descriptor (%d)", err);
        return err;
    }

    /* Add configuration */
    err = usbd_add_configuration(usbd_ctx, USBD_SPEED_FS, &foc_fs_config);
    if (err) {
        LOG_ERR("Failed to add configuration (%d)", err);
        return err;
    }

    /* Register CDC-ACM class */
    err = usbd_register_all_classes(usbd_ctx, USBD_SPEED_FS, 1, NULL);
    if (err) {
        LOG_ERR("Failed to register classes (%d)", err);
        return err;
    }

    /* Set device code triple for CDC */
    usbd_device_set_code_triple(usbd_ctx, USBD_SPEED_FS,
                                USB_BCC_MISCELLANEOUS, 0x02, 0x01);

    /* Register message callback */
    err = usbd_msg_register_cb(usbd_ctx, usb_msg_cb);
    if (err) {
        LOG_ERR("Failed to register message callback (%d)", err);
        return err;
    }

    /* Initialize USB device */
    err = usbd_init(usbd_ctx);
    if (err) {
        LOG_ERR("Failed to initialize USB device (%d)", err);
        return err;
    }

    /* Enable USB if VBUS detection not supported */
    if (!usbd_can_detect_vbus(usbd_ctx)) {
        err = usbd_enable(usbd_ctx);
        if (err) {
            LOG_ERR("Failed to enable USB device (%d)", err);
            return err;
        }
    }

    usb_enabled = true;
    LOG_INF("USB CDC-ACM initialized");

    return 0;
}

bool usb_cdc_is_ready(void)
{
    if (!usb_enabled) {
        return false;
    }

    uint32_t dtr = 0;
    uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr);

    return (dtr != 0);
}

int usb_cdc_wait_for_dtr(k_timeout_t timeout)
{
    return k_sem_take(&dtr_sem, timeout);
}
