/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Simple USB CDC-ACM module
 * Based on Zephyr USB CDC-ACM sample
 */

#include "cdcacm.h"
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(cdcacm, LOG_LEVEL_INF);

#define USB_VID 0x2FE3  /* Test VID - do not use in production */
#define USB_PID 0x0001

/* USB descriptor strings */
USBD_DESC_LANG_DEFINE(usb_lang);
USBD_DESC_MANUFACTURER_DEFINE(usb_mfr, "FOC");
USBD_DESC_PRODUCT_DEFINE(usb_product, "FOC STM32G431");
USBD_DESC_CONFIG_DEFINE(usb_cfg_desc, "CDC-ACM Configuration");

/* USB configuration */
USBD_CONFIGURATION_DEFINE(usb_fs_config,
			  USB_SCD_SELF_POWERED,
			  250, /* 500mA max power */
			  &usb_cfg_desc);

/* USB device context instantiation */
USBD_DEVICE_DEFINE(usb_ctx,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   USB_VID, USB_PID);

static struct usbd_context *usb_device_ctx = &usb_ctx;
static bool usb_initialized = false;

/**
 * @brief USB message callback
 */
static void usb_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
	LOG_INF("USB message: %s", usbd_msg_type_string(msg->type));

	/* Handle VBUS detection if supported */
	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(ctx)) {
				LOG_INF("Failed to enable USB on VBUS ready");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(ctx)) {
				LOG_INF("Failed to disable USB on VBUS removed");
			}
		}
	}
}

int cdcacm_init(void)
{
	int err;

	if (usb_initialized) {
		LOG_INF("USB already initialized");
		return 0;
	}

	/* Add string descriptors */
	err = usbd_add_descriptor(usb_device_ctx, &usb_lang);
	if (err) {
		LOG_INF("Failed to add language descriptor: %d", err);
		return err;
	}

	err = usbd_add_descriptor(usb_device_ctx, &usb_mfr);
	if (err) {
		LOG_INF("Failed to add manufacturer descriptor: %d", err);
		return err;
	}

	err = usbd_add_descriptor(usb_device_ctx, &usb_product);
	if (err) {
		LOG_INF("Failed to add product descriptor: %d", err);
		return err;
	}

	/* Add configuration */
	err = usbd_add_configuration(usb_device_ctx, USBD_SPEED_FS, &usb_fs_config);
	if (err) {
		LOG_INF("Failed to add configuration: %d", err);
		return err;
	}

	/* Register CDC-ACM class - auto-registered from device tree */
	err = usbd_register_all_classes(usb_device_ctx, USBD_SPEED_FS, 1, NULL);
	if (err) {
		LOG_INF("Failed to register classes: %d", err);
		return err;
	}

	/* Set device code triple for CDC-ACM */
	usbd_device_set_code_triple(usb_device_ctx, USBD_SPEED_FS,
				    USB_BCC_MISCELLANEOUS, 0x02, 0x01);

	/* Register message callback */
	err = usbd_msg_register_cb(usb_device_ctx, usb_msg_cb);
	if (err) {
		LOG_INF("Failed to register message callback: %d", err);
		return err;
	}

	/* Initialize USB device */
	err = usbd_init(usb_device_ctx);
	if (err) {
		LOG_INF("Failed to initialize USB device: %d", err);
		return err;
	}

	/* Enable USB if VBUS detection not supported */
	if (!usbd_can_detect_vbus(usb_device_ctx)) {
		err = usbd_enable(usb_device_ctx);
		if (err) {
			LOG_INF("Failed to enable USB device: %d", err);
			return err;
		}
	}

	usb_initialized = true;

	return 0;
}

struct usbd_context *cdcacm_get_context(void)
{
	return usb_initialized ? usb_device_ctx : NULL;
}
