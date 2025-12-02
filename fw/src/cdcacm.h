/*
 * Copyright (c) 2025 FOC Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CDCACM_H
#define CDCACM_H

#ifdef CONFIG_ENABLE_USB
#include <zephyr/usb/usbd.h>

/**
 * @brief Initialize USB CDC-ACM device
 *
 * This function initializes the USB device stack and enables
 * CDC-ACM virtual serial port.
 *
 * @return 0 on success, negative error code on failure
 */
int cdcacm_init(void);

/**
 * @brief Get USB device context
 *
 * @return Pointer to USB device context, or NULL if not initialized
 */
struct usbd_context *cdcacm_get_context(void);

#else
/* Stub functions when USB is disabled */
static inline int cdcacm_init(void) { return 0; }
static inline void *cdcacm_get_context(void) { return NULL; }
#endif /* CONFIG_ENABLE_USB */

#endif /* CDCACM_H */
