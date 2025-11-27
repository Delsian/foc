#ifndef USB_CDC_H
#define USB_CDC_H

#include <zephyr/kernel.h>

/**
 * Initialize USB CDC-ACM device
 *
 * Sets up USB device stack, registers CDC-ACM class, and enables USB.
 * Must be called before using any other USB CDC functions.
 *
 * @return 0 on success, negative errno on failure
 */
int usb_cdc_init(void);

/**
 * Check if USB CDC-ACM is ready for communication
 *
 * Checks if USB is enabled and DTR (Data Terminal Ready) signal is set,
 * which indicates a host terminal is connected.
 *
 * @return true if ready for communication, false otherwise
 */
bool usb_cdc_is_ready(void);

/**
 * Wait for DTR signal from host
 *
 * Blocks until the host sets the DTR signal, indicating a terminal
 * application has opened the serial port.
 *
 * @param timeout Maximum time to wait (use K_FOREVER to wait indefinitely)
 * @return 0 if DTR received, -EAGAIN if timeout occurred
 */
int usb_cdc_wait_for_dtr(k_timeout_t timeout);

#endif /* USB_CDC_H */
