/**
 * @file main.c
 * @author Hugh Roberts
 * @brief Application main entry point
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <sys/printk.h>
#include <settings/settings.h>
#include <devicetree.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/hwinfo.h>
#include <sys/byteorder.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>       
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr.h>
#include <shell/shell.h>
#include <time.h>
#include <logging/log.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/util.h>
#include <inttypes.h>

#include "board.h"
#include "mesh.h"

#define COMMAND_LINE_STACK_SIZE 1024
#define COMMAND_LINE_PRIORITY 10

void command_line_thread(void);

K_THREAD_DEFINE(command_line, COMMAND_LINE_STACK_SIZE, command_line_thread, NULL, NULL, NULL, COMMAND_LINE_PRIORITY, 0, 0);

LOG_MODULE_REGISTER(base, LOG_LEVEL_DBG);

void command_line_thread(void) {
	/* Setup DTR */
	const struct device *console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	/* Wait on DTR - 'Data Terminal Ready'
	 * Will wait here until a terminal has been attached to the device
	 * This is not necessary, however, can be useful from reading early data
	 */
	while (!dtr) {
		uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}
}

void main(void)
{
	/* Enable the USB Driver */
    if (usb_enable(NULL)){
        return;
	}

	int err = -1;

	err = board_init();

	if (err) {
		printk("Board init failed (err: %d)\n", err);
		return;
	}

	err = bt_init();

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	provision();
}