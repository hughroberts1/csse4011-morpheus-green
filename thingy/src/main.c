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
#include <stdio.h>
#include <stdlib.h>
#include <settings/settings.h>
#include <devicetree.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/hwinfo.h>
#include <sys/byteorder.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <zephyr.h>

#include "board.h"
#include "mesh.h"

void main(void)
{
	int err = -1;

	printk("Initializing...\n");

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
}