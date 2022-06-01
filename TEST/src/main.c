
/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

#include <string.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/gatt.h>
#include <drivers/sensor.h>

#include "mesh.h"
#include "board.h"

static uint8_t dev_uuid[16];


static void bt_ready(int err) 
{
    if (err) {
        printk("Bluetooth init failed\n");
        return;
    }

    printk("Bluetooth initialised\n");

    err = mesh_init();
    if (err) {
        printk("Initialising mesh failed\n");
        return;
    }

    printk("Mesh initialised\n");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    printk("Board started\n");
}

void main(void)
{
    int err;

    if (IS_ENABLED(CONFIG_HWINFO)) {
		err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
	}

    if (err < 0) {
        dev_uuid[0] = 0xdd;
        dev_uuid[1] = 0xdd;
    }

    err = board_init();

    if (err) {
        printk("Board init failed\n");
        return;
    }

    err = bt_enable(bt_ready); 
    if (err) {
        printk("Bluetooth init failed\n");
    }
}