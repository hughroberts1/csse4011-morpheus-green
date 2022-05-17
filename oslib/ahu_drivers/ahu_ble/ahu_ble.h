/**
 * @file ahu_ble.h
 * @author Hugh Roberts
 * @brief Contains required globals and defines for ahu_ble.c
 * @version 0.1
 * @date 2022-04-04
 * @copyright Copyright (c) 2022
 * 
 */

void thread_ble_ahu(void);
void thread_button(void);

// DIDs
#define HTS_TEMP 1
#define HTS_HUM 2
#define LPS 3
#define CCS 4
#define LIS2DH_X 5
#define LIS2DH_Y 6
#define LIS2DH_Z 7
#define RGB 8
#define BUZZ 9
#define PB 10

#include "kernel.h"
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#define WAIT_TIME (30 * 1e6) /*seconds*/

#define UUID_BUFFER_SIZE 16

#define CREATE_FLAG(flag) static atomic_t flag = (atomic_t)false
#define SET_FLAG(flag) (void)atomic_set(&flag, (atomic_t)true)
#define UNSET_FLAG(flag) (void)atomic_set(&flag, (atomic_t)false)
#define WAIT_FOR_FLAG(flag) \
	while (!(bool)atomic_get(&flag)) { \
		(void)k_sleep(K_MSEC(1)); \
	}

#define SCU_UUID \
	BT_UUID_DECLARE_128(0x89, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38, 0xad, 0xe9, 0xd2, 0x96, 0x23, 0x3d, 0x57, 0x3c)
#define SCU_CHRC_UUID \
	BT_UUID_DECLARE_128(0x8a, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38, 0xad, 0xe9, 0xd2, 0x96, 0x23, 0x3d, 0x57, 0x3c)

#define AHU_UUID \
	BT_UUID_DECLARE_128(0x8b, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38, 0xad, 0xe9, 0xd2, 0x96, 0x23, 0x3d, 0x57, 0x3c)

#define AHU_CHRC_UUID \
	BT_UUID_DECLARE_128(0x8c, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38, 0xad, 0xe9, 0xd2, 0x96, 0x23, 0x3d, 0x57, 0x3c)


#define NEW_CHRC_UUID \
	BT_UUID_DECLARE_128(0x03, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, \
			    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0x00)



void button_module_init(void);


