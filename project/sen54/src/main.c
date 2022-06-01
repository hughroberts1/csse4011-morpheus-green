#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <sys/printk.h>

/* Include files from our library */
#include "scu_ble.h"
#include "scu_power_management.h"
#include "scu_sensors.h"
#include "sen5x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

K_THREAD_DEFINE(scu_sen54_thread_tid, THREAD_SCU_SEN54_POLL_STACK, thread_scu_sen54_poll,
		NULL, NULL, NULL, THREAD_SCU_SEN54_POLL_PRIORITY, 0, 0);
