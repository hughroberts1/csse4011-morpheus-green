/**
 * @file cmd.c
 * @author Hugh Roberts
 * @brief LED and time commands for controlling AHU
 * @version 0.1
 * @date 2022-04-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <kernel.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>
#include <shell/shell.h>

#include "led_driver.h"
#include "cmd.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(time_module);
		
/* Command Handler for printing System time
 */
static int cmd_sys_time(const struct shell *shell, size_t argc, 
						char **argv)
{
	if (argc == 2) {
		if ((strcmp(argv[1], "f"))) {
			LOG_ERR("invalid command");
			return 0;
		}	
	}

	int64_t uptime = k_uptime_get();
	LOG_INF("System Time: %lld seconds", uptime / 1000);
	return 0;
}

/* Command Handler for printing system time, formatted */
static int cmd_sys_time_f(const struct shell *shell, size_t argc, 
						char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int64_t uptime = k_uptime_get();
	int64_t seconds = uptime / 1000;
	int minutes = seconds / 60;
	int hours = minutes / 60;
	LOG_INF("System Time: %d hours, %d minutes, %lld seconds", hours, minutes % 60, seconds % 60);

	return 0;
}

/* Specify Shell Commands for System Time */
/* Creating subcommands (level 1 command) array for command "time". */
SHELL_STATIC_SUBCMD_SET_CREATE(sys_time, 
		SHELL_CMD(f, NULL, "Time formatted.", cmd_sys_time_f),
		SHELL_SUBCMD_SET_END
);

/* Creating root (level 0) commmand "time" */
SHELL_CMD_REGISTER(time, &sys_time, "System Time", cmd_sys_time);
