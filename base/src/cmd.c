/**
 * @file cmd.c
 * @author Hugh Roberts
 * @brief shell commands for base node
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
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

#include "mesh.h"

static int cmd_pressure(const struct shell *, size_t, char **);
static int cmd_humidity(const struct shell *, size_t, char **);
static int cmd_temperature(const struct shell *, size_t, char **);
static int cmd_voc(const struct shell *, size_t, char **);
static int cmd_co2(const struct shell *, size_t, char **);
static int cmd_pm10(const struct shell *, size_t, char **);

static int cmd_list_nodes(const struct shell *, size_t, char **);

SHELL_CMD_REGISTER(pressure, NULL, "Read pressure from all nodes", cmd_pressure);
SHELL_CMD_REGISTER(humidity, NULL, "Read humidity from all nodes", cmd_humidity);
SHELL_CMD_REGISTER(temperature, NULL, "Read temperature from all nodes", cmd_temperature);
SHELL_CMD_REGISTER(voc, NULL, "Read VOC from all nodes", cmd_voc);
SHELL_CMD_REGISTER(co2, NULL, "Read CO2 from all nodes", cmd_co2);
SHELL_CMD_REGISTER(pm10, NULL, "Read PM10 from all nodes", cmd_pm10);

SHELL_CMD_REGISTER(list_nodes, NULL, "List all nodes currently connected to mesh network", cmd_list_nodes);

static int cmd_list_nodes(const struct shell *shell, size_t argc, char **argv)
{
	return list_nodes();
}

static int cmd_pressure(const struct shell *shell, size_t argc, char **argv) 
{
	return sensor_request(PRESSURE);
}

static int cmd_humidity(const struct shell *shell, size_t argc, char **argv) 
{
	return sensor_request(HUMIDITY);
}

static int cmd_temperature(const struct shell *shell, size_t argc, char **argv) 
{
	return sensor_request(TEMPERATURE);
}

static int cmd_voc(const struct shell *shell, size_t argc, char **argv) {

	return sensor_request(VOC);
}

static int cmd_co2(const struct shell *shell, size_t argc, char **argv) {

	return sensor_request(CO2);
}

static int cmd_pm10(const struct shell *shell, size_t argc, char **argv) {
	return sensor_request(PM10);
}