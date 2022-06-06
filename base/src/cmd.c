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

/* Shell command prototypes */
static int cmd_pressure(const struct shell *, size_t, char **);
static int cmd_humidity(const struct shell *, size_t, char **);
static int cmd_temperature(const struct shell *, size_t, char **);
static int cmd_voc(const struct shell *, size_t, char **);
static int cmd_co2(const struct shell *, size_t, char **);
static int cmd_pm10(const struct shell *, size_t, char **);
static int cmd_list_nodes(const struct shell *, size_t, char **);
static int cmd_all_on(const struct shell *, size_t, char **);
static int cmd_all_off(const struct shell *, size_t, char **);
static int cmd_set_sample_period(const struct shell *, size_t, char **);

/* Register shell commands with callbacks */
SHELL_CMD_REGISTER(pressure, NULL, "Read pressure from all nodes", cmd_pressure);
SHELL_CMD_REGISTER(humidity, NULL, "Read humidity from all nodes", cmd_humidity);
SHELL_CMD_REGISTER(temperature, NULL, "Read temperature from all nodes", cmd_temperature);
SHELL_CMD_REGISTER(voc, NULL, "Read VOC from all nodes", cmd_voc);
SHELL_CMD_REGISTER(co2, NULL, "Read CO2 from all nodes", cmd_co2);
SHELL_CMD_REGISTER(pm10, NULL, "Read PM10 from all nodes", cmd_pm10);
SHELL_CMD_REGISTER(list_nodes, NULL, "List all nodes currently connected to mesh network", cmd_list_nodes);
SHELL_CMD_REGISTER(all_on, NULL, "Continuously request all nodes data", cmd_all_on);
SHELL_CMD_REGISTER(all_off, NULL, "Turn off continuous sampling", cmd_all_off);
SHELL_CMD_REGISTER(sample_period, NULL, "Set sample period of continuous sampling (seconds)", cmd_set_sample_period);

/**
 * @brief Display a list of all nodes active in the mesh network 
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_list_nodes(const struct shell *shell, size_t argc, char **argv)
{
	return list_nodes();
}

/**
 * @brief Request pressure reading from all nodes in the network 
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_pressure(const struct shell *shell, size_t argc, char **argv) 
{
	return sensor_request(PRESSURE);
}

/**
 * @brief Request humidity reading from all nodes in the network
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_humidity(const struct shell *shell, size_t argc, char **argv) 
{
	return sensor_request(HUMIDITY);
}

/**
 * @brief Request temperature reading from all nodes in the network 
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_temperature(const struct shell *shell, size_t argc, char **argv) 
{
	return sensor_request(TEMPERATURE);
}

/**
 * @brief Request VOC (volatile organic compounds) reading from all nodes in the network 
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_voc(const struct shell *shell, size_t argc, char **argv) 
{
	return sensor_request(VOC);
}

/**
 * @brief Request CO2 reading from all nodes in the network 
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_co2(const struct shell *shell, size_t argc, char **argv) 
{
	return sensor_request(CO2);
}

/**
 * @brief Reqeust PM10 (particulate matter) from all nodes in the network 
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_pm10(const struct shell *shell, size_t argc, char **argv) 
{
	return sensor_request(PM10);
}

/**
 * @brief Start continuously sampling all known devices from all nodes in the network 
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_all_on(const struct shell *shell, size_t argc, char **argv) 
{
	return sensor_continuous_on(); 
}

/**
 * @brief Stop continuous sampling 
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_all_off(const struct shell *shell, size_t argc, char **argv)
{
	return sensor_continuous_off();
}

/**
 * @brief Set the sampling period for continuous sampling 
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_set_sample_period(const struct shell *shell, size_t argc, char **argv) 
{
	return set_sample_period(argc, argv);
}