/**
 * @file led_driver.c
 * @author Hugh Roberts
 * @brief Driver to controlling RGB LEDs
 * @version 0.1
 * @date 2022-04-04
 * @copyright Copyright (c) 2022
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "led_driver.h"
#include <logging/log.h>

#include <shell/shell.h>

LOG_MODULE_REGISTER(led_module);

bool red_level = 1; 
bool green_level = 1; 
bool blue_level = 1; 


/* Toggle LED corresponding to colour 
 */
int led_toggle(int colour)
{
	const struct device *dev; 

	if (colour == RED) {
		dev = device_get_binding(RED_LED);
	} else if (colour == GREEN) {
		dev = device_get_binding(GREEN_LED);
	} else if (colour == BLUE) {
		dev = device_get_binding(BLUE_LED);
	} else {

	}
	if (dev == NULL) {
		return ENODEV;
	}
	if (colour == RED) {
		red_level = !red_level;
		LOG_INF("red led is %s", red_level ? "on" : "off");
		return gpio_pin_toggle(dev, RED_PIN);
	} else if (colour == GREEN) {
		green_level = !green_level;
		LOG_INF("green led is %s", green_level ? "on" : "off");
		return gpio_pin_toggle(dev, GREEN_PIN);
	} else if (colour == BLUE) {
		blue_level = !blue_level;
		LOG_INF("blue led is %s", blue_level ? "on" : "off");
		return gpio_pin_toggle(dev, BLUE_PIN);
	} else {
		return ENODEV;
	}
}

/* Turn on or off LED corresponding to colour 
 */
int led_on_off(int colour, int setting)
{
	const struct device *dev; 

	if (colour == RED) {
		dev = device_get_binding(RED_LED);
	} else if (colour == GREEN) {
		dev = device_get_binding(GREEN_LED);
	} else if (colour == BLUE) {
		dev = device_get_binding(BLUE_LED);
	} else {

	}
	if (dev == NULL) {
		return ENODEV;
	}
	if (colour == RED) {
		if (red_level == setting) {
			LOG_WRN("red led already %s", red_level ? "on" : "off");
		}
		red_level = (bool) setting;
		LOG_INF("red led is %s", setting ? "on" : "off");
		return gpio_pin_set(dev, RED_PIN, setting);
	} else if (colour == GREEN) {
		if (green_level == setting) {
			LOG_WRN("green led already %s", green_level ? "on" : "off");
		}
		green_level = (bool) setting;
		LOG_INF("green led is %s", setting ? "on" : "off");
		return gpio_pin_set(dev, GREEN_PIN, setting);
	} else if (colour == BLUE) {
		if (blue_level == setting) {
			LOG_WRN("blue led already %s", blue_level ? "on" : "off");
		}
		blue_level = (bool) setting;
		LOG_INF("blue led is %s", setting ? "on" : "off");
		return gpio_pin_set(dev, BLUE_PIN, setting);
	} else {
		return ENODEV;
	}
}

/*
 * initialise LEDs
 */
int led_module_init(void)
{
	const struct device *dev;
	int ret;

	/* #################### Setup LEDs ######################### */
	dev = device_get_binding(RED_LED);
	if (dev == NULL) {
		return ENODEV;
	}

	ret = gpio_pin_configure(dev, RED_PIN, GPIO_OUTPUT_ACTIVE | RED_FLAGS);
	if (ret < 0) {
		return ENODEV;
	}

	dev = device_get_binding(GREEN_LED);
	if (dev == NULL) {
		return ENODEV;
	}

	ret = gpio_pin_configure(dev, GREEN_PIN, GPIO_OUTPUT_ACTIVE | GREEN_FLAGS);
	if (ret < 0) {
		return ENODEV;
	}

	dev = device_get_binding(BLUE_LED);
	if (dev == NULL) {
		return ENODEV;
	}

	ret = gpio_pin_configure(dev, BLUE_PIN, GPIO_OUTPUT_ACTIVE | BLUE_FLAGS);
	if (ret < 0) {
		return ENODEV;
	}
	
	LOG_DBG("led init OK");

	return 0;

	/* ########################## Setup LEDs ####################### */
}

/* Command handler for turning red led on, note that is assumes 
 * the pin has been preconfigured
 */
static int cmd_red_led_ctrl_on(const struct shell *shell, size_t argc, char **argv)
{

	if ((strcmp(argv[0], "r")) && (strcmp(argv[0], "g")) && (strcmp(argv[0], "b"))) {
		LOG_ERR("invalid command");
	}

	led_on_off(RED, ON);
	return 0;
	
}

/* Command handler for turning green led on, note that is assumes 
 * the pin has been preconfigured
 */
static int cmd_green_led_ctrl_on(const struct shell *shell, size_t argc, char **argv) 
{
	led_on_off(GREEN, ON);
	return 0;
}

/* Command handler for turning blue led on, note that is assumes 
 * the pin has been preconfigured
 */
static int cmd_blue_led_ctrl_on(const struct shell *shell, size_t argc, char **argv)
{
	led_on_off(BLUE, ON);
	return 0;
}


/* Command handler for turning red led off, note that is assumes 
 * the pin has been preconfigured
 */
static int cmd_red_led_ctrl_off(const struct shell *shell, size_t argc, char **argv)
{
	led_on_off(RED, OFF);
	return 0;
}

/* Command handler for turning green led off, note that is assumes 
 * the pin has been preconfigured
 */
static int cmd_green_led_ctrl_off(const struct shell *shell, size_t argc, char **argv)
{
	led_on_off(GREEN, OFF);
	return 0;
}

/* Command handler for turning blue led off, note that is assumes 
 * the pin has been preconfigured
 */
static int cmd_blue_led_ctrl_off(const struct shell *shell, size_t argc, char **argv)
{
	led_on_off(BLUE, OFF);
	return 0;
}

/* Command handler for toggling red led, note that is assumes 
 * the pin has been preconfigured
 */
static int cmd_red_led_ctrl_toggle(const struct shell *shell, size_t argc, char **argv)
{
	led_toggle(RED);
	return 0;
}

/* Command handler for toggling green led, note that is assumes 
 * the pin has been preconfigured
 */
static int cmd_green_led_ctrl_toggle(const struct shell *shell, size_t argc, char **argv)
{
	led_toggle(GREEN);
	return 0;
}

/* Command handler for toggling blue led, note that is assumes 
 * the pin has been preconfigured
 */
static int cmd_blue_led_ctrl_toggle(const struct shell *shell, size_t argc, char **argv)
{
	led_toggle(BLUE);
	return 0;
}

/* Creating subcommands (level 2 command) array for command "led o" */
SHELL_STATIC_SUBCMD_SET_CREATE(led_ctrl_on, 
		SHELL_CMD(r, NULL, "Turn red led on.", cmd_red_led_ctrl_on),
		SHELL_CMD(g, NULL, "Turn green led on.", cmd_green_led_ctrl_on),
		SHELL_CMD(b, NULL, "Turn blue led on.", cmd_blue_led_ctrl_on),
		SHELL_SUBCMD_SET_END
);

/* Creating subcommands (level 2 command) array for command "led f" */
SHELL_STATIC_SUBCMD_SET_CREATE(led_ctrl_off, 
		SHELL_CMD(r, NULL, "Turn red led off.", cmd_red_led_ctrl_off),
		SHELL_CMD(g, NULL, "Turn green led off.", cmd_green_led_ctrl_off),
		SHELL_CMD(b, NULL, "Turn blue led off.", cmd_blue_led_ctrl_off),
		SHELL_SUBCMD_SET_END
);

/* Creating subcommands (level 2 command) array for command "led t" */
SHELL_STATIC_SUBCMD_SET_CREATE(led_ctrl_toggle, 
		SHELL_CMD(r, NULL, "Toggle red led.", cmd_red_led_ctrl_toggle),
		SHELL_CMD(g, NULL, "Toggle green led.", cmd_green_led_ctrl_toggle),
		SHELL_CMD(b, NULL, "Toggle blue led.", cmd_blue_led_ctrl_toggle),
		SHELL_SUBCMD_SET_END
);

/* error checking */
static int cmd_led(const struct shell *shell, size_t argc, char **argv)
{
	if ((strcmp(argv[1], "r")) && (strcmp(argv[1], "g")) && (strcmp(argv[1], "b"))) {
		LOG_ERR("invalid command");
	}
	return 0;
}

/* error checking */
static int cmd_led_root(const struct shell *shell, size_t argc, char **argv)
{
	if ((strcmp(argv[1], "o")) && (strcmp(argv[1], "f")) && (strcmp(argv[1], "t"))) {
		LOG_ERR("invalid command");
	}
	return 0;
}

/* Specify Shell Commands for LED Toggling */
/* Creating subcommands (level 1 command) array for command "led". */ 
SHELL_STATIC_SUBCMD_SET_CREATE(led_ctrl,
        SHELL_CMD(o, &led_ctrl_on, "Turn led on.", cmd_led),
        SHELL_CMD(f, &led_ctrl_off, "Turn led off.", cmd_led),
		SHELL_CMD(t, &led_ctrl_toggle, "Toggle led.", cmd_led),
        SHELL_SUBCMD_SET_END
); 

/* Creating root (level 0) command "led" */
SHELL_CMD_REGISTER(led, &led_ctrl, "Led Commands", cmd_led_root);
