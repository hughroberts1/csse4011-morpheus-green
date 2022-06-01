/* board.c - Generic HW interaction hooks */

/*
 * Copyright (c) 2021 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <bluetooth/mesh.h>
#include <drivers/gpio.h>

#include "board.h"
#include "mesh.h"

static bool pressed;
static const struct device *gpio;

/* Locate led0 as alias or label by that name */
#if DT_NODE_EXISTS(DT_ALIAS(led0))
#define LED0 DT_ALIAS(led0)
#elif DT_NODE_EXISTS(DT_NODELABEL(led0))
#define LED0 DT_NODELABEL(led0)
#else
#define LED0 DT_INVALID_NODE
#endif

/* Locate button0 as alias or label by sw0 or button0 */
#if DT_NODE_EXISTS(DT_ALIAS(sw0))
#define BUTTON0 DT_ALIAS(sw0)
#elif DT_NODE_EXISTS(DT_ALIAS(button0))
#define BUTTON0 DT_ALIAS(button0)
#elif DT_NODE_EXISTS(DT_NODELABEL(sw0))
#define BUTTON0 DT_NODELABEL(sw0)
#elif DT_NODE_EXISTS(DT_NODELABEL(button0))
#define BUTTON0 DT_NODELABEL(button0)
#else
#define BUTTON0 DT_INVALID_NODE
#endif

#if DT_NODE_EXISTS(LED0)
#define LED0_DEV DT_PHANDLE(LED0, gpios)
#define LED0_PIN DT_PHA(LED0, gpios, pin)
#define LED0_FLAGS DT_PHA(LED0, gpios, flags)

static const struct device *led_dev = DEVICE_DT_GET(LED0_DEV);
#endif /* LED0 */

#if DT_NODE_EXISTS(BUTTON0)
#define BUTTON0_DEV DT_PHANDLE(BUTTON0, gpios)
#define BUTTON0_PIN DT_PHA(BUTTON0, gpios, pin)
#define BUTTON0_FLAGS DT_PHA(BUTTON0, gpios, flags)

static const struct device *button_dev = DEVICE_DT_GET(BUTTON0_DEV);


static bool button_is_pressed(void)
{
	return gpio_pin_get(gpio, DT_GPIO_PIN(DT_ALIAS(sw0), gpios)) > 0;
}

static void button_interrupt(const struct device *port, struct gpio_callback *cb,
		      gpio_port_pins_t pins)
{
	set_led_state(pressed);
	
	if (button_is_pressed() == pressed) {
		return;
	}

	pressed = !pressed;
	printk("Button %s\n", pressed ? "pressed" : "released");

	
	if (!mesh_is_initialized()) {
		return;
	}

	mesh_send_hello();

	
	
}
#endif /* BUTTON0 */

static int led_init(void)
{
#if DT_NODE_EXISTS(LED0)
	int err;

	if (!device_is_ready(led_dev)) {
		return -ENODEV;
	}

	err = gpio_pin_configure(led_dev, LED0_PIN,
				 LED0_FLAGS | GPIO_OUTPUT_INACTIVE);
	if (err) {
		return err;
	}
#else
	printk("WARNING: LEDs not supported on this board.\n");
#endif

	return 0;
}

static int button_init(void)
{
#if DT_NODE_EXISTS(BUTTON0)
	static struct gpio_callback button_cb;

	gpio = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(sw0), gpios));
	if (!gpio) {
		return -ENODEV;
	}
	gpio_pin_configure(gpio, BUTTON0_PIN, 
		GPIO_INPUT | BUTTON0_FLAGS);

	gpio_pin_interrupt_configure(gpio, BUTTON0_PIN, 
		GPIO_INT_EDGE_BOTH);

	gpio_init_callback(&button_cb, button_interrupt, 
		BIT(BUTTON0_PIN));

	gpio_add_callback(gpio, &button_cb);

	return 0;
#else
	printk("WARNING: Buttons not supported on this board.\n");
#endif

	return 0;
}

int board_init(void)
{
	int err;

	err = led_init();
	if (err) {
		return err;
	}

	set_led_state(true);

	err = button_init(); 
    if (err) {
        return err;
    }

	return 0;
}

int set_led_state(bool state)
{
#if DT_NODE_EXISTS(LED0)
	return gpio_pin_set(led_dev, LED0_PIN, state);
#endif
}

void board_output_number(bt_mesh_output_action_t action, uint32_t number)
{
}

void board_prov_complete(void)
{
}
