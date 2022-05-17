/**
 ***************************************************************************************************
 * @file scu_io.c
 * @author Oliver Roman
 * @date 31.03.2022
 * @brief Contains the functionality required to control the IO devices such as the RGB Well LED,
 * 	  buzzer and the pushbutton
 ***************************************************************************************************
 **/

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/regulator.h>
#include <drivers/pwm.h>

#include "scu_io.h"

static int is_led_init_flag = 0;
static int is_pb_init_flag = 0;
static int is_spk_init_flag = 0;

#ifdef CONFIG_MOBILE

/**
 * @brief Initialise the RGB LED
 * 
 * @param colour Character that selects the colour of the LED to initialise
 * @return int Error code
 */
int scu_io_led_init(uint8_t colour) {
	if (!is_led_init_flag) {
		const struct device* dev;
		
		switch (colour) {
		case 'r':
			dev = device_get_binding(LED0);
			if (dev == NULL)
				return ENODEV;
			is_led_init_flag = 1;
			return gpio_pin_configure(dev, PIN0, GPIO_OUTPUT_ACTIVE | FLAGS0);
		case 'g': 
			dev = device_get_binding(LED1);
			if (dev == NULL)
				return ENODEV;
			is_led_init_flag = 1;
			return gpio_pin_configure(dev, PIN1, GPIO_OUTPUT_ACTIVE | FLAGS1);
		case 'b':
			dev = device_get_binding(LED2);
			if (dev == NULL)
				return ENODEV;
			is_led_init_flag = 1;
			return gpio_pin_configure(dev, PIN2, GPIO_OUTPUT_ACTIVE | FLAGS2);
		default:
			return EINVAL;
		}
	}
}


/**
 * @brief Initialise the RGB LED
 * 
 * @param colour Character that selects the colour of the LED to initialise
 * @return int Error code
 */
int scu_io_led_deinit(uint8_t colour) {
	if (is_led_init_flag) {
		const struct device* dev;
		
		switch (colour) {
		case 'r':
			dev = device_get_binding(LED0);
			if (dev == NULL)
				return ENODEV;
			is_led_init_flag = 0;
			return gpio_pin_configure(dev, PIN0, GPIO_DISCONNECTED);
		case 'g': 
			dev = device_get_binding(LED1);
			if (dev == NULL)
				return ENODEV;
			is_led_init_flag = 0;
			return gpio_pin_configure(dev, PIN1, GPIO_DISCONNECTED);
		case 'b':
			dev = device_get_binding(LED2);
			if (dev == NULL)
				return ENODEV;
			is_led_init_flag = 0;
			return gpio_pin_configure(dev, PIN2, GPIO_DISCONNECTED);
		default:
			return EINVAL;
		}
	}
}

/**
 * @brief Set particular colour of RGB to on
 * 
 * @param colour Character representing the colour
 * @return int Error code
 */
int scu_io_led_on(uint8_t colour) {
	
	const struct device* dev;
	switch (colour) {
	case 'r':
		printk("Case: r (144)\n");
		dev = device_get_binding(LED0);
		if (dev == NULL)
			return ENODEV;
		return gpio_pin_set(dev, PIN0, 1);
	case 'g': 
		dev = device_get_binding(LED1);
		if (dev == NULL)
			return ENODEV;
		return gpio_pin_set(dev, PIN1, 1);
	case 'b':
		dev = device_get_binding(LED2);
		if (dev == NULL)
			return ENODEV;
		return gpio_pin_set(dev, PIN2, 1);
	default:
		return EINVAL;
	}
}


/**
 * @brief Set particular colour of RGB to toggle
 * 
 * @param colour Character representing the colour
 * @return int int Error code
 */
int scu_io_led_toggle(uint8_t colour) {
	
	const struct device* dev;
	switch (colour) {
	case 'r':
		dev = device_get_binding(LED0);
		if (dev == NULL)
			return ENODEV;
		return gpio_pin_toggle(dev, PIN0);
	case 'g': 
		dev = device_get_binding(LED1);
		if (dev == NULL)
			return ENODEV;
		return gpio_pin_toggle(dev, PIN1);
	case 'b':
		dev = device_get_binding(LED2);
		if (dev == NULL)
			return ENODEV;
		return gpio_pin_toggle(dev, PIN2);
	default:
		return EINVAL;
	}
}



/**
 * @brief Set particular colour of RGB to off
 * 
 * @param colour Character representing the colour
 * @return int int Error code
 */
int scu_io_led_off(uint8_t colour) {
	
	const struct device* dev;
	switch (colour) {
	case 'r':
		dev = device_get_binding(LED0);
		if (dev == NULL)
			return ENODEV;
		return gpio_pin_set(dev, PIN0, 0);
	case 'g': 
		dev = device_get_binding(LED1);
		if (dev == NULL)
			return ENODEV;
		return gpio_pin_set(dev, PIN1, 0);
	case 'b':
		dev = device_get_binding(LED2);
		if (dev == NULL)
			return ENODEV;
		return gpio_pin_set(dev, PIN2, 0);
	default:
		return EINVAL;
	}
}


/**
 * @brief Initialise the PB
 * 
 * @return int Error code
 */
int scu_io_pb_init(void) {
	if (!is_pb_init_flag) {
		const struct device* dev;
		dev = device_get_binding(PB);
		if (dev == NULL)
			return ENODEV;
		is_pb_init_flag = 1;
		return gpio_pin_configure(dev, PB_PIN, GPIO_INPUT | PB_FLAGS);
	}
}


/**
 * @brief Deinitialise the PB
 * 
 * @return int Error code
 */
int scu_io_pb_deinit(void) {
	if (is_pb_init_flag) {
		const struct device* dev;
		dev = device_get_binding(PB);
		if (dev == NULL)
			return ENODEV;
		is_pb_init_flag = 0;
		return gpio_pin_configure(dev, PB_PIN, GPIO_DISCONNECTED);
	}
}


/**
 * @brief Reads the PB pin logic value
 * 
 * @return int the logic level of the PB or < 0 if an error
 */
int scu_io_pb_pin_read(void) {
	const struct device* dev;
	dev = device_get_binding(PB);
	if (dev == NULL)
		return ENODEV;
	return gpio_pin_get(dev, PB_PIN);
}

#endif
/*

int scu_io_spk_init(void) {
	if (!is_spk_init_flag) {
		const struct device* dev = device_get_binding(SPK_PWR);
		if (dev == NULL)
			return ENODEV;
		is_spk_init_flag = 1;
		// Need to make an onoff manager struct here
		//struct onoff_manager spkr_pwr_mgr;

		//onoff_manager_init(&spkr_pwr_mgr, const struct onoff_transitions *transitions);
		return regulator_enable(dev, NULL);
	}
}


void pwm_something(void) { 
	struct device *dev = DEVICE_DT_GET(PWM_CTLR);
	uint64_t cycles;
    	if (!dev) {
       		printk("Cannot find %s!\n", "PWM_0");
       		return;
    	}
	//pwm_get_cycles_per_sec(dev, PWM_CHANNEL, &cycles);
}*/