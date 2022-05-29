/**
 ***************************************************************************************************
 * @file scu_.c
 * @author Oliver Roman
 * @date 31.03.2022
 * @brief Contains the funcionality required to conserve power by adjusting duty cycle of sensors
 ***************************************************************************************************
 **/
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <pm.h>

/* Local Library Include */
#include "scu_power_management.h"


struct device *power_mngment_init()
{
        
        

        // Initialising the device
	if (dev == NULL) {
		printk("Could not get %s device\n", devName);
		return;
	}
	
	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", devName);
		return;
	}

}
/**
 * @brief 
 * 
 * @param power 
 * @return int 
 */
int vdd_pwr_ctlr(int power)
{       
        struct device *vdd_rail = device_get_binding(VDD_NODE);
        gpio_pin_configure(dev, VDD_PIN, GPIO_INPUT | VDD_FLAGS);
}

int ccs_pwr_ctlr(int power)
{
        struct device *ccs_rail = device_get_binding(CCS_VDD_NODE);
        gpio_pin_configure(dev, CCS_VDD_PIN, GPIO_INPUT | CCS_VDD_FLAGS);
}