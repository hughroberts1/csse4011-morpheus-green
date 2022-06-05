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

/* Local Library Include */
#include "scu_power_management.h"

struct device *scu_power_management_init(char* devName)
{
        struct device *dev;
        if(!strcmp(devName, "VDD PWR"))
                dev = device_get_binding(DT_LABEL(VDD_NODE)); 
        else if(!strcmp(devName, "CCS PWR"))
                dev = device_get_binding(DT_LABEL(CCS_NODE));
        
        if (dev == NULL) {
		printk("Could not get %s device\n", devName);
		return;
	}
	
	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", devName);
		return;
	}

	return dev;
}


void scu_power_management_on(void)
{
        int err = 0;
        struct device *vdd_dev = scu_power_management_init("VDD PWR");
        struct device *ccs_dev = scu_power_management_init("CCS PWR");
        err = regulator_enable(vdd_dev, NULL);
        if(err < 0 )
                printk("VDD had error: %d when trying to enable\n", err);
        err = regulator_enable(ccs_dev, NULL);
        if(err < 0 )
                printk("CCS had error: %d when trying to enable\n", err);
}


void scu_power_management_off(void)
{
        int err = 0;
        struct device *vdd_dev = scu_power_management_init("VDD PWR");
        struct device *ccs_dev = scu_power_management_init("CCS PWR");
        err = regulator_disable(vdd_dev, NULL);
        if(err < 0 )
                printk("VDD had error: %d when trying to disable\n", err);
        err = regulator_disable(ccs_dev, NULL);
        if(err < 0 )
                printk("CCS had error: %d when trying to disable\n", err);

}