#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <sys/printk.h>

/* Include files from our library */
#include "scu_sensors.h"




void main (void)
{       
        while(1) {
	        scu_sensors_temp_get();
                scu_sensors_hum_get();
	        scu_sensors_voc_get();
                scu_sensors_pressure_get();
                k_sleep(K_SECONDS(1));
        }
}