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

