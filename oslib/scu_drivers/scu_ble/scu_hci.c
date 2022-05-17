/**
 ******************************************************************************
 * @file scu_hci.h
 * @author Oliver Roman
 * @date 31.03.2022
 * @brief Contains implementation of HCI protocol functinos for the SCU to communicate with
 *	  the AHU over Bluetooth. Contains functions to parse the HCI packet to request data
 *	  from the SCU Sensors and package it back into another HCI packet.  
 ******************************************************************************
 **/

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <sys/util.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "scu_sensors.h"
#include "scu_hci.h"
#include "scu_io.h"


/**
 * @brief Function that interprets a packet, requests data and packages it into another HCI packet 
 * 	  to give to the AHU.
 * 
 * @param hci_recieve_packet The HCI packet recieved from the AHU
 * @param hci_response_packet The HCI packet to send to the AHU
 * @return int Error code, 0 is successful < 0 is an error
 */
int hci_command_interpret(struct HCI_Message *hci_recieve_packet,
			  struct HCI_Message *hci_response_packet) {
	// Container for sending sampling time data
	int samp_time = 0;

	// Preamble must be 0xAA
	if(hci_recieve_packet->preamble != 0xAA)
		return -1;
	
	// Recieving a packet must be from type 0x01 (request type)
	if(hci_recieve_packet->type != 0x01)
		return -1;

	// Switching over the DID portion of the data (first byte is DID)
	switch(hci_recieve_packet->did) {
#ifdef CONFIG_MOBILE
	case HTS221_TEMPERATURE:
		scu_process_hts221_sample();
		hci_response_packet->did = HTS221_TEMPERATURE;
		memcpy(hci_response_packet->data, &(current_sensor_data.temperature),
			sizeof(current_sensor_data.temperature));
		break;
	case HTS221_HUMIDITY:
		scu_process_hts221_sample();
		hci_response_packet->did = HTS221_HUMIDITY;
		memcpy(hci_response_packet->data, &(current_sensor_data.humidity),
			sizeof(current_sensor_data.humidity));
		break;
	case LPS22_AIR_PRESSURE:
		scu_process_lps22hb_sample();
		hci_response_packet->did = LPS22_AIR_PRESSURE;
		memcpy(hci_response_packet->data, &(current_sensor_data.pressure),
			sizeof(current_sensor_data.pressure));
		break;
	case CCS811_VOC:
		scu_process_ccs811_sample();
		hci_response_packet->did = CCS811_VOC;
		memcpy(hci_response_packet->data, &(current_sensor_data.voc),
			sizeof(current_sensor_data.voc));
		break;
	case LIS2DH_X_ACCELERATION:
		scu_process_lis2dh_sample();
		hci_response_packet->did = LIS2DH_X_ACCELERATION;
		memcpy(hci_response_packet->data, &(current_sensor_data.acceleration[0]),
			sizeof(current_sensor_data.acceleration[0]));
		break;
	case LIS2DH_Y_ACCELERATION:
		scu_process_lis2dh_sample();
		hci_response_packet->did = LIS2DH_Y_ACCELERATION;
		memcpy(hci_response_packet->data, &(current_sensor_data.acceleration[1]),
			sizeof(current_sensor_data.acceleration[1]));
		break;
	case LIS2DH_Z_ACCELERATION:
		scu_process_lis2dh_sample();
		hci_response_packet->did = LIS2DH_Z_ACCELERATION;
		memcpy(hci_response_packet->data, &(current_sensor_data.acceleration[2]),
			sizeof(current_sensor_data.acceleration[2]));
		break;
	case RGB_LED:
		if (scu_io_led_init(hci_recieve_packet->data[1]) < 0)
			return -1;
		if (scu_io_led_toggle(hci_recieve_packet->data[1]) < 0) {
			printk("Toggle returned -ve\n");
			return -1;
		}
		printk("Toggle has passed through\n");
		break;
	case BUZZER:
		printk("Speaker should be working");
		break;
	case PUSHBUTTON:
		if(scu_io_pb_init() < 0)
			return -1;
		int val = scu_io_pb_pin_read();
		if(val < 0) {
			printk("The button press screwed up");
			return -1;
		} else
			hci_response_packet->did = PUSHBUTTON;
			hci_response_packet->data[0] = val;
		break;
	case 'a':
		if(!thread_running_flag) {
			thread_running_flag = 1;
			k_sem_give(&cts_mode_sem);
		} else {
			thread_running_flag = 0;
			k_sem_take(&cts_mode_sem, K_FOREVER);
		}
		break;
#endif
	case 'd':

		break;
	case 's':
		samp_time += hci_recieve_packet->data[1] << 8;
		samp_time += hci_recieve_packet->data[2];
		printk("The sampling time recieved is: %d", samp_time);
		scu_sensors_samp_time_set(samp_time);
		break;
	
	default:
		printk("Not a valid DID\n");
		return -1;
	}
}
