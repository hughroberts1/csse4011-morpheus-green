/**
 ******************************************************************************
 * @file scu_sensors.c
 * @author Oliver Roman
 * @date 31.03.2022
 * @brief Contains driving and threading functionality for the sensors that the
 * 	  SCU or Argon board might use
 ******************************************************************************
 **/

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <drivers/sensor/ccs811.h>
#include <stdio.h>
#include <string.h>
#include <sys/util.h>
#include <sys/printk.h>

// Local Library Include
#include "scu_sensors.h"
#include "scu_ble.h"
#include "sen5x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

// Name of the port used for the ultrasonic TODO: see how to change this with devicetree
// Can change here now for modularity purposes
char* portName = "GPIO_1";

K_SEM_DEFINE(usSem, 0, 1);
K_MUTEX_DEFINE(scuMutex);

// Zeroing the struct that will hold the current sensor data
struct scuSensorData currentSensorData = {0,0,0,{0,0,0},0,0};

// Setting up the ultrasonic callback data struct
static struct gpio_callback usCbData;

// Flag for whether the continuous sensor sampling thread is running or not 
int threadRunFlag = 0;
static int samplingTime = SAMPLING_TIME_DEFAULT;
uint32_t usStartTime;
uint32_t usEndTime;

#ifdef CONFIG_LPS22HB
/**
 * @brief Function for reading pressure values from the lps22hb
 * 
 */
void scu_process_lps22hb_sample(void)
{

	const struct device *dev = scu_sensors_init("LPS22HB");
	struct sensor_value pressure;

	if (k_mutex_lock(&scuMutex, K_FOREVER) != 0) {
        	printk("The mutex could not lock\n");
		return;
    	}

	if (sensor_sample_fetch(dev) < 0) {
		printk("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) < 0) {
		printk("Cannot read LPS22HB pressure channel\n");
		return;
	}

	/* Display pressure and save*/
	currentSensorData.pressure = sensor_value_to_double(&pressure);
	printk("Pressure:%.1f kPa\n", currentSensorData.pressure);
	k_mutex_unlock(&scuMutex);
}
#endif

#ifdef CONFIG_HTS221
/**
 * @brief Function for reading temperature and humidity values from the hts221 
 * 
 */
void scu_process_hts221_sample(void)
{
	
	const struct device *dev = scu_sensors_init("HTS221");
	struct sensor_value temp, hum;

	if (k_mutex_lock(&scuMutex, K_FOREVER) != 0) {
        	printk("The mutex could not lock\n");
		return;
    	}
	
	if (sensor_sample_fetch(dev) < 0) {
		printk("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printk("Cannot read HTS221 temperature channel\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum) < 0) {
		printk("Cannot read HTS221 humidity channel\n");
		return;
	}

	// Save & display readings
	currentSensorData.temperature = sensor_value_to_double(&temp);
	currentSensorData.humidity = sensor_value_to_double(&hum);
	printk("Temperature:%.1f C & Relative Humidity:%.1f%%\n", currentSensorData.temperature,
		currentSensorData.humidity);
	k_mutex_unlock(&scuMutex);
}
#endif

#ifdef CONFIG_LIS2DH
/**
 * @brief Function for reading acceleration values from the lisd2h
 * 
 */
void scu_process_lis2dh_sample(void)
{
	
	const struct device *dev = scu_sensors_init("LIS2DH");
	struct sensor_value accel[3];
	int err = sensor_sample_fetch(dev);

	if (k_mutex_lock(&scuMutex, K_FOREVER) != 0) {
        	printk("The mutex could not lock\n");
		return;
    	}

	if (err == 0) {
		err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	}

	if (err < 0) {
		printk("ERROR: Update failed: %d\n", err);
	} else {
		currentSensorData.acceleration[0] = sensor_value_to_double(&accel[0]);
		currentSensorData.acceleration[1] = sensor_value_to_double(&accel[1]);
		currentSensorData.acceleration[2] = sensor_value_to_double(&accel[2]);
		printk("Acceleration values are x:%f, y:%f, z:%f",
		       currentSensorData.acceleration[0],
		       currentSensorData.acceleration[1],
		       currentSensorData.acceleration[2]);
	}
	k_mutex_unlock(&scuMutex);
}
#endif

#ifdef CONFIG_CCS811
/**
 * @brief Function for reading VOC value from the ccs811
 * 
 */
void scu_process_ccs811_sample(void)
{
	
	const struct device *dev = scu_sensors_init("CCS811");
	struct sensor_value tvoc, co2;
	int err = 0;

	if (k_mutex_lock(&scuMutex, K_FOREVER) != 0) {
        	printk("The mutex could not lock\n");
		return;
    	}

	if (err == 0) {
		err = sensor_sample_fetch(dev);
	}

	if (err == 0) {
		const struct ccs811_result_type *res = ccs811_result(dev);
		sensor_channel_get(dev, SENSOR_CHAN_VOC, &tvoc);
		currentSensorData.voc = (float)sensor_value_to_double(&tvoc);
		sensor_channel_get(dev, SENSOR_CHAN_CO2, &c02);
		currentSensorData.co2 = (float)sensor_value_to_double(&co2);
		printk("Amount of VOC is: %f ppb eTVOC and CO2 is: %f ppb\n", currentSensorData.voc,
		       currentSensorData.co2);
		if (rp->status & CCS811_STATUS_ERROR) {
			printk("ERROR: %02x\n", rp->error);
		}
	}
	k_mutex_unlock(&scuMutex);
}
#endif

#ifdef CONFIG_ULTRASONIC
/**
 * @brief Function for reading the distance value from the ultrasonic sensor
 * 
 */
void scu_process_ultrasonic_sample(void)
{

	const struct device *dev = scu_sensors_init(portName);

    	if (k_mutex_lock(&scuMutex, K_FOREVER) != 0) {
        	printk("The mutex could not lock\n");
		return;
    	}
	
	// Default value for the sensor will be -1, if the function is to return before it is
	// updated then it shows that the sensor value is invalid (negative)
	currentSensorData.distance = -1;
    	
	if (gpio_add_callback(dev, &usCbData) != 0) {
        	printk("Failed to add HC-SR04 echo callback");
        	k_mutex_unlock(&scuMutex);
        	return;
    	}

	// Sets the trig pin to high to start the reading (after initialising to low)
	gpio_pin_set(dev, TRIG_PIN, 0);
	k_busy_wait(1);
	gpio_pin_set(dev, TRIG_PIN, 1);
	k_busy_wait(TRIG_PULSE_US);
	// Set the trig pin back to 0
	gpio_pin_set(dev, TRIG_PIN, 0);

	// Set a semaphore to signal when interrupt has occured
	if (k_sem_take(&usSem, K_MSEC(MAX_WAIT_MS)) != 0) {
		printk("No response from the ultrasonic sensor\n");
		k_mutex_unlock(&scuMutex);
		if (gpio_remove_callback(dev, &usCbData)) {
			printk("Could not remove the callback\n");
		}
		return;
    	}

	int usSampleDelta = usEndTime - usStartTime;
	usSampleDelta = k_cyc_to_us_near32(usSampleDelta);

	if ((usSampleDelta < INVALID_PULSE_US) && (usSampleDelta > TRIG_PULSE_US)) {
		// Calculate distance in centimeters and update the current sensor data struct
		currentSensorData.distance = (((float)usSampleDelta/USEC_PER_SEC) *
				SPEED_OF_SOUND/2) * 100;
	} else {
		printk("Measurement just recieved was invalid\n");
		k_usleep(ERR_WAIT_US);
	}
	k_mutex_unlock(&scuMutex);
	
}

/**
 * @brief Callback funciton for the interrupt connected to the ultrasonic sensor, times difference
 * 	  between the echo pin start and finish
 * 
 * @param dev The device handle for the ultrasonic sensor
 */
void echo_recieved_callback(const struct device *dev)
{

// Removed "struct gpio_callback *cb" if the code breaks it could be why
	if (gpio_pin_get(dev, ECHO_PIN)) {
		// Start timing how long it takes for the echo to end
		usStartTime = k_cycle_get_32();
	} else if (!gpio_pin_get(dev, ECHO_PIN)) {
		// Stop the timer here
		usEndTime = k_cycle_get_32();
		// Signal that the echo has now completed
		k_sem_give(&usSem);
	}
}
#endif

#ifdef CONFIG_SEN54
/**
 * @brief Function used to take a sample of all data sources from the SEN54
 * 
 */
void scu_process_sen54_sample(void)
{
	int err = sen5x_start_measurement();
    	if (err)
        	printf("Error executing sen5x_start_measurement(): %i\n", error);

	const struct device *dev = scu_sensors_init("I2C_0");
	
	if (k_mutex_lock(&scuMutex, K_FOREVER) != 0) {
        	printk("The mutex could not lock\n");
		return;
    	}

	// TODO: check if its &struct.data or &struct->data to be passed in here (make sure to pass
	//	 by reference)
	err = sen5x_read_measured_values(NULL, NULL, NULL, &currentSensorData->particle,
            				 &currentSensorData->hum, &currentSensorData->temp,
					 &currentSensorData->voc, NULL);
	// Adjust scaling of data
	currentSensorData.particle /= 10;
	currentSensorData.hum /= 100;
	currentSensorData.temp /= 200;
	currentSensorData.voc /= 10;
	int err = sen5x_stop_measurement();
	if (err)
        	printf("Error executing sen5x_stop_measurement(): %i\n", error);
	k_mutex_unlock(&scuMutex);
}
#endif

/**
 * @brief Initialise one of the desired scu sensors, by default sampling time is 1 seconds
 * 
 * @param devName The name of the device/sensor to be initialised
 * @return *dev The pointer to the device that has been initialised
 */
const struct device* scu_sensors_init(char *devName)
{

	/* The aim of this is to set up so it will automatically look for the device that is in the
	   parameters, if an option that is meant to work fails it will be picked up later on and 
	   should work */
	const struct device *dev = device_get_binding(devName);

	#ifdef CONFIG_LIS2DH
	if (!strcmp(devName, "LIS2DH"))
		dev = device_get_binding(DT_LABEL(DT_INST(0, st_lis2dh)));
	#endif

	#ifdef CONFIG_CCS881
	if (!sterrmp(devName, "CCS811"))
		dev = device_get_binding(DT_LABEL(DT_INST(0, ams_ccs811)));
	#endif

	#ifdef CONFIG_ULTRASONIC
	/* TODO: Make the name here modular (change it to be a variable that is defined and updated 
	   earlier) */
	if (!strcmp(devName, portName)) {
		dev = device_get_binding(portName); 
	
		if (gpio_pin_configure(dev, TRIG_PIN, GPIO_OUTPUT) != 0) {
			printk("Error failed to configure trigger pin\n");
			return;
		}

		if (gpio_pin_configure(dev, ECHO_PIN, GPIO_INPUT) != 0) {
			printk("Error failed to configure echo pin\n");
			return;
		}
		
		/* TODO: This line keeps giving an error when its technically fine, it will be because
		   the interrupt is already configured so just find and confirm that is the error 
		   code and add it to the if check to ignore */  
		if (gpio_pin_interrupt_configure(dev, ECHO_PIN, GPIO_INT_EDGE_BOTH) != 0) {
			printk("Error failed to configure interrupt for on echo pin\n");
			return;
		}
		gpio_init_callback(&usCbData, echo_recieved_callback, BIT(ECHO_PIN));
		printk("Set up ultrasonic sensor callback on echo pin\n");
	}
	#endif
	
	#ifdef CONFIG_SEN54
	// TODO: Change the last part of the strcmp if check to properly check for the SEN54
	if (!strcmp(devName, "SEN54")) {

	}
	#endif

	// Initialising the device
	if (dev == NULL) {
		printk("Could not get %s device\n", devName);
		return;
	}
	
	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", devName);
		return;
	}

	printk("All good device %s is set up\n", devName);
	return dev;
}


/**
 * @brief Set the scu sensors' sampling time between once per 10 seconds to once per 5 mins
 * 
 * @param newSampTime The sampling time to set on the device in seconds
 * @return int The error code
 */
int scu_sensors_samp_time_set(int newSampTime)
{

	if(newSampTime >= 1 && newSampTime <= 300) {
		samplingTime = newSampTime;
		return 0;
	} else
		return ENOTSUP;
}