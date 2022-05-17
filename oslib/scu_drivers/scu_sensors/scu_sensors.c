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

#define SAMPLING_TIME_DEFAULT 10

K_SEM_DEFINE(sensor_logging_sem, 0, 1);
K_SEM_DEFINE(cts_mode_sem, 0,1);
K_SEM_DEFINE(us_sem, 0, 1);
K_MUTEX_DEFINE(us_mutex);
//K_THREAD_DEFINE(scu_sensors_thread_tid, THREAD_SCU_SENSOR_STACK, thread_scu_sensors,
//		NULL, NULL, NULL, THREAD_SCU_SENSOR_PRIORITY, 0, 0);
#ifdef CONFIG_STATIC
K_THREAD_DEFINE(scu_ultrasonic_thread_tid, THREAD_SCU_SENSOR_STACK, thread_scu_ultrasonic,
		NULL, NULL, NULL, THREAD_SCU_SENSOR_PRIORITY, 0, 0);
#endif
// Zeroing the struct that will hold the current sensor data
struct scuSensorData current_sensor_data = {0,0,0,{0,0,0},0,0};

// Setting up the ultrasonic callback data struct
static struct gpio_callback us_cb_data;

static int sampling_time = SAMPLING_TIME_DEFAULT;

// Flag for whether the continuous sensor sampling thread is running or not 
int thread_running_flag = 0;

uint32_t us_start_time;
uint32_t us_end_time;

#ifdef CONFIG_MOBILE
/**
 * @brief Function for reading pressure values from the lps22hb
 * 
 * @param dev The device handle for the lps22hb
 */
void scu_process_lps22hb_sample(void) {

	const struct device *dev = scu_sensors_init("LPS22HB");
	static unsigned int obs;
	struct sensor_value pressure;

	if (sensor_sample_fetch(dev) < 0) {
		printk("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) < 0) {
		printk("Cannot read LPS22HB pressure channel\n");
		return;
	}

	current_sensor_data.pressure = sensor_value_to_double(&pressure);
	++obs;
	printk("Observation:%u\n", obs);

	/* display pressure and save*/
	printk("Pressure:%.1f kPa\n", sensor_value_to_double(&pressure));
}


/**
 * @brief Function for reading temperature and humidity values from the hts221 
 * 
 * @param dev The device handle for the hts221
 */
void scu_process_hts221_sample(void) {
	
	const struct device *dev = scu_sensors_init("HTS221");
	static unsigned int obs;
	struct sensor_value temp, hum;
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

	current_sensor_data.temperature = sensor_value_to_double(&temp);
	current_sensor_data.humidity = sensor_value_to_double(&hum);
	++obs;
	printk("Observation:%u\n", obs);

	// display temperature
	printk("Temperature:%.1f C\n", sensor_value_to_double(&temp));

	// display humidity
	printk("Relative Humidity:%.1f%%\n", sensor_value_to_double(&hum));
}


/**
 * @brief Function for reading acceleration values from the lisd2h
 * 
 * @param dev The device handle for the lis2dh
 */
void scu_process_lis2dh_sample(void) {
	
	const struct device *dev = scu_sensors_init("LIS2DH");
	static unsigned int count;
	struct sensor_value accel[3];
	const char *overrun = "";
	int rc = sensor_sample_fetch(dev);

	++count;
	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		if (IS_ENABLED(CONFIG_LIS2DH_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev,
					SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc < 0) {
		printk("ERROR: Update failed: %d\n", rc);
	} else {
		current_sensor_data.acceleration[0] = sensor_value_to_double(&accel[0]);
		current_sensor_data.acceleration[1] = sensor_value_to_double(&accel[1]);
		current_sensor_data.acceleration[2] = sensor_value_to_double(&accel[2]);
		printk("#%u @ %u ms: %sx %f , y %f , z %f", count, k_uptime_get_32(), overrun,
		       current_sensor_data.acceleration[0], current_sensor_data.acceleration[1],
		       current_sensor_data.acceleration[2]);
	}
}


/**
 * @brief Function for reading VOC value from the ccs811
 * 
 * @param dev The device handle for the ccs811
 */
void scu_process_ccs811_sample(void) {
	
	const struct device *dev = scu_sensors_init("CCS811");
	struct sensor_value tvoc;
	int rc = 0;
	int baseline = -1;

	if (rc == 0) {
		rc = sensor_sample_fetch(dev);
	}
	if (rc == 0) {
		const struct ccs811_result_type *rp = ccs811_result(dev);


		sensor_channel_get(dev, SENSOR_CHAN_VOC, &tvoc);
		current_sensor_data.voc = sensor_value_to_double(&tvoc);
		printk("CCS811: %f ppb eTVOC\n", current_sensor_data.voc);

		if (rp->status & CCS811_STATUS_ERROR) {
			printk("ERROR: %02x\n", rp->error);
		}
	}
}
#endif

#ifdef CONFIG_STATIC
/**
 * @brief Function for reading the distance value from the ultrasonic sensor
 * 
 * @param dev The device handle for the ultrasonic sensor
 */
void scu_process_ultrasonic_sample(void) {
	const struct device *dev = scu_sensors_init("GPIO_1");

    	if (k_mutex_lock(&us_mutex, K_FOREVER) != 0) {
        	printk("The mutex could not lock\n");
		return;
    	}
	
	// Default value for the sensor will be -1, if the function is to return before it is
	// updated then it shows that the sensor value is invalid (negative)
	current_sensor_data.distance = -1;
    	
	if (gpio_add_callback(dev, &us_cb_data) != 0) {
        	printk("Failed to add HC-SR04 echo callback");
        	k_mutex_unlock(&us_mutex);
        	return;
    	}

	// Sets the trig pin to high to start the reading (initialise to low)
	gpio_pin_set(dev, TRIG_PIN, 0);
	k_busy_wait(1);
	gpio_pin_set(dev, TRIG_PIN, 1);
	k_busy_wait(TRIG_PULSE_US);
	// Set the trig pin back to 0
	gpio_pin_set(dev, TRIG_PIN, 0);

	// Set a semaphore to signal when interrupt has occured
	if (0 != k_sem_take(&us_sem, K_MSEC(MAX_WAIT_MS))) {
		printk("No response from the ultrasonic sensor\n");
		k_mutex_unlock(&us_mutex);
		if (gpio_remove_callback(dev, &us_cb_data)) {
			printk("Could not remove the callback\n");
		}
		return;
    	}

	int us_sample_delta = us_end_time - us_start_time;
	us_sample_delta = k_cyc_to_us_near32(us_sample_delta);

	if ((us_sample_delta < INVALID_PULSE_US) && (us_sample_delta > TRIG_PULSE_US)) {
		// Calculate distance in centimeters and update the current sensor data struct
		current_sensor_data.distance = (((float)us_sample_delta/USEC_PER_SEC) *
				SPEED_OF_SOUND/2) * 100;
	} else {
		printk("Measurement just recieved was invalid\n");
		k_usleep(ERR_WAIT_US);
	}
	k_mutex_unlock(&us_mutex);
	
}


void echo_recieved_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	if (gpio_pin_get(dev, ECHO_PIN)) {
		// Start timing how long it takes for the echo to end
		us_start_time = k_cycle_get_32();
	} else if (!gpio_pin_get(dev, ECHO_PIN)) {
		// Stop the timer here
		us_end_time = k_cycle_get_32();
		// Signal that the echo has now completed
		k_sem_give(&us_sem);
	}
}
#endif

/**
 * @brief Initialise one of the desired scu sensors, by default sampling time is 5 seconds
 * 
 * @param dev_str The name of the device/sensor to be initialised
 * @return *dev The pointer to the device that has been initialised
 */
const struct device* scu_sensors_init(char *dev_name) {

	/* The aim of this is to set up so it will automatically look for the device that is in the
	   parameters */
	const struct device *dev = device_get_binding(dev_name);

	#ifdef CONFIG_MOBILE
	if (!strcmp(dev_name, "LIS2DH")) {
		dev = device_get_binding(DT_LABEL(DT_INST(0, st_lis2dh)));
	} else if (!strcmp(dev_name, "CCS811")) {
		dev = device_get_binding(DT_LABEL(DT_INST(0, ams_ccs811)));
	}
	#endif

	#ifdef CONFIG_STATIC
	if (!strcmp(dev_name, "GPIO_1")) {
		dev = device_get_binding("GPIO_1"); 
	
		if (gpio_pin_configure(dev, TRIG_PIN, GPIO_OUTPUT) != 0) {
			printk("Error failed to configure trigger pin\n");
			return;
		}

		if (gpio_pin_configure(dev, ECHO_PIN, GPIO_INPUT) != 0) {
			printk("Error failed to configure echo pin\n");
			return;
		}

		if (gpio_pin_interrupt_configure(dev, ECHO_PIN, GPIO_INT_EDGE_BOTH) != 0) {
			printk("Error failed to configure interrupt for on echo pin\n");
			return;
		}
		gpio_init_callback(&us_cb_data, echo_recieved_callback, BIT(ECHO_PIN));
		printk("Set up ultrasonic sensor callback on echo pin\n");
	}
	#endif
	
	// Initialising the device
	if (dev == NULL) {
		printk("Could not get %s device\n", dev_name);
		return;
	}
	
	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev_name);
		return;
	}

	printk("All good device %s is set up\n", dev_name);
	return dev;
}


/**
 * @brief Set the scu sensors' sampling time between once per 10 seconds to once per 5 mins
 * 
 * @param new_samp_time The sampling time to set on the device in seconds
 * @return int The error code
 */
int scu_sensors_samp_time_set(int new_samp_time) {
	if(new_samp_time >= 1 && new_samp_time <= 300) {
		sampling_time = new_samp_time;
		return 0;
	} else
		return -1;
}

#ifdef CONFIG_MOBILE
/**
 * @brief Continuous sensor logging thread that initialises all the sensors, then sends it
 * 	  via a data 
 */
void thread_scu_sensors(void) {
	while (1) {
		// Process a sample from each sensor and wait until next sample
		k_sem_take(&cts_mode_sem, K_FOREVER);
		printk("About to sample all of the data\n");
		scu_process_hts221_sample();
		scu_process_lps22hb_sample();
		scu_process_ccs811_sample();
		scu_process_lis2dh_sample();
		k_sem_give(&sensor_logging_sem);
		
		printk("Done sampling all of the data\n");
		k_sem_give(&cts_mode_sem);
		k_sleep(K_SECONDS(sampling_time));
	}
}
#endif

#ifdef CONFIG_STATIC
/**
 * @brief Ultrasonic reading thread, samples the ultrasonic sensor every set sampling period and 
 * 	  outputs the distance read 
 */
void thread_scu_ultrasonic(void) {
	const struct device *dev = scu_sensors_init("GPIO_1");
	while (1) {
		// Process a sample from the ultrasonic sensor and output the distance
		scu_process_ultrasonic_sample();
		k_mutex_lock(&us_mutex, K_FOREVER);
		printk("The object in front of the ultrasonic sensor is %d centimeters away\n",
		       current_sensor_data.distance);
		k_mutex_unlock(&us_mutex);
		k_sleep(K_MSEC(100));
	}
}
#endif