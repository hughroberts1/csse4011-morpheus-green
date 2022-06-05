/**
 * @file scu_sensors.h
 * @author Oliver Roman
 * @brief Contains required definitions by scu_sensors.c
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef SCU_SENSORS
#define SCU_SENSORS

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>

#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/sensor/ccs811.h>

// Other Defines
#define SAMPLING_TIME_DEFAULT 5

// Hash defines for the ultrasonic sensor
#define TRIG_PULSE_US 11
#define INVALID_PULSE_US 25000
#define MAX_WAIT_MS 130
#define ERR_WAIT_US 145
#define SPEED_OF_SOUND 343 // in m/s (meters per second)
// Adjust these pins based on the usage of your board
#define TRIG_PIN 2
#define ECHO_PIN 1

/* Thread Stack Sizes */
#define THREAD_SCU_SEN54_POLL_STACK 2048

/* Thread Priorities */
#define THREAD_SCU_SEN54_POLL_PRIORITY 3

struct scuSensorData
{
	float temperature;
	float humidity;
	float pressure;
	float acceleration[3];  // In order x, y, z
	float voc;
	float co2;
	float nox;
	float pm10;		// Mass concentration of particles sized 0.3 - 10 um in ug/m^3
	int16_t distance;
};

/* Thread variables */
extern struct k_mutex scuMutex;
extern struct k_sem usSem;
extern const k_tid_t scu_sen54_thread_tid; 

/* Program variables */
extern struct scuSensorData currentSensorData;
extern int threadRunFlag;

/* Function Prototypes..*/
const struct device* scu_sensors_init(char *);
int scu_sensors_samp_time_set(int);
float scu_sensors_temp_get(void);
float scu_sensors_hum_get(void);
float scu_sensors_pressure_get(void);
float *scu_sensors_acel_get(void);
float scu_sensors_voc_get(void);
float scu_sensors_co2_get(void);
float scu_sensors_pm10_get(void);
int16_t scu_sensors_dist_get(void);
/* Thread function prototypes */
void thread_scu_sen54_poll(void);
#endif