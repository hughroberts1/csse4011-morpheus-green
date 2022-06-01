/**
 ******************************************************************************
 * @file scu_sensors.h
 * @author Oliver Roman
 * @date 31.03.2022
 * @brief Contains required definitions by scu_sensors.c
 ******************************************************************************
 **/
#ifndef SCU_SENSORS
#define SCU_SENSORS

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>

#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/sensor/ccs811.h>

// Other Defines
#define SAMPLING_TIME_DEFAULT 60

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

struct scuSensorData {
	float temperature;
	float humidity;
	float pressure;
	// In order x, y, z
	float acceleration[3];
	float voc;
	float co2;
	float nox;
	float particle[4];
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
void scu_process_lps22hb_sample(void);
void scu_process_hts221_sample(void);
void scu_process_lis2dh_sample(void);
void scu_process_ccs811_sample(void);
void scu_process_ultrasonic_sample(void);

/* Thread function prototypes */
void thread_scu_sen54_poll(void);
#endif