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

//Thread Definitions
#define THREAD_SCU_SENSOR_STACK 1024
#define THREAD_SCU_SENSOR_PRIORITY 3

// Other Defines
#define SAMPLING_TIME_DEFAULT 1
#define TRIG_PIN 2
#define ECHO_PIN 1
#define TRIG_PULSE_US 11
#define INVALID_PULSE_US 25000
#define MAX_WAIT_MS 130
#define ERR_WAIT_US 145
#define SPEED_OF_SOUND 343 // in m/s (meters per second)

struct scuSensorData {
	double temperature;
	double humidity;
	double pressure;
	// In order x, y, z
	double acceleration[3];
	double voc;
	int16_t distance;
};

/* Thread variables */
extern const k_tid_t scu_sensors_thread_tid;
extern const k_tid_t scu_ultrasonic_thread_tid; 
extern struct k_sem sensor_logging_sem;
extern struct k_sem cts_mode_sem;
extern struct k_mutex us_mutex;
extern struct k_sem us_sem;

/* Program variables */
extern struct scuSensorData current_sensor_data;
extern int thread_running_flag;

/* Function Prototypes..*/
const struct device* scu_sensors_init(char *);
int scu_sensors_samp_time_set(int);
void scu_process_lps22hb_sample(void);
void scu_process_hts221_sample(void);
void scu_process_lis2dh_sample(void);
void scu_process_ccs811_sample(void);
void scu_process_ultrasonic_sample(void);

/* Thread Prototypes */
void thread_scu_sensors(void);
void thread_scu_ultrasonic(void);

#endif