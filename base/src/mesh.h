/**
 * @file mesh.h
 * @author Hugh Roberts
 * @brief mesh.c defines
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#define REQUEST 0xFF
#define RESPONSE 0xAF

#define TEMPERATURE 0x01
#define PRESSURE 0x02
#define HUMIDITY 0x03
#define VOC 0x04
#define CO2 0x05

#define PROVISION_WAIT_TIME 10

#define NUM_DEVICES 5

#define SENSOR_CLIENT_MODEL 4

#define SENSOR_STATUS BT_MESH_MODEL_OP_2(0x00, 0x52)
#define SENSOR_GET BT_MESH_MODEL_OP_2(0x82, 0x31) 

#define UUID_LENGTH 16

#define RECEIVED_QUEUE_LENGTH 32
#define RECEIVED_QUEUE_ALIGNMENT 32

typedef struct {
	uint8_t uuid[16];
	uint32_t time; 
	uint8_t device; 
	uint32_t data; 
} ReceivedMessageQueueItem;

void provision(void);

int sensor_request(uint8_t device);

int bt_init(void);

#define RECEIVE_THREAD_STACK_SIZE 500
#define RECEIVE_THREAD_PRIORITY 5

uint8_t bluetoothListen(void *args);