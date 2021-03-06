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

#define THINGY 0xF0
#define ARGON 0xF1

#define REQUEST 0xFF
#define RESPONSE 0xAF

#define TEMPERATURE 0x01
#define PRESSURE 0x02
#define HUMIDITY 0x03
#define VOC 0x04
#define CO2 0x05
#define PM10 0x06

#define NUM_DEVICES 6

#define PROVISION_WAIT_TIME 10
#define SENSOR_CLIENT_MODEL 4

#define GEN_ONOFF_CLIENT_MODEL 5

// opcodes for sensor messages
#define SENSOR_STATUS BT_MESH_MODEL_OP_2(0x00, 0x52)
#define SENSOR_GET BT_MESH_MODEL_OP_2(0x82, 0x31)

// opcodes for ping messages
#define OP_ONOFF_GET       BT_MESH_MODEL_OP_2(0x82, 0x01)
#define OP_ONOFF_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x04)

#define MAX_NODES 10

#define UUID_LENGTH 16

#define RECEIVED_QUEUE_LENGTH 100
#define RECEIVED_QUEUE_ALIGNMENT 32

#define RECEIVE_THREAD_STACK_SIZE 500
#define RECEIVE_THREAD_PRIORITY 5
#define CONTINUOUS_THREAD_STACK_SIZE 1024
#define CONTINUOUS_THREAD_PRIORITY 2
#define LIST_NODES_THREAD_STACK_SIZE 500
#define LIST_NODES_THREAD_PRIORITY 5

#define LIST_WAIT_TIME 2
#define PRINT_SLEEP_TIME_MS 20

#define DEFAULT_SAMPLE_PERIOD 5000
#define MIN_SAMPLING_PERIOD_SECONDS 2
#define MAX_SAMPLING_PERIOD_SECONDS 5 * 60

typedef struct {
	uint8_t uuid[16];
	uint8_t board_type; 
	uint32_t time; 
	uint8_t device; 
	uint32_t data; 
} ReceivedMessageQueueItem;

typedef struct {
	uint8_t board_type; 
	struct bt_mesh_cdb_node* cdb_node; 
} node;

struct Map {
	uint8_t device_id; 
	char* device_name;
};

void provision(void);
int sensor_request(uint8_t device);
int sensor_continuous_on(void);
int sensor_continuous_off(void);
int set_sample_period(size_t, char**);
int list_nodes(void);
int bt_init(void);
uint8_t bluetoothListen(void *args);


