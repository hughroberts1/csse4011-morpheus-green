/**
 * @file mesh.h
 * @author Hugh Roberts
 * @brief mesh defines for weather station thingy
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#define THINGY 0x01

#define REQUEST 0xFF
#define RESPONSE 0xAF

#define TEMPERATURE 0x01
#define PRESSURE 0x02
#define HUMIDITY 0x03
#define VOC 0x04
#define CO2 0x05

#define NUM_DEVICES 5

#define SENSOR_STATUS BT_MESH_MODEL_OP_2(0x00, 0x52)
#define SENSOR_GET BT_MESH_MODEL_OP_2(0x82, 0x31)

#define TIMER_THREAD_STACK_SIZE 1024
#define TIMER_THREAD_PRIORITY 5

#define OP_ONOFF_GET       BT_MESH_MODEL_OP_2(0x82, 0x01)
#define OP_ONOFF_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x04)

int bt_init(void);