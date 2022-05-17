/**
 ******************************************************************************
 * @file scu_ble.h
 * @author Oliver Roman
 * @date 31.03.2022
 * @brief Contains the required definitions by scu_ble.c
 ******************************************************************************
 **/
#ifndef SCU_BLE
#define SCU_BLE

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

/* Thread Stack Sizes */
#define THREAD_BLE_CONNECT_STACK 2048
#define THREAD_BLE_RESPONSE_STACK 2048

/* Thread Priorities */
#define THREAD_BLE_CONNECT_PRIORITY 1
#define THREAD_BLE_RESPONSE_PRIORITY 5

#define NUM_STATIC_NODES 4

/* Thread Semaphores & Comms Variables */
extern struct k_sem rx_queue_sem;
extern struct k_sem adv_sem;
extern struct k_msgq rx_queue;
extern const k_tid_t ble_connect_thread_tid; 
extern const k_tid_t ble_response_thread_tid; 

struct staticAdvPacket {
        char addr[BT_ADDR_LE_STR_LEN];
        int time;
        int RSSI;
        float distance;
        int dataReady;
};

extern struct staticAdvPacket node_packets[NUM_STATIC_NODES];

/* Function Prototypes */
void thread_ble_connect(void);
void thread_ble_response(void);

#endif