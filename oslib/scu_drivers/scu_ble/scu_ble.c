/**
 ***************************************************************************************************
 * @file scu_ble.c
 * @author Oliver Roman
 * @date 31.03.2022
 * @brief Contains the functionality required to make a Bluetooth connection and talk to the AHU
 ***************************************************************************************************
 **/
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>

#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <sys/byteorder.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/util.h>

/* Local Library Include */
#include "scu_sensors.h"
#include "scu_hci.h"
#include "scu_ble.h"

#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000
#define SHORT_SLEEP_MS 50


// Keeps Track of BLE connection within APP
bool ble_connected = false;

static struct bt_conn *g_conn;
static uint16_t ahu_handle;

/* Custom Service Variables */
static struct bt_uuid_128 scu_uuid = BT_UUID_INIT_128(
	0x89, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38,
	0xad, 0xe9, 0xd2, 0x96, 0x23, 0x3d, 0x57, 0x3c);

static struct bt_uuid_128 scu_rx_uuid = BT_UUID_INIT_128(
	0x8a, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38,
	0xad, 0xe9, 0xd2, 0x96, 0x23, 0x3d, 0x57, 0x3c);

/* Custom Service Variables */
static struct bt_uuid_128 ahu_uuid = BT_UUID_INIT_128(
	0x8b, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38,
	0xad, 0xe9, 0xd2, 0x96, 0x23, 0x3d, 0x57, 0x3c);

static struct bt_uuid_128 ahu_rx_uuid = BT_UUID_INIT_128(
	0x8c, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38,
	0xad, 0xe9, 0xd2, 0x96, 0x23, 0x3d, 0x57, 0x3c);

#define AHU_UUID \
	BT_UUID_DECLARE_128(0x8b, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38, 0xad, 0xe9, 0xd2, 0x96,\
	0x23, 0x3d, 0x57, 0x3c)

static struct bt_uuid *ahu_uuid_ptr = AHU_UUID;

#define AHU_RX_UUID \
	BT_UUID_DECLARE_128(0x8c, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38, 0xad, 0xe9, 0xd2, 0x96,\
	0x23, 0x3d, 0x57, 0x3c)

static struct bt_uuid *ahu_rx_uuid_ptr = AHU_RX_UUID;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
	  0x89, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38, 
	  0xad, 0xe9, 0xd2, 0x96, 0x23, 0x3d, 0x57, 0x3c),
};

// Initialise defaults for the hci messages
struct HCI_Message request_hci_msg = {0,0,0,{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
struct HCI_Message response_hci_msg = {0xAA,0x02,0xF,0,{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
struct staticAdvPacket node_packets[NUM_STATIC_NODES] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};

static uint16_t ahu_handle;

K_SEM_DEFINE(rx_queue_sem, 0, 1);

K_SEM_DEFINE(adv_sem, 0, 1);

K_MSGQ_DEFINE(rx_queue, sizeof(struct HCI_Message), 10, 4);

K_THREAD_DEFINE(ble_connect_thread_tid, THREAD_BLE_CONNECT_STACK, thread_ble_connect,
		NULL, NULL, NULL, THREAD_BLE_CONNECT_PRIORITY, 0, 0);

//K_THREAD_DEFINE(ble_response_thread_tid, THREAD_BLE_RESPONSE_STACK, thread_ble_response,
//		NULL, NULL, NULL, THREAD_BLE_RESPONSE_PRIORITY, 0, 0);

/**
 * @brief 
 * Connection handler
 * @param conn Connection handle 
 * @param attr Attribute data/user data
 * @param params 
 * @return uint8_t 
 */
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params) {
	int err;

	if (attr == NULL) {
		if (ahu_handle == 0) {
			printk("Did not discover scu chrc (%x)",
				 ahu_handle);
		}
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (params->type == BT_GATT_DISCOVER_PRIMARY &&
		bt_uuid_cmp(params->uuid, AHU_UUID) == 0) {
		printk("Found test service\n");
		
		params->uuid = NULL;
		params->start_handle = attr->handle + 1;
		params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, params);
		if (err != 0) {
			printk("Discover failed (err %d)\n", err);
		}
		return BT_GATT_ITER_STOP;
	} else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
		if (bt_uuid_cmp(chrc->uuid, AHU_RX_UUID) == 0) {
			printk("Found ahu chrc\n");
			ahu_handle = chrc->value_handle;
		} 
	}
	return BT_GATT_ITER_CONTINUE;
}


/**
 * @brief Function used to discover the services and characteristics of the other side of the 
 * 		  connection
 * 
 */
static void gatt_discover(void) {
	static struct bt_gatt_discover_params discover_params;
	int err;
	printk("Discovering services and characteristics\n");

	discover_params.uuid = ahu_uuid_ptr;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(g_conn, &discover_params);
	if (err != 0)
		printk("Discover failed(err %d)\n", err);
	printk("Discover complete\n");
}


/**
 * @brief Callback function after data is sent to the AHU
 * 
 * @param conn Connection Handler
 * @param err err val
 * @param params 
 */
static void write_ahu_rx(struct bt_conn *conn, uint8_t err,
				struct bt_gatt_write_params *params) {
	if (err != BT_ATT_ERR_SUCCESS)
		printk("Write failed: 0x%02X\n", err);
	(void)memset(params, 0, sizeof(*params));
}


/**
 * @brief Function to write a HCI Packet from the rx_queue to a handle
 * 
 * @param handle The handle to a GATT Characteristic
 * @param response_msg The pointer to the HCI message to send as a response to the AHU
 */
static void gatt_write(uint16_t handle, struct HCI_Message response_msg) {
	static struct bt_gatt_write_params write_params;
	int err;

	if (handle == ahu_handle) {
		printk("Writing to chrc\n");
		write_params.data = &response_msg;
		//TODO: Changed this from sizeof(response_hci_msg) to sizeof(response_msg) check
		write_params.length = sizeof(response_msg);
	}
	write_params.func = write_ahu_rx;
	write_params.handle = handle;

	err = bt_gatt_write(g_conn, &write_params);
	if (err != 0) {
		printk("Write failed: 0x%02X\n", err);
	}

	printk("success\n");
}


/**
 * @brief Callback function after the SCU's GATT has been written to, stores data sent by AHU's
 * 	  write
 * 
 * @param conn connection handler
 * @param attr Attribute data/user data
 * @param buf Buffer storing the value
 * @param len Length of data
 * @param flags Write flags
 * @return ssize_t number of bytes written in case of success or negative values in case of error. 
 */
static ssize_t write_scu_rx(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			    uint16_t len, uint8_t flags) {
	
	if (len > sizeof(struct HCI_Message)) {
		printk("Invalid chrc length\n");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (flags != 0) {
		printk("Invalid flags %u\n", flags);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	
	// Append the HCI_Message packet to the rx queue
	if (k_msgq_put(&rx_queue, buf, K_NO_WAIT) != 0) {
		// Queue is full
		printk("Queue is full\n");
	}
	memcpy(&request_hci_msg, buf, len);
	printk("The HCI packet has been recieved: p:%u t:%u l:%u d0:%u d1:%u\n",
	       request_hci_msg.preamble, request_hci_msg.type, request_hci_msg.len,
	       request_hci_msg.did, request_hci_msg.data[0]);
	k_sem_give(&rx_queue_sem);
	return len;
}


// Set up the request and response characteristics
BT_GATT_SERVICE_DEFINE(server_svc, BT_GATT_PRIMARY_SERVICE(&scu_uuid),
		       BT_GATT_CHARACTERISTIC(&scu_rx_uuid.uuid,
		       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
		       BT_GATT_PERM_WRITE | BT_GATT_PERM_READ, NULL, write_scu_rx, NULL)
);


/**
 * @brief Connection call back
 * 
 * @param conn Connection handler
 * @param err err val
 */
static void connected(struct bt_conn *conn, uint8_t err) {
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
		ble_connected = false;
	} else {
		printk("BLE Connected to Device\n");
		ble_connected = true;
		struct bt_le_conn_param *param = BT_LE_CONN_PARAM(6, 6, 0, 400);
		g_conn = conn;
		gatt_discover();

		if (bt_conn_le_param_update(conn, param) < 0) {
			while (1) {
				printk("Connection Update Error\n");
				k_msleep(10);
			}
		}
	}
}


/**
 * @brief Disconnect Callback, used to keep track of connection status 
 *	  in the application layer
 * 
 * @param conn connection handler
 * @param reason disconnect reason.
 */
static void disconnected(struct bt_conn *conn, uint8_t reason) {
	printk("Disconnected (reason 0x%02x)\n", reason);
	ble_connected = false;
}


/**
 * @brief Conn callback data structs, holds
 *	  function pointers.
 * 
 */
static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};


/**
 * @brief Initialises bluetooth, and begins advertising data
 *	on BLE.
 * 
 */
void bt_ready(void) {
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		//settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
	printk("Advertising successfully started\n");
}

/**
 * @brief Enables bluetooth, and sets connection callback handler, awaits central to connect to
 *	peripheral (mobile)
 * 
 */
void thread_ble_connect(void) {
	bt_ready();

	bt_conn_cb_register(&conn_callbacks);
	
	while (1) {
		k_msleep(SHORT_SLEEP_MS);
	}
}

/* TODO: Figure out how to implement HCI into the gatt write and abstract BLE as much as possible
void thread_ble_response(void) {
	while(1) {
		// Checks to see if ready message signal has been sent by the ble response callback
		// if not don't wait for it
		if (!k_sem_take(&rx_queue_sem, K_NO_WAIT)) {
			printk("\n\n\n\nBefore sensor\n");
			k_msgq_get(&rx_queue, &request_hci_msg, K_FOREVER);
			if (hci_command_interpret(&request_hci_msg, &response_hci_msg) < 0)
				printk("An error occured while parsing the packet\n");
			else {
				printk("\n\n\n\nAfter sensor\n");
				gatt_write(ahu_handle, response_hci_msg);
			}
		}

		// Checks to see if a ready message signal has been sent by the logging thread
		
		// TODO: Hardcoded format for repeated sensing (could maybe change/clean up?) 
		if(!k_sem_take(&sensor_logging_sem, K_NO_WAIT)) {
			
			response_hci_msg.len = sizeof(current_sensor_data.temperature);
			response_hci_msg.did = HTS221_TEMPERATURE;
			memcpy(response_hci_msg.data, &(current_sensor_data.temperature),
			       response_hci_msg.len);
			gatt_write(ahu_handle, response_hci_msg);

			response_hci_msg.did = HTS221_HUMIDITY;
			response_hci_msg.len = sizeof(current_sensor_data.humidity);
			memcpy(response_hci_msg.data, &(current_sensor_data.humidity),
			       sizeof(current_sensor_data.humidity));
			gatt_write(ahu_handle, response_hci_msg);

			response_hci_msg.did = LPS22_AIR_PRESSURE;
			response_hci_msg.len = sizeof(current_sensor_data.pressure);
			memcpy(response_hci_msg.data, &(current_sensor_data.pressure),
			       sizeof(current_sensor_data.pressure));
			gatt_write(ahu_handle, response_hci_msg);

			response_hci_msg.did = CCS811_VOC;
			response_hci_msg.len = sizeof(current_sensor_data.voc);
			memcpy(response_hci_msg.data, &(current_sensor_data.voc),
			       sizeof(current_sensor_data.voc));
			gatt_write(ahu_handle, response_hci_msg);

			response_hci_msg.did = LIS2DH_X_ACCELERATION;
			response_hci_msg.len = sizeof(current_sensor_data.acceleration[0]);
			memcpy(response_hci_msg.data, &(current_sensor_data.acceleration[0]),
			       sizeof(current_sensor_data.acceleration[0]));
			gatt_write(ahu_handle, response_hci_msg);

			response_hci_msg.did = LIS2DH_Y_ACCELERATION;
			response_hci_msg.len = sizeof(current_sensor_data.acceleration[1]);
			memcpy(response_hci_msg.data, &(current_sensor_data.acceleration[1]),
			       sizeof(current_sensor_data.acceleration[1]));
			gatt_write(ahu_handle, response_hci_msg);

			response_hci_msg.did = LIS2DH_Z_ACCELERATION;
			response_hci_msg.len = sizeof(current_sensor_data.acceleration[2]);
			memcpy(response_hci_msg.data, &(current_sensor_data.acceleration[2]),
			       sizeof(current_sensor_data.acceleration[2]));
			gatt_write(ahu_handle, response_hci_msg);
		}

		k_msleep(5);
	}
}
*/