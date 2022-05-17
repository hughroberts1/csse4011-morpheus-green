/**
 * @file ahu_ble.c
 * @author Hugh Roberts
 * @brief AHU device driver that will connect to the SCU and
 * send required data to serial via USB console.
 * @version 0.1
 * @date 2022-04-04
 * @copyright Copyright (c) 2022
 */
#include <bluetooth/gatt.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <kernel.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>
#include <shell/shell.h>

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <string.h>
#include <stdlib.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ble_module);


#include "led_driver.h"
#include "ahu_ble.h"
#include "hci_message.h"

/* ################### PUSH BUTTON ###################### */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

static struct k_sem buttonSem;

static const struct shell *shell;

void button_module_init(void);
static void scu_gatt_write(uint16_t handle);

bool cont_samp_enabled = false;

bool cont_sampling = false;

CREATE_FLAG(flag_is_connected);
CREATE_FLAG(flag_discover_complete);
CREATE_FLAG(flag_write_complete);

static struct bt_conn *g_conn;
static uint16_t scu_handle;
static struct bt_uuid *scu_svc_uuid = SCU_UUID;
//Used to as a key to test against scanned UUIDs
uint16_t scu_uuid[] = {0x89, 0x8e, 0x72, 0x42, 0x81, 0xf0, 0x48, 0x38, 0xad, 0xe9, 0xd2, 0x96, 0x23, 0x3d, 0x57, 0x3c};

#define ARRAY_ITEM(i, _) i

/* holds HCI packet received */
struct HCI_Message *rx_packet = &(struct HCI_Message) {
	.preamble = 0,
	.type = 0,
	.length = 0,
	.data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	.did = 0
};

/* holds HC packet to send */
struct HCI_Message *tx_packet = &(struct HCI_Message) {
	.preamble = 0xAA,
	.type = 0x01,
	.length = 0,
	.data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	.did = 0
};

/**
 * @brief Callback function for button interrupt
 * @param dev device
 * @param cb callback
 * @param pins
 */
void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	if (cont_samp_enabled) {
		k_sem_give(&buttonSem);
	}

}

/**
 * @brief Initialise push button for interrupts
 */
void button_module_init(void) 
{
    int ret;

	if (!device_is_ready(button.port)) {
		LOG_ERR("Error: button device %s is not ready\n",
		       button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return;
	}

    ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	LOG_DBG("Set up button at %s pin %d\n", button.port->name, button.pin);

}

void start_scan(void);


/**
 * @brief Callback function to discover SCU GATT handles
 * @param conn connection
 * @param attr attribute
 * @param params parameters
 * @return uint8_t result
 */
static uint8_t discover_func(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		struct bt_gatt_discover_params *params)
{
	int err;

	if (attr == NULL) {
		if (scu_handle == 0) {
			LOG_ERR("Did not discover scu chrc (%x)",
			     scu_handle);
		}

		(void)memset(params, 0, sizeof(*params));

		SET_FLAG(flag_discover_complete);

		return BT_GATT_ITER_STOP;
	}

	LOG_INF("[ATTRIBUTE] handle %u", attr->handle);

	if (params->type == BT_GATT_DISCOVER_PRIMARY &&
	    bt_uuid_cmp(params->uuid, SCU_UUID) == 0) {
		LOG_INF("Found scu service");
		params->uuid = NULL;
		params->start_handle = attr->handle + 1;
		params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, params);
		if (err != 0) {
			LOG_ERR("Discover failed (err %d)", err);
		}

		return BT_GATT_ITER_STOP;
	} else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;

		if (bt_uuid_cmp(chrc->uuid, SCU_CHRC_UUID) == 0) {
			LOG_INF("Found scu chrc");
			scu_handle = chrc->value_handle;
		} 
	}

	return BT_GATT_ITER_CONTINUE;
}

/**
 * @brief BLE Device connected callback function. If an error is detected
 * scan is restarted. After connected discovers SCU GATT handle
 * @param conn Connection handler
 * @param err BLE ERR 
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err != 0) {
		LOG_ERR("failed to connect to %s (%u)", addr, err);

		bt_conn_unref(g_conn);
		g_conn = NULL;

		start_scan(); 

		return;
	}

	LOG_INF("Connected to %s", addr);

	g_conn = conn;
	SET_FLAG(flag_is_connected);

	static struct bt_gatt_discover_params discover_params;

	LOG_DBG("Discovering services and characteristics");

	discover_params.uuid = scu_svc_uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	int error = bt_gatt_discover(g_conn, &discover_params);
	if (error != 0) {
		LOG_ERR("Discover failed(err %d)", error);
	}
}

/**
 * @brief BLE Disconnected callback. When disconnected, restarts BLE scanning.
 * @param conn Connection handler
 * @param reason Disconnect reason (ERR VAL)
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != g_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	bt_conn_unref(g_conn);

	g_conn = NULL;
	UNSET_FLAG(flag_is_connected);

	start_scan();
}

/**
 * @brief Construct a new bt conn cb define object
 */
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/**
 * @brief Callback for BLE scanning, checks whether the returned UUID
 * matches the custom UUID of the SCU. If matched, attempts to connect
 * to the device.
 * @param data Callback data from scanning
 * @param user_data Device user data
 * @return true
 * @return false
 */
static bool eir_found(struct bt_data *data, void *user_data)
{
	bt_addr_le_t *addr = user_data;
	int i;
	int matchedCount = 0; 

	LOG_INF("[AD]: %u data_len %u", data->type, data->data_len);
	
	if (data->type == BT_DATA_UUID128_ALL) {

		uint16_t temp = 0; 
		for (i = 0; i < data->data_len; i++) {
			temp = data->data[i];
			if (temp == scu_uuid[i]) {
				matchedCount++;
			}
		}
		if (matchedCount == UUID_BUFFER_SIZE) {
			// scu uuid matched
			LOG_INF("mobile uuid found, attempting to connect");
			int err = bt_le_scan_stop();
			k_msleep(10);
			if (err) {
				LOG_ERR("Stop LE scan failed");
				return true;
			}
			struct bt_le_conn_param *param = BT_LE_CONN_PARAM_DEFAULT;
			err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
						param, &g_conn);
			if (err)
            {
                LOG_ERR("Create conn failed (err %d)", err);
                start_scan();
            }

			return false;
		}
	}
	return true;
}

/**
 * @brief Callback function for when scan detects a device, scanned devices
 * are filtered by their connectibility and scan data is parsed.
 * @param addr Device address
 * @param rssi RSSI
 * @param type Device type
 * @param ad Advertisement data
 */
void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
		  struct net_buf_simple *ad)
{
	char dev[BT_ADDR_LE_STR_LEN];
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, dev, sizeof(dev));
	LOG_INF("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i",
	       dev, type, ad->len, rssi);

	if (type == BT_GAP_ADV_TYPE_ADV_IND ||
	    type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {

		bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
		LOG_INF("Device found: %s (RSSI %d)", addr_str, rssi);

		bt_data_parse(ad, eir_found, (void *)addr);
	}
}

/**
 * @brief Starts passive BLE scanning for nearby devices.
 */
void start_scan(void)
{
	int err;

	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_ACTIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return;
	}

	LOG_DBG("Scanning successfully started");
}


/**
 * @brief Discovers GATT characteristic handles of SCU
 */
static void scu_gatt_discover(void)
{
	static struct bt_gatt_discover_params discover_params;
	int err;

	LOG_DBG("Discovering services and characteristics");

	discover_params.uuid = scu_svc_uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(g_conn, &discover_params);
	if (err != 0) {
		LOG_ERR("Discover failed(err %d)", err);
	}

	WAIT_FOR_FLAG(flag_discover_complete);
	LOG_DBG("Discover complete");
}

/**
 * @brief Callback function for writing to SCU's GATT
 * @param conn Connection
 * @param err Error
 * @param params write parameters
 */
static void scu_gatt_write_cb(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_write_params *params)
{
	if (err != BT_ATT_ERR_SUCCESS) {
		LOG_ERR("Write failed: 0x%02X", err);
	}

	(void)memset(params, 0, sizeof(*params));

	SET_FLAG(flag_write_complete);
}

/**
 * @brief Write to GATT of SCU
 * @param handle
 */
static void scu_gatt_write(uint16_t handle)
{
	static struct bt_gatt_write_params write_params;
	int err;

	if (handle == scu_handle) {
		
		write_params.data = tx_packet;
		write_params.length = sizeof(*tx_packet);
		
	} 

	/* ############### print what im sending ################## */

	LOG_DBG("\n\nSENDING: did: %d \npreamble: %d\ntype: %d\nlength: %d\ndata: %d %d %d %d %d %d %d %d %d %d\n\n", \
						tx_packet->did, \
						tx_packet->preamble, \
						tx_packet->type, \
						tx_packet->length, \
						tx_packet->data[0], \
						tx_packet->data[1],\
						tx_packet->data[2],\
						tx_packet->data[3],\
						tx_packet->data[4],\
						tx_packet->data[5],\
						tx_packet->data[6],\
						tx_packet->data[7],\
						tx_packet->data[8],\
						tx_packet->data[9], \
						tx_packet->data[10],\
						tx_packet->data[11],\
						tx_packet->data[12],\
						tx_packet->data[13],\
						tx_packet->data[14],\
						tx_packet->data[15]);

	write_params.func = scu_gatt_write_cb;
	write_params.handle = handle;

	UNSET_FLAG(flag_write_complete);

	err = bt_gatt_write(g_conn, &write_params);
	if (err != 0) {
		LOG_ERR("bt_gatt_write failed: %d", err);
	}

	WAIT_FOR_FLAG(flag_write_complete);
	LOG_DBG("success");
}

/**
 * @brief Callback function for when AHU's GATT is written to by SCU
 * 
 * @param conn Connection
 * @param attr GATT attribute
 * @param buf buffer that holds data
 * @param len length of data
 * @param offset offset of data
 * @param flags flags
 * @return ssize_t
 */
static ssize_t write_ahu_chrc(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len,
			       uint16_t offset, uint8_t flags)
{

	if (!cont_sampling) {

		rx_packet = (struct HCI_Message *) buf;

		double value; 
		char reading_type[20]; 
		strcpy(reading_type, "Unknown");

		int expected = tx_packet->did;

		if (expected == HTS_TEMP) {
			strcpy(reading_type, "HTS temperature");
		} else if (expected == HTS_HUM) {
			strcpy(reading_type, "HTS humidity");
		} else if (expected == LPS) {
			strcpy(reading_type, "LPS pressure");
		} else if (expected == CCS) {
			strcpy(reading_type, "CCS VOC");
		} else if (expected == LIS2DH_X) {
			strcpy(reading_type, "Acceleration X");
		} else if (expected == LIS2DH_Y) {
			strcpy(reading_type, "Acceleration Y");
		} else if (expected == LIS2DH_Z) {
			strcpy(reading_type, "Acceleration Z");
		} else if (expected == PB) {
			strcpy(reading_type, "Push button");
		} else {
			strcpy(reading_type, "Unknown");
		}
	
	
		if (expected != PB) {
			memcpy(&value, rx_packet->data, sizeof(double));
			LOG_INF("%s: %f", reading_type, value);
		} else {
			LOG_INF("%s: %d", reading_type, rx_packet->data[0]);
		}
	

		return len;
	} else {
		rx_packet = (struct HCI_Message *) buf;
		// JSON

		double value; 

		if (rx_packet->did != PB) {
			// receiving a float
			memcpy(&value, rx_packet->data, sizeof(double));
			shell_print(shell, "{<%d>,[<%f>]}",
						rx_packet->did,
						value);
		} else {
			// receiving a 1 or 0 for push button
			shell_print(shell, "{<%d>,[<%d>]}", 
						rx_packet->did,
						rx_packet->data[0]);
		}
		

		return len;
	}

	
}

/**
 * @brief Construct a new bt gatt service define object.
 * Defines AHU's GATT for SCU to write to.
 */
BT_GATT_SERVICE_DEFINE(ahu_svc,
	BT_GATT_PRIMARY_SERVICE(AHU_UUID),
	BT_GATT_CHARACTERISTIC(AHU_CHRC_UUID,
			       BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE,
			       NULL, write_ahu_chrc, NULL),
);

/**
 * @brief Thread to send continuous sampling request packet
 * when push button is pressed
 */
void thread_button(void)
{
	while (1) {
		k_sem_take(&buttonSem, K_FOREVER); 

		cont_sampling = !cont_sampling;

		tx_packet->length = 1;
	
		memset(tx_packet->data, 0, 16);
		tx_packet->did = 'a';

		scu_gatt_write(scu_handle);

	}
}

/**
 * @brief BLE AHU  thread, starts initial BLE scanning.
 */
void thread_ble_ahu(void)
{
	int err;
	shell = shell_backend_uart_get_ptr();
	k_sem_init(&buttonSem, 0, 1);
    
	err = bt_enable(NULL);
	if (err != 0) {
		LOG_ERR("Bluetooth discover failed (err %d)", err);
	}

	start_scan();

	WAIT_FOR_FLAG(flag_is_connected);

	scu_gatt_discover();

	LOG_DBG("GATT client Passed");
}

/**
 * @brief Command handler for reading temperature
 * @param shell
 * @param argc
 * @param argv
 * @return int
 */
static int cmd_hts221_r_t(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tx_packet->length = 1;

	tx_packet->did = HTS_TEMP; 

	memset(tx_packet->data, 0, 16);

	scu_gatt_write(scu_handle);

	return 0;
}

/**
 * @brief Command handler for reading humidity
 * @param shell
 * @param argc
 * @param argv
 * @return int
 */
static int cmd_hts221_r_h(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tx_packet->length = 1;
	memset(tx_packet->data, 0, 16);
	tx_packet->did = HTS_HUM;

	scu_gatt_write(scu_handle);

	return 0;
}

/* Specify shell commands for reading from HTS221 */
/* creating subcommands (level 2 command) array for command "hts221 r"  */
SHELL_STATIC_SUBCMD_SET_CREATE(hts221_r,
		SHELL_CMD(t, NULL, "Read sensor temperature", cmd_hts221_r_t),
		SHELL_CMD(h, NULL, "Read sensor humidity", cmd_hts221_r_h),
		SHELL_SUBCMD_SET_END
);

/* error checking */
static int cmd_read_error(const struct shell *shell, size_t argc, char **argv)
{
	if ((strcmp(argv[1], "t")) && (strcmp(argv[1], "h"))) {
		LOG_ERR("invalid command");
	}
	return 0;
}

/* error checking */
static int cmd_accel_read_error(const struct shell *shell, size_t argc, char **argv)
{
	if ((strcmp(argv[1], "x")) && (strcmp(argv[1], "y")) && (strcmp(argv[1], "z"))) {
		LOG_ERR("invalid command");
	}
	return 0;
}

/* error checking */
static int cmd_sensor_error_root(const struct shell *shell, size_t argc, char **argv)
{
	if ((strcmp(argv[1], "r")) ) {
		LOG_ERR("invalid command");
	}
	return 0;
}

/* error checking */
static int cmd_led(const struct shell *shell, size_t argc, char **argv)
{
	if ((strcmp(argv[1], "r")) && (strcmp(argv[1], "g")) && (strcmp(argv[1], "b"))) {
		LOG_ERR("invalid command");
	}
	return 0;
}

/* error checking */
static int cmd_write_root(const struct shell *shell, size_t argc, char **argv)
{
	if ((strcmp(argv[1], "w")) ) {
		LOG_ERR("invalid command");
	}
	return 0;
}

/* creating subocommand (level 1 command) array for command "hts221" */
SHELL_STATIC_SUBCMD_SET_CREATE(cmdhts221,
		SHELL_CMD(r, &hts221_r, "Read sensor", cmd_read_error),
		SHELL_SUBCMD_SET_END
);

/* Creating root (level 0) command "hts221" */
SHELL_CMD_REGISTER(hts221, &cmdhts221, "HTS Sensor - temperature, humidity", cmd_sensor_error_root);

/**
 * @brief Command handler for reading humidity
 * @param shell
 * @param argc
 * @param argv
 * @return int
 */
static int cmd_lps22_r(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tx_packet->length = 1;
	memset(tx_packet->data, 0, 16);

	tx_packet->did = LPS;

	scu_gatt_write(scu_handle);

	return 0;
}

/* creating subcommand (level 1 command) array for command "lps22" */
SHELL_STATIC_SUBCMD_SET_CREATE(cmdlps22,
		SHELL_CMD(r, NULL, "Read sensor pressure", cmd_lps22_r),
		SHELL_SUBCMD_SET_END
);

/* creating root (level 0) command "lps2" */
SHELL_CMD_REGISTER(lps22, &cmdlps22, "LPS Sensor - pressure", cmd_sensor_error_root);

/* Command handler for reading acceleration x direction */
static int cmd_lis2dh_r_x(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tx_packet->length = 1;
	memset(tx_packet->data, 0, 16);
	tx_packet->did = LIS2DH_X;

	scu_gatt_write(scu_handle);

	return 0;
}

/* command handler for reading acceleration y direction */
static int cmd_lis2dh_r_y(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tx_packet->length = 1;
	memset(tx_packet->data, 0, 16);
	tx_packet->did = LIS2DH_Y;

	scu_gatt_write(scu_handle);

	return 0;
}

/* command handler for reading acceleration z direction */
static int cmd_lis2dh_r_z(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tx_packet->length = 1;
	memset(tx_packet->data, 0, 16);
	tx_packet->did = LIS2DH_Z;

	scu_gatt_write(scu_handle);

	return 0;
}

/* creating subcommand (level 2 command) array for command "lis2sh r" */
SHELL_STATIC_SUBCMD_SET_CREATE(lis2dh_r,
		SHELL_CMD(x, NULL, "Read acceleration x", cmd_lis2dh_r_x),
		SHELL_CMD(y, NULL, "Read acceleration y", cmd_lis2dh_r_y),
		SHELL_CMD(z, NULL, "Read acceleration z", cmd_lis2dh_r_z),
		SHELL_SUBCMD_SET_END
);

/* creating subcommand (level 1 command) array for command "lis2dh" */
SHELL_STATIC_SUBCMD_SET_CREATE(cmdlis2dh,
		SHELL_CMD(r, &lis2dh_r, "Read sensor", cmd_accel_read_error),
		SHELL_SUBCMD_SET_END
);

/* creatinng root (level 0) command "lis2dh" */
SHELL_CMD_REGISTER(lis2dh, &cmdlis2dh, "LIS2DH Sensor - acceleration", cmd_sensor_error_root);

/* Command handler for reading VOC */
static int cmd_ccs811_r(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tx_packet->length = 1;
	memset(tx_packet->data, 0, 16);
	tx_packet->did = CCS;

	scu_gatt_write(scu_handle);

	return 0;
}

/* creating subcommand (level 1 command) array for command "ccs811" */
SHELL_STATIC_SUBCMD_SET_CREATE(cmdccs811,
		SHELL_CMD(r, NULL, "Read sensor VOC", cmd_ccs811_r),
		SHELL_SUBCMD_SET_END
);

/* command handler for setting red RGB led  on SCU*/
static int cmd_rgb_w_r(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tx_packet->length = 1;
	memset(tx_packet->data, 0, 16);
	tx_packet->did = RGB;

	tx_packet->data[1] = 'r';

	scu_gatt_write(scu_handle);

	LOG_INF("Setting red led on SCU");

	return 0;
}

/* command handler for setting green RGB led on SCU */
static int cmd_rgb_w_g(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tx_packet->length = 1;
	memset(tx_packet->data, 0, 16);
	tx_packet->did = RGB;

	tx_packet->data[1] = 'g';

	scu_gatt_write(scu_handle);

	LOG_INF("Setting green led on SCU");

	return 0;
}

/* command handler for setting blue RGB led on SCU */
static int cmd_rgb_w_b(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tx_packet->length = 1;
	memset(tx_packet->data, 0, 16);
	tx_packet->did = RGB;

	tx_packet->data[1] = 'b';

	scu_gatt_write(scu_handle);

	LOG_INF("Setting blue led on SCU");

	return 0;
}

/* creating subcommand (level 2 command) array for command "rgb w" */
SHELL_STATIC_SUBCMD_SET_CREATE(rgb_w,
		SHELL_CMD(r, NULL, "Write red led", cmd_rgb_w_r),
		SHELL_CMD(g, NULL, "Write green led", cmd_rgb_w_g),
		SHELL_CMD(b, NULL, "Write blue led", cmd_rgb_w_b),
		SHELL_SUBCMD_SET_END
);

/* creating subcommand (level 1 command) array for command "rgb" */
SHELL_STATIC_SUBCMD_SET_CREATE(cmdrgb,
		SHELL_CMD(w, &rgb_w, "Write RGB", cmd_led),
		SHELL_SUBCMD_SET_END
);

/* creatingn root (level 0) command "rgb" */
SHELL_CMD_REGISTER(rgb, &cmdrgb, "RGB LED", cmd_write_root);

/* command handler for reading pushbutton state on SCU */
static int cmd_pb_r(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tx_packet->length = 1;
	memset(tx_packet->data, 0, 16);
	tx_packet->did = PB;

	scu_gatt_write(scu_handle);

	return 0;
}

/* creating subcommand(level 1 command) array for command "pb" */
SHELL_STATIC_SUBCMD_SET_CREATE(cmdpb,
		SHELL_CMD(r, NULL, "Read push button state", cmd_pb_r),
		SHELL_SUBCMD_SET_END
);

/* creating root (level 0) command "pb" */
SHELL_CMD_REGISTER(pb, &cmdpb, "Push button", cmd_sensor_error_root);

/* command handler for writing the duty cycle to SCU */
static int cmd_dc_w(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t duty_cycle = atoi(argv[1]);

	tx_packet->length = 1;
	memset(tx_packet->data, 0, 16);
	tx_packet->did = 'd';
	tx_packet->data[1] = (uint8_t) (duty_cycle & 0xFF);

	scu_gatt_write(scu_handle);

	LOG_INF("Setting duty cycle to %d", duty_cycle);

	return 0;
}

/* creating subcommand (level 1 commmand) array for command "dc" */
SHELL_STATIC_SUBCMD_SET_CREATE(cmddc,
		SHELL_CMD(w, NULL, "Configure duty cycle", cmd_dc_w),
		SHELL_SUBCMD_SET_END
);

/* creating root (level 0) command "dc" */
SHELL_CMD_REGISTER(dc, &cmddc, "Duty cycle", cmd_write_root);

/* command handler for writing sample time to SCU */
static int cmd_sample_w(const struct shell *shell, size_t argc, char **argv)
{
	uint16_t sample_time = atoi(argv[1]);


	tx_packet->length = 3;
	
	memset(tx_packet->data, 0, 16);
	tx_packet->did = 's';
	tx_packet->data[1] = (uint8_t) (sample_time >> 8) & 0xFF;
	tx_packet->data[2] = (uint8_t) (sample_time & 0xFF);

	scu_gatt_write(scu_handle);

	LOG_INF("Setting sample time to %d", sample_time);

	return 0;
}

/* creating subcommand (level 1 command) array for command "sample" */
SHELL_STATIC_SUBCMD_SET_CREATE(cmdsample,
		SHELL_CMD(w, NULL, "Set sampling time (seconds)", cmd_sample_w),
		SHELL_SUBCMD_SET_END
);

/* creating root (level 0) command "sample" */
SHELL_CMD_REGISTER(sample, &cmdsample, "Sampling time", cmd_write_root);

/* error checking */
static int cmd_write_all(const struct shell *shell, size_t argc, char **argv)
{
	if ((strcmp(argv[1], "o")) && (strcmp(argv[1], "f"))) {
		LOG_ERR("invalid command");
	}
	return 0;
}

/* command handler for enabling continuous sampling */
static int cmd_all_on(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	cont_samp_enabled = true;

	LOG_INF("Continuous sampling enabled");

	return 0;
}

/* command handler for disabling continuous sampling */
static int cmd_all_off(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	cont_samp_enabled = false;

	LOG_INF("Continuous sampling disabled");

	return 0;
}

/* creating subcommand (level 1 command) array for command "all" */
SHELL_STATIC_SUBCMD_SET_CREATE(cmdall,
		SHELL_CMD(o, NULL, "Enable continuous sampling", cmd_all_on),
		SHELL_CMD(f, NULL, "Disable continuous sampling", cmd_all_off),
		SHELL_SUBCMD_SET_END
);

/* creatingn root (level 0) command "all" */
SHELL_CMD_REGISTER(all, &cmdall, "Continuous sampling", cmd_write_all);

/* creating root (level 0) command "ccs811" */
SHELL_CMD_REGISTER(ccs811, &cmdccs811, "CCS811 Sensor - VOC", cmd_sensor_error_root);

/* command handler for writing the buzzer frequency to SCU */
static int cmd_buzzer_w(const struct shell *shell, size_t argc, char **argv)
{
	uint16_t buzzer_freq = atoi(argv[1]);

	tx_packet->length = 3;
	memset(tx_packet->data, 0, 16);

	tx_packet->did = BUZZ;

	// pack freq
	tx_packet->data[1] = (uint8_t) (buzzer_freq >> 8) & 0xFF;
	tx_packet->data[2] = (uint8_t) (buzzer_freq & 0xFF);
	
	scu_gatt_write(scu_handle);

	LOG_INF("Setting buzzer frequency: %d", buzzer_freq);

	return 0;
}

/* creating subcommand (level 1 command) array for command "buzzer" */
SHELL_STATIC_SUBCMD_SET_CREATE(cmdbuzzer,
		SHELL_CMD(w, NULL, "Set buzzer frequency", cmd_buzzer_w),
		SHELL_SUBCMD_SET_END
);

/* creating root (level 0) command "buzzer" */
SHELL_CMD_REGISTER(buzzer, &cmdbuzzer, "Buzzer", cmd_write_root);

