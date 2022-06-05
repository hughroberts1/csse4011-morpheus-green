/**
 * @file mesh.c
 * @author Hugh Roberts
 * @brief Mesh functionality for the base node
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <sys/printk.h>
#include <settings/settings.h>
#include <devicetree.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/hwinfo.h>
#include <sys/byteorder.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>       
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr.h>
#include <shell/shell.h>
#include <time.h>
#include <logging/log.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/util.h>
#include <inttypes.h>
#include <kernel.h>

#include "mesh.h"
#include "board.h"

#define TIMER_STACK_SIZE 1024
#define TIMER_PRIORITY 5

K_SEM_DEFINE(sem_unprov_beacon, 0, 1);
K_SEM_DEFINE(sem_node_added, 0, 1);
K_SEM_DEFINE(sem_list_nodes, 0, 1);

static const uint16_t net_idx = 0;
static const uint16_t app_idx = 0;

static uint16_t self_addr = 1, node_addr;
static uint8_t node_uuid[UUID_LENGTH]; 

struct k_msgq ReceivedMessageQueue; 

// Initialise message queue (received messages from mesh nodes)
K_MSGQ_DEFINE(ReceivedMessageQueue, sizeof(ReceivedMessageQueueItem), 
        RECEIVED_QUEUE_LENGTH, RECEIVED_QUEUE_ALIGNMENT);

// initialise thread for processing incoming message
K_THREAD_DEFINE(receiveIncoming, RECEIVE_THREAD_STACK_SIZE, 
	bluetoothListen, NULL, NULL, NULL, RECEIVE_THREAD_PRIORITY, 0, 0);

void thread_list_nodes(void);

K_THREAD_DEFINE(list_nodes_thread, LIST_NODES_THREAD_STACK_SIZE, thread_list_nodes, NULL, NULL, NULL, LIST_NODES_THREAD_PRIORITY, 0, 0);

struct Map {
	uint8_t device_id; 
	char* device_name;
};

struct Map devices[NUM_DEVICES] = {
	{.device_id = TEMPERATURE, .device_name = "TEMPERATURE"},
	{.device_id = PRESSURE, .device_name = "PRESSURE"},
	{.device_id = HUMIDITY, .device_name = "HUMIDITY"},
	{.device_id = VOC, .device_name = "VOC"},
	{.device_id = CO2, .device_name = "CO2"}
};

char* get_device_name(uint8_t device_id)
{
	for (uint8_t i = 0; i < NUM_DEVICES; i++) {
		if (devices[i].device_id == device_id) {
			return devices[i].device_name;
		}
	}
	return "No device with ID";
}

static void attention_on(struct bt_mesh_model *mod)
{
	board_led_set(true);
}

static void attention_off(struct bt_mesh_model *mod)
{
	board_led_set(false);
}

static const struct bt_mesh_health_srv_cb health_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);


static struct bt_mesh_cfg_cli cfg_cli = {
};

static void health_current_status(struct bt_mesh_health_cli *cli, uint16_t addr,
				  uint8_t test_id, uint16_t cid, uint8_t *faults,
				  size_t fault_count)
{
	size_t i;

	printk("Health Current Status from 0x%04x\n", addr);

	if (!fault_count) {
		printk("Health Test ID 0x%02x Company ID 0x%04x: no faults\n",
		       test_id, cid);
		return;
	}

	printk("Health Test ID 0x%02x Company ID 0x%04x Fault Count %zu:\n",
	       test_id, cid, fault_count);

	for (i = 0; i < fault_count; i++) {
		printk("\t0x%02x\n", faults[i]);
	}
}

static struct bt_mesh_health_cli health_cli = {
	.current_status = health_current_status,
};

static int sensor_status(struct bt_mesh_model *model, 
				struct bt_mesh_msg_ctx *ctx, 
				struct net_buf_simple *buf)
{
	uint16_t addr = ctx->addr;
	struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(addr);

	uint8_t type = net_buf_simple_pull_u8(buf);
	uint32_t time = net_buf_simple_pull_le32(buf);
	uint8_t device = net_buf_simple_pull_u8(buf);
	uint32_t data = net_buf_simple_pull_le32(buf);

	ReceivedMessageQueueItem rxMessage; 
	rxMessage.time = time;
	rxMessage.data = data;
	rxMessage.device = device;
	memcpy(rxMessage.uuid, node->uuid, sizeof(node->uuid));

	while (k_msgq_put(&ReceivedMessageQueue, &rxMessage, K_NO_WAIT) != 0) {
		k_msgq_purge(&ReceivedMessageQueue);
	}

	return 0;
}

uint8_t bluetoothListen(void *args)
{
	ReceivedMessageQueueItem rxMessage;

	while (1) {
		
		k_msgq_get(&ReceivedMessageQueue, &rxMessage, K_FOREVER);

		// unpack the incoming message
		uint8_t uuid[UUID_LENGTH];
		uint32_t time = rxMessage.time;
		uint32_t data = rxMessage.data;
		uint8_t device = rxMessage.device;
		memcpy(uuid, &rxMessage.uuid, sizeof(uuid));

		// print out data
		printk("UUID: ");
		for (uint8_t i = 0; i < UUID_LENGTH; i++) {
			printk("%02x", uuid[i]);
		}
		printk("\n");
		printk("Time: %d Device: %d Data: %f\n\n", time, device, *((float *)(&data)));

	}
	return 0;
}


static const struct bt_mesh_model_op sensor_cli_op[] = {
	{SENSOR_STATUS, BT_MESH_LEN_MIN(1), sensor_status},
	BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_cdb_node* activeNodes[MAX_NODES];
static uint8_t numNodesActive = 0; 

static void gen_onoff_status(struct bt_mesh_model *model,
			     struct bt_mesh_msg_ctx *ctx,
			     struct net_buf_simple *buf)
{
	net_buf_simple_pull_u8(buf);

	activeNodes[numNodesActive++] = bt_mesh_cdb_node_get(ctx->addr);
}

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
	{OP_ONOFF_STATUS, 1, gen_onoff_status},
	BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_CLI(&health_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),

	BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_CLI, sensor_cli_op, NULL, NULL),

	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, NULL,
		      NULL),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

int sensor_request(uint8_t device)
{
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = models[SENSOR_CLIENT_MODEL].keys[0], 
		.addr = BT_MESH_ADDR_ALL_NODES,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	if (ctx.app_idx == BT_MESH_KEY_UNUSED) {
		printk("The Generic OnOff Client must be bound to a key before "
		       "sending.\n");
		return -ENOENT;
	}
	uint32_t currentTime = k_uptime_get_32();

	BT_MESH_MODEL_BUF_DEFINE(buf, SENSOR_GET, 1 + 4 + 1);
	bt_mesh_model_msg_init(&buf, SENSOR_GET);

	net_buf_simple_add_u8(&buf, REQUEST);
	net_buf_simple_add_le32(&buf, currentTime);
	net_buf_simple_add_u8(&buf, device);

	printk("Sending request for device %d (%s) at system time %d\n", device, get_device_name(device),  currentTime);

	return bt_mesh_model_send(&models[SENSOR_CLIENT_MODEL], &ctx, &buf, NULL, NULL);
}


static void configure_self(struct bt_mesh_cdb_node *self)
{
	struct bt_mesh_cdb_app_key *key;
	uint8_t status = 0;
	int err;

	printk("Configuring self...\n");

	key = bt_mesh_cdb_app_key_get(app_idx);
	if (key == NULL) {
		printk("No app-key 0x%04x\n", app_idx);
		return;
	}

	/* Add Application Key */
	err = bt_mesh_cfg_app_key_add(self->net_idx, self->addr, self->net_idx,
				      app_idx, key->keys[0].app_key, &status);
	if (err || status) {
		printk("Failed to add app-key (err %d, status %d)\n", err,
		       status);
		return;
	}

	err = bt_mesh_cfg_mod_app_bind(self->net_idx, self->addr, self->addr,
				       app_idx, BT_MESH_MODEL_ID_HEALTH_CLI,
				       &status);
	if (err || status) {
		printk("Failed to bind app-key (err %d, status %d)\n", err,
		       status);
		return;
	}

	/* Bind the client models */

	err = bt_mesh_cfg_mod_app_bind(self->net_idx, self->addr, self->addr,
				       app_idx, BT_MESH_MODEL_ID_SENSOR_CLI,
				       &status);

	err = bt_mesh_cfg_mod_app_bind(self->net_idx, self->addr, self->addr, 
						app_idx, BT_MESH_MODEL_ID_GEN_ONOFF_CLI, 
						&status);

	if (err || status) {
		printk("Failed to bind app-key (err %d, status %d)\n", err,
		       status);
		return;
	}

	atomic_set_bit(self->flags, BT_MESH_CDB_NODE_CONFIGURED);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_node_store(self);
	}

	printk("Configuration complete\n");
}


static void configure_node(struct bt_mesh_cdb_node *node)
{
	NET_BUF_SIMPLE_DEFINE(buf, BT_MESH_RX_SDU_MAX);
	struct bt_mesh_comp_p0_elem elem;
	struct bt_mesh_cdb_app_key *key;
	struct bt_mesh_comp_p0 comp;
	uint8_t status;
	int err, elem_addr;

	printk("Configuring node 0x%04x...\n", node->addr);

	key = bt_mesh_cdb_app_key_get(app_idx);
	if (key == NULL) {
		printk("No app-key 0x%04x\n", app_idx);
		return;
	}

	/* Add Application Key */
	err = bt_mesh_cfg_app_key_add(net_idx, node->addr, net_idx, app_idx,
				      key->keys[0].app_key, &status);
	if (err || status) {
		printk("Failed to add app-key (err %d status %d)\n", err, status);
		return;
	}

	/* Get the node's composition data and bind all models to the appkey */
	err = bt_mesh_cfg_comp_data_get(net_idx, node->addr, 0, &status, &buf);
	if (err || status) {
		printk("Failed to get Composition data (err %d, status: %d)\n",
		       err, status);
		return;
	}

	err = bt_mesh_comp_p0_get(&comp, &buf);
	if (err) {
		printk("Unable to parse composition data (err: %d)\n", err);
		return;
	}

	elem_addr = node->addr;
	while (bt_mesh_comp_p0_elem_pull(&comp, &elem)) {
		printk("Element @ 0x%04x: %u + %u models\n", elem_addr,
		       elem.nsig, elem.nvnd);
		for (int i = 0; i < elem.nsig; i++) {
			uint16_t id = bt_mesh_comp_p0_elem_mod(&elem, i);

			if (id == BT_MESH_MODEL_ID_CFG_CLI ||
			    id == BT_MESH_MODEL_ID_CFG_SRV) {
				continue;
			}
			printk("Binding AppKey to model 0x%03x:%04x\n",
			       elem_addr, id);

			err = bt_mesh_cfg_mod_app_bind(net_idx, node->addr,
						       elem_addr, app_idx, id,
						       &status);
			if (err || status) {
				printk("Failed (err: %d, status: %d)\n", err,
				       status);
			}
		}

		for (int i = 0; i < elem.nvnd; i++) {
			struct bt_mesh_mod_id_vnd id =
				bt_mesh_comp_p0_elem_mod_vnd(&elem, i);

			printk("Binding AppKey to model 0x%03x:%04x:%04x\n",
			       elem_addr, id.company, id.id);

			err = bt_mesh_cfg_mod_app_bind_vnd(net_idx, node->addr,
							   elem_addr, app_idx,
							   id.id, id.company,
							   &status);
			if (err || status) {
				printk("Failed (err: %d, status: %d)\n", err,
				       status);
			}
		}

		elem_addr++;
	}

	atomic_set_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_node_store(node);
	}

	printk("Configuration complete\n");
}

static uint8_t check_unconfigured(struct bt_mesh_cdb_node *node, void *data)
{
	if (!atomic_test_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED)) {
		if (node->addr == self_addr) {
			configure_self(node);
		} else {
			configure_node(node);
		}
	}

	return BT_MESH_CDB_ITER_CONTINUE;
}

void provision(void)
{
    char uuid_hex_str[32 + 1];
    int err = -1;

    while (1) {
        k_sem_reset(&sem_unprov_beacon);
        k_sem_reset(&sem_node_added);
        bt_mesh_cdb_node_foreach(check_unconfigured, NULL);

        //printk("Waiting for unprovisioned beacon...\n");
        err = k_sem_take(&sem_unprov_beacon, K_SECONDS(PROVISION_WAIT_TIME));
        if (err == -EAGAIN) {
            continue;
        }

        bin2hex(node_uuid, UUID_LENGTH, uuid_hex_str, sizeof(uuid_hex_str));

        printk("Provisioning %s\n", uuid_hex_str);
        err = bt_mesh_provision_adv(node_uuid, net_idx, 0, 0);
        if (err < 0) {
            printk("Provisioning failed (err %d)\n", err);
            continue;
        }

        printk("Waiting for node to be added...\n");
        err = k_sem_take(&sem_node_added, K_SECONDS(PROVISION_WAIT_TIME));
        printk("After sem take\n");
        if (err == -EAGAIN) {
            printk("Timeout waiting for node to be added\n");
            continue;
        }

        printk("Added node 0x%04x\n", node_addr);
    }
}

static void unprovisioned_beacon(uint8_t uuid[UUID_LENGTH],
				 bt_mesh_prov_oob_info_t oob_info,
				 uint32_t *uri_hash)
{
	memcpy(node_uuid, uuid, UUID_LENGTH);
	k_sem_give(&sem_unprov_beacon);
}

static void node_added(uint16_t net_idx, uint8_t uuid[UUID_LENGTH], uint16_t addr, uint8_t num_elem)
{
	node_addr = addr;
	k_sem_give(&sem_node_added);
}

static void setup_cdb(void)
{
	struct bt_mesh_cdb_app_key *key;

	key = bt_mesh_cdb_app_key_alloc(net_idx, app_idx);
	if (key == NULL) {
		printk("Failed to allocate app-key 0x%04x\n", app_idx);
		return;
	}

	bt_rand(key->keys[0].app_key, UUID_LENGTH);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_app_key_store(key);
	}
}

static int mesh_init(void)
{
    static uint8_t dev_uuid[UUID_LENGTH];

    int err = -1; 

    if (IS_ENABLED(CONFIG_HWINFO)) {
		err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
	}

	if (err < 0) {
		dev_uuid[0] = 0xdd;
		dev_uuid[1] = 0xdd;
	}

    static const struct bt_mesh_prov prov = {
		.uuid = dev_uuid,
		.unprovisioned_beacon = unprovisioned_beacon,
		.node_added = node_added,
    };
    err = bt_mesh_init(&prov, &comp);
    return err;
}

static void bt_ready(int err)
{
	uint8_t net_key[16], dev_key[16];

	err = mesh_init();

	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	printk("Mesh initialized\n");

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		printk("Loading stored settings\n");
		settings_load();
	}

	bt_rand(net_key, 16);

	err = bt_mesh_cdb_create(net_key);
	if (err == -EALREADY) {
		printk("Using stored CDB\n");
	} else if (err) {
		printk("Failed to create CDB (err %d)\n", err);
		return;
	} else {
		printk("Created CDB\n");
		setup_cdb();
	}

	bt_rand(dev_key, 16);

	err = bt_mesh_provision(net_key, BT_MESH_NET_PRIMARY, 0, 0, self_addr,
				dev_key);
	if (err == -EALREADY) {
		printk("Using stored settings\n");
	} else if (err) {
		printk("Provisioning failed (err %d)\n", err);
		return;
	} else {
		printk("Provisioning completed\n");
	}
}

static int gen_onoff_send(void)
{
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = models[GEN_ONOFF_CLIENT_MODEL].keys[0], /* Use the bound key */
		.addr = BT_MESH_ADDR_ALL_NODES,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	if (ctx.app_idx == BT_MESH_KEY_UNUSED) {
		printk("The Generic OnOff Client must be bound to a key before "
		       "sending.\n");
		return -ENOENT;
	}

	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_GET, 1);
	bt_mesh_model_msg_init(&buf, OP_ONOFF_GET);

	net_buf_simple_add_u8(&buf, 0xFF);

	printk("Sending a ping. Waiting for %d seconds for replies\n", LIST_WAIT_TIME);

	return bt_mesh_model_send(&models[GEN_ONOFF_CLIENT_MODEL], &ctx, &buf, NULL, NULL);
}

int list_nodes(void)
{
	gen_onoff_send();
	numNodesActive = 0; 
	k_sem_give(&sem_list_nodes);
	return 0;
}

void thread_list_nodes(void)
{
	while (1) {

		k_sem_take(&sem_list_nodes, K_FOREVER);
		// now need to wait 2 seconds and print out what was collected in that time 

		k_sleep(K_SECONDS(LIST_WAIT_TIME));

		// finished sleeping, print out what we got

		printk("Number of active nodes: %d\n", numNodesActive);
		for (uint8_t i = 0; i < numNodesActive; i++) {
			printk("Node at: ");
			for (uint8_t j = 0; j < UUID_LENGTH; j++) {
				printk("%02x", activeNodes[i]->uuid[j]);
			}
			printk("\n");
		}	
	}
}

int bt_init(void)
{
	return bt_enable(bt_ready);
}