/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/printk.h>

#include <settings/settings.h>
#include <devicetree.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/hwinfo.h>
#include <sys/byteorder.h>
#include <bluetooth/hci.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>

#include "board.h"

/*########################### NEW INCLUDES ####################################### */
#define MOD_LF            0x0000
#define OP_HELLO          0xbb
#define OP_HEARTBEAT      0xbc
#define OP_BADUSER        0xbd
#define OP_VND_HELLO      BT_MESH_MODEL_OP_3(OP_HELLO, BT_COMP_ID_LF)
#define OP_VND_HEARTBEAT  BT_MESH_MODEL_OP_3(OP_HEARTBEAT, BT_COMP_ID_LF)
#define OP_VND_BADUSER    BT_MESH_MODEL_OP_3(OP_BADUSER, BT_COMP_ID_LF)

#define IV_INDEX          0
#define DEFAULT_TTL       31
#define GROUP_ADDR        0xc123
#define NET_IDX           0x000
#define APP_IDX           0x000
#define FLAGS             0

/* Maximum characters in "hello" message */
#define HELLO_MAX         8

#define MAX_SENS_STATUS_LEN 8

#define SENS_PROP_ID_PRESENT_DEVICE_TEMP 0x0054

enum {
	SENSOR_HDR_A = 0,
	SENSOR_HDR_B = 1,
};

struct sensor_hdr_a {
	uint16_t prop_id:11;
	uint16_t length:4;
	uint16_t format:1;
} __packed;

struct sensor_hdr_b {
	uint8_t length:7;
	uint8_t format:1;
	uint16_t prop_id;
} __packed;

static struct k_work hello_work;
static struct k_work baduser_work;
static struct k_work mesh_start_work;


#define STATE_OFF	0x00
#define STATE_ON	0x01
#define STATE_DEFAULT	0x01
#define STATE_RESTORE	0x02

/* Model Operation Codes */
#define BT_MESH_MODEL_OP_GEN_ONOFF_GET		BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET		BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS	BT_MESH_MODEL_OP_2(0x82, 0x04)

#define BT_MESH_MODEL_OP_SENS_DESC_GET		BT_MESH_MODEL_OP_2(0x82, 0x30)
#define BT_MESH_MODEL_OP_SENS_GET		BT_MESH_MODEL_OP_2(0x82, 0x31)
#define BT_MESH_MODEL_OP_SENS_COL_GET		BT_MESH_MODEL_OP_2(0x82, 0x32)
#define BT_MESH_MODEL_OP_SENS_SERIES_GET	BT_MESH_MODEL_OP_2(0x82, 0x33)

#define BT_MESH_MODEL_OP_SENS_DESC_STATUS	BT_MESH_MODEL_OP_1(0x51)
#define BT_MESH_MODEL_OP_SENS_STATUS		BT_MESH_MODEL_OP_1(0x52)

struct led_onoff_state {
	uint8_t current;
	uint8_t previous;
	uint8_t dev_id;

	uint8_t last_tid;
	uint16_t last_tx_addr;
	int64_t last_msg_timestamp;
};

void mesh_send_hello(void);
void mesh_send_baduser(void);

uint16_t mesh_get_addr(void);
bool mesh_is_initialized(void);
void mesh_start(void);
int mesh_init(void);


/* ################################ NEW INCLUDES ############################# */


#define OP_ONOFF_GET       BT_MESH_MODEL_OP_2(0x82, 0x01)
#define OP_ONOFF_SET       BT_MESH_MODEL_OP_2(0x82, 0x02)
#define OP_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)
#define OP_ONOFF_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x04)

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

static const char *const onoff_str[] = { "off", "on" };

static struct {
	bool val;
	uint8_t tid;
	uint16_t src;
	uint32_t transition_time;
	struct k_work_delayable work;
} onoff;

/* OnOff messages' transition time and remaining time fields are encoded as an
 * 8 bit value with a 6 bit step field and a 2 bit resolution field.
 * The resolution field maps to:
 * 0: 100 ms
 * 1: 1 s
 * 2: 10 s
 * 3: 20 min
 */
static const uint32_t time_res[] = {
	100,
	MSEC_PER_SEC,
	10 * MSEC_PER_SEC,
	10 * 60 * MSEC_PER_SEC,
};

static inline int32_t model_time_decode(uint8_t val)
{
	uint8_t resolution = (val >> 6) & BIT_MASK(2);
	uint8_t steps = val & BIT_MASK(6);

	if (steps == 0x3f) {
		return SYS_FOREVER_MS;
	}

	return steps * time_res[resolution];
}

static inline uint8_t model_time_encode(int32_t ms)
{
	if (ms == SYS_FOREVER_MS) {
		return 0x3f;
	}

	for (int i = 0; i < ARRAY_SIZE(time_res); i++) {
		if (ms >= BIT_MASK(6) * time_res[i]) {
			continue;
		}

		uint8_t steps = ceiling_fraction(ms, time_res[i]);

		return steps | (i << 6);
	}

	return 0x3f;
}

static int onoff_status_send(struct bt_mesh_model *model,
			     struct bt_mesh_msg_ctx *ctx)
{
	uint32_t remaining;

	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_STATUS, 3);
	bt_mesh_model_msg_init(&buf, OP_ONOFF_STATUS);

	remaining = k_ticks_to_ms_floor32(
			    k_work_delayable_remaining_get(&onoff.work)) +
		    onoff.transition_time;

	/* Check using remaining time instead of "work pending" to make the
	 * onoff status send the right value on instant transitions. As the
	 * work item is executed in a lower priority than the mesh message
	 * handler, the work will be pending even on instant transitions.
	 */
	if (remaining) {
		net_buf_simple_add_u8(&buf, !onoff.val);
		net_buf_simple_add_u8(&buf, onoff.val);
		net_buf_simple_add_u8(&buf, model_time_encode(remaining));
	} else {
		net_buf_simple_add_u8(&buf, onoff.val);
	}

	return bt_mesh_model_send(model, ctx, &buf, NULL, NULL);
}

static void onoff_timeout(struct k_work *work)
{
	if (onoff.transition_time) {
		/* Start transition.
		 *
		 * The LED should be on as long as the transition is in
		 * progress, regardless of the target value, according to the
		 * Bluetooth Mesh Model specification, section 3.1.1.
		 */
		board_led_set(true);

		k_work_reschedule(&onoff.work, K_MSEC(onoff.transition_time));
		onoff.transition_time = 0;
		return;
	}

	board_led_set(onoff.val);
}

/* Generic OnOff Server message handlers */

static int gen_onoff_get(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	onoff_status_send(model, ctx);
	return 0;
}

static int gen_onoff_set_unack(struct bt_mesh_model *model,
			       struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf)
{
	uint8_t val = net_buf_simple_pull_u8(buf);
	uint8_t tid = net_buf_simple_pull_u8(buf);
	int32_t trans = 0;
	int32_t delay = 0;

	if (buf->len) {
		trans = model_time_decode(net_buf_simple_pull_u8(buf));
		delay = net_buf_simple_pull_u8(buf) * 5;
	}

	/* Only perform change if the message wasn't a duplicate and the
	 * value is different.
	 */
	if (tid == onoff.tid && ctx->addr == onoff.src) {
		/* Duplicate */
		return 0;
	}

	if (val == onoff.val) {
		/* No change */
		return 0;
	}

	printk("set: %s delay: %d ms time: %d ms\n", onoff_str[val], delay,
	       trans);

	onoff.tid = tid;
	onoff.src = ctx->addr;
	onoff.val = val;
	onoff.transition_time = trans;

	/* Schedule the next action to happen on the delay, and keep
	 * transition time stored, so it can be applied in the timeout.
	 */
	k_work_reschedule(&onoff.work, K_MSEC(delay));

	return 0;
}

static int gen_onoff_set(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	(void)gen_onoff_set_unack(model, ctx, buf);
	onoff_status_send(model, ctx);

	return 0;
}

static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ OP_ONOFF_GET,       BT_MESH_LEN_EXACT(0), gen_onoff_get },
	{ OP_ONOFF_SET,       BT_MESH_LEN_MIN(2),   gen_onoff_set },
	{ OP_ONOFF_SET_UNACK, BT_MESH_LEN_MIN(2),   gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};

/* Generic OnOff Client */

static int gen_onoff_status(struct bt_mesh_model *model,
			    struct bt_mesh_msg_ctx *ctx,
			    struct net_buf_simple *buf)
{
	uint8_t present = net_buf_simple_pull_u8(buf);

	if (buf->len) {
		uint8_t target = net_buf_simple_pull_u8(buf);
		int32_t remaining_time =
			model_time_decode(net_buf_simple_pull_u8(buf));

		printk("OnOff status: %s -> %s: (%d ms)\n", onoff_str[present],
		       onoff_str[target], remaining_time);
		return 0;
	}

	printk("OnOff status: %s\n", onoff_str[present]);

	return 0;
}

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
	{OP_ONOFF_STATUS, BT_MESH_LEN_MIN(1), gen_onoff_status},
	BT_MESH_MODEL_OP_END,
};

static int sensor_desc_get(struct bt_mesh_model *model,
			   struct bt_mesh_msg_ctx *ctx,
			   struct net_buf_simple *buf)
{
	/* TODO */
	return 0;
}

static void sens_temperature_celsius_fill(struct net_buf_simple *msg)
{
	struct sensor_hdr_a hdr;
	/* TODO Get only temperature from sensor */
	//struct sensor_value val[2];
	int16_t temp_degrees = 19; 

	hdr.format = SENSOR_HDR_A;
	hdr.length = sizeof(temp_degrees);
	hdr.prop_id = SENS_PROP_ID_PRESENT_DEVICE_TEMP;

	//get_hdc1010_val(val);
	//temp_degrees = sensor_value_to_double(&val[0]) * 100;

	net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));
	net_buf_simple_add_le16(msg, temp_degrees);
}

static void sens_unknown_fill(uint16_t id, struct net_buf_simple *msg)
{
	struct sensor_hdr_b hdr;

	/*
	 * When the message is a response to a Sensor Get message that
	 * identifies a sensor property that does not exist on the element, the
	 * Length field shall represent the value of zero and the Raw Value for
	 * that property shall be omitted. (Mesh model spec 1.0, 4.2.14).
	 *
	 * The length zero is represented using the format B and the special
	 * value 0x7F.
	 */
	hdr.format = SENSOR_HDR_B;
	hdr.length = 0x7FU;
	hdr.prop_id = id;

	net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));
}

static void sensor_create_status(uint16_t id, struct net_buf_simple *msg)
{
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS);

	switch (id) {
	case SENS_PROP_ID_PRESENT_DEVICE_TEMP:
		sens_temperature_celsius_fill(msg);
		break;
	default:
		sens_unknown_fill(id, msg);
		break;
	}
}

static int sensor_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
		      struct net_buf_simple *buf)
{
	NET_BUF_SIMPLE_DEFINE(msg, 1 + MAX_SENS_STATUS_LEN + 4);
	uint16_t sensor_id;

	sensor_id = net_buf_simple_pull_le16(buf);
	sensor_create_status(sensor_id, &msg);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("Unable to send Sensor get status response\n");
	}

	return 0;
}

static int sensor_col_get(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	/* TODO */
	return 0;
}

static int sensor_series_get(struct bt_mesh_model *model,
			     struct bt_mesh_msg_ctx *ctx,
			     struct net_buf_simple *buf)
{
	/* TODO */
	return 0;
}

/* Mapping of message handlers for Sensor Server (0x1100) */
static const struct bt_mesh_model_op sensor_srv_op[] = {
	{ BT_MESH_MODEL_OP_SENS_DESC_GET,   BT_MESH_LEN_EXACT(0), sensor_desc_get },
	{ BT_MESH_MODEL_OP_SENS_GET,        BT_MESH_LEN_EXACT(2), sensor_get },
	{ BT_MESH_MODEL_OP_SENS_COL_GET,    BT_MESH_LEN_EXACT(2), sensor_col_get },
	{ BT_MESH_MODEL_OP_SENS_SERIES_GET, BT_MESH_LEN_EXACT(2), sensor_series_get },
};


static int sensor_status(struct bt_mesh_model *model,
			    struct bt_mesh_msg_ctx *ctx,
			    struct net_buf_simple *buf)
{

	printk("sensor status\n");
	uint8_t data1 = net_buf_simple_pull_u8(buf);
	uint8_t data2 = net_buf_simple_pull_u8(buf);

	printk("Data: %d, %d\n", data1, data2);

	return 0;
}


static const struct bt_mesh_model_op sensor_cli_op[] = {
	{BT_MESH_MODEL_OP_SENS_STATUS, BT_MESH_LEN_EXACT(2), sensor_status},
	BT_MESH_MODEL_OP_END,
};


/* This application only needs one element to contain its models */
static struct bt_mesh_model models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),


	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op, NULL,
		      NULL),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, NULL,
		      NULL),


	BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV, sensor_srv_op, NULL, NULL),

	BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_CLI, sensor_cli_op, NULL, NULL),

};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/* Provisioning */

static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
	printk("OOB Number: %u\n", number);

	board_output_number(action, number);

	return 0;
}

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	board_prov_complete();
}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static uint8_t dev_uuid[16];

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 4,
	.output_actions = BT_MESH_DISPLAY_NUMBER,
	.output_number = output_number,
	.complete = prov_complete,
	.reset = prov_reset,
};

/** Send an OnOff Set message from the Generic OnOff Client to all nodes. */
static int gen_onoff_send(bool val)
{
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = models[3].keys[0], /* Use the bound key */
		.addr = BT_MESH_ADDR_ALL_NODES,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};
	static uint8_t tid;

	if (ctx.app_idx == BT_MESH_KEY_UNUSED) {
		printk("The Generic OnOff Client must be bound to a key before "
		       "sending.\n");
		return -ENOENT;
	}

	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_SET_UNACK, 2);
	bt_mesh_model_msg_init(&buf, OP_ONOFF_SET_UNACK);
	net_buf_simple_add_u8(&buf, val);
	net_buf_simple_add_u8(&buf, tid++);

	printk("Sending OnOff Set: %s\n", onoff_str[val]);

	return bt_mesh_model_send(&models[3], &ctx, &buf, NULL, NULL);
}

static int sensor_status_send(uint16_t data)
{

	printk("Sensor status send\n");

	struct bt_mesh_msg_ctx ctx = {
		.app_idx = models[4].keys[0],
		.addr = BT_MESH_ADDR_ALL_NODES,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	if (ctx.app_idx == BT_MESH_KEY_UNUSED) {
		printk("The Sensor Server must be bound to a key before sending\n");
		return -ENOENT;
	}


	BT_MESH_MODEL_BUF_DEFINE(buf, BT_MESH_MODEL_OP_SENS_STATUS, 2);
	bt_mesh_model_msg_init(&buf, BT_MESH_MODEL_OP_SENS_STATUS);
	net_buf_simple_add_u8(&buf, (data >> 8) & 0xFF);
	net_buf_simple_add_u8(&buf, data & 0xFF);

	return bt_mesh_model_send(&models[4], &ctx, &buf, NULL, NULL);

}

/** Send an OnOff Status message from the Generic OnOff Server to all nodes. */
static int gen_onoff_status_send(void)
{
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = models[2].keys[0], /* Use the bound key */
		.addr = BT_MESH_ADDR_ALL_NODES,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};
	static uint8_t tid;

	if (ctx.app_idx == BT_MESH_KEY_UNUSED) {
		printk("The Generic OnOff Server must be bound to a key before "
		       "sending.\n");
		return -ENOENT;
	}

	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_STATUS, 1);
	bt_mesh_model_msg_init(&buf, OP_ONOFF_STATUS);
	net_buf_simple_add_u8(&buf, onoff.val);

	printk("Sending OnOff Status: %s\n", onoff_str[onoff.val]);

	return bt_mesh_model_send(&models[2], &ctx, &buf, NULL, NULL);
}


uint16_t dummy = 0; 


static void button_pressed(struct k_work *work)
{
	if (bt_mesh_is_provisioned()) {
		//(void)gen_onoff_send(!onoff.val);
		
		//onoff.val = !onoff.val;
		//board_led_set(onoff.val);
		//(void)gen_onoff_status_send();
		sensor_status_send(dummy++);
		onoff.val = !onoff.val;
		board_led_set(onoff.val);
		return;
	}

	/* Self-provision with an arbitrary address.
	 *
	 * NOTE: This should never be done in a production environment.
	 *       Addresses should be assigned by a provisioner, and keys should
	 *       be generated from true random numbers. It is done in this
	 *       sample to allow testing without a provisioner.
	 */
	static uint8_t net_key[16];
	static uint8_t dev_key[16];
	static uint8_t app_key[16];
	uint16_t addr;
	int err;

	if (IS_ENABLED(CONFIG_HWINFO)) {
		addr = sys_get_le16(&dev_uuid[0]) & BIT_MASK(15);
	} else {
		addr = k_uptime_get_32() & BIT_MASK(15);
	}

	printk("Self-provisioning with address 0x%04x\n", addr);
	err = bt_mesh_provision(net_key, 0, 0, 0, addr, dev_key);
	if (err) {
		printk("Provisioning failed (err: %d)\n", err);
		return;
	}

	/* Add an application key to both Generic OnOff models: */
	err = bt_mesh_app_key_add(0, 0, app_key);
	if (err) {
		printk("App key add failed (err: %d)\n", err);
		return;
	}

	/* Models must be bound to an app key to send and receive messages with
	 * it:
	 */
	models[2].keys[0] = 0;
	models[3].keys[0] = 0;

	models[4].keys[0] = 0; 
	models[5].keys[0] = 0; 	

	printk("Provisioned and configured!\n");
}

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	printk("Mesh initialized\n");
}

void main(void)
{
	static struct k_work button_work;
	int err = -1;

	printk("Initializing...\n");

	if (IS_ENABLED(CONFIG_HWINFO)) {
		err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
	}

	if (err < 0) {
		dev_uuid[0] = 0xdd;
		dev_uuid[1] = 0xdd;
	}

	k_work_init(&button_work, button_pressed);

	err = board_init(&button_work);
	if (err) {
		printk("Board init failed (err: %d)\n", err);
		return;
	}

	k_work_init_delayable(&onoff.work, onoff_timeout);

	/* Initialize the Bluetooth Subsystem */

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

}