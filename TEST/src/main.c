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

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>

#include "board.h"

#define STATE_OFF	0x00
#define STATE_ON	0x01
#define STATE_DEFAULT	0x01
#define STATE_RESTORE	0x02

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

#define MOD_LF            0x0000
#define OP_HELLO          0xbb
#define OP_HEARTBEAT      0xbc
#define OP_BADUSER        0xbd
#define OP_VND_HELLO      BT_MESH_MODEL_OP_3(OP_HELLO, BT_COMP_ID_LF)
#define OP_VND_HEARTBEAT  BT_MESH_MODEL_OP_3(OP_HEARTBEAT, BT_COMP_ID_LF)
#define OP_VND_BADUSER    BT_MESH_MODEL_OP_3(OP_BADUSER, BT_COMP_ID_LF)

/* Maximum characters in "hello" message */
#define HELLO_MAX         8

#define MAX_SENS_STATUS_LEN 8

#define IV_INDEX          0
#define DEFAULT_TTL       31
#define GROUP_ADDR        0xc123
#define NET_IDX           0x000
#define APP_IDX           0x000
#define FLAGS             0
#define SENS_PROP_ID_PRESENT_DEVICE_TEMP 0x0054

struct led_onoff_state {
	uint8_t current;
	uint8_t previous;
	uint8_t dev_id;

	uint8_t last_tid;
	uint16_t last_tx_addr;
	int64_t last_msg_timestamp;
};

/* Definitions of models user data (Start) */
static struct led_onoff_state led_onoff_state[] = {
	/* Use LED 0 for this model */
	{ .dev_id = 0 },
};


void mesh_send_hello(void);

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

static struct k_work mesh_start_work;
static struct k_work hello_work;

static void attention_on(struct bt_mesh_model *mod)
{
	board_led_set(true);
}

static void attention_off(struct bt_mesh_model *mod)
{
	board_led_set(false);
}

static void heartbeat(const struct bt_mesh_hb_sub *sub, uint8_t hops,
		      uint16_t feat)
{
	printk("Heartbeat received\n");
}

BT_MESH_HB_CB_DEFINE(hb_cb) = {
	.recv = heartbeat,
};

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_srv_pub_root, NULL, 2 + 3);


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

/* Generic OnOff Server message handlers */
static int gen_onoff_get(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	NET_BUF_SIMPLE_DEFINE(msg, 2 + 1 + 4);
	struct led_onoff_state *state = model->user_data;

	printk("addr 0x%04x onoff 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, state->current);
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
	net_buf_simple_add_u8(&msg, state->current);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("Unable to send On Off Status response\n");
	}

	return 0;
}

static int gen_onoff_set_unack(struct bt_mesh_model *model,
			       struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf)
{
	struct net_buf_simple *msg = model->pub->msg;
	struct led_onoff_state *state = model->user_data;
	int err;
	uint8_t tid, onoff;
	int64_t now;

	onoff = net_buf_simple_pull_u8(buf);
	tid = net_buf_simple_pull_u8(buf);

	if (onoff > STATE_ON) {
		printk("Wrong state received\n");

		return 0;
	}

	now = k_uptime_get();
	if (state->last_tid == tid && state->last_tx_addr == ctx->addr &&
	    (now - state->last_msg_timestamp <= (6 * MSEC_PER_SEC))) {
		printk("Already received message\n");
	}

	state->current = onoff;
	state->last_tid = tid;
	state->last_tx_addr = ctx->addr;
	state->last_msg_timestamp = now;

	printk("addr 0x%02x state 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, state->current);

	if (board_led_set(onoff)) {
		printk("Failed to set led state\n");
		return 0;
	}

	/*
	 * If a server has a publish address, it is required to
	 * publish status on a state change
	 *
	 * See Mesh Profile Specification 3.7.6.1.2
	 *
	 * Only publish if there is an assigned address
	 */

	if (state->previous != state->current &&
	    model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		printk("publish last 0x%02x cur 0x%02x\n",
		       state->previous, state->current);
		state->previous = state->current;
		bt_mesh_model_msg_init(msg,
				       BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
		net_buf_simple_add_u8(msg, state->current);
		err = bt_mesh_model_publish(model);
		if (err) {
			printk("bt_mesh_model_publish err %d\n", err);
		}
	}

	return 0;
}

static int gen_onoff_set(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	(void)gen_onoff_set_unack(model, ctx, buf);
	(void)gen_onoff_get(model, ctx, buf);

	return 0;
}

/* Mapping of message handlers for Generic OnOff Server (0x1000) */
static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ BT_MESH_MODEL_OP_GEN_ONOFF_GET,       BT_MESH_LEN_EXACT(0), gen_onoff_get },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET,       BT_MESH_LEN_MIN(2),   gen_onoff_set },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK, BT_MESH_LEN_MIN(2),   gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};


static struct bt_mesh_cfg_cli cfg_cli = {
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
	int16_t temp_degrees = 19;

	hdr.format = SENSOR_HDR_A;
	hdr.length = sizeof(temp_degrees);
	hdr.prop_id = SENS_PROP_ID_PRESENT_DEVICE_TEMP;


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

static int sensor_series_get(struct bt_mesh_model *model,
			     struct bt_mesh_msg_ctx *ctx,
			     struct net_buf_simple *buf)
{
	/* TODO */
	return 0;
}


static int sensor_col_get(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	/* TODO */
	return 0;
}


static int vnd_hello(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
		     struct net_buf_simple *buf)
{
	char str[32];
	size_t len;

	printk("Hello message from 0x%04x\n", ctx->addr);

	if (ctx->addr == bt_mesh_model_elem(model)->addr) {
		printk("Ignoring message from self\n");
		return 0;
	}

	len = MIN(buf->len, HELLO_MAX);
	memcpy(str, buf->data, len);
	str[len] = '\0';


	strcat(str, " says hi!");


	return 0;
}

static int vnd_baduser(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
		       struct net_buf_simple *buf)
{
	char str[32];
	size_t len;

	printk("\"Bad user\" message from 0x%04x\n", ctx->addr);

	if (ctx->addr == bt_mesh_model_elem(model)->addr) {
		printk("Ignoring message from self\n");
		return 0;
	}

	len = MIN(buf->len, HELLO_MAX);
	memcpy(str, buf->data, len);
	str[len] = '\0';

	strcat(str, " is misbehaving!");

	return 0;
}

static int vnd_heartbeat(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	uint8_t init_ttl, hops;

	/* Ignore messages from self */
	if (ctx->addr == bt_mesh_model_elem(model)->addr) {
		return 0;
	}

	init_ttl = net_buf_simple_pull_u8(buf);
	hops = init_ttl - ctx->recv_ttl + 1;

	printk("Heartbeat from 0x%04x over %u hop%s\n", ctx->addr,
	       hops, hops == 1U ? "" : "s");


	return 0;
}

/* Mapping of message handlers for Sensor Server (0x1100) */
static const struct bt_mesh_model_op sensor_srv_op[] = {
	{ BT_MESH_MODEL_OP_SENS_DESC_GET,   BT_MESH_LEN_EXACT(0), sensor_desc_get },
	{ BT_MESH_MODEL_OP_SENS_GET,        BT_MESH_LEN_EXACT(2), sensor_get },
	{ BT_MESH_MODEL_OP_SENS_COL_GET,    BT_MESH_LEN_EXACT(2), sensor_col_get },
	{ BT_MESH_MODEL_OP_SENS_SERIES_GET, BT_MESH_LEN_EXACT(2), sensor_series_get },
};

static const struct bt_mesh_model_op vnd_ops[] = {
	{ OP_VND_HELLO,     BT_MESH_LEN_MIN(1), vnd_hello },
	{ OP_VND_HEARTBEAT, BT_MESH_LEN_MIN(1), vnd_heartbeat },
	{ OP_VND_BADUSER,   BT_MESH_LEN_MIN(1), vnd_baduser },
	BT_MESH_MODEL_OP_END,
};

/* This application only needs one element to contain its models */
static struct bt_mesh_model models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op, &gen_onoff_srv_pub_root,
		      &led_onoff_state[0]),

	BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV,
		      sensor_srv_op, NULL, NULL),
};

static int pub_update(struct bt_mesh_model *mod)
{
	struct net_buf_simple *msg = mod->pub->msg;

	printk("Preparing to send heartbeat\n");

	bt_mesh_model_msg_init(msg, OP_VND_HEARTBEAT);
	net_buf_simple_add_u8(msg, DEFAULT_TTL);

	return 0;
}

BT_MESH_MODEL_PUB_DEFINE(vnd_pub, pub_update, 3 + 1);

static struct bt_mesh_model vnd_models[] = {
	BT_MESH_MODEL_VND(BT_COMP_ID_LF, MOD_LF, vnd_ops, &vnd_pub, NULL),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, models, vnd_models),
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



static int provision(void)
{
	/* Self-provision with an arbitrary address.
	 *
	 * NOTE: This should never be done in a production environment.
	 *       Addresses should be assigned by a provisioner, and keys should
	 *       be generated from true random numbers. It is done in this
	 *       sample to allow testing without a provisioner.
	 */

	static const uint8_t net_key[16] = {
		0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc,
		0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc,
	};
	static const uint8_t app_key[16] = {
		0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
		0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	};
	struct bt_mesh_cfg_mod_pub pub = {
		.addr = GROUP_ADDR,
		.app_idx = APP_IDX,
		.ttl = DEFAULT_TTL,
		.period = BT_MESH_PUB_PERIOD_SEC(10),
	};
	uint8_t dev_key[16];
	uint16_t addr;
	int err;

	err = bt_rand(dev_key, sizeof(dev_key));
	if (err) {
		return err;
	}

	do {
		err = bt_rand(&addr, sizeof(addr));
		if (err) {
			return err;
		}
	} while (!addr);


	printk("Self-provisioning with address 0x%04x\n", addr);

	err = bt_mesh_provision(net_key, NET_IDX, FLAGS, IV_INDEX, addr, dev_key);
	if (err) {
		printk("Provisioning failed (err: %d)\n", err);
		return err;
	}

	printk("Configuring...\n");

	//err = bt_mesh_cfg_app_key_add(NET_IDX, addr, NET_IDX, APP_IDX, app_key, NULL);

	err = bt_mesh_app_key_add(0, 0, app_key);

	if (err) {
		printk("App key add failed (err: %d)\n", err);
		return err;
	}


	bt_mesh_cfg_mod_app_bind_vnd(NET_IDX, addr, addr, APP_IDX,
				     MOD_LF, BT_COMP_ID_LF, NULL);

	printk("1\n");

	bt_mesh_cfg_mod_app_bind(NET_IDX, addr, addr, APP_IDX,
				 BT_MESH_MODEL_ID_GEN_ONOFF_SRV, NULL);

	printk("2\n");

	bt_mesh_cfg_mod_app_bind(NET_IDX, addr, addr, APP_IDX,
				 BT_MESH_MODEL_ID_SENSOR_SRV, NULL);


	
	printk("3\n");

	models[2].keys[0] = 0;
	models[3].keys[0] = 0;

	bt_mesh_cfg_mod_app_bind(NET_IDX, addr, addr, APP_IDX,
				 BT_MESH_MODEL_ID_HEALTH_SRV, NULL);

	printk("4\n");

	bt_mesh_cfg_mod_sub_add_vnd(NET_IDX, addr, addr, GROUP_ADDR,
				    MOD_LF, BT_COMP_ID_LF, NULL);


	printk("5\n");

	bt_mesh_cfg_mod_pub_set_vnd(NET_IDX, addr, addr, MOD_LF, BT_COMP_ID_LF,
				    &pub, NULL);

	printk("6\n");

	

	printk("Configuration complete\n");

	printk("Provisioned and configured!\n");

	return addr;
}

static void button_pressed(struct k_work *work)
{
	if (bt_mesh_is_provisioned()) {

		mesh_send_hello();

		return;
	}	

	/*
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

	err = bt_mesh_app_key_add(0, 0, app_key);
	if (err) {
		printk("App key add failed (err: %d)\n", err);
		return;
	}

	models[2].keys[0] = 0;
	models[3].keys[0] = 0;

	printk("Provisioned and configured!\n");
	*/

	provision();
	
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



static void start_mesh(struct k_work *work)
{
	int err;

	if (err < 0) {
		printk("Starting mesh failed\n");
	} else {
		char buf[32];

		snprintk(buf, sizeof(buf),
			 "Mesh Started\nAddr: 0x%04x", err);
		printk("HI\n");
	}
}

static size_t first_name_len(const char *name)
{
	size_t len;

	for (len = 0; *name; name++, len++) {
		switch (*name) {
		case ' ':
		case ',':
		case '\n':
			return len;
		}
	}

	return len;
}

static void send_hello(struct k_work *work)
{
	NET_BUF_SIMPLE_DEFINE(msg, 3 + HELLO_MAX + 4);
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = APP_IDX,
		.addr = GROUP_ADDR,
		.send_ttl = DEFAULT_TTL,
	};
	const char *name = bt_get_name();

	bt_mesh_model_msg_init(&msg, OP_VND_HELLO);
	net_buf_simple_add_mem(&msg, name,
			       MIN(HELLO_MAX, first_name_len(name)));

	if (bt_mesh_model_send(&vnd_models[0], &ctx, &msg, NULL, NULL) == 0) {
		printk("Saying Hi to everyone\n");
	} else {
		printk("Sending failed\n");
	}
}

void mesh_send_hello(void)
{
	k_work_submit(&hello_work);
}

void mesh_start(void)
{
	k_work_submit(&mesh_start_work);
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

	k_work_init(&hello_work, send_hello);

	//k_work_init(&mesh_start_work, start_mesh);


	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

}