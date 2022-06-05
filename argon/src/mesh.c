/**
 * @file mesh.c
 * @author Hugh Roberts
 * @brief mesh functionality for weather station particle argon node
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <settings/settings.h>
#include <drivers/hwinfo.h>
#include <sys/byteorder.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <zephyr.h>

#include "board.h"
#include "mesh.h"
#include "scu_sensors.h"


uint32_t currentTime = 0;

void timer_func(void* argv) 
{    
    while(1) {
        k_msleep(1);
        currentTime++;
    }
}

K_THREAD_DEFINE(timer_thread, TIMER_THREAD_STACK_SIZE,
                timer_func, NULL, NULL, NULL,
                TIMER_THREAD_PRIORITY, 0, 0);

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

static int send_sensor_data(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, uint8_t device, float data)
{
	if (bt_mesh_is_provisioned()) {

		BT_MESH_MODEL_BUF_DEFINE(buf, SENSOR_STATUS, 1 + 4 + 1 + 4);
		bt_mesh_model_msg_init(&buf, SENSOR_STATUS);

		net_buf_simple_add_u8(&buf, RESPONSE);
		net_buf_simple_add_le32(&buf, currentTime);
		net_buf_simple_add_u8(&buf, device);
		net_buf_simple_add_le32(&buf, *((uint32_t*)(&data)));

		printk("Sending data");
		for (int i = 0; i < buf.len; i++) {
			printk("%02x ", buf.data[i]);
		}
		printk("\n");

		k_sleep(K_MSEC(20));
		board_led_set(true);
		k_sleep(K_MSEC(20));
		board_led_set(false);

		return bt_mesh_model_send(model, ctx, &buf, NULL, NULL);
	}
	return 0; 
}

void sensor_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("Received ");
	for (int i = 0; i < buf->len; i++) {
		printk("%02x ", buf->data[i]);
	}
	printk("\n");

	uint8_t type = net_buf_simple_pull_u8(buf);
	uint32_t requestingTime = net_buf_simple_pull_le32(buf);
	uint8_t device = net_buf_simple_pull_u8(buf);	

	// update the system time 
	currentTime = requestingTime; 

	board_led_set(true);
	k_sleep(K_MSEC(20));
	board_led_set(false);
	
	if (type == REQUEST) {
		switch(device) {
			case TEMPERATURE: 
				send_sensor_data(model, ctx, device, (double) scu_sensors_temp_get());
				break;
			case HUMIDITY: 
				send_sensor_data(model, ctx, device, (double) scu_sensors_hum_get());
				break;
			case VOC: 
				send_sensor_data(model, ctx, device, (double) scu_sensors_voc_get());
				break;
			case PM10: 
				send_sensor_data(model, ctx, device, (double) scu_sensors_pm10_get());
		}
	}

}

static int onoff_status_send(struct bt_mesh_model *model,
			     struct bt_mesh_msg_ctx *ctx)
{

	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_STATUS, 1);
	bt_mesh_model_msg_init(&buf, OP_ONOFF_STATUS);

	net_buf_simple_add_u8(&buf, 0xFF);

	return bt_mesh_model_send(model, ctx, &buf, NULL, NULL);
}

const struct bt_mesh_model_op sensor_srv_op[] = {
	{SENSOR_GET, 0, sensor_get},
	BT_MESH_MODEL_OP_END,
};

static void gen_onoff_get(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	onoff_status_send(model, ctx);
}

static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ OP_ONOFF_GET, 0, gen_onoff_get },
	BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),

	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op, NULL,
		      NULL),

	BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV, sensor_srv_op, NULL, NULL),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

static int mesh_init(void)
{
    static uint8_t dev_uuid[16];
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
    };

    err = bt_mesh_init(&prov, &comp);
    return err;

}

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = mesh_init();

	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		printk("Loading stored settings\n");
		settings_load();
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	printk("Mesh initialized\n");
}

int bt_init(void)
{
    return bt_enable(bt_ready);
}