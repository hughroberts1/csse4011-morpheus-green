
#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <drivers/i2c.h>
#include <stdint.h>

#define I2C0 DT_NODELABEL(i2c0)

int read_bytes(const struct device *dev, uint16_t addr,
				 const void *write_buf, size_t num_write,
				 void *read_buf, size_t num_read)
{
	int err;
	err = i2c_write(dev, write_buf, num_write, addr);
	if (err) {
		return err;
	}

	k_msleep(20);

	err = i2c_read(dev, read_buf, num_read, addr);
	if (err) {
		return err;
	}
}

void main(void)
{
	int err;

	const struct device *device = device_get_binding(DT_LABEL(I2C0));

	if (device == NULL) {
		printk("No device found\n");
		return;
	}
	if (!device_is_ready(device)) {
		printk("Device %s is not ready\n", device->name);
		return;
	}

	k_msleep(500);

	err = i2c_configure(device, I2C_MODE_MASTER | I2C_SPEED_SET(I2C_SPEED_STANDARD));
	printk("err %d\n", err);

	k_msleep(500);

	uint8_t buffer[30];
	memset(buffer, 0, 30);
	uint8_t data[48];
	memset(data, 0, 48);

	
	buffer[0] = (0xFF00 & 0x0021) >> 8;
	buffer[1] = 0x00FF & 0x0021;

	err = i2c_write(device, buffer, 2, 0x69);
	printk("0x%02X%02X err %d\n", buffer[0], buffer[1], err);
	
	/*
	k_msleep(1000);
	buffer[0] = (0xFF00 & 0xD304) >> 8;
	buffer[1] = 0x00FF & 0xD304;
	err = i2c_write(device, buffer, 2, 0x69);
	printk("0x%02X%02X err %d\n", buffer[0], buffer[1], err);
	
	k_msleep(1000);
	buffer[0] = (0xFF00 & 0x0021) >> 8;
	buffer[1] = 0x00FF & 0x0021;
	err = i2c_write(device, buffer, 2, 0x69);
	printk("0x%02X%02X err %d\n", buffer[0], buffer[1], err);
	*/
	k_msleep(1000);

	while(1) {
		buffer[0] = (0xFF00 & 0x0202) >> 8;
		buffer[1] = 0x00FF & 0x0202;
		printk("address: 0x%02X%02X ", buffer[0], buffer[1]);
		err = read_bytes(device, 0x69, buffer, 2, data, 3);
		printk("err %d %02X %02X %02X %02X\n", err, data[0], data[1], data[2], data[3]);


		if (data[1] == 0x01) {
			buffer[0] = (0xFF00 & 0x03C4) >> 8;
			buffer[1] = 0x00FF & 0x03C4;

			err = read_bytes(device, 0x69, buffer, 2, data, 24);
			printk("%d %d\n", err, data[0]);

			printk("PM1.0: %.1f ug/m^2\n", (float)((uint16_t)((data[0] << 8) | data[1]))/10);
			printk("PM2.5: %.1f ug/m^2\n", (float)((uint16_t)((data[3] << 8) | data[4]))/10);
			printk("PM4.0: %.1f ug/m^2\n", (float)((uint16_t)((data[6] << 8) | data[7]))/10);
			printk("PM10: %.1f ug/m^2\n", (float)((uint16_t)((data[9] << 8) | data[10]))/10);
			printk("Humidity: %.2f \%RH\n", (float)((uint16_t)((data[12] << 8) | data[13]))/100);
			printk("Temperature: %.2f C\n", (float)((uint16_t)((data[15] << 8) | data[16]))/200);
			printk("VOC Index: %.1f\n", (float)((uint16_t)((data[18] << 8) | data[19]))/10);
			printk("NOx Index: %.1f\n", (float)((uint16_t)((data[21] << 8) | data[22]))/10);
		}

		k_msleep(1000);
	}
}
