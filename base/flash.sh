#!/bin/bash

nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
	--application build/zephyr/zephyr.hex \
	--application-version 1 base.zip

sudo chmod a+rw /dev/ttyACM1

nrfutil dfu usb-serial -pkg base.zip -p /dev/ttyACM1
