# Project - Argon Weather Station Node

## Overview

This is the code for the weather station node (Particle Argon) in the Bluetooth mesh network. This node starts by beaconing to be provisions, and the base node
will receive this beacon and provision the node. Then the weather station node will respond to sensor requests from the base node. 
The thingy node has the following sensors: 
* Temperature
* Humidity
* VOC
* PM10

## Requirements

* Particle Argon

## Folder Structure

```
├── ./CMakeLists.txt
├── ./Kconfig
├── ./prj.conf
├── ./segger.sh
└── ./src
    ├── ./src/board.c
    ├── ./src/board.h
    ├── ./src/main.c
    ├── ./src/mesh.c
    ├── ./src/mesh.h
    ├── ./src/scu_sensors.c
    ├── ./src/scu_sensors.h
    ├── ./src/sen5x_i2c.c
    ├── ./src/sen5x_i2c.h
    ├── ./src/sensirion_common.c
    ├── ./src/sensirion_common.h
    ├── ./src/sensirion_config.h
    ├── ./src/sensirion_i2c.c
    ├── ./src/ sensirion_i2c.h
    ├── ./src/sensirion_i2c.h
    ├── ./src/sensirion_i2c_hal.c
    └── ./src/sensirion_i2c_hal.h
```

## Building and Running 
```SHELL
$ cd argon/
$ west build -p

$ west flash 
$ ./segger.sh
```

Within the directory `argon`, run `west build -p` to compile the code. Then, run `west flash` to flash the code onto the Argon. Then, (optional), 
view the debug output of the node through segger RTT by running `./segger.sh`. 
