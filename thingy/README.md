# Project - Thingy Weather Station Node

## Overview

This is the code for the weather station node (Thingy:52) in the Bluetooth mesh network. This node starts by beaconing to be provisions, and the base node
will receive this beacon and provision the node. Then the weather station node will respond to sensor requests from the base node. 
The thingy node has the following sensors: 
* Temperature
* Humidity
* Pressure
* VOC
* CO2

## Requirements

* Thingy:52

## Folder Structure

```
├── ./CMakeLists.txt
├── ./prj.conf
├── ./segger.sh
└── ./src
    ├── ./src/board.c
    ├── ./src/board.h
    ├── ./src/main.c
    ├── ./src/mesh.c
    ├── ./src/mesh.h
    ├── ./src/scu_sensors.c
    └── ./src/scu_sensors.h
```

## Building and Running 
```SHELL
$ cd thingy/
$ west build -p

$ west flash 
$ ./segger.sh
```

Within the directory `thingy`, run `west build -p` to compile the code. Then, run `west flash` to flash the code onto the Thingy. Then, (optional), 
view the debug output of the node through segger RTT by running `./segger.sh`. 

