# Project - Base Node

## Overview

This is the code for the base node in the Bluetooth mesh network. The base is responsible for setting up the mesh network, including provisioning new devices 
when they beacon. The base also implements a shell interface to communicate and send data requests to the mesh network. 

## Requirements

* nrf52840 dongle

## Folder Structure

```
├── ./base.zip
├── ./CMakeLists.txt
├── ./dtc_shell.overlay
├── ./flash.sh
├── ./prj.conf
└── ./src
    ├── ./src/board.c
    ├── ./src/board.h
    ├── ./src/cmd.c
    ├── ./src/main.c
    ├── ./src/mesh.c
    └── ./src/mesh.h
```

## Building and Running 
```SHELL
$ cd base/
$ west build -p

$ ./flash.sh
$ sudo screen /dev/ttyACM0
```

Within the directory `base`, run `west build -p` to compile the code. Then, after putting the dongle into bootloader mode, run the script `./flash.sh` to flash the 
code onto the dongle. 

Once code has been flashed, open a terminal window and run `sudo screen /dev/ttyACM0`. Then the shell interface is exposed and one can interact with the base. 
For a list of commands, press tab. 
