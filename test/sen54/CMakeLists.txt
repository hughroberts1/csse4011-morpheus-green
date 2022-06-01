set(BOARD particle_argon)
set(BOARD_FLASH_RUNNER jlink)
set(CONF_FILE prj.conf segger_rtt.conf bt.conf)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test)

#Add Include Directories
include_directories(
			./			
			)

#Add Target Sources
target_sources(app PRIVATE 
			src/main.c
            		./sen5x_i2c.c
            		./sensirion_common.c
            		./sensirion_i2c.c
            		./sensirion_i2c_hal.c
			)
