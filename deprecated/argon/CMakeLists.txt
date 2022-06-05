set(BOARD particle_argon)
set(BOARD_FLASH_RUNNER jlink)
set(CONF_FILE prj.conf segger_rtt.conf bt.conf)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(project)

#Add Include Directories
include_directories(
			../../oslib/scu_drivers/scu_ble/
            		../../oslib/scu_drivers/scu_power_management/
            		../../oslib/scu_drivers/scu_sensors/
	    		../../oslib/scu_drivers/scu_io/
			)


#Add Target Sources
target_sources(app PRIVATE 
			src/main.c
			../../oslib/scu_drivers/scu_ble/scu_hci.c
			../../oslib/scu_drivers/scu_ble/scu_ble.c
            		../../oslib/scu_drivers/scu_power_management/scu_power_management.c
            		../../oslib/scu_drivers/scu_sensors/scu_sensors.c
            		../../oslib/scu_drivers/scu_sensors/sen5x_i2c.c
            		../../oslib/scu_drivers/scu_sensors/sensirion_common.c
            		../../oslib/scu_drivers/scu_sensors/sensirion_i2c.c
            		../../oslib/scu_drivers/scu_sensors/sensirion_i2c_hal.c
	    		../../oslib/scu_drivers/scu_io/scu_io.c
			)
