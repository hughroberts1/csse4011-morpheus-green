# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD thingy52_nrf52832)

set(BOARD_FLASH_RUNNER jlink)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(thingy)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
