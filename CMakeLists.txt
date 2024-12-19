# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w)

include(pico_sdk_import.cmake)

project(pico_uart_bridge C CXX ASM)

pico_sdk_init()

add_executable(uart_bridge uart-bridge.c usb-descriptors.c)

target_include_directories(uart_bridge PUBLIC
	./
	pico-sdk/lib/tinyusb/src)

target_link_libraries(uart_bridge
	hardware_flash
	hardware_pwm
	pico_multicore
	pico_stdlib
	tinyusb_device
	pico_cyw43_arch_lwip_poll)

pico_add_extra_outputs(uart_bridge)
