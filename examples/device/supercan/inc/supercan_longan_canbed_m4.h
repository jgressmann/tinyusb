/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#ifndef LONGAN_CANBED_M4
#	error "Only include this file for Longan CANBED M4 boards"
#endif

#include <sam.h>
#include <mcu.h>


#define SC_BOARD_USB_BCD_DEVICE (HWREV << 8)
#define SC_BOARD_USB_MANUFACTURER_STRING "Longan"
#define SC_BOARD_CAN_COUNT 1
#define SC_BOARD_NAME BOARD_NAME
#define SC_BOARD_CAN_CLK_HZ 48000000

enum {
	SC_BOARD_DEBUG_DEFAULT,
	SC_BOARD_LED_COUNT
};

#define sc_board_led_can_status_set(index, on)
#define sc_board_led_usb_burst()
#define sc_board_led_can_traffic_burst(index)

#include <supercan_same5x.h>

