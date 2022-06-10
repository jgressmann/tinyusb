/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#ifndef FEATHER_M4_CAN_EXPRESS
#	error "Only include this file for Adafruit Feather M4 CAN Express boards"
#endif


#define BOARD_NAME "Feather M4 CE" // full name too long
#define SC_BOARD_USB_BCD_DEVICE (1 << 8)
#define SC_BOARD_USB_MANUFACTURER_STRING "Adafruit"
#define SC_BOARD_CAN_COUNT 1
#define SC_BOARD_NAME BOARD_NAME
#define SC_BOARD_CAN_CLK_HZ 48000000


enum {
	SC_BOARD_DEBUG_DEFAULT,
	SC_BOARD_PIXEL_RED,
	SC_BOARD_PIXEL_GREEN,
	SC_BOARD_PIXEL_BLUE,
	SC_BOARD_LED_COUNT
};

SC_RAMFUNC extern void sc_board_led_can_status_set(uint8_t index, int status);

#define sc_board_led_usb_burst()
#define sc_board_led_can_traffic_burst(index)

#include <supercan_same5x.h>
