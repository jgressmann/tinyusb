/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Jean Gressmann <jean@0x42.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#pragma once

#ifndef SAME54XPLAINEDPRO
#	error "Only include this file for AT SAM E54 Xplained Pro boards"
#endif

#define BOARD_NAME "Adafruit Feather M4 CAN Express"
#define SC_BOARD_USB_BCD_DEVICE (1 << 8)
#define SC_BOARD_USB_MANUFACTURER_STRING "Microchip"
#define SC_BOARD_CAN_COUNT 1
#define SC_BOARD_NAME BOARD_NAME


enum {
	SC_BOARD_DEBUG_DEFAULT,
	SC_BOARD_LED_COUNT
};

#define sc_board_led_can_status_set(index, on)
#define sc_board_led_usb_burst()
#define sc_board_led_can_traffic_burst(index)

#include <supercan_same5x.h>
