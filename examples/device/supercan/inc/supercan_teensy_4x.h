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

#include <stdbool.h>
#include <stdint.h>


#ifndef TEENSY_4X
#	error "Only include this file for Teensy 4.x boards"
#endif

#define SC_BOARD_NAME "Teensy 4.x"


#define SC_BOARD_USB_BCD_DEVICE (HWREV << 8)
#define SC_BOARD_USB_MANUFACTURER_STRING "2guys"
#define SC_BOARD_CAN_CLK_HZ 80000000
#define SC_BOARD_CAN_COUNT 2


// #define SC_BOARD_CAN_NMBT_BRP_MIN           0x0001
// #define SC_BOARD_CAN_NMBT_BRP_MAX           0x0200
// #define SC_BOARD_CAN_NMBT_SJW_MIN           0x0001
// #define SC_BOARD_CAN_NMBT_SJW_MAX           0x0080
// #define SC_BOARD_CAN_NMBT_TSEG1_MIN         0x0002
// #define SC_BOARD_CAN_NMBT_TSEG1_MAX         0x0100
// #define SC_BOARD_CAN_NMBT_TSEG2_MIN         0x0002
// #define SC_BOARD_CAN_NMBT_TSEG2_MAX         0x0080

// #define SC_BOARD_CAN_DTBT_BRP_MIN           0x01
// #define SC_BOARD_CAN_DTBT_BRP_MAX           0x20
// #define SC_BOARD_CAN_DTBT_SJW_MIN           0x01
// #define SC_BOARD_CAN_DTBT_SJW_MAX           0x10
// #define SC_BOARD_CAN_DTBT_TSEG1_MIN         0x01
// #define SC_BOARD_CAN_DTBT_TSEG1_MAX         0x20
// #define SC_BOARD_CAN_DTBT_TSEG2_MIN         0x01
// #define SC_BOARD_CAN_DTBT_TSEG2_MAX         0x10

#define SC_BOARD_CAN_TX_FIFO_SIZE 8
#define SC_BOARD_CAN_RX_FIFO_SIZE 8

enum {
	SC_BOARD_DEBUG_DEFAULT,
	SC_BOARD_LED_COUNT
};

#define sc_board_can_ts_request(index) do { } while (0)
#define sc_board_can_ts_wait(index) 0
#define sc_board_led_usb_burst() do { } while (0)
#define sc_board_led_can_traffic_burst(index) do { } while (0)
#define sc_board_led_can_status_set(index, status) do { } while (0)