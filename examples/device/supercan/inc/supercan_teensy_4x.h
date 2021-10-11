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

#ifndef D5035_03
	#define D5035_03 0
#endif

#include <MIMXRT1062.h>


#define SC_BOARD_CAN_CLK_HZ 80000000
#define SC_BOARD_CAN_COUNT 2

#define SC_BOARD_CAN_TX_FIFO_SIZE 32
#define SC_BOARD_CAN_RX_FIFO_SIZE 32

enum {
	SC_BOARD_DEBUG_DEFAULT,
#if D5035_03
	LED_DEBUG_0,
	LED_DEBUG_1,
	LED_DEBUG_2,
	LED_DEBUG_3,
	LED_CAN0_STATUS_GREEN,
	LED_CAN0_STATUS_RED,
	LED_CAN1_STATUS_GREEN,
	LED_CAN1_STATUS_RED,
#endif
	SC_BOARD_LED_COUNT,
};

#define sc_board_can_ts_request(index) do { } while (0)
#define sc_board_can_ts_wait(index) (GPT2->CNT)


#if D5035_03
#define SC_BOARD_NAME "D5035-03"
#define SC_BOARD_USB_BCD_DEVICE (HWREV << 8)
#define SC_BOARD_USB_MANUFACTURER_STRING "2guys"
#define CAN0_TRAFFIC_LED LED_DEBUG_1
#define CAN1_TRAFFIC_LED LED_DEBUG_2

#define sc_board_led_usb_burst() led_burst(LED_DEBUG_3, SC_LED_BURST_DURATION_MS)
#define sc_board_led_can_traffic_burst(index) \
	do { \
		switch (index) { \
		case 0: led_burst(CAN0_TRAFFIC_LED, SC_LED_BURST_DURATION_MS); break; \
		case 1: led_burst(CAN1_TRAFFIC_LED, SC_LED_BURST_DURATION_MS); break; \
		default: break; \
		} \
	} while (0)

SC_RAMFUNC extern void sc_board_led_can_status_set(uint8_t index, int status);
#else
#define SC_BOARD_NAME "Teensy 4.x"
#define SC_BOARD_USB_BCD_DEVICE (4 << 8)
#define SC_BOARD_USB_MANUFACTURER_STRING "PJRC"
#define sc_board_led_usb_burst() do { } while (0)
#define sc_board_led_can_traffic_burst(index) do { } while (0)
#define sc_board_led_can_status_set(index, status) do { } while (0)
#endif // #if D5035_03
