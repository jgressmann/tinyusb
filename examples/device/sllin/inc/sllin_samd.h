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


#include <sam.h>



#define SLLIN_BOARD_USB_BCD_DEVICE (1 << 8)
#define SLLIN_BOARD_USB_MANUFACTURER_STRING "Adafruit"
#define SLLIN_BOARD_LIN_COUNT 1
#define SLLIN_BOARD_NAME "Trinket M0"

enum {
	SLLIN_BOARD_DEBUG_DEFAULT,
	// LED_DEBUG_0,
	// LED_DEBUG_1,
	// LED_DEBUG_2,
	SLLIN_BOARD_LED_COUNT
};



// #define sl_board_led_usb_burst() led_burst(LED_DEBUG_3, SLLIN_LED_BURST_DURATION_MS)
// #define sl_board_led_lin_traffic_burst(index) \
// 	do { \
// 		switch (index) { \
// 		case 0: led_burst(LED_DEBUG_0, SLLIN_LED_BURST_DURATION_MS); break; \
// 		default: break; \
// 		} \
// 	} while (0)


// SLLIN_RAMFUNC extern void sl_board_led_lin_status_set(uint8_t index, int status);



