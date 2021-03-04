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

#include <stdint.h>
#include <stdbool.h>

#define BASE64_ASSERT LP_DEBUG_ASSERT
#define base64_likely likely
#define base64_unlikely unlikely
#include <base64.h>
#undef BASE64_ASSERT
#undef base64_likely
#undef base64_unlikely

#ifdef __cplusplus
extern "C" {
#endif



union rle_bit {
	uint8_t mux;
	struct {
		uint8_t count:7;
		uint8_t value:1;
	} bit;
};


#define RLE_BIT_MAX_COUNT 127


#define LP_USB_BUFFER_SIZE 64

struct linchpin {
	// StackType_t usb_device_stack[configMINIMAL_SECURE_STACK_SIZE];
	// StaticTask_t usb_device_task_mem;
	// StackType_t cdc_task_stack_mem[configMINIMAL_SECURE_STACK_SIZE];
	// StaticTask_t cdc_task_mem;
	// StackType_t lin_task_stack_mem[configMINIMAL_STACK_SIZE];
	// StaticTask_t lin_task_mem;
	// char error_message[64];
	// int error_code;
	// TaskHandle_t usb_task_handle;
	volatile uint8_t state;
	uint32_t signal_frequency;
	uint8_t cmd_buffer[LP_USB_BUFFER_SIZE];
	uint8_t usb_rx_buffer[LP_USB_BUFFER_SIZE];
	uint8_t usb_tx_buffer[LP_USB_BUFFER_SIZE];
	uint8_t signal_tx_buffer[LP_USB_BUFFER_SIZE];
	uint8_t signal_rx_buffer[LP_USB_BUFFER_SIZE];
	uint8_t cmd_count;
	uint8_t usb_rx_count;
	uint8_t usb_tx_count;
	struct base64_state usb_rx_base64_state;
	volatile uint8_t signal_tx_buffer_pi;
	volatile uint8_t signal_tx_buffer_gi;
	volatile uint8_t signal_rx_buffer_pi;
	volatile uint8_t signal_rx_buffer_gi;
	bool running;
	bool started;
	bool finished;
	bool tx_overflow;
	volatile uint32_t output_count_total;
	volatile uint32_t output_flags;
	// union rle_bit input;
	// union rle_bit output;
	uint8_t input_bit;
	uint8_t input_count;
	// uint8_t output_bit;
	uint8_t output_count;
};

#define OUTPUT_FLAG_STALLED     0x1
#define OUTPUT_FLAG_RX_OVERFLOW 0x2


#ifdef __cplusplus
} // extern "C"
#endif
