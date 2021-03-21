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


enum lp_state {
	LP_DISCONNECTED,
	LP_CONNECTED,
	LP_RUNNING
};

enum lp_run_state {
	LP_RUN_REQUESTED,
	LP_RUN_STARTED,
	LP_RUN_STOPPING,
//	LP_RUN_STOPPING2,
	LP_RUN_STOPPED,
};

#define LP_CMD_BUFFER_SIZE 64

struct linchpin {
	size_t usb_rx_buffer_gi;
	size_t usb_rx_buffer_pi;
	size_t usb_tx_buffer_gi;
	size_t usb_tx_buffer_pi;
	size_t usb_tx_compressed_gi;
	size_t usb_tx_compressed_pi;
	volatile size_t signal_tx_buffer_pi;
	volatile size_t signal_tx_buffer_gi;
	volatile size_t signal_rx_buffer_pi;
	volatile size_t signal_rx_buffer_gi;
	size_t usb_rx_compressed_offset;
	uint32_t signal_frequency;
	uint8_t cmd_buffer[LP_CMD_BUFFER_SIZE];
	uint8_t usb_rx_buffer[LP_SIGNAL_BUFFER_SIZE];
	uint8_t usb_tx_buffer[LP_SIGNAL_BUFFER_SIZE];
	uint8_t signal_tx_buffer[2*LP_SIGNAL_BUFFER_SIZE];
	uint8_t signal_rx_buffer[2*LP_SIGNAL_BUFFER_SIZE];
	uint8_t usb_rx_compressed_buffer[LP_SIGNAL_BUFFER_SIZE];
	uint8_t usb_tx_compressed_buffer[LP_SIGNAL_BUFFER_SIZE];
	uint8_t fastlz_buffer[LP_SIGNAL_BUFFER_SIZE];
	uint8_t cmd_count;
	uint8_t state;
	uint8_t run_state;
	struct base64_state usb_rx_base64_state;
	struct base64_state usb_tx_base64_state;
	uint8_t signal_priv_input_bit;
	uint8_t signal_priv_input_count;
	volatile uint8_t signal_flags;
	uint8_t signal_priv_output_count;
	uint8_t signal_priv_output_value;
};

#define OUTPUT_FLAG_TX_STALLED  0x1
#define OUTPUT_FLAG_RX_OVERFLOW 0x2
#define OUTPUT_FLAG_OUTPUT_DONE 0x4
#define OUTPUT_FLAG_INPUT_DONE  0x8


extern struct linchpin lp;
void lp_init(void);
LP_RAMFUNC void lp_cdc_task(void);
LP_RAMFUNC void lp_signal_next_bit(void);


#ifndef lp_rx_pin_read
	#error Define bool lp_rx_pin_read(void);
#endif

#ifndef lp_tx_pin_set
	#error Define void lp_tx_pin_set(void);
#endif

#ifndef lp_tx_pin_clear
	#error Define void lp_tx_pin_clear(void);
#endif

#ifndef lp_timer_stop
	#error Define void lp_timer_stop(void);
#endif

#ifndef lp_timer_start
	#error Define void lp_timer_start(void);
#endif

#ifndef lp_cdc_is_connected
	#error Define bool lp_cdc_is_connected(void);
#endif

#ifndef lp_cdc_rx_available
	#error Define uint32_t lp_cdc_rx_available(void);
#endif

#ifndef lp_cdc_tx_available
	#error Define uint32_t lp_cdc_tx_available(void);
#endif

#ifndef lp_cdc_rx
	#error Define uint32_t lp_cdc_rx(uint8_t *ptr, uint32_t count);
#endif

#ifndef lp_cdc_rx
	#error Define uint32_t lp_cdc_tx(uint8_t const *ptr, uint32_t count);
#endif

#ifndef lp_cdc_tx_flush
	#error Define void lp_cdc_tx_flush(void);
#endif

#ifndef lp_cdc_rx_clear
	#error Define void lp_cdc_rx_clear(void);
#endif

#ifndef lp_cdc_tx_clear
	#error Define void lp_cdc_tx_clear(void);
#endif


void lp_delay_ms(uint32_t ms);
bool lp_pin_set(uint32_t pin, bool value);
void lp_version(char* ptr, uint32_t capacity);

#ifdef __cplusplus
} // extern "C"
#endif
