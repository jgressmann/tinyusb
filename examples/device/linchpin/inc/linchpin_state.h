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

enum lp_lin_mode {
	LP_LIN_MODE_OFF,
	LP_LIN_MODE_RELAXED,
	LP_LIN_MODE_STRICT,
};



enum lp_lin_state {
	LP_LIN_DISCONNECTED,
	LP_LIN_CONNECTED,
//	LP_LIN_RUNNING,
//	LP_LIN_STOPPED,
};

enum lp_cmd_state {
	LP_CMD_DISCONNECTED,
	LP_CMD_CONNECTED,
};


#define LP_CMD_BUFFER_SIZE 64





struct lin_task_state {
	uint32_t signal_tx_buffer[LP_SIGNAL_BUFFER_SIZE];
	uint32_t signal_rx_buffer[LP_SIGNAL_BUFFER_SIZE];
	volatile size_t signal_tx_buffer_pi;
	volatile size_t signal_tx_buffer_gi;
	volatile size_t signal_rx_buffer_pi;
	volatile size_t signal_rx_buffer_gi;
	uint32_t signal_frequency;
	enum lp_lin_state state;
	volatile uint8_t signal_flags;
	volatile uint8_t mode;
	struct rlew_decoder dec;
	struct rlew_encoder enc;
};

struct cmd_task_state {
	uint8_t usb_rx_buffer[LP_CMD_BUFFER_SIZE];
	uint8_t usb_tx_buffer[LP_CMD_BUFFER_SIZE];
	size_t usb_rx_buffer_gi;
	size_t usb_rx_buffer_pi;
	size_t usb_tx_buffer_gi;
	size_t usb_tx_buffer_pi;

	uint8_t cmd_count;
	uint8_t state;
};

struct linchpin {
	struct cmd_task_state cmd;
	struct lin_task_state lin;
};

#define OUTPUT_FLAG_INPUT_BAD   0x01
#define OUTPUT_FLAG_ERROR       0x02
#define OUTPUT_FLAG_INPUT_DONE  0x04
#define OUTPUT_FLAG_TX_STALLED  0x08
#define OUTPUT_FLAG_RX_OVERFLOW 0x10
#define OUTPUT_FLAG_OUTPUT_DONE 0x20



typedef void (*lp_timer_callback_t)(void);
extern lp_timer_callback_t lp_timer_callback;
extern struct linchpin lp;
void lp_init(void);
void lp_cdc_cmd_task(void);
LP_RAMFUNC void lp_cdc_lin_task(void);
LP_RAMFUNC void lp_rle_task(void);
// LP_RAMFUNC void lp_signal_next_bit(void);


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
	#error Define bool lp_cdc_is_connected(uint8_t itf);
#endif

#ifndef lp_cdc_rx_available
	#error Define uint32_t lp_cdc_rx_available(uint8_t itf);
#endif

#ifndef lp_cdc_tx_available
	#error Define uint32_t lp_cdc_tx_available(uint8_t itf);
#endif

#ifndef lp_cdc_rx
	#error Define uint32_t lp_cdc_rx(uint8_t itf, uint8_t *ptr, uint32_t count);
#endif

#ifndef lp_cdc_rx
	#error Define uint32_t lp_cdc_tx(uint8_t itf, uint8_t const *ptr, uint32_t count);
#endif

#ifndef lp_cdc_tx_flush
	#error Define void lp_cdc_tx_flush(uint8_t itf);
#endif

#ifndef lp_cdc_rx_clear
	#error Define void lp_cdc_rx_clear(uint8_t itf);
#endif

#ifndef lp_cdc_tx_clear
	#error Define void lp_cdc_tx_clear(uint8_t itf);
#endif

// #ifndef lp_state_lock
// 	#error Define void lp_state_lock(void);
// #endif

// #ifndef lp_state_unlock
// 	#error Define void lp_state_unlock(void);
// #endif


void lp_delay_ms(uint32_t ms);
bool lp_pin_set(uint32_t pin, bool value);
void lp_version(char* ptr, uint32_t capacity);

#ifdef __cplusplus
} // extern "C"
#endif
