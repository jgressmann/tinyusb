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

#include <sections.h>
#include <supercan_debug.h>

#define SC_PACKED __packed
#include <supercan.h>

#include <supercan_version.h>

#ifndef likely
#define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x),0)
#endif



#define CMD_BUFFER_SIZE 64
#define MSG_BUFFER_SIZE 512

enum {
	SC_LED_BURST_DURATION_MS = 8,
	SC_BUS_ACTIVITY_TIMEOUT_MS = 256,

	SC_CAN_LED_STATUS_DISABLED = 0,
	SC_CAN_LED_STATUS_ENABLED_BUS_OFF,
	SC_CAN_LED_STATUS_ENABLED_BUS_ON_PASSIVE,
	SC_CAN_LED_STATUS_ENABLED_BUS_ON_ACTIVE,
	SC_CAN_LED_STATUS_ERROR_ACTIVE,
	SC_CAN_LED_STATUS_ERROR_PASSIVE,

	SC_CAN_LED_BLINK_DELAY_PASSIVE_MS = 512,
	SC_CAN_LED_BLINK_DELAY_ACTIVE_MS = 128,

	SC_CAN_STATUS_FIFO_TYPE_BUS_STATUS = 0,
	SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR,
	SC_CAN_STATUS_FIFO_TYPE_RXTX_ERRORS,
	SC_CAN_STATUS_FIFO_TYPE_TXR_DESYNC,
	SC_CAN_STATUS_FIFO_TYPE_RX_LOST,

	SC_TS_MAX = 0xffffffff,
};

typedef struct _sc_can_bit_timing {
	uint16_t brp;
	uint16_t tseg1;
	uint8_t tseg2;
	uint8_t sjw;
} sc_can_bit_timing;

typedef struct _sc_can_bit_timing_range {
	sc_can_bit_timing min, max;
} sc_can_bit_timing_range;


typedef struct _sc_can_status {
	volatile uint32_t timestamp_us;
	volatile uint8_t type;

	union {
		volatile struct {
			uint8_t tx : 1;
			uint8_t data_part : 1;
			uint8_t code : 6;
		} bus_error;

		volatile uint8_t bus_state;

		volatile struct {
			uint8_t tx;
			uint8_t rx;
		} counts;

		volatile uint8_t rx_lost;
	};

} sc_can_status;



__attribute__((noreturn)) extern void sc_board_reset(void);
extern uint32_t sc_board_identifier(void);
extern void sc_board_init_begin(void);
extern void sc_board_init_end(void);
extern void sc_board_led_set(uint8_t index, bool on);
extern void sc_board_leds_on_unsafe(void);
extern uint16_t sc_board_can_feat_perm(uint8_t index);
extern uint16_t sc_board_can_feat_conf(uint8_t index);
extern void sc_board_can_feat_set(uint8_t index, uint16_t features);
extern sc_can_bit_timing_range const* sc_board_can_nm_bit_timing_range(uint8_t index);
extern sc_can_bit_timing_range const* sc_board_can_dt_bit_timing_range(uint8_t index);
extern void sc_board_can_nm_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt);
extern void sc_board_can_dt_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt);
extern void sc_board_can_go_bus(uint8_t index, bool on);
SC_RAMFUNC extern bool sc_board_can_tx_queue(uint8_t index, struct sc_msg_can_tx const * msg);


extern void sc_board_can_reset(uint8_t index);
// SC_RAMFUNC extern void sc_board_can_status_fill(uint8_t index, struct sc_msg_can_status *msg);
/* place rx / tx / error messages into buffer
 *
 * return  -1 if no messages
 *         > 0 if messages placed
 *         0 if insufficient space in buffer
 */
SC_RAMFUNC extern int sc_board_can_place_msgs(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end);

SC_RAMFUNC static inline uint8_t dlc_to_len(uint8_t dlc)
{
	static const uint8_t map[16] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
	};
	return map[dlc & 0xf];
}

SC_RAMFUNC extern void sc_can_notify_task_def(uint8_t index, uint32_t count);
SC_RAMFUNC extern void sc_can_notify_task_isr(uint8_t index, uint32_t count);
extern void sc_can_log_bit_timing(sc_can_bit_timing const *c, char const* name);
SC_RAMFUNC extern void sc_can_status_queue(uint8_t index, sc_can_status const *status);

#if defined(D5035_01)
#	include "supercan_D5035_01.h"
#elif defined(SAME54XPLAINEDPRO)
#	include "supercan_same54_xplained_pro.h"
#elif defined(TEENSY_4X)
#	include "supercan_teensy_4x.h"
#else
#	error "Unsupported board!"
#endif

#ifndef D5035_01
#	define D5035_01 0
#endif

#ifndef SAME54XPLAINEDPRO
#	define SAME54XPLAINEDPRO 0
#endif

#ifndef TEENSY_4X
#	define TEENSY_4X 0
#endif

