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
#include <sllin_debug.h>
#include <sllin_version.h>
#include <sllin.h>

#ifndef likely
#define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x),0)
#endif


#define SLLIN_NAME "slLIN"

enum {
	SLLIN_QUEUE_ELEMENT_TYPE_RX_FRAME,
};

typedef struct _sllin_queue_element {
	uint8_t type;
	union {
		struct {
			uint8_t id;
			uint8_t crc;
			uint8_t flags;
			uint8_t len;
			uint8_t data[8];
		} lin_frame;
	};
} sllin_queue_element;

enum {
	SLLIN_FRAME_FLAG_ENHANCED_CHECKSUM = 0x1,
	SLLIN_FRAME_FLAG_CRC_ERROR = 0x2,
	SLLIN_FRAME_FLAG_NO_RESPONSE = 0x4,
};


__attribute__((noreturn)) extern void sllin_board_reset(void);
extern uint32_t sllin_board_identifier(void);
extern void sllin_board_init_begin(void);
extern void sllin_board_init_end(void);
extern void sllin_board_led_set(uint8_t index, bool on);
extern void sllin_board_leds_on_unsafe(void);
extern void sllin_board_lin_init(uint8_t index, uint16_t bitrate, bool master);
SLLIN_RAMFUNC extern bool sllin_board_lin_master_tx(
	uint8_t index,
	uint8_t pi,
	uint8_t len,
	uint8_t const *data,
	uint8_t crc,
	uint8_t flags);
SLLIN_RAMFUNC extern void sllin_board_lin_slave_tx(
	uint8_t index,
	uint8_t id,
	uint8_t len,
	uint8_t const *data,
	uint8_t crc,
	uint8_t flags);
SLLIN_RAMFUNC extern void sllin_lin_task_notify_def(uint8_t index, uint32_t count);
SLLIN_RAMFUNC extern void sllin_lin_task_notify_isr(uint8_t index, uint32_t count);
SLLIN_RAMFUNC extern void sllin_lin_task_queue(uint8_t index, sllin_queue_element const *element);


#if defined(SAME54XPLAINEDPRO)
#	include "sllin_same54_xplained_pro.h"
#else
#	error "Unknown board"
#endif
