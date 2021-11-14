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

#ifndef likely
#define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x),0)
#endif


#define SLLIN_NAME "slLIN"
#define CMD_BUFFER_SIZE 64
#define MSG_BUFFER_SIZE 512


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
};


static inline uint8_t sllin_id_to_pid(uint8_t id)
{
	static const uint8_t map[] = {
		0x80,
		0xC1,
		0x42,
		0x03,
		0xC4,
		0x85,
		0x06,
		0x47,
		0x08,
		0x49,
		0xCA,
		0x8B,
		0x4C,
		0x0D,
		0x8E,
		0xCF,
		0x50,
		0x11,
		0x92,
		0xD3,
		0x14,
		0x55,
		0xD6,
		0x97,
		0xD8,
		0x99,
		0x1A,
		0x5B,
		0x9C,
		0xDD,
		0x5E,
		0x1F,
		0x20,
		0x61,
		0xE2,
		0xA3,
		0x64,
		0x25,
		0xA6,
		0xE7,
		0xA8,
		0xE9,
		0x6A,
		0x2B,
		0xEC,
		0xAD,
		0x2E,
		0x6F,
		0xF0,
		0xB1,
		0x32,
		0x73,
		0xB4,
		0xF5,
		0x76,
		0x37,
		0x78,
		0x39,
		0xBA,
		0xFB,
		0x3C,
		0x7D,
		0xFE,
		0xBF,
	};

	return map[id & 0x3f];
}

static inline uint8_t sllin_pid_to_id(uint8_t pid)
{
	return pid & 0x3f;
}

#define sllin_crc_start() 0
#define sllin_crc_update1(crc, byte) (crc + byte)


static inline unsigned sllin_crc_update(unsigned crc, uint8_t const * data, uint8_t bytes)
{
	for (unsigned i = 0; i < bytes; ++i) {
		crc += data[i];
	}

	return crc;
}

static inline uint8_t sllin_crc_finalize(unsigned crc)
{
	unsigned factor = crc / 256;
	crc -= factor * 255;

	return (~crc) & 0xff;
}



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
SLLIN_RAMFUNC extern void sllin_board_lin_slave_tx(uint8_t index, uint8_t id, uint8_t len, uint8_t const *data, uint8_t crc);
SLLIN_RAMFUNC extern void sllin_lin_task_notify_def(uint8_t index, uint32_t count);
SLLIN_RAMFUNC extern void sllin_lin_task_notify_isr(uint8_t index, uint32_t count);
SLLIN_RAMFUNC extern void sllin_lin_task_queue(uint8_t index, sllin_queue_element const *element);


#if defined(SAME54XPLAINEDPRO)
#	include "sllin_same54_xplained_pro.h"
#elif defined(__SAMD21E18A__)
#	include "sllin_samd.h"
#else
#	error "Unknown board"
#endif
