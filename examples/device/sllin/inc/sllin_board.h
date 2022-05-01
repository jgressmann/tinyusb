/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <sections.h>
#include <sllin_debug.h>
#include <sllin_version.h>
#include <sllin.h>

#include <FreeRTOSConfig.h>

#define SLLIN_TASK_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY-1)
#define SLLIN_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1)

#ifndef likely
#define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x),0)
#endif


#define SLLIN_NAME "slLIN"

typedef struct sllin_conf {
	uint16_t bitrate;
	uint16_t sleep_timeout_ms;
	bool master;
} sllin_conf;

enum {
	SLLIN_QUEUE_ELEMENT_TYPE_FRAME,
	SLLIN_QUEUE_ELEMENT_TYPE_COUNT,
};

enum {
	SLLIN_CRC_TYPE_UNKNOWN = -1,
	SLLIN_CRC_TYPE_CLASSIC,
	SLLIN_CRC_TYPE_ENHANCED,
};

typedef struct _sllin_queue_element {
	uint8_t type;
	uint8_t reserved;
	uint16_t time_stamp_ms;
	union {
		struct {
			uint32_t id;
			uint8_t crc;
			uint8_t len;
			uint8_t data[8];
		} frame;
	};
} sllin_queue_element;

enum {
	SLLIN_LIN_LED_BLINK_DELAY_SLEEPING_MS = 768,
	SLLIN_LIN_LED_BLINK_DELAY_AWAKE_PASSIVE_MS = 512,
	SLLIN_LIN_LED_BLINK_DELAY_AWAKE_ACTIVE_MS = 128,

	SLLIN_LIN_LED_STATUS_DISABLED = 0,
	SLLIN_LIN_LED_STATUS_ENABLED_OFF_BUS,
	SLLIN_LIN_LED_STATUS_ON_BUS_SLEEPING,
	SLLIN_LIN_LED_STATUS_ON_BUS_AWAKE_PASSIVE,
	SLLIN_LIN_LED_STATUS_ON_BUS_AWAKE_ACTIVE,
	SLLIN_LIN_LED_STATUS_ERROR,
};


__attribute__((noreturn)) extern void sllin_board_reset(void);
extern uint32_t sllin_board_identifier(void);
extern void sllin_board_init_begin(void);
extern void sllin_board_init_end(void);
extern void sllin_board_led_set(uint8_t index, bool on);
extern void sllin_board_leds_on_unsafe(void);
extern void sllin_board_lin_uninit(uint8_t index);
extern void sllin_board_lin_init(uint8_t index, sllin_conf *conf);
extern void sllin_board_lin_sleep_timeout(uint8_t index, uint16_t timeout_ms);
SLLIN_RAMFUNC extern bool sllin_board_lin_master_break(uint8_t index);
SLLIN_RAMFUNC extern bool sllin_board_lin_master_request(uint8_t index, uint8_t id);
SLLIN_RAMFUNC extern void sllin_board_lin_slave_respond(uint8_t index, uint8_t id, bool respond);
SLLIN_RAMFUNC extern void sllin_board_led_lin_status_set(uint8_t index, int status);
SLLIN_RAMFUNC extern void sllin_lin_task_notify_def(uint8_t index, uint32_t count);
SLLIN_RAMFUNC extern void sllin_lin_task_notify_isr(uint8_t index, uint32_t count);
SLLIN_RAMFUNC extern void sllin_lin_task_queue(uint8_t index, sllin_queue_element const *element);


extern uint16_t _sllin_time_stamp_ms;
#define sllin_time_stamp_ms() __atomic_load_n(&_sllin_time_stamp_ms, __ATOMIC_ACQUIRE)


#ifndef SAME54XPLAINEDPRO
#	define SAME54XPLAINEDPRO 0
#endif

#ifndef D5035_50
#	define D5035_50 0
#endif

#if SAME54XPLAINEDPRO
#	include "sllin_same54_xplained_pro.h"
#elif D5035_50
#	include "sllin_D5035_50.h"
#else
#	error "Unknown board"
#endif


struct sllin_frame_data {
	__attribute__ ((aligned(4))) uint8_t data[64][8];
	uint8_t len[64];
	uint8_t crc[64];
};

extern struct sllin_frame_data sllin_frame_data[SLLIN_BOARD_LIN_COUNT];