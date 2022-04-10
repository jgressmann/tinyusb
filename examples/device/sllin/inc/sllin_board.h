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
	SLLIN_QUEUE_ELEMENT_TYPE_RX_FRAME,
	SLLIN_QUEUE_ELEMENT_TYPE_SLEEP,
	SLLIN_QUEUE_ELEMENT_TYPE_WAKE_UP,
	SLLIN_QUEUE_ELEMENT_TYPE_COUNT,
};

typedef struct _sllin_queue_element {
	uint8_t type;
	union {
		struct {
			uint16_t time_stamp_ms;
			uint8_t id;
			uint8_t crc;
			uint8_t flags;
			uint8_t len;
			uint8_t data[8];
		} lin_frame;
		uint16_t time_stamp_ms;
	};
} sllin_queue_element;

enum {
	SLLIN_FRAME_FLAG_ENHANCED_CHECKSUM = 0x01,
	SLLIN_FRAME_FLAG_PID_ERROR = 0x02,
	SLLIN_FRAME_FLAG_CRC_ERROR = 0x04,
	SLLIN_FRAME_FLAG_NO_RESPONSE = 0x08,
	SLLIN_FRAME_FLAG_FOREIGN = 0x10,
	SLLIN_FRAME_FLAG_MASTER_TX = 0x20,
};

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
