/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2023 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <sections.h>
#include <supercan_debug.h>

#define SC_PACKED  __attribute__((packed))
#include <supercan.h>

#include <supercan_version.h>

#include <FreeRTOSConfig.h>

#define SC_TASK_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1)
#define SC_ISR_PRIORITY configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
// #define SC_TASK_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+2)
// #define SC_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1)

#ifndef likely
#define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x),0)
#endif



#define CMD_BUFFER_SIZE 64


enum {
	SC_LED_BURST_DURATION_MS = 8,
	SC_BUS_ACTIVITY_TIMEOUT_MS = 256,

	SC_CAN_LED_STATUS_DISABLED = 0,
	SC_CAN_LED_STATUS_ENABLED_OFF_BUS,
	SC_CAN_LED_STATUS_ENABLED_ON_BUS_PASSIVE,
	SC_CAN_LED_STATUS_ENABLED_ON_BUS_ACTIVE,
	SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_PASSIVE,
	SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_ACTIVE,
	SC_CAN_LED_STATUS_ENABLED_ON_BUS_BUS_OFF,

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
#define sc_board_name() SC_BOARD_NAME
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

/* retrieve rx / tx / error messages into buffer
 *
 * Callee places messages into the
 *
 * return  -1 if no messages
 *         > 0 if messages where placed in buffer
 *         0 if insufficient space in buffer
 */
SC_RAMFUNC extern int sc_board_can_retrieve(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end);

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

#ifndef D5035_01
#	define D5035_01 0
#endif

#ifndef SAME54XPLAINEDPRO
#	define SAME54XPLAINEDPRO 0
#endif

#ifndef TEENSY_4X
#	define TEENSY_4X 0
#endif

#ifndef FEATHER_M4_CAN_EXPRESS
#	define FEATHER_M4_CAN_EXPRESS 0
#endif

#ifndef LONGAN_CANBED_M4
#	define LONGAN_CANBED_M4 0
#endif

#ifndef STM32F3DISCOVERY
#	define STM32F3DISCOVERY 0
#endif

#ifndef D5035_04
#	define D5035_04 0
#endif

#ifndef D5035_05
#	define D5035_05 0
#endif



#	include "supercan_D5035_01.h"
#elif SAME54XPLAINEDPRO
#	include "supercan_same54_xplained_pro.h"
#elif TEENSY_4X
#	include "supercan_teensy_4x.h"
#elif FEATHER_M4_CAN_EXPRESS
#	include "supercan_feather_m4_can_express.h"
#elif LONGAN_CANBED_M4
#	include "supercan_longan_canbed_m4.h"
#elif STM32F3DISCOVERY
#	include "supercan_stm32f3discovery.h"
#elif D5035_05
#	include "supercan_D5035_05.h"
#elif STM32H7A3NUCLEO
#	include "supercan_stm32h7a3nucleo.h"
#elif defined(D5035_04)
#	include "supercan_D5035_04.h"
#else
#	pragma GCC warning "unknown board, using dummy CAN implementation"
#	define SUPERCAN_DUMMY 1
#	include "supercan_dummy.h"
#endif




SC_RAMFUNC static inline uint32_t sc_bitrate(unsigned brp, unsigned tseg1, unsigned tseg2)
{
	SC_DEBUG_ASSERT(brp >= 1);

	return SC_BOARD_CAN_CLK_HZ / ((uint32_t)brp * (1 + tseg1 + tseg2));
}

#ifndef MSG_BUFFER_SIZE
#	define MSG_BUFFER_SIZE 512
#endif