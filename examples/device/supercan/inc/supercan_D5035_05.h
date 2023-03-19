/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#ifndef D5035_05
#	error "Only include this file for D5035-05 boards"
#endif

#define HWREV 1

#ifndef HWREV
#	error "Define HWREV"
#endif

#define SC_BOARD_USB_BCD_DEVICE (HWREV << 8)
#define SC_BOARD_USB_MANUFACTURER_STRING "2guys"
#define SC_BOARD_CAN_COUNT 2
#define SC_BOARD_NAME "D5035-05"
#define SC_BOARD_CAN_CLK_HZ 64000000


enum {
	SC_BOARD_DEBUG_DEFAULT,
	LED_DEBUG_0,
	LED_DEBUG_1,
	LED_DEBUG_2,
	LED_DEBUG_3,
	LED_CAN0_STATUS_GREEN,
	LED_CAN0_STATUS_RED,
	LED_CAN1_STATUS_GREEN,
	LED_CAN1_STATUS_RED,
	SC_BOARD_LED_COUNT
};

#define CAN0_TRAFFIC_LED LED_DEBUG_1
#define CAN1_TRAFFIC_LED LED_DEBUG_2


#define sc_board_led_usb_burst() led_burst(LED_DEBUG_3, SC_LED_BURST_DURATION_MS)
#define sc_board_led_can_traffic_burst(index) \
	do { \
		switch (index) { \
		case 0: led_burst(CAN0_TRAFFIC_LED, SC_LED_BURST_DURATION_MS); break; \
		case 1: led_burst(CAN1_TRAFFIC_LED, SC_LED_BURST_DURATION_MS); break; \
		default: break; \
		} \
	} while (0)


SC_RAMFUNC extern void sc_board_led_can_status_set(uint8_t index, int status);

// #undef sc_board_name
// extern char const* sc_board_name(void);

#define sc_board_can_ts_request(index)
#define sc_board_can_ts_wait(index) 0

#include <stm32g0b1xx.h>

#define SUPERCAN_MCAN 1
#define MCAN_MESSAGE_RAM_CONFIGURABLE 0
#define MCAN_HW_RX_FIFO_SIZE 3
#define MCAN_HW_TX_FIFO_SIZE 3

#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef volatile       uint32_t RoReg;   /**< Read only 32-bit register (volatile const unsigned int) */
typedef volatile       uint16_t RoReg16; /**< Read only 16-bit register (volatile const unsigned int) */
typedef volatile       uint8_t  RoReg8;  /**< Read only  8-bit register (volatile const unsigned int) */
// typedef volatile       uint32_t WoReg;   /**< Write only 32-bit register (volatile unsigned int) */
// typedef volatile       uint16_t WoReg16; /**< Write only 16-bit register (volatile unsigned int) */
// typedef volatile       uint8_t  WoReg8;  /**< Write only  8-bit register (volatile unsigned int) */
// typedef volatile       uint32_t RwReg;   /**< Read-Write 32-bit register (volatile unsigned int) */
// typedef volatile       uint16_t RwReg16; /**< Read-Write 16-bit register (volatile unsigned int) */
// typedef volatile       uint8_t  RwReg8;  /**< Read-Write  8-bit register (volatile unsigned int) */
#endif

#include <supercan_mcan.h>

