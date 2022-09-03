/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#ifndef D5035_04
#	error "Only include this file for D5035-04 boards"
#endif

#ifndef HWREV
#	error "Define HWREV"
#endif

#include <gd32c10x.h>
#include <sections.h>
#include <FreeRTOS.h>


#define SC_BOARD_USB_BCD_DEVICE (HWREV << 8)
#define SC_BOARD_USB_MANUFACTURER_STRING "2guys"
#define SC_BOARD_CAN_COUNT 1
#define SC_BOARD_NAME "D5035-04"
#define SC_BOARD_CAN_CLK_HZ 60000000

#define SC_BOARD_CAN_TX_FIFO_SIZE 32
#define SC_BOARD_CAN_RX_FIFO_SIZE 32
/* This board is special, greater sizes lead to the USB ports begin disabled during
 * transfer. The Linux kernel prints this helpful (?) diagnostic message:
 * usb usb4-port4: disabled by hub (EMI?), re-enabling..
 */

// works *sometimes*
// #define MSG_BUFFER_SIZE 256
// seems to *mostly* work
// #define MSG_BUFFER_SIZE 128

#define MSG_BUFFER_SIZE 128

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
SC_RAMFUNC extern uint32_t sc_board_can_ts_fetch_isr(void);



#define sc_board_can_ts_request(index) do { } while (0)
static inline uint32_t sc_board_can_ts_wait(uint8_t index)
{
	uint32_t ts;

	(void)index;

	taskENTER_CRITICAL();
	ts = sc_board_can_ts_fetch_isr();
	taskEXIT_CRITICAL();

	return ts;
}

