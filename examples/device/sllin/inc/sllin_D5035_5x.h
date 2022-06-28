/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#if SUPERDFU_APP
	#include <dfu_ram.h>
	#include <dfu_app.h>
	#include <dfu_usb_descriptors.h>
	#include <mcu.h>
	#define DFU_USB_RESET_TIMEOUT_MS 100

extern void dfu_init_begin(void);
#define dfu_init_end(void) dfu_app_watchdog_disable()
extern void dfu_timer_start(uint16_t ms);
extern void dfu_timer_expired(void);

#endif



#define SLLIN_BOARD_USB_BCD_DEVICE (1 << 8)
#define SLLIN_BOARD_USB_MANUFACTURER_STRING "2guys"


enum {
	SLLIN_BOARD_DEBUG_DEFAULT,
	LED_DEBUG_0,
	LED_DEBUG_1,
	LED_DEBUG_2,
	LED_DEBUG_3,
	LED_LIN0_STATUS_GREEN,
	LED_LIN0_STATUS_RED,
	LED_LIN1_STATUS_GREEN,
	LED_LIN1_STATUS_RED,
	SLLIN_BOARD_LED_COUNT
};

#define SAM_CONF_LIN_UART_FREQUENCY 48000000

#include "sllin_sam.h"

