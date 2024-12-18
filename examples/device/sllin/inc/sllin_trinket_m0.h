/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once



#define SLLIN_BOARD_USB_BCD_DEVICE (1 << 8)
#define SLLIN_BOARD_USB_MANUFACTURER_STRING "Adafruit"
#define SLLIN_BOARD_LIN_COUNT 1
#define SLLIN_BOARD_NAME "Trinket M0"

enum {
	SLLIN_BOARD_DEBUG_DEFAULT,
	SLLIN_BOARD_DOTSTAR_RED,
	SLLIN_BOARD_DOTSTAR_GREEN,
	SLLIN_BOARD_DOTSTAR_BLUE,
	SLLIN_BOARD_LED_COUNT
};


/*
 * According to the manual, the SAMD21 USART supports auto-baud and break detection.
 * However, it fails to detect any PID which starts with a zero bit like 0xCA (0x0A).
 */
#define SAM_CONF_LIN_UART_FREQUENCY 48000000
#define sam_timer_sync_wait(tc) while ((tc)->COUNT16.STATUS.bit.SYNCBUSY)
#define sam_usart_clear_pending(index) NVIC_ClearPendingIRQ(SERCOM2_IRQn)
#define SAM_UART_RX_PORT_PIN_MUX 7u

#include "sllin_sam.h"
