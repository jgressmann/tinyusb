/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once


#include <sam.h>

#define SLLIN_BOARD_LIN_COUNT 2
#define SLLIN_BOARD_NAME "D5035-50"

#define sam_timer_sync_wait(tc) while ((tc)->COUNT16.SYNCBUSY.bit.CTRLB)
#define sam_usart_clear_pending() \
	do { \
		NVIC_ClearPendingIRQ(SERCOM1_0_IRQn); \
		NVIC_ClearPendingIRQ(SERCOM1_2_IRQn); \
		NVIC_ClearPendingIRQ(SERCOM1_3_IRQn); \
	} while (0)
#define SAM_UART_RX_PORT_PIN_MUX 8u

#include "sllin_D5035_5x.h"


