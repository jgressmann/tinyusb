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

#if defined(D5035_01)
#	include "supercan_D5035_01.h"
#elif defined(SAME54XPLAINEDPRO)
#	include "supercan_same54_xplained_pro.h"
#else
#	error "Unsupported board!"
#endif

#ifndef D5035_01
#	define D5035_01 0
#endif

#ifndef SAME54XPLAINEDPRO
#	define SAME54XPLAINEDPRO 0
#endif

enum {
	CANLED_STATUS_DISABLED,
	CANLED_STATUS_ENABLED_BUS_OFF,
	CANLED_STATUS_ENABLED_BUS_ON_PASSIVE,
	CANLED_STATUS_ENABLED_BUS_ON_ACTIVE,
	CANLED_STATUS_ERROR_ACTIVE,
	CANLED_STATUS_ERROR_PASSIVE,
};


extern void sc_board_led_init(void);
extern void sc_board_led_set(uint8_t index, bool on);
extern void sc_board_leds_on_unsafe(void);

extern void sc_board_can_init_module(void);
extern void sc_board_can_init_pins(void);
extern void sc_board_can_init_clock(void);
extern void sc_board_can_interrupt_enable(uint8_t index, bool on);
extern void* sc_board_can_m_can(uint8_t index);
extern void sc_board_can_burst_led(uint8_t index, uint16_t duration_ms);
extern void sc_board_can_led_set_status(uint8_t index, int status);
extern void sc_board_power_led_on(void);
extern void sc_board_usb_led_burst(uint16_t duration_ms);
extern void sc_board_counter_1MHz_init(void);

#include <supercan_debug.h>

#ifndef likely
#define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x),0)
#endif


__attribute__((section(".ramfunc"))) static inline void sc_board_counter_1MHz_request_current_value(void)
{
	TC0->COUNT32.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_READSYNC_Val;
}

__attribute__((section(".ramfunc"))) static inline void sc_board_counter_1MHz_request_current_value_lazy(void)
{
	uint8_t reg;

	reg = __atomic_load_n(&TC0->COUNT32.CTRLBSET.reg, __ATOMIC_ACQUIRE);

	while (1) {
		uint8_t cmd = reg & TC_CTRLBSET_CMD_Msk;
		SC_DEBUG_ASSERT(cmd == TC_CTRLBSET_CMD_READSYNC || cmd == TC_CTRLBSET_CMD_NONE);
		if (cmd == TC_CTRLBSET_CMD_READSYNC) {
			break;
		}

		if (likely(__atomic_compare_exchange_n(
			&TC0->COUNT32.CTRLBSET.reg,
			&reg,
			TC_CTRLBSET_CMD_READSYNC,
			false, /* weak? */
			__ATOMIC_RELEASE,
			__ATOMIC_ACQUIRE))) {
				break;
			}
	}
}

__attribute__((section(".ramfunc"))) static inline bool sc_board_counter_1MHz_is_current_value_ready(void)
{
	return (__atomic_load_n(&TC0->COUNT32.CTRLBSET.reg, __ATOMIC_ACQUIRE) & TC_CTRLBSET_CMD_Msk) == TC_CTRLBSET_CMD_NONE;
	//return TC0->COUNT32.CTRLBSET.bit.CMD == TC_CTRLBSET_CMD_NONE_Val;
}

__attribute__((section(".ramfunc"))) static inline uint32_t sc_board_counter_1MHz_read_unsafe(void)
{
	return TC0->COUNT32.COUNT.reg;
}