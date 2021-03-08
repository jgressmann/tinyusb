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


#include <tusb.h>
#include <sam.h>

#define LP_FASTLZ_HASH_LOG 6
#define LP_SIGNAL_BUFFER_SIZE (1u<<12)
#define LP_USB_BUFFER_SIZE 64
#define lp_cdc_is_connected tud_cdc_connected
#define lp_cdc_rx_available tud_cdc_available
#define lp_cdc_tx_available tud_cdc_write_available
#define lp_cdc_rx tud_cdc_read
#define lp_cdc_tx tud_cdc_write
#define lp_cdc_tx_flush tud_cdc_write_flush
#define lp_cdc_rx_clear tud_cdc_read_flush
#define lp_rx_pin_read() ({ (PORT->Group[2].IN.reg & 0b100000) == 0b100000; })
#define lp_tx_pin_set() PORT->Group[2].OUTSET.reg = 0b10000
#define lp_tx_pin_clear() PORT->Group[2].OUTCLR.reg = 0b10000
#define lp_timer_stop() TC0->COUNT32.CTRLA.bit.ENABLE = 0
#define lp_timer_start() \
	do { \
		TC0->COUNT32.CC[0].reg = CONF_CPU_FREQUENCY / lp.signal_frequency; \
		TC0->COUNT32.CTRLA.bit.ENABLE = 1; \
	} while (0)


#include <linchpin_misc.h>
#include <linchpin_debug.h>
#include <linchpin_error.h>

#define BASE64_ASSERT LP_DEBUG_ASSERT
#define base64_likely likely
#define base64_unlikely unlikely
#define BASE64_FUNC LP_RAMFUNC
#define BASE64_H
#include <base64.h>
#undef BASE64_H

#include <linchpin_api.h>
#include <linchpin_state.h>


