/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jean Gressmann <jean@0x42.de>
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

#include <supercan_debug.h>
#include <leds.h>
#include <FreeRTOS.h>

#if SUPERCAN_DEBUG && !CFG_TUSB_DEBUG
char sc_log_buffer[SUPERCAN_DEBUG_LOG_BUFFER_SIZE];
#endif

__attribute__((noreturn)) extern void sc_assert_failed(char const * const msg)
{
	taskDISABLE_INTERRUPTS();
	board_uart_write(msg, -1);
	led_on();
	while (1);
}

__attribute__((noreturn)) extern void sc_isr_assert_failed(char const * const msg)
{
	board_uart_write(msg, -1);
	led_on();
	while (1);
}