/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020-2021 Jean Gressmann <jean@0x42.de>
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
#include <supercan_board.h>
#include <leds.h>
#include <FreeRTOS.h>

#ifndef ARRAY_SIZE
#	define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif

#if SUPERCAN_DEBUG && !CFG_TUSB_DEBUG
char sc_log_buffer[SUPERCAN_DEBUG_LOG_BUFFER_SIZE];
#endif

__attribute__((noreturn)) extern void sc_assert_failed(char const * const msg, size_t len)
{
	taskDISABLE_INTERRUPTS();
	board_uart_write(msg, len);
	sc_board_leds_on_unsafe();
	while (1);
}

extern void sc_dump_mem(void const * _ptr, size_t count)
{
	char buf[8];
	int chars = 0;
	uint8_t const *ptr = (uint8_t const *)_ptr;

	for (size_t i = 0; i < count; i += 16) {
		// usnprintf doesn't support width or fill
		chars = usnprintf(buf, sizeof(buf), "%X", (unsigned)i);

		for (int k = chars; k < 3; ++k) {
			board_uart_write("0", 1);
		}

		board_uart_write(buf, chars);
		board_uart_write(" ", 1);
		board_uart_write(" ", 1);

		size_t end = i + 16;
		if (end > count) {
			end = count;
		}

		for (size_t j = i; j < end; ++j) {
			chars = usnprintf(buf, sizeof(buf), "%X", ptr[j]);

			for (int k = chars; k < 2; ++k) {
				board_uart_write("0", 1);
			}

			board_uart_write(buf, chars);
			board_uart_write(" ", 1);
		}

		board_uart_write("\n", 1);
	}
}

