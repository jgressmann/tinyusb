/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020-2023 Jean Gressmann <jean@0x42.de>
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



#ifndef ARRAY_SIZE
#	define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif

#if SUPERCAN_DEBUG
char sc_log_buffer[SUPERCAN_DEBUG_LOG_BUFFER_SIZE];

extern int sc_log(char const* fmt, ...)
{
	int chars;
	va_list vl;

	va_start(vl, fmt);
	chars = uvsnprintf(sc_log_buffer, sizeof(sc_log_buffer), fmt, vl);
	va_end(vl);

	board_uart_write(sc_log_buffer, chars);

	return chars;
}

#endif

__attribute__((noreturn)) extern void sc_assert_failed(char const * const msg, size_t len)
{
	volatile uint32_t* ARM_CM_DHCSR =  ((volatile uint32_t*) 0xE000EDF0UL); /* Cortex M CoreDebug->DHCSR */ \

	taskDISABLE_INTERRUPTS();
	board_uart_write(msg, len);
	sc_board_leds_on_unsafe();

	if ( (*ARM_CM_DHCSR) & 1UL ) {  /* Only halt mcu if debugger is attached */
		__asm("BKPT #0\n"); \
	}

	while (1);
}

extern void sc_dump_mem(void const * _ptr, size_t count)
{
	char buf[16];
	char prefix[8];
	int chars = 0;
	uint8_t const *ptr = (uint8_t const *)_ptr;
	size_t rows = count / 16;
	unsigned width = 1;

	while (rows) {
		rows /= 16;
		++width;
	}

	chars = usnprintf(prefix, sizeof(prefix), "%%0%uX  ", width);

	for (size_t i = 0; i < count; i += 16) {
		chars = usnprintf(buf, sizeof(buf), prefix, (unsigned)i);

		board_uart_write(buf, chars);

		size_t end = i + 16;
		if (end > count) {
			end = count;
		}

		for (size_t j = i; j < end; ++j) {
			chars = usnprintf(buf, sizeof(buf), "%02X ", ptr[j]);

			board_uart_write(buf, chars);
		}

		board_uart_write("\n", 1);
	}
}

