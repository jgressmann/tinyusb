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

#include <bsp/board.h>
#include <usnprintf.h>


#if LINCHPIN_DEBUG
#	if CFG_TUSB_DEBUG > 0
#		define LOG TU_LOG1
#	else
#		define LINCHPIN_DEBUG_LOG_BUFFER_SIZE 128
extern char lp_log_buffer[LINCHPIN_DEBUG_LOG_BUFFER_SIZE];

#		define LOG(...) \
	do { \
		int chars = usnprintf(lp_log_buffer, sizeof(lp_log_buffer), __VA_ARGS__); \
		board_uart_write(lp_log_buffer, chars); \
	} while (0)
#	endif

#	define LP_DEBUG_ASSERT LP_ASSERT

#else
#	define LOG(...)
#	define LP_DEBUG_ASSERT(...)
#endif

#define LP_DEBUG_JOIN2(x, y) x##y
#define LP_DEBUG_JOIN(x, y) LP_DEBUG_JOIN2(x, y)

#define LP_DEBUG_STR2(x) #x
#define LP_DEBUG_STR(x) LP_DEBUG_STR2(x)


#define LP_ASSERT(x) \
	do { \
		if (__builtin_expect(!(x), 0)) { \
			lp_assert_failed("ASSERT FAILED: " #x " " __FILE__ ":" LP_DEBUG_STR(__LINE__) "\n"); \
		} \
	} while (0)

#define LP_ISR_ASSERT(x) \
	do { \
		if (__builtin_expect(!(x), 0)) { \
			lp_assert_failed("ISR ASSERT FAILED: " #x " " __FILE__ ":" LP_DEBUG_STR(__LINE__) "\n"); \
		} \
	} while (0)


__attribute__((noreturn)) extern void lp_assert_failed(char const * const msg);
extern void lp_dump_mem(void const * _ptr, size_t count);
