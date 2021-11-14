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

#pragma once

#include <stddef.h>
#include <usnprintf.h>


#if SLLIN_DEBUG
#	define SLLIN_DEBUG_LOG_BUFFER_SIZE 128

extern char sllin_log_buffer[SLLIN_DEBUG_LOG_BUFFER_SIZE];

#	define LOG(...) \
	do { \
		int sllin_debug_chars = usnprintf(sllin_log_buffer, sizeof(sllin_log_buffer), __VA_ARGS__); \
		board_uart_write(sllin_log_buffer, sllin_debug_chars); \
	} while (0)

#	define SLLIN_DEBUG_ASSERT SLLIN_ASSERT
#	define SLLIN_DEBUG_ISR_ASSERT SLLIN_ISR_ASSERT
#else
#	define LOG(...)
#	define SLLIN_DEBUG_ASSERT(...)
#	define SLLIN_DEBUG_ISR_ASSERT(...)
#endif

#define SLLIN_DEBUG_JOIN2(x, y) x##y
#define SLLIN_DEBUG_JOIN(x, y) SLLIN_DEBUG_JOIN2(x, y)

#define SLLIN_DEBUG_STR2(x) #x
#define SLLIN_DEBUG_STR(x) SLLIN_DEBUG_STR2(x)

#define SLLIN_ASSERT(x) \
	do { \
		if (__builtin_expect(!(x), 0)) { \
			sllin_assert_failed("ASSERT FAILED: " #x " " __FILE__ ":" SLLIN_DEBUG_STR(__LINE__) "\n"); \
		} \
	} while (0)

#define SLLIN_ISR_ASSERT(x) \
	do { \
		if (__builtin_expect(!(x), 0)) { \
			sllin_assert_failed("ISR ASSERT FAILED: " #x " " __FILE__ ":" SLLIN_DEBUG_STR(__LINE__) "\n"); \
		} \
	} while (0)


__attribute__((noreturn)) extern void sllin_assert_failed(char const *msg);
extern void sllin_dump_mem(void const * _ptr, size_t count);
