/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2020-2023 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#include <stddef.h>
#include <usnprintf.h>

extern int board_uart_write(void const * buf, int len);

#if SUPERCAN_DEBUG
#	define SUPERCAN_DEBUG_LOG_BUFFER_SIZE 128
extern char sc_log_buffer[SUPERCAN_DEBUG_LOG_BUFFER_SIZE];

#	define LOG(...) \
	do { \
		int sc_debug_chars = usnprintf(sc_log_buffer, sizeof(sc_log_buffer), __VA_ARGS__); \
		board_uart_write(sc_log_buffer, sc_debug_chars); \
	} while (0)

#	define SC_DEBUG_ASSERT SC_ASSERT

#else
#	define LOG(...)
#	define SC_DEBUG_ASSERT(...)
#endif

#define SC_DEBUG_JOIN2(x, y) x##y
#define SC_DEBUG_JOIN(x, y) SC_DEBUG_JOIN2(x, y)

#define SC_DEBUG_STR2(x) #x
#define SC_DEBUG_STR(x) SC_DEBUG_STR2(x)


#define SC_ASSERT_FAILED(...) sc_assert_failed(__VA_ARGS__, sizeof(__VA_ARGS__) - 1)

#define SC_ASSERT(x) \
	do { \
		if (__builtin_expect(!(x), 0)) { \
			SC_ASSERT_FAILED("ASSERT FAILED: " #x " " __FILE__ ":" SC_DEBUG_STR(__LINE__) "\n"); \
		} \
	} while (0)

#define SC_ISR_ASSERT(x) \
	do { \
		if (__builtin_expect(!(x), 0)) { \
			SC_ASSERT_FAILED("ISR ASSERT FAILED: " #x " " __FILE__ ":" SC_DEBUG_STR(__LINE__) "\n"); \
		} \
	} while (0)


__attribute__((noreturn)) extern void sc_assert_failed(char const * const msg, size_t len);
extern void sc_dump_mem(void const * _ptr, size_t count);
