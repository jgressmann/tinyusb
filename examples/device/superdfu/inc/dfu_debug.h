/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2020-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#include <bsp/board.h>
#include <usnprintf.h>

#ifndef SUPERDFU_DEBUG
	#define SUPERDFU_DEBUG 0
#endif

#if SUPERDFU_DEBUG
	#define SUPERDFU_DEBUG_LOG_BUFFER_SIZE 128
extern char dfu_log_buffer[SUPERDFU_DEBUG_LOG_BUFFER_SIZE];

	#define LOG(...) \
		do { \
			int chars = usnprintf(dfu_log_buffer, sizeof(dfu_log_buffer), __VA_ARGS__); \
			board_uart_write(dfu_log_buffer, chars); \
		} while (0)
#else
	#define LOG(...)
#endif

extern void dfu_dump_mem(void const * _ptr, size_t count);
