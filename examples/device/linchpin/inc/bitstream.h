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


#include <stdint.h>
#include <string.h>

#ifndef BS_ASSERT
	#include <assert.h>
	#define BS_ASSERT assert
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Write a packed byte to the output stream
 *
 * Return 0 on success.
 */
typedef int (*bs_write_t)(void* ctx, uint8_t byte);
/* Read a byte for unpacking from input stream
 *
 * Return 0 on success.
 */
typedef int (*bs_read_t)(void* ctx, uint8_t* byte);

struct bitstream {
	uint8_t incoming_byte;
	uint8_t outgoing_byte;
	uint8_t incoming_count;
	uint8_t outgoing_count;
};

static inline void bs_init(struct bitstream *s)
{
	BS_ASSERT(s);
	memset(s, 0, sizeof(*s));
}

static inline int bs_write(
		struct bitstream* s,
		void* ctx,
		bs_write_t callback,
		unsigned bit)
{
	BS_ASSERT(s);
	BS_ASSERT(callback);

	s->incoming_byte <<= 1;
	s->incoming_byte |= bit & 1;
	++s->incoming_count;

	if (s->incoming_count == 8) {
		int e = callback(ctx, s->incoming_byte);

		if (e) {
			return e;
		}

		s->incoming_count = 0;
	}

	return 0;
}

static inline int bs_flush(
		struct bitstream* s,
		void* ctx,
		bs_write_t callback,
		uint8_t term)
{
	BS_ASSERT(s);
	BS_ASSERT(callback);

	if (s->incoming_count) {
		s->incoming_byte <<= 8 - s->incoming_count;
		s->incoming_byte |= term & ((1u << (8 - s->incoming_count))-1);

		int e = callback(ctx, s->incoming_byte);
		if (e) {
			return e;
		}

		s->incoming_count = 0;
	}

	return 0;
}

static inline int bs_read(
		struct bitstream* s,
		void* ctx,
		bs_read_t callback,
		unsigned *bit)
{
	unsigned mask = 0;

	BS_ASSERT(s);
	BS_ASSERT(callback);
	BS_ASSERT(bit);

	if (!s->outgoing_count) {
		int e = callback(ctx, &s->outgoing_byte);

		if (e) {
			return e;
		}

		s->outgoing_count = 8;
	}

	mask = 1u << --s->outgoing_count;
	*bit = (s->outgoing_byte & mask) == mask;

	return 0;
}

#ifdef __cplusplus
} // extern "C"
#endif
