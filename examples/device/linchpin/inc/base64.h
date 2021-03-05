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

#include <string.h>
#include <stdint.h>


#ifndef BASE64_ASSERT
	#include <assert.h>
	#define BASE64_ASSERT assert
#endif

#ifndef BASE64_FUNC
	#define BASE64_FUNC
#endif

#ifndef base64_likely
	#define base64_likely(x) x
#endif

#ifndef base64_unlikely
	#define base64_unlikely(x) x
#endif


#define BASE64_FLAG_OVERFLOW     0x1
#define BASE64_FLAG_INVALID_CHAR 0x2

#ifdef __cplusplus
extern "C" {
#endif




struct base64_state {
	uint8_t state;
	uint8_t bits:4;
	uint8_t rem:2;
	uint8_t flags:2;
};

extern const char base64_to_ascii_table[64];

#define base64_to_ascii(x) (base64_to_ascii_table[(x) & 0x3f])

BASE64_FUNC static inline void base64_init(struct base64_state* state)
{
	BASE64_ASSERT(state);
	memset(state, 0, sizeof(*state));
}

BASE64_FUNC static inline void base64_flush_bits(
	struct base64_state* state,
	uint8_t volatile* gi_ptr,
	uint8_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	uint8_t buf_size,
	uint8_t expected_bits)
{
	BASE64_ASSERT(state);
	BASE64_ASSERT(gi_ptr);
	BASE64_ASSERT(pi_ptr);
	BASE64_ASSERT(buf_ptr);
	BASE64_ASSERT(buf_size > 0);
	BASE64_ASSERT(state->bits == expected_bits);

	uint8_t gi = __atomic_load_n(gi_ptr, __ATOMIC_ACQUIRE);
	uint8_t pi = *pi_ptr;
	uint8_t used = pi - gi;
	if (base64_likely(used < buf_size)) {
		uint8_t index = pi % buf_size;
		buf_ptr[index] = state->state;
		__atomic_store_n(pi_ptr, pi + 1, __ATOMIC_RELEASE);
	} else {
		state->flags |= BASE64_FLAG_OVERFLOW;
	}
}

BASE64_FUNC static inline void base64_encode_flush(
	struct base64_state* state,
	uint8_t volatile* gi_ptr,
	uint8_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	uint8_t buf_size)
{
	base64_flush_bits(state, gi_ptr, pi_ptr, buf_ptr, buf_size, 6);
}


BASE64_FUNC static inline void base64_encode_shift(
	uint8_t bits,
	struct base64_state* state,
	uint8_t volatile* gi_ptr,
	uint8_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	uint8_t buf_size)
{
	BASE64_ASSERT(state);
	BASE64_ASSERT(gi_ptr);
	BASE64_ASSERT(pi_ptr);
	BASE64_ASSERT(buf_ptr);
	BASE64_ASSERT(buf_size > 0);
	BASE64_ASSERT(!(state->bits & 0xc0));

	state->rem = (state->rem + 1) % 3;

	switch (state->bits) {
	case 0:
		state->bits = 6;
		state->state = base64_to_ascii((bits >> 2) & 0x3f);
		base64_encode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 2;
		state->state = bits & 0x03;
		state->state <<= 4;
		break;
	case 2:
		state->bits = 6;
		state->state |= (bits >> 4) & 0xf;
		state->state = base64_to_ascii(state->state);
		base64_encode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 4;
		state->state = (bits & 0xf);
		state->state <<= 2;
		break;
	case 4:
		state->bits = 6;
		state->state |= (bits >> 6) & 0x03;
		state->state = base64_to_ascii(state->state);
		base64_encode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 6;
		state->state = base64_to_ascii(bits & 0x3f);
		base64_encode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 0;
		break;
	default:
		BASE64_ASSERT(0 && "base64_encode_shift unhandled no bits");
		break;
	}
}

BASE64_FUNC static inline void base64_encode_finalize(
	struct base64_state* state,
	uint8_t volatile* gi_ptr,
	uint8_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	uint8_t buf_size)
{
	BASE64_ASSERT(state);
	BASE64_ASSERT(gi_ptr);
	BASE64_ASSERT(pi_ptr);
	BASE64_ASSERT(buf_ptr);
	BASE64_ASSERT(buf_size > 0);

	switch (state->rem) {
	case 0:
		break;
	case 1:
		state->bits = 6;
		state->state = base64_to_ascii(state->state);
		base64_encode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 6;
		state->state = '=';
		base64_encode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 6;
		state->state = '=';
		base64_encode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 0;
		break;
	case 2:
		state->bits = 6;
		state->state = base64_to_ascii(state->state);
		base64_encode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 6;
		state->state = '=';
		base64_encode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 0;
		break;
	default:
		BASE64_ASSERT(0 && "base64_encode_finalize");
		break;
	}

	state->rem = 0;
}

BASE64_FUNC static inline void base64_decode_flush(
	struct base64_state* state,
	uint8_t volatile* gi_ptr,
	uint8_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	uint8_t buf_size)
{
	base64_flush_bits(state, gi_ptr, pi_ptr, buf_ptr, buf_size, 8);
}


BASE64_FUNC void base64_decode_shift(
	uint8_t c,
	struct base64_state* state,
	uint8_t volatile* gi_ptr,
	uint8_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	uint8_t buf_size);

#ifdef __cplusplus
}
#endif

