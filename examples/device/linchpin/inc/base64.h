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

#include <string.h>
#include <stdint.h>
#include <stddef.h>

#if defined(BASE64_STATIC)
	#define BASE64_EXTERN static inline
#else
	#define BASE64_EXTERN extern
#endif

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

#define BASE64_TO_ASCII_TABLE_INITIALIZER \
{ \
	'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', \
	'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', \
	'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', \
	'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', \
	'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', \
	'o', 'p', 'q', 'r', 's', 't', 'u', 'v', \
	'w', 'x', 'y', 'z', '0', '1', '2', '3', \
	'4', '5', '6', '7', '8', '9', '+', '/' \
}


#define BASE64_FLAG_OVERFLOW     0x1
#define BASE64_FLAG_INVALID_CHAR 0x2
#define BASE64_FLAG_DONE         0x4

#ifdef __cplusplus
extern "C" {
#endif

#if defined(BASE64_H)

struct base64_state {
	uint8_t state;
	uint8_t bits;
	uint8_t rem:2;
	uint8_t flags:4;
};

#if defined(BASE64_C) && defined(BASE64_STATIC)
	static const char base64_to_ascii_table[64] = BASE64_TO_ASCII_TABLE_INITIALIZER;
#else
	extern const char base64_to_ascii_table[64];
#endif

#define base64_to_ascii(x) (base64_to_ascii_table[(x) & 0x3f])

BASE64_FUNC static inline bool base64_is_base64_char(int c)
{
	return (c == '+' || c == '/' || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'));
}

BASE64_FUNC static inline void base64_init(struct base64_state* state)
{
	BASE64_ASSERT(state);
	memset(state, 0, sizeof(*state));
}

BASE64_FUNC static inline void base64_flush(
	struct base64_state* state,
	size_t volatile* gi_ptr,
	size_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	size_t buf_size,
	uint8_t expected_bits)
{
	BASE64_ASSERT(state);
	BASE64_ASSERT(gi_ptr);
	BASE64_ASSERT(pi_ptr);
	BASE64_ASSERT(buf_ptr);
	BASE64_ASSERT(buf_size > 0);
	BASE64_ASSERT(state->bits == expected_bits);

	size_t gi = __atomic_load_n(gi_ptr, __ATOMIC_ACQUIRE);
	size_t pi = *pi_ptr;
	size_t used = pi - gi;
	if (base64_likely(used < buf_size)) {
		size_t index = pi % buf_size;
		buf_ptr[index] = state->state;
		__atomic_store_n(pi_ptr, pi + 1, __ATOMIC_RELEASE);
	} else {
		state->flags |= BASE64_FLAG_OVERFLOW;
	}
}

BASE64_FUNC static inline void base64_encode_flush(
	struct base64_state* state,
	size_t volatile* gi_ptr,
	size_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	size_t buf_size)
{
	base64_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size, 6);
}


BASE64_FUNC static inline void base64_encode_shift(
	uint8_t bits,
	struct base64_state* state,
	size_t volatile* gi_ptr,
	size_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	size_t buf_size)
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
	size_t volatile* gi_ptr,
	size_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	size_t buf_size)
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
	size_t volatile* gi_ptr,
	size_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	size_t buf_size)
{
	base64_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size, 8);
}


BASE64_FUNC BASE64_EXTERN void base64_decode_shift(
	uint8_t c,
	struct base64_state* state,
	size_t volatile* gi_ptr,
	size_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	size_t buf_size);

#endif // defined(BASE64_H)

#ifdef BASE64_C

#ifndef BASE64_STATIC
const char base64_to_ascii_table[64] = BASE64_TO_ASCII_TABLE_INITIALIZER;
#endif


BASE64_EXTERN void base64_decode_shift(
	uint8_t c,
	struct base64_state* state,
	size_t volatile* gi_ptr,
	size_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	size_t buf_size)
{
	uint8_t bits = 0;

	BASE64_ASSERT(state);
	BASE64_ASSERT(gi_ptr);
	BASE64_ASSERT(pi_ptr);
	BASE64_ASSERT(buf_ptr);
	BASE64_ASSERT(buf_size > 0);

	if (c >= 'A' && c <= 'Z') {
		bits = c - 'A';
	} else if (c >= 'a' && c <= 'z') {
		bits = (c - 'a') + 26;
	} else if (c >= '0' && c <= '9') {
		bits = (c - '0') + 52;
	} else if (c == '+') {
		bits = 62;
	} else if (c == '/') {
		bits = 63;
	} else if (c == '=') {
		state->bits = 0;
		state->flags |= BASE64_FLAG_DONE;
		return;
	} else {
		state->flags |= BASE64_FLAG_INVALID_CHAR;
		return;
	}

	switch (state->bits) {
	case 0:
		state->bits = 6;
		state->state = bits << 2;
		break;
	case 2:
		state->bits = 8;
		state->state |= bits;
		base64_decode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 0;
		break;
	case 4:
		state->bits = 8;
		state->state |= (bits >> 2) & 0xf;
		base64_decode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 2;
		state->state = (bits & 0x3) << 6;
		break;
	case 6:
		state->bits = 8;
		state->state |= (bits >> 4) & 0x3;
		base64_decode_flush(state, gi_ptr, pi_ptr, buf_ptr, buf_size);
		state->bits = 4;
		state->state = (bits & 0xf) << 4;
		break;
	default:
		BASE64_ASSERT(0 && "base64_decode_shift unhandled no bits");
		break;
	}
}

#endif // #ifdef BASE64_C

#ifdef __cplusplus
}
#endif



#undef BASE64_TO_ASCII_TABLE_INITIALIZER
