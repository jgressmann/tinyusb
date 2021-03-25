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
#include <stdlib.h>

#if defined(RLE_STATIC)
	#define RLE_EXTERN static inline
#else
	#define RLE_EXTERN extern
#endif

#ifndef RLE_ASSERT
	#include <assert.h>
	#define RLE_ASSERT assert
#endif

#ifndef RLE_FUNC
	#define RLE_FUNC
#endif

#ifndef rle_likely
	#define rle_likely(x) x
#endif

#ifndef rle_unlikely
	#define rle_unlikely(x) x
#endif

#ifndef RLE_INT_TYPE
	#error Define RLE_INT_TYPE
#endif

#ifdef __cplusplus
extern "C" {
#endif


#if defined(RLE_H)

typedef int (*rle_gen_t)(RLE_INT_TYPE* bits, unsigned bit_count);
/* \brief Reads bits during decode
 *
 * \param ctx: user supplied pointer
 * \param ptr: pointer to bit storage
 * \param count: number of bits to read
 *
 * \return: the number of bits placed or negative on error
 */
typedef int (*rle_read_bits_t)(void* ctx, uint8_t* ptr, unsigned count);

#define RLE_MAX_BIT_COUNT ((sizeof(RLE_INT_TYPE) * 8)-1)
#define RLE_MAX_COUNT (((RLE_INT_TYPE)1) << RLE_MAX_BIT_COUNT)


#define RLE_FLAG_ENC_VALUE     0x1

#define RLE_FLAG_DEC_ERROR     0x01
#define RLE_FLAG_DEC_UNDERFLOW 0x02
#define RLE_FLAG_DEC_AVAILABLE 0x04
#define RLE_FLAG_DEC_BAD       0x08

#define RLE_DEC_STATE_START 0

#define RLEE_NONE 0

struct rle {
	RLE_INT_TYPE count;
	uint8_t value;
	uint8_t flags;
//	uint8_t bit_offset;
//	union {
//		struct {

//		} enc;
//		struct {
//			uint8_t flags;
//		//	RLE_INT_TYPE dec_state:2;

//		} dec;
//	};
//	RLE_INT_TYPE value:1;
//	RLE_INT_TYPE flags:1;
//	RLE_INT_TYPE flags:2;
////	RLE_INT_TYPE dec_state:2;
//	RLE_INT_TYPE bit_offset:3;
//	RLE_INT_TYPE dec_rem:7;
	rle_gen_t generator;
};

RLE_FUNC static inline void rle_init(struct rle *rle)
{
	RLE_ASSERT(rle);
	memset(rle, 0, sizeof(*rle));
}


RLE_FUNC static inline int rle_encode_flush(
	struct rle *rle)
{
	RLE_ASSERT(rle);
	RLE_ASSERT(rle->generator);

	if (!rle->count) {
		return RLEE_NONE;
	}

	unsigned lz;
	if (sizeof(RLE_INT_TYPE) <= sizeof(int)) {
		lz = __builtin_clz(rle->count);
	} else if (sizeof(RLE_INT_TYPE) <= sizeof(long)) {
		lz = __builtin_clzl(rle->count);
	} else {
		lz = __builtin_clzll(rle->count);
	}

	unsigned count_bits = sizeof(RLE_INT_TYPE) * 8 - 1 - lz;
	unsigned rem = rle->count - (((RLE_INT_TYPE)1) << count_bits);

	RLE_INT_TYPE buf[2] = {0, 0};
	unsigned left = sizeof(RLE_INT_TYPE) * 8;
	unsigned total_bit_count = (count_bits + 1) * 2;

	if (rle->value) {
		buf[0] = ((((RLE_INT_TYPE)1) << (count_bits + 1)) - 1) << 1;
	} else {
		buf[0] = 1;
	}

	left -= count_bits + 2;

	if (count_bits) {
		if (left < count_bits) {
			buf[0] <<= left;
			buf[0] |= (rem >> (count_bits - left)) & ((((RLE_INT_TYPE)1) << left)-1);
			buf[1] = rem & ((((RLE_INT_TYPE)1) << (count_bits - left))-1);
		} else {
			buf[0] <<= count_bits;
			buf[0] |= rem;
		}
	}

	rle->count = 0;
	rle->flags = 0;

	return rle->generator(buf, total_bit_count);
}

RLE_FUNC static inline int rle_encode_push_bit(struct rle *rle, int bit)
{
	RLE_ASSERT(rle);

	if (rle_likely(rle->flags & RLE_FLAG_ENC_VALUE)) {
		if (rle_likely(bit == rle->value)) {
			if (rle_unlikely(rle->count == RLE_MAX_COUNT)) {
				int error = rle_encode_flush(rle);
				if (rle_unlikely(error)) {
					return error;
				}
			} else {
				++rle->count;
				return RLEE_NONE;
			}
		} else {
			int error = rle_encode_flush(rle);
			if (rle_unlikely(error)) {
				return error;
			}
		}
	}

	rle->count = 1;
	rle->value = bit;
	rle->flags = RLE_FLAG_ENC_VALUE;

	return RLEE_NONE;
}

RLE_FUNC static void rle_load(
		struct rle *rle,
		void* ctx,
		rle_read_bits_t callback);

RLE_FUNC static inline void rle_decode_bit(
		struct rle *rle,
		void* ctx,
		rle_read_bits_t callback,
		int* bit)
{
	RLE_ASSERT(rle);
	RLE_ASSERT(callback);
	RLE_ASSERT(bit);

	if (rle_likely(rle->flags & RLE_FLAG_DEC_AVAILABLE)) {
available:
		if (rle_likely(rle->count > 1)) {
			*bit = rle->value;
			--rle->count;
		} else {
			RLE_ASSERT(rle->count == 1);
			*bit = rle->value;
			rle->flags &= ~RLE_FLAG_DEC_AVAILABLE;
		}
	} else {
		rle_load(rle, ctx, callback);
		if (rle_likely(rle->flags == RLE_FLAG_DEC_AVAILABLE)) {
			goto available;
		}
	}
}

#endif // #ifdef RLE_H

#ifdef RLE_C

RLE_FUNC static void rle_load(
		struct rle *rle,
		void* ctx,
		rle_read_bits_t callback)
{
	RLE_ASSERT(rle);
	RLE_ASSERT(callback);

	uint8_t byte;
	int r = callback(ctx, &byte, 1);
	if (rle_unlikely(r < 0)) {
		rle->flags |= RLE_FLAG_DEC_ERROR;
		return;
	}

	if (rle_unlikely(r < 1)) {
		rle->flags |= RLE_FLAG_DEC_UNDERFLOW;
		return;
	}

	unsigned value = byte & 1;
	unsigned next_bit;
	unsigned bit_count = -1;

	// find first inverted bit
	do {
		++bit_count;

		r = callback(ctx, &byte, 1);
		if (rle_unlikely(r < 0)) {
			rle->flags |= RLE_FLAG_DEC_ERROR;
			return;
		}

		if (rle_unlikely(r < 1)) {
			rle->flags |= RLE_FLAG_DEC_UNDERFLOW;
			return;
		}

		next_bit = byte & 1;

		if (rle_unlikely(bit_count == RLE_MAX_BIT_COUNT)) {
			rle->flags |= RLE_FLAG_DEC_BAD;
			return;
		}
	} while (next_bit == value);


	RLE_INT_TYPE count = ((RLE_INT_TYPE)1) << bit_count;
	RLE_INT_TYPE rem = 0;

	// read out n-bit counter
	if (bit_count) {
		uint8_t counter_buffer[sizeof(RLE_INT_TYPE)];
		r = callback(ctx, counter_buffer, bit_count);
		if (rle_unlikely(r < 0)) {
			rle->flags |= RLE_FLAG_DEC_ERROR;
			return;
		}

		if (rle_unlikely((unsigned)r < bit_count)) {
			rle->flags |= RLE_FLAG_DEC_UNDERFLOW;
			return;
		}

		unsigned i = 0;
		for (; bit_count >= 8; bit_count -= 8) {
			rem <<= 8;
			rem |= counter_buffer[i++];
		}

		if (bit_count) {
			rem <<= bit_count;
			rem |= counter_buffer[i] & ((1u << bit_count)-1);
		}
	}

	count += rem;
	rle->value = value;
	rle->count = count;
	rle->flags |= RLE_FLAG_DEC_AVAILABLE;
}

#endif // #ifdef RLE_C

#ifdef __cplusplus
} // extern "C"
#endif
