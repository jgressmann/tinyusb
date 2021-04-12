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

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#if defined(RLE_STATIC)
	#define RLE_EXTERN static
#else
	#define RLE_EXTERN
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


/* \brief Write bits during encode
 *
 * \param ctx: user supplied pointer
 * \param ptr: pointer to bit storage
 * \param count: number of bits to write
 *
 * \return: the number of bits placed or negative on error
 */
typedef int (*rle_write_bits_t)(void* ctx, uint8_t const* ptr, unsigned count);

/* \brief Read bits during decode
 *
 * \param ctx: user supplied pointer
 * \param ptr: pointer to bit storage
 * \param count: number of bits to read
 *
 * \return: the number of bits placed or negative on error
 */
typedef int (*rle_read_bits_t)(void* ctx, uint8_t* ptr, unsigned count);

#define RLE_MAX_BIT_COUNT ((sizeof(RLE_INT_TYPE) * 8)-2)
#define RLE_MAX_COUNT ((((RLE_INT_TYPE)1) << (RLE_MAX_BIT_COUNT+1))-1)


#define RLE_FLAG_ENC_ERROR     0x01
#define RLE_FLAG_ENC_OVERFLOW  0x02
#define RLE_FLAG_ENC_VALUE     0x04


#define RLE_FLAG_DEC_ERROR     0x01
#define RLE_FLAG_DEC_EOS       0x02
#define RLE_FLAG_DEC_UNDERFLOW 0x04
#define RLE_FLAG_DEC_AVAILABLE 0x08

struct rle {
	RLE_INT_TYPE count;
	uint8_t value;
	uint8_t flags;
	uint8_t priv;
};

RLE_FUNC static inline void rle_init(struct rle *rle)
{
	RLE_ASSERT(rle);
	memset(rle, 0, sizeof(*rle));
}

RLE_FUNC RLE_EXTERN void rle_encode_flush(
	struct rle *rle,
	void* ctx,
	rle_write_bits_t callback);



RLE_FUNC static inline void rle_encode_bit(
		struct rle *rle,
		void* ctx,
		rle_write_bits_t callback,
		unsigned bit)
{
	RLE_ASSERT(rle);
	RLE_ASSERT(callback);

	if (rle_unlikely(rle->flags)) {
		return;
	}

	if (rle_likely(rle->priv)) {
		if (rle_likely((bit == rle->value) & (rle->count < RLE_MAX_COUNT))) {
			++rle->count;
		} else {
			rle_encode_flush(rle, ctx, callback);
			if (rle_likely(!rle->flags)) {
				// goto increases by 4 cycles
				rle->count = 1;
				rle->value = bit;
				// goto store;
			}
		}
	} else {
		rle->priv = 1;
// store:
		rle->count = 1;
		rle->value = bit;
	}
}

RLE_FUNC RLE_EXTERN void rle_load(
		struct rle *rle,
		void* ctx,
		rle_read_bits_t callback);

RLE_FUNC static inline void rle_decode_bit(
		struct rle *rle,
		void* ctx,
		rle_read_bits_t callback,
		unsigned *bit)
{
	RLE_ASSERT(rle);
	RLE_ASSERT(callback);
	RLE_ASSERT(bit);

	if (rle_unlikely(rle->flags)) {
		return;
	}

	if (rle_likely(rle->priv)) {
available:
		if (rle_likely(rle->count > 1)) {
			*bit = rle->value;
			--rle->count;
		} else {
			RLE_ASSERT(rle->count == 1);
			*bit = rle->value;
			rle->priv = 0;
		}
	} else {
		rle_load(rle, ctx, callback);
		if (rle_likely(!rle->flags)) {
			rle->priv = 1;
			goto available;
		}
	}
}

#endif // #ifdef RLE_H

#ifdef RLE_C

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshift-count-overflow"


RLE_FUNC RLE_EXTERN void rle_encode_flush(
	struct rle *rle,
	void* ctx,
	rle_write_bits_t callback)
{
	RLE_ASSERT(rle);
	RLE_ASSERT(callback);

	if (rle_unlikely(!rle->count)) {
		return;
	}

	if (rle_unlikely(rle->flags & (RLE_FLAG_ENC_ERROR | RLE_FLAG_ENC_OVERFLOW))) {
		return;
	}

	unsigned lz;
	if (sizeof(RLE_INT_TYPE) <= sizeof(int)) {
		lz = __builtin_clz(rle->count);
	} else if (sizeof(RLE_INT_TYPE) <= sizeof(long)) {
		lz = __builtin_clzl(rle->count);
	} else {
		lz = __builtin_clzll(rle->count);
	}

	switch (sizeof(RLE_INT_TYPE)) {
	case 8:
		lz &= 63;
		break;
	case 4:
		lz &= 31;
		break;
	case 2:
		lz &= 15;
		break;
	case 1:
		lz &= 7;
		break;
	default:
		RLE_ASSERT(0 && "sizeof(RLE_INT_TYPE) unhandled");
		break;
	}

	unsigned count_bits = sizeof(RLE_INT_TYPE) * 8 - 1 - lz;
	unsigned rem = rle->count - (((RLE_INT_TYPE)1) << count_bits);

	uint8_t byte_buf[2*sizeof(RLE_INT_TYPE)];
	unsigned left = sizeof(RLE_INT_TYPE) * 8;
	unsigned total_bit_count = (count_bits + 1) * 2;
	unsigned byte_offset = 0;
	RLE_INT_TYPE value;

	if (rle->value) {
		value = ((((RLE_INT_TYPE)1) << (count_bits + 1)) - 1) << 1;
	} else {
		value = 1;
	}

	left -= count_bits + 2;

	if (count_bits) {
		if (left < count_bits) {
			value <<= left;
			value |= (rem >> (count_bits - left)) & ((((RLE_INT_TYPE)1) << left)-1);

			switch (sizeof(RLE_INT_TYPE)) {
			case 8:
				byte_buf[byte_offset++] = (uint8_t)(value >> 56);
				byte_buf[byte_offset++] = (uint8_t)(value >> 48);
				byte_buf[byte_offset++] = (uint8_t)(value >> 40);
				byte_buf[byte_offset++] = (uint8_t)(value >> 32);
				__attribute__ ((fallthrough));
			case 4:
				byte_buf[byte_offset++] = (uint8_t)(value >> 24);
				byte_buf[byte_offset++] = (uint8_t)(value >> 16);
				__attribute__ ((fallthrough));
			case 2:
				byte_buf[byte_offset++] = (uint8_t)(value >> 8);
				__attribute__ ((fallthrough));
			case 1:
				byte_buf[byte_offset++] = (uint8_t)(value >> 0);
				break;
			}

			value = rem & ((((RLE_INT_TYPE)1) << (count_bits - left))-1);
			left = sizeof(RLE_INT_TYPE) * 8 - (count_bits - left);
		} else {
			value <<= count_bits;
			value |= rem;
			left -= count_bits;
		}
	}

	if ((sizeof(RLE_INT_TYPE) * 8u - left) & 7u) {
		unsigned align_shift = 8u - ((sizeof(RLE_INT_TYPE) * 8u - left) & 7u);

		value <<= align_shift;
	}

	switch ((sizeof(RLE_INT_TYPE) * 8u - left + 7u) / 8u) {
	case 8:
		byte_buf[byte_offset++] = (uint8_t)(value >> 56);
		__attribute__ ((fallthrough));
	case 7:
		byte_buf[byte_offset++] = (uint8_t)(value >> 48);
		__attribute__ ((fallthrough));
	case 6:
		byte_buf[byte_offset++] = (uint8_t)(value >> 40);
		__attribute__ ((fallthrough));
	case 5:
		byte_buf[byte_offset++] = (uint8_t)(value >> 32);
		__attribute__ ((fallthrough));
	case 4:
		byte_buf[byte_offset++] = (uint8_t)(value >> 24);
		__attribute__ ((fallthrough));
	case 3:
		byte_buf[byte_offset++] = (uint8_t)(value >> 16);
		__attribute__ ((fallthrough));
	case 2:
		byte_buf[byte_offset++] = (uint8_t)(value >> 8);
		__attribute__ ((fallthrough));
	case 1:
		byte_buf[byte_offset++] = (uint8_t)(value >> 0);
		break;
	}

	int w = callback(ctx, byte_buf, total_bit_count);
	if (rle_unlikely((unsigned)w != total_bit_count)) {
		if (w < 0) {
			rle->flags |= RLE_FLAG_ENC_ERROR;
		} else {
			rle->flags |= RLE_FLAG_ENC_OVERFLOW;
		}
	} else {
		rle->count = 0;
		rle->flags = 0;
	}
}

RLE_FUNC RLE_EXTERN void rle_load(
		struct rle *rle,
		void* ctx,
		rle_read_bits_t callback)
{
	RLE_ASSERT(rle);
	RLE_ASSERT(callback);

	uint8_t byte;
	int r = callback(ctx, &byte, 1);
	if (rle_unlikely((unsigned)r != 1)) {
		if (r < 0) {
			rle->flags |= RLE_FLAG_DEC_ERROR;
		} else {
			rle->flags |= RLE_FLAG_DEC_UNDERFLOW;
		}
		return;
	}

	unsigned value = (byte & 0x80) == 0x80;
	unsigned next_bit;
	unsigned bit_count = -1;

	// find first inverted bit
	do {
		++bit_count;

		r = callback(ctx, &byte, 1);
		if (rle_unlikely((unsigned)r != 1)) {
			if (r < 0) {
				rle->flags |= RLE_FLAG_DEC_ERROR;
			} else {
				rle->flags |= RLE_FLAG_DEC_UNDERFLOW;
			}
			return;
		}

		next_bit = (byte & 0x80) == 0x80;

		if (rle_unlikely(bit_count == RLE_MAX_BIT_COUNT)) {
			rle->flags |= RLE_FLAG_DEC_EOS;
			return;
		}
	} while (next_bit == value);


	RLE_INT_TYPE count = ((RLE_INT_TYPE)1) << bit_count;
	RLE_INT_TYPE rem = 0;

	// read out n-bit counter
	if (bit_count) {
		uint8_t counter_buffer[sizeof(RLE_INT_TYPE)];

		r = callback(ctx, counter_buffer, bit_count);
		if (rle_unlikely((unsigned)r != bit_count)) {
			if (r < 0) {
				rle->flags |= RLE_FLAG_DEC_ERROR;
			} else {
				rle->flags |= RLE_FLAG_DEC_UNDERFLOW;
			}
			return;
		}

		unsigned i = 0;
		for (; bit_count >= 8; bit_count -= 8, ++i) {
			rem <<= 8;
			rem |= counter_buffer[i];
		}

		if (bit_count) {
			rem <<= 8;
			rem |= counter_buffer[i];
			rem >>= 8 - bit_count;
		}
	}

	count += rem;
	rle->value = value;
	rle->count = count;
}

#pragma GCC diagnostic pop

#endif // #ifdef RLE_C

#ifdef __cplusplus
} // extern "C"
#endif
