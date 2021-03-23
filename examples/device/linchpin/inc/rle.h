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
#include <stddef.h>
#include <stdlib.h>
#include <toe/preshing_hash.h>

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

#ifdef __cplusplus
extern "C" {
#endif




typedef int (*rle_gen_t)(size_t* bits, unsigned bit_count);

#define RLE_FLAG_VALUE  0x1
#define RLE_MAX_COUNT (((size_t)1) << (sizeof(size_t) * 4))

#define RLEE_NONE 0

struct rle {
	size_t count;
	size_t value:1;
	size_t flags:1;
	rle_gen_t generator;
};

static inline size_t rle_ceil2(size_t v) {
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	if (sizeof(v) == 8) {
		v |= v >> 32;
	}
	v++;
	return v;
}

static inline size_t rle_floor2(size_t v) {
	return rle_ceil2(v) >> 1;
}

static inline void rle_init(struct rle *rle)
{
	RLE_ASSERT(rle);
	memset(rle, 0, sizeof(*rle));
}


static inline int rle_encode_flush(struct rle *rle)
{
	RLE_ASSERT(rle);
	RLE_ASSERT(rle->generator);

	if (!rle->count) {
		return RLEE_NONE;
	}

	unsigned lz;
	if (sizeof(size_t) == sizeof(int)) {
		lz = __builtin_clz(rle->count);
	}
	else if (sizeof(size_t) == sizeof(long)) {
		lz = __builtin_clzl(rle->count);
	} else {
		lz = __builtin_clzll(rle->count);
	}

	unsigned count_bits = sizeof(size_t) * 8 - 1 - lz;
	unsigned rem = rle->count - (((size_t)1) << count_bits);

	size_t buf[2] = {0, 0};
	unsigned left = sizeof(size_t) * 8;
	unsigned total_bit_count = (count_bits + 1) * 2;
	int error = RLEE_NONE;

	if (rle->value) {
		buf[0] |= rle->value;
		--left;

		for (unsigned i = 0; i < count_bits; ++i) {
			buf[0] <<= 1;
			buf[0] |= rle->value;
			--left;
		}

		buf[0] <<= 1;
		buf[0] |= !rle->value;
		--left;

		// fix me optimize
	} else {
		left -= count_bits + 2;
		buf[0] = 1;
	}

	if (count_bits) {
		if (left < count_bits) {
			buf[0] <<= left;
			buf[0] |= (rem >> (count_bits - left)) & ((((size_t)1) << left)-1);
			buf[1] = rem & ((((size_t)1) << (count_bits - left))-1);
		} else {
			buf[0] <<= count_bits;
			buf[0] |= rem;
		}
	}

	rle->count = 0;
	rle->flags = 0;

	return rle->generator(buf, total_bit_count);
}

static inline int rle_encode_push_bit(struct rle *rle, int bit)
{
	RLE_ASSERT(rle);

	if (rle_likely(rle->flags & RLE_FLAG_VALUE)) {
		if (rle_likely(bit == rle->value)) {
			if (rle_unlikely(rle->count == RLE_MAX_COUNT)) {
				int error = rle_encode_flush(rle);
				if (rle_unlikely(error)) {
					return error;
				}
				goto start;
			} else {
				++rle->count;
			}
		} else {
			int error = rle_encode_flush(rle);
			if (rle_unlikely(error)) {
				return error;
			}
			goto start;
		}
	} else {
start:
		rle->count = 1;
		rle->value = bit;
		rle->flags = RLE_FLAG_VALUE;
	}

	return RLEE_NONE;
}

#ifdef __cplusplus
} // extern "C"
#endif
