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

#if defined(RLEW_STATIC)
	#define RLEW_EXTERN static
#else
	#define RLEW_EXTERN
#endif

#ifndef RLEW_ASSERT
	#include <assert.h>
	#define RLEW_ASSERT assert
#endif

#ifndef RLEW_FUNC
	#define RLEW_FUNC
#endif

#ifndef rlew_likely
	#define rlew_likely(x) x
#endif

#ifndef rlew_unlikely
	#define rlew_unlikely(x) x
#endif

#ifndef RLEW_INT_TYPE
	#error Define RLEW_INT_TYPE
#endif

#ifdef __cplusplus
extern "C" {
#endif


#if defined(RLEW_H)


typedef int (*rlew_store_t)(void* ctx, RLEW_INT_TYPE i);
typedef int (*rlew_load_t)(void* ctx, RLEW_INT_TYPE* i);

#define RLEW_MAX_BIT_COUNT ((sizeof(RLEW_INT_TYPE) * 8)-2)
#define RLEW_MAX_COUNT ((((RLEW_INT_TYPE)1) << (RLEW_MAX_BIT_COUNT+1))-1)


#define RLEW_FLAG_ENC_OVERFLOW  0x01


#define RLEW_FLAG_DEC_UNDERFLOW 0x01
#define RLEW_FLAG_DEC_EOS       0x02





struct rlew_encoder {
	RLEW_INT_TYPE count;
	RLEW_INT_TYPE state;
	uint8_t used;
	uint8_t value;
	uint8_t flags;
};

struct rlew_decoder {
	RLEW_INT_TYPE count;
	RLEW_INT_TYPE state;
	uint8_t used;
	uint8_t value;
	uint8_t flags;
};

RLEW_FUNC static inline void rlew_enc_init(struct rlew_encoder *rle)
{
	RLEW_ASSERT(rle);
	memset(rle, 0, sizeof(*rle));
}


RLEW_FUNC static inline void rlew_dec_init(struct rlew_decoder *rle)
{
	RLEW_ASSERT(rle);
	memset(rle, 0, sizeof(*rle));
}

RLEW_FUNC static inline unsigned int rlew_lz(RLEW_INT_TYPE value)
{
	unsigned int lz;

	if (sizeof(RLEW_INT_TYPE) <= sizeof(int)) {
		lz = __builtin_clz(value);
	} else if (sizeof(RLEW_INT_TYPE) <= sizeof(long)) {
		lz = __builtin_clzl(value);
	} else {
		lz = __builtin_clzll(value);
	}

	switch (sizeof(RLEW_INT_TYPE)) {
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
		RLEW_ASSERT(0 && "sizeof(RLEW_INT_TYPE) unhandled");
		break;
	}

	return lz;
}

RLEW_FUNC static inline unsigned int rlew_clrsb(RLEW_INT_TYPE value)
{
	unsigned int r;

	if (sizeof(RLEW_INT_TYPE) < sizeof(int)) {
		r = __builtin_clrsb(value << ((sizeof(int) - sizeof(RLEW_INT_TYPE)) * 8));
	} else if (sizeof(RLEW_INT_TYPE) == sizeof(int)) {
		r = __builtin_clrsb(value);
	} else if (sizeof(RLEW_INT_TYPE) == sizeof(long)) {
		r = __builtin_clrsbl(value);
	} else {
		r = __builtin_clrsbll(value);
	}

	switch (sizeof(RLEW_INT_TYPE)) {
	case 8:
		r &= 63;
		break;
	case 4:
		r &= 31;
		break;
	case 2:
		r &= 15;
		break;
	case 1:
		r &= 7;
		break;
	default:
		RLEW_ASSERT(0 && "sizeof(RLEW_INT_TYPE) unhandled");
		break;
	}

	return r;
}



RLEW_FUNC RLEW_EXTERN void rlew_enc_flush(
	struct rlew_encoder *rle,
	void* ctx,
	rlew_store_t callback);



RLEW_FUNC static inline void rlew_enc_bit(
		struct rlew_encoder *rle,
		void* ctx,
		rlew_store_t callback,
		unsigned bit)
{
	RLEW_ASSERT(rle);
	RLEW_ASSERT(callback);

	if (rlew_unlikely(rle->flags)) {
		return;
	}

	if (rlew_likely(rle->count)) {
		if (rlew_likely((bit == rle->value) & (rle->count < RLEW_MAX_COUNT))) {
			++rle->count;
		} else {
			rlew_enc_flush(rle, ctx, callback);
			if (rlew_likely(!rle->flags)) {
				rle->count = 1;
				rle->value = bit;
			}
		}
	} else {
		rle->count = 1;
		rle->value = bit;
	}
}



RLEW_FUNC static inline void rlew_enc_finish(
		struct rlew_encoder *rle,
		void* ctx,
		rlew_store_t callback,
		int term)
{
	RLEW_ASSERT(rle);
	RLEW_ASSERT(callback);

	rlew_enc_flush(rle, ctx, callback);

	if (rlew_unlikely(rle->flags)) {
		return;
	}

	if (rle->used) {
		rle->state <<= sizeof(RLEW_INT_TYPE) * 8 - rle->used;
		int e = callback(ctx, rle->state);
		if (rlew_unlikely(e)) {
			rle->flags |= RLEW_FLAG_ENC_OVERFLOW;
			return;
		}

		rle->used = 0;
	}

	if (term) {
		int e = callback(ctx, 0);
		if (rlew_unlikely(e)) {
			rle->flags |= RLEW_FLAG_ENC_OVERFLOW;
		}
	}
}

RLEW_FUNC RLEW_EXTERN void rlew_dec_load(
		struct rlew_decoder *rle,
		void* ctx,
		rlew_load_t callback);

RLEW_FUNC static inline int rlew_dec_bit(
		struct rlew_decoder *rle,
		void* ctx,
		rlew_load_t callback)
{
	int r;

	RLEW_ASSERT(rle);
	RLEW_ASSERT(callback);

start:
	if (rlew_unlikely(rle->flags)) {
		r = -1;
	} else {
		if (rlew_likely(rle->count)) {
			--rle->count;
			r = rle->value;
		} else {
			rlew_dec_load(rle, ctx, callback);
			goto start;
		}
	}

	return r;
}

#endif // #ifdef RLEW_H

#ifdef RLEW_C


RLEW_FUNC RLEW_EXTERN void rlew_enc_flush(
	struct rlew_encoder *rle,
	void* ctx,
	rlew_store_t callback)
{
	RLEW_ASSERT(rle);
	RLEW_ASSERT(callback);

	if (rlew_unlikely(!rle->count)) {
		return;
	}

	if (rlew_unlikely(rle->flags)) {
		return;
	}

	unsigned lz = rlew_lz(rle->count);
	unsigned count_bits = sizeof(RLEW_INT_TYPE) * 8 - 1 - lz;
	unsigned rem = rle->count - (((RLEW_INT_TYPE)1) << count_bits);
	unsigned left = sizeof(RLEW_INT_TYPE) * 8 - rle->used;
	unsigned prefix_bits = count_bits + 2;
	RLEW_INT_TYPE value;

	if (rle->value) {
		value = ((((RLEW_INT_TYPE)1) << (count_bits + 1)) - 1) << 1;
	} else {
		value = 1;
	}

	if (left < prefix_bits) {
		rle->state <<= left;
		rle->state |= (value >> (prefix_bits - left)) & ((((RLEW_INT_TYPE)1) << left)-1);
		int e = callback(ctx, rle->state);
		if (rlew_unlikely(e)) {
			rle->flags |= RLEW_FLAG_ENC_OVERFLOW;
			return;
		}

		value &= ((((RLEW_INT_TYPE)1) << (prefix_bits - left))-1);
		prefix_bits -= left;

		rle->state = 0;
		rle->used = 0;
		left = sizeof(RLEW_INT_TYPE) * 8;
	}

	rle->state <<= prefix_bits;
	rle->state |= value;
	rle->used += prefix_bits;
	left = sizeof(RLEW_INT_TYPE) * 8 - rle->used;

	if (count_bits) {
		if (left < count_bits) {
			rle->state <<= left;
			rle->state |= (rem >> (count_bits - left)) & ((((RLEW_INT_TYPE)1) << left)-1);
			int e = callback(ctx, rle->state);
			if (rlew_unlikely(e)) {
				rle->flags |= RLEW_FLAG_ENC_OVERFLOW;
				return;
			}

			rem &= ((((RLEW_INT_TYPE)1) << (count_bits - left))-1);
			count_bits -= left;

			rle->state = 0;
			rle->used = 0;
			left = sizeof(RLEW_INT_TYPE) * 8;
		}

		rle->state <<= count_bits;
		rle->state |= rem;
		rle->used += count_bits;
	}

	rle->count = 0;
}

RLEW_FUNC RLEW_EXTERN void rlew_dec_load(
		struct rlew_decoder *rle,
		void* ctx,
		rlew_load_t callback)
{
	RLEW_ASSERT(rle);
	RLEW_ASSERT(callback);

	if (rlew_unlikely(!rle->used)) {
		int e = callback(ctx, &rle->state);
		if (rlew_unlikely(e)) {
			rle->flags |= RLEW_FLAG_DEC_UNDERFLOW;
			return;
		}

		rle->used = sizeof(RLEW_INT_TYPE) * 8;
	}

	RLEW_ASSERT(rle->used >= 2);

	RLEW_INT_TYPE bit = ((RLEW_INT_TYPE)1) << (sizeof(RLEW_INT_TYPE) * 8 - 1);
	RLEW_INT_TYPE count = 1;

	// first bit
	unsigned first = (rle->state & bit) == bit;
	rle->state <<= 1;
	--rle->used;

//	if (rlew_unlikely(!rle->used)) {
//		int e = callback(ctx, &rle->state);
//		if (rlew_unlikely(e)) {
//			rle->flags |= RLEW_FLAG_DEC_UNDERFLOW;
//			return;
//		}

//		rle->used = sizeof(RLEW_INT_TYPE) * 8;
//	}

	// second bit, but keep bit so we can
	// easily do a check of same sign bit
	// in case we need to read the counter
	unsigned second = (rle->state & bit) == bit;

	// need to read counter?
	if (rlew_likely(second == first)) {
		unsigned count_bits = 0;

		unsigned same = rlew_clrsb(rle->state);

		// now remove 'second'
		--rle->used;
		rle->state <<= 1;

		if (same > rle->used) {
			same = rle->used;
		}

		if (same == rle->used) {
			// account for second
			++same;

			int e = callback(ctx, &rle->state);
			if (rlew_unlikely(e)) {
				if (same + 1 >= sizeof(RLEW_INT_TYPE) * 8) {
					rle->flags |= RLEW_FLAG_DEC_EOS;
				} else {
					rle->flags |= RLEW_FLAG_DEC_UNDERFLOW;
				}
				return;
			}

			rle->used = sizeof(RLEW_INT_TYPE) * 8;


			if (first == ((rle->state & bit) == bit)) {
				unsigned more_same = rlew_clrsb(rle->state) + 1;

				if (more_same > rle->used) {
					more_same = rle->used;
				}

				same += more_same;

				if (rlew_unlikely(same + 1 >= sizeof(RLEW_INT_TYPE) * 8)) {
					rle->flags |= RLEW_FLAG_DEC_EOS;
					return;
				}

				rle->used -= more_same + 1;
				rle->state <<= more_same + 1;
			} else {
				rle->used -= 1;
				rle->state <<= 1;
			}

			count_bits = same;
		} else {
			// account for second
			++same;
			count_bits = same;
			rle->state <<= same;
			rle->used -= same;

			if (rlew_unlikely(!rle->used)) {
				int e = callback(ctx, &rle->state);
				if (rlew_unlikely(e)) {
					rle->flags |= RLEW_FLAG_DEC_UNDERFLOW;
					return;
				}

				rle->used = sizeof(RLEW_INT_TYPE) * 8;
			}
		}

		RLEW_INT_TYPE rem = 0;
		count = (((RLEW_INT_TYPE)1) << count_bits);

		RLEW_ASSERT(rle->used);

		if (count_bits > rle->used) {
			rem = (rle->state >> (sizeof(RLEW_INT_TYPE) * 8 - count_bits)) << (count_bits - rle->used);
			count_bits -= rle->used;

			int e = callback(ctx, &rle->state);
			if (rlew_unlikely(e)) {
				rle->flags |= RLEW_FLAG_DEC_UNDERFLOW;
				return;
			}

			rle->used = sizeof(RLEW_INT_TYPE) * 8;
		}

		rem |= rle->state >> (sizeof(RLEW_INT_TYPE) * 8 - count_bits);
		rle->used -= count_bits;
		rle->state <<= count_bits;

		count += rem;
	} else {
		// remove second
		rle->state <<= 1;
		--rle->used;
	}

	rle->value = first;
	rle->count = count;
}


#endif // #ifdef RLEW_C

#ifdef __cplusplus
} // extern "C"
#endif
