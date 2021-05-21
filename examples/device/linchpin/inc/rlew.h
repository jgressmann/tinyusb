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

#define RLEW_ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))


typedef int (*rlew_store_t)(void* ctx, RLEW_INT_TYPE i);
typedef int (*rlew_load_t)(void* ctx, RLEW_INT_TYPE* i);

#define RLEW_MAX_BIT_COUNT ((sizeof(RLEW_INT_TYPE) * 8)-2)
#define RLEW_MAX_COUNT ((((RLEW_INT_TYPE)1) << (RLEW_MAX_BIT_COUNT+1))-1)
//#define RLEW_MAX_COUNT ((RLEW_INT_TYPE)-1)


#define RLEW_ERROR_NONE      0
#define RLEW_ERROR_FALSE     0
#define RLEW_ERROR_TRUE      1
#define RLEW_ERROR_UNDERFLOW (1<<1)
#define RLEW_ERROR_OVERFLOW  (2<<1)
#define RLEW_ERROR_EOS       (3<<1)

#define rlew_is_error(x) ((x) > 1)

#define RLEW_TERM_COUNT 3

struct rlew_encoder {
	RLEW_INT_TYPE count;
	RLEW_INT_TYPE state;
	RLEW_INT_TYPE output_buffer[2];
	uint8_t used;
	uint8_t value;
	uint8_t output_pi; // full range
	uint8_t output_gi; // full range
};

struct rlew_decoder {
	RLEW_INT_TYPE count;
	RLEW_INT_TYPE state;
	RLEW_INT_TYPE input_buffer[3];
	uint8_t used;
	uint8_t value;
	uint8_t input_pi; // full range
	uint8_t input_gi; // full range
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



RLEW_FUNC RLEW_EXTERN void rlew_enc_flush(struct rlew_encoder *rle);
RLEW_FUNC RLEW_EXTERN int rlew_enc_finish(struct rlew_encoder *rle);


RLEW_FUNC static inline int rlew_enc_bit(
		struct rlew_encoder *rle,
		unsigned bit)
{
	RLEW_ASSERT(rle);

	if (rlew_likely(rle->count)) {
		if (rlew_likely((bit == rle->value))) {
			if (rlew_likely((rle->count < RLEW_MAX_COUNT))) {
				++rle->count;
				goto out;
			} else {
flush:
				if (rle->output_pi != rle->output_gi) {
					// require the output buffer to be cleared first
					return RLEW_ERROR_OVERFLOW;
				}

				rlew_enc_flush(rle);
				goto store1;
			}
		} else {
			goto flush;
		}
	}

store1:
	rle->count = 1;
	rle->value = (uint8_t)bit;

out:
	return RLEW_ERROR_NONE;
}



RLEW_FUNC RLEW_EXTERN int rlew_dec_load(struct rlew_decoder *rle);

RLEW_FUNC static inline int rlew_dec_bit(struct rlew_decoder *rle)
{
	RLEW_ASSERT(rle);

	if (rlew_unlikely(!rle->count)) {
		int error = rlew_dec_load(rle);
		if (error) {
			return error;
		}
	}

	RLEW_ASSERT(rle->count);

	--rle->count;
	return rle->value;
}

#endif // #ifdef RLEW_H

#ifdef RLEW_C


RLEW_FUNC RLEW_EXTERN int rlew_enc_finish(struct rlew_encoder *rle)
{
	RLEW_ASSERT(rle);

	if (rle->count) {
		if (rle->output_pi != rle->output_gi) {
			return RLEW_ERROR_OVERFLOW;
		}

		rlew_enc_flush(rle);
	}

	if (rle->used) {
		if (rle->output_pi != rle->output_gi) {
			return RLEW_ERROR_OVERFLOW;
		}

		rle->state <<= sizeof(RLEW_INT_TYPE) * 8 - rle->used;
		rle->output_buffer[rle->output_pi++ % RLEW_ARRAY_SIZE(rle->output_buffer)] = rle->state;
		rle->used = 0;

		return RLEW_ERROR_OVERFLOW;
	}

//	if (term) {
//		if (rle->output_pi != rle->output_gi) {
//			return RLEW_ERROR_OVERFLOW;
//		}

//		rle->output_buffer[rle->output_pi++ % RLEW_ARRAY_SIZE(rle->output_buffer)] = rle->state;
//		return RLEW_ERROR_OVERFLOW;
//	}

	return RLEW_ERROR_NONE;
}


RLEW_FUNC RLEW_EXTERN void rlew_enc_flush(struct rlew_encoder *rle)
{
	RLEW_ASSERT(rle);
	RLEW_ASSERT(rle->output_pi == rle->output_gi);
	RLEW_ASSERT(rle->count);
	RLEW_ASSERT(!(rle->used & 1));

	unsigned lz = rlew_lz(rle->count);
	lz -= lz != 0;
	unsigned count_bits = RLEW_MAX_BIT_COUNT - lz;
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

		rle->output_buffer[rle->output_pi++ % RLEW_ARRAY_SIZE(rle->output_buffer)] = rle->state;

		value &= ((((RLEW_INT_TYPE)1) << (prefix_bits - left))-1);
		prefix_bits -= left;

		rle->state = 0;
		rle->used = 0;
		left = sizeof(RLEW_INT_TYPE) * 8;
	}

	rle->state <<= prefix_bits;
	rle->state |= value;
	rle->used += prefix_bits;
	left -= prefix_bits;

	if (count_bits) {
		if (left < count_bits) {
			rle->state <<= left;
			rle->state |= (rem >> (count_bits - left)) & ((((RLEW_INT_TYPE)1) << left)-1);
			rle->output_buffer[rle->output_pi++ % RLEW_ARRAY_SIZE(rle->output_buffer)] = rle->state;

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
	RLEW_ASSERT(!(rle->used & 1));
}

RLEW_FUNC RLEW_EXTERN int rlew_dec_load(struct rlew_decoder *rle)
{
	RLEW_ASSERT(rle);

	if ((uint8_t)(rle->input_pi - rle->input_gi) < RLEW_ARRAY_SIZE(rle->input_buffer)) {
		return RLEW_ERROR_UNDERFLOW;
	}

	if (rlew_unlikely(!rle->used)) {
		rle->state = rle->input_buffer[rle->input_gi++ % RLEW_ARRAY_SIZE(rle->input_buffer)];
		rle->used = sizeof(RLEW_INT_TYPE) * 8;
	}

	RLEW_ASSERT(rle->used >= 2);

	RLEW_INT_TYPE bit = ((RLEW_INT_TYPE)1) << (sizeof(RLEW_INT_TYPE) * 8 - 1);
	RLEW_INT_TYPE count = 1;

	// first bit
	unsigned first = (rle->state & bit) == bit;
	rle->state <<= 1;
	--rle->used;

	// get second bit, but keep it in state so
	// we can save the reload of the state
	// in case there are no more bits after 'second'

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

			rle->state = rle->input_buffer[rle->input_gi++ % RLEW_ARRAY_SIZE(rle->input_buffer)];
			rle->used = sizeof(RLEW_INT_TYPE) * 8;

			if (first == ((rle->state & bit) == bit)) {
				unsigned more_same = rlew_clrsb(rle->state) + 1;

				same += more_same;

				if (rlew_unlikely(same + 1 >= sizeof(RLEW_INT_TYPE) * 8)) {
					return RLEW_ERROR_EOS;
				}

				count_bits = same;
				rle->used -= more_same + 1;
				rle->state <<= more_same + 1;
			} else {
				count_bits = same;
				rle->used -= 1;
				rle->state <<= 1;
			}
		} else {
			// account for second
			++same;
			count_bits = same;
			rle->used -= same;
			rle->state <<= same;
		}

		RLEW_INT_TYPE rem = 0;
		count = (((RLEW_INT_TYPE)1) << count_bits);

//		RLEW_ASSERT(rle->used);

		if (count_bits > rle->used) {
			rem = (rle->state >> (sizeof(RLEW_INT_TYPE) * 8 - count_bits)) << (count_bits - rle->used);
			count_bits -= rle->used;

			rle->state = rle->input_buffer[rle->input_gi++ % RLEW_ARRAY_SIZE(rle->input_buffer)];
			rle->used = sizeof(RLEW_INT_TYPE) * 8;
		}

		rem |= rle->state >> (sizeof(RLEW_INT_TYPE) * 8 - count_bits);
		rle->used -= count_bits;
		rle->state <<= count_bits;

		count |= rem;
	} else {
		// remove second
		rle->state <<= 1;
		--rle->used;
	}

	rle->value = first ? RLEW_ERROR_TRUE : RLEW_ERROR_FALSE;
	rle->count = count;

	RLEW_ASSERT(!(rle->used & 1));

	return RLEW_ERROR_NONE;
}


#endif // #ifdef RLEW_C

#ifdef __cplusplus
} // extern "C"
#endif
