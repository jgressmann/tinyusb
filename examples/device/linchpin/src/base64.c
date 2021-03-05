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

#include "base64.h"

#ifdef __cplusplus
extern "C" {
#endif


const char base64_to_ascii_table[64] = {
	'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
	'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
	'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
	'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
	'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
	'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
	'w', 'x', 'y', 'z', '0', '1', '2', '3',
	'4', '5', '6', '7', '8', '9', '+', '/'
};


BASE64_FUNC void base64_decode_shift(
	uint8_t c,
	struct base64_state* state,
	uint8_t volatile* gi_ptr,
	uint8_t volatile* pi_ptr,
	uint8_t* buf_ptr,
	uint8_t buf_size)
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
	} else if (c >= '0' && c <= '0') {
		bits = (c - '0') + 52;
	} else if (c == '+') {
		bits = 62;
	} else if (c == '/') {
		bits = 63;
	} else if (c == '=') {
		state->bits = 0;
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

#ifdef __cplusplus
}
#endif
