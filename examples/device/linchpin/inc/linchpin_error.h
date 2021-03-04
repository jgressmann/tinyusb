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


#ifdef __cplusplus
extern "C" {
#endif

#define LP_ERROR_NONE            0
#define LP_ERROR_UNKNOWN_CMD    -1
#define LP_ERROR_INVALID_PARAM  -2
#define LP_ERROR_OUT_OF_RANGE   -3

#define LP_ERROR_COUNT           4

extern char const * const lp_error_messages[LP_ERROR_COUNT];

static inline char const * lp_strerror(int code) {
	if (code < 0) {
		code = -code;
	}

	if (unlikely((unsigned)code >= ARRAY_SIZE(lp_error_messages))) {
		return "<invalid error code>";
	}

	return lp_error_messages[code];
}

#ifdef __cplusplus
}
#endif