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

#define RLEW_ASSERT(...)
#define RLEW_H
#define RLEW_INT_TYPE uint32_t
#include <rlew.h>
#undef RLEW_INT_TYPE
#undef RLEW_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct rlew_encoder rlew32_encoder;
typedef struct rlew_decoder rlew32_decoder;

extern rlew32_decoder* rlew32_dec_new();
extern void rlew32_dec_free(rlew32_decoder* d);
extern int rlew32_dec_bit(rlew32_decoder* d);

extern rlew32_encoder* rlew32_enc_new();
extern void rlew32_enc_free(rlew32_encoder* e);
extern int rlew32_enc_bit(rlew32_encoder* e, unsigned int bit);
extern int rlew32_enc_finish(rlew32_encoder* e);
extern int rlew32_enc_output_take(rlew32_encoder* e, uint32_t* value);

#ifdef __cplusplus
} // extern "C"
#endif
