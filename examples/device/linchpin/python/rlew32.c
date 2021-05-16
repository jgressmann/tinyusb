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


#include <stdlib.h>
#include "rlew32.h"

#define RLEW_C
#define RLEW_INT_TYPE uint32_t
#include <rlew.h>


extern rlew32_decoder* rlew32_dec_new()
{
    rlew32_decoder* d = malloc(sizeof(*d));
    if (d) {
        rlew_dec_init(d);
    }

    return d;
}

extern void rlew32_dec_free(rlew32_decoder* d)
{
    free(d);
}

extern void rlew32_dec_init(rlew32_decoder* d)
{
    rlew_dec_init(d);
}

extern int rlew32_dec_bit(rlew32_decoder* d, void* ctx, rlew_load_t callback)
{
    return rlew_dec_bit(d, ctx, callback);
}

extern uint8_t rlew32_dec_flags(rlew32_decoder* rle)
{
    return rle->flags;
}

extern rlew32_encoder* rlew32_enc_new()
{
    rlew32_encoder* e = malloc(sizeof(*e));
    if (e) {
        rlew_enc_init(e);
    }

    return e;
}

extern void rlew32_enc_free(rlew32_encoder* e)
{
    free(e);
}

extern void rlew32_enc_bit(rlew32_encoder* e, void* ctx, rlew_store_t callback, unsigned int bit)
{
    rlew_enc_bit(e, ctx, callback, bit);
}

//extern void rlew32_enc_flush(rlew32_encoder* e, void* ctx, rlew_store_t callback)
//{
//    rlew_enc_flush(e, ctx, callback);
//}

extern void rlew32_enc_finish(rlew32_encoder* e, void* ctx, rlew_store_t callback, int term)
{
    rlew_enc_finish(e, ctx, callback, term);
}

extern uint8_t rlew32_enc_flags(rlew32_encoder* rle)
{
    return rle->flags;
}
