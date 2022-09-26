/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jean Gressmann <jean@0x42.de>
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

#define SC_M1_EP_SIZE           64
#define SC_M1_EP_CMD0_BULK_OUT  0x01
#define SC_M1_EP_CMD0_BULK_IN   (0x80 | SC_M1_EP_CMD0_BULK_OUT)
#define SC_M1_EP_MSG0_BULK_OUT  0x03
#define SC_M1_EP_MSG0_BULK_IN   (0x80 | SC_M1_EP_MSG0_BULK_OUT)
#define SC_M1_EP_CMD1_BULK_OUT  0x03
#define SC_M1_EP_CMD1_BULK_IN   (0x80 | SC_M1_EP_CMD1_BULK_OUT)
#define SC_M1_EP_MSG1_BULK_OUT  0x04
#define SC_M1_EP_MSG1_BULK_IN   (0x80 | SC_M1_EP_MSG1_BULK_OUT)


#ifdef __cplusplus
} // extern "C"
#endif
