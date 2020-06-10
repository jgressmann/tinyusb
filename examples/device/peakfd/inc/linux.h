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

#include <stdint.h>


typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef u16 __le16;
typedef u32 __le32;
typedef u16 __be16;
typedef u32 __be32;

#if TU_BIG_ENDIAN == TU_BYTE_ORDER
static inline u16 le16_to_cpu(__le16 value) { return __builtin_bswap16(value); }
static inline u32 le32_to_cpu(__le32 value) { return __builtin_bswap32(value); }
static inline u16 cpu_to_le16(__le16 value) { return __builtin_bswap16(value); }
static inline u32 cpu_to_le32(__le32 value) { return __builtin_bswap32(value); }
#else
static inline u16 le16_to_cpu(__le16 value) { return value; }
static inline u32 le32_to_cpu(__le32 value) { return value; }
static inline u16 cpu_to_le16(__le16 value) { return value; }
static inline u32 cpu_to_le32(__le32 value) { return value; }
#endif



