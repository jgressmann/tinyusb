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


#define DFU_RAM_SECTION_NAME ".dfuram"
#define DFU_RAM_MAGIC_STRING "SuperDFU RHv1\0\0\0"
#define DFU_RAM_FLAG_DFU_REQ 0x1



struct dfu_hdr {
	uint8_t magic[16];
	uint32_t flags;
	uint32_t counter;
} __packed;


extern struct dfu_hdr dfu_hdr __attribute__((used,section(DFU_RAM_SECTION_NAME)));

static inline void dfu_request_dfu(int req)
{
	if (req) {
		dfu_hdr.flags |= DFU_RAM_FLAG_DFU_REQ;
	} else {
		dfu_hdr.flags &= ~DFU_RAM_FLAG_DFU_REQ;
	}
}

static inline bool dfu_requested_dfu(void)
{
	return (dfu_hdr.flags & DFU_RAM_FLAG_DFU_REQ) == DFU_RAM_FLAG_DFU_REQ;
}

static inline void dfu_mark_stable(void)
{
	dfu_hdr.counter = 0;
}

