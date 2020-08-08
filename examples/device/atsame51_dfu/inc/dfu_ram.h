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
#include <stdbool.h>
#include <string.h>


#define DFU_RAM_HDR_SECTION_NAME ".dfuram"
#define DFU_RAM_HDR_MAGIC_STRING "SuperDFU RH\0\0\0\0\0"
#define DFU_RAM_HDR_VERSION 1
#define DFU_RAM_HDR_FLAG_DFU_REQ 0x1


/**
 * struct dfu_hdr - SuperDFU bootloader interaction (runtime)
 * @magic: fill by the bootloader to contain DFU_RAM_MAGIC_STRING
 * @version: fill by the bootloader to contain DFU_RAM_HDR_VERSION
 * @flags: flags to communicate with the bootloader to provide a naming hint for a possible
 *	device node to create.
 * @counter: counter to check for app termination
 *
 * This structure is created an initialized by the bootloader.
 * You can signal the bootloader to keep running after device reset
 * by setting DFU_RAM_FLAG_DFU_REQ in flags.
 *
 * To reference the structure from your application include this like somewhere
 *
 * struct dfu_hdr dfu_hdr __attribute__((section(DFU_RAM_HDR_SECTION_NAME)));
 */
struct dfu_hdr {
	uint8_t magic[16];
	uint8_t version;
	uint8_t reserved[3];
	uint32_t flags;
	uint32_t counter;
} __packed;


extern struct dfu_hdr dfu_hdr __attribute__((section(DFU_RAM_HDR_SECTION_NAME)));

/**
 * Get a pointer to the runtime bootloader header
 */
static inline struct dfu_hdr* dfu_hdr_ptr(void)
{
	return &dfu_hdr;
}

/**
 * Verifies that the bootloader version running matches the
 * version the application was compiled with.
 */
static inline bool dfu_signature_compatible(void)
{
	return
		0 == memcmp(dfu_hdr_ptr()->magic, DFU_RAM_HDR_MAGIC_STRING, sizeof(dfu_hdr_ptr()->magic))
		&& DFU_RAM_HDR_VERSION == dfu_hdr_ptr()->version;
}


/**
 * Request bootloader from your application
 *
 * @req: non-zero requests bootloader, zero disables
 *
 * After the bootloader has been requested, the application needs to reset
 * the device.
 */
static inline void dfu_request_dfu(int req)
{
	if (req) {
		dfu_hdr_ptr()->flags |= DFU_RAM_HDR_FLAG_DFU_REQ;
	} else {
		dfu_hdr_ptr()->flags &= ~DFU_RAM_HDR_FLAG_DFU_REQ;
	}
}

/**
 * Queries if the bootloader has been requested
 */
static inline bool dfu_requested_dfu(void)
{
	return (dfu_hdr_ptr()->flags & DFU_RAM_HDR_FLAG_DFU_REQ) == DFU_RAM_HDR_FLAG_DFU_REQ;
}

/**
 * Marks the application as stable.
 *
 * When started under SuperDFU, the bootloader will stop starting
 * and stay active, after the application has been started (and failed
 * causing device reset) 3 times.
 *
 * Marking the application as stable resets the counter.
 */
static inline void dfu_mark_stable(void)
{
	dfu_hdr_ptr()->counter = 0;
}

