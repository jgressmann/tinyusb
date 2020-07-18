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
#include <assert.h>

#define DFU_APP_HDR_SECTION_NAME ".dfuapp"
#define DFU_APP_HDR_MAGIC_STRING "SuperDFU AHv1\0\0\0"
#define DFU_APP_HDR_VERSION 1

#define DFU_APP_ERROR_NONE                      0x00
#define DFU_APP_ERROR_MAGIC_MISMATCH            0x01
#define DFU_APP_ERROR_UNSUPPORED_HDR_VERSION    0x02
#define DFU_APP_ERROR_INVALID_SIZE              0x03
#define DFU_APP_ERROR_CRC_CALC_FAILED           0x04
#define DFU_APP_ERROR_CRC_VERIFICATION_FAILED   0x05
#define DFU_APP_HDR_REGION_SIZE                 0x80

// typedef uint32_t dfu_be32;

struct dfu_app_hdr {
	uint8_t magic[16];
	uint8_t dfu_app_hdr_version;
	uint8_t app_version_major;
	uint8_t app_version_minor;
	uint8_t app_version_patch;
	uint8_t app_name[64];
    uint32_t app_crc;
	uint32_t app_size;
	// uint32_t app_vector_addr;
	uint32_t reserved[9];
} __packed;

_Static_assert(128 == sizeof(struct dfu_app_hdr), "structure size must be 128 bytes");



int dfu_app_validate(struct dfu_app_hdr const *hdr);
