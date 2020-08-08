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
#include <mcu.h>

#define DFU_APP_HDR_SECTION_NAME ".dfuapphdr"
#define DFU_APP_FTR_SECTION_NAME ".dfuappftr"
#define DFU_APP_HDR_MAGIC_STRING "SuperDFU AH\0\0\0\0\0"
#define DFU_APP_FTR_MAGIC_STRING "SuperDFU AF\0\0\0\0\0"
#define DFU_APP_HDR_VERSION 1
#define DFU_APP_HDR_SIZE 0x40

#define DFU_APP_ERROR_NONE                      0x00
#define DFU_APP_ERROR_MAGIC_MISMATCH            0x01
#define DFU_APP_ERROR_UNSUPPORED_HDR_VERSION    0x02
#define DFU_APP_ERROR_INVALID_SIZE              0x03
#define DFU_APP_ERROR_CRC_CALC_FAILED           0x04
#define DFU_APP_ERROR_CRC_APP_HEADER_MISMATCH   0x05
#define DFU_APP_ERROR_CRC_APP_DATA_MISMATCH     0x06

/**
 * struct dfu_app_hdr - SuperDFU bootloader application header
 * @hdr_magic: must contain DFU_APP_HDR_MAGIC_STRING, initialized by the application
 * @hdr_version: must contain DFU_APP_HDR_VERSION, initialized by the application
 * @hdr_crc: CRC32 of header, filled by superdfu-patch.py.
 * @app_size: size in bytes of the app. Filled in by superdfu-patch.py.
 * @app_crc: CRC32 of the application code (excluding struct dfu_app_hdr and struct dfu_app_ftr).
 *            Filled in by superdfu-patch.py.
 * @app_version_major: app major version, filled by app
 * @app_version_minor: app minor version, filled by app
 * @app_name: Name of the application (zero terminated), filled by app
 * @app_watchdog_timeout_s: timeout in seconds before the bootloader watchdog expires, filled by app

 *
 * SuperDFU uses this structure to verify and load your application.
 * To work under the bootloader, your application code must include an instance
 * of this struct and struct dfu_app_ftr (see below) in its application code.
 *
 * Example
 * static struct dfu_app_hdr dfu_app_hdr __attribute__((used,section(DFU_APP_HDR_SECTION_NAME))) = {
 *   .hdr_magic = DFU_APP_HDR_MAGIC_STRING,
 *   .hdr_version = DFU_APP_HDR_VERSION,
 *   .app_version_major = 0,
 *   .app_version_minor = 1,
 *   .app_version_patch = 0,
 *   .app_watchdog_timeout_s = 1,
 *   .app_name = "my app",
 * };
 *
 * static struct dfu_app_ftr dfu_app_ftr __attribute__((used,section(DFU_APP_FTR_SECTION_NAME))) = {
 *    .magic = DFU_APP_FTR_MAGIC_STRING
 * };  *
 *
 */
struct dfu_app_hdr {
	uint8_t hdr_magic[16];
	uint8_t hdr_version;
	uint8_t hdr_reserved[3];
	uint32_t hdr_crc;
	uint32_t app_size;
	uint32_t app_crc;
	uint8_t app_version_major;
	uint8_t app_version_minor;
	uint8_t app_version_patch;
	uint8_t app_watchdog_timeout_s;
	uint8_t app_name[28];
} __packed;

_Static_assert((sizeof(struct dfu_app_hdr) & 3) == 0, "structure size must be a multiple of 4");
_Static_assert(DFU_APP_HDR_SIZE  == sizeof(struct dfu_app_hdr), "structure size mismatches define");
_Static_assert(MCU_VECTOR_TABLE_ALIGNMENT >= sizeof(struct dfu_app_hdr), "structure size must not exceed vector table alignment");

/**
 * struct dfu_app_hdr - SuperDFU bootloader application header
 * @magic: must contain DFU_APP_FTR_MAGIC_STRING, filled by the application
 *
 * This struct together with the header enable superdfu-patch.py to compute the application
 * size and checksums.
 */
struct dfu_app_ftr {
	uint8_t magic[16];
};


int dfu_app_hdr_validate(struct dfu_app_hdr const *hdr);

/**
 * Disables the bootloader watchdog
 *
 * The application must call this in time to disable the bootloader watchdog.
 * Else the device will reset.
 */
static inline void dfu_app_watchdog_disable(void)
{
	WDT->CTRLA.bit.ENABLE = 0;
}
