/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2020-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#include <stdint.h>
#include <assert.h>
#include <mcu.h>

#define DFU_APP_TAG_SECTION_NAME ".dfutag"
#define DFU_APP_TAG_PTR_SECTION_NAME ".dfutagptr"
#define DFU_APP_TAG_MAGIC_STRING "SuperDFU AT\0\0\0\0\0"

#define DFU_APP_TAG_VERSION 1
#define DFU_APP_TAG_SIZE 0x40
#define DFU_APP_TAG_FLAG_BOOTLOADER 1
#define DFU_APP_TAG_BOM 0x1234

#define DFU_APP_ERROR_NONE                      0x00
#define DFU_APP_ERROR_MAGIC_MISMATCH            0x01
#define DFU_APP_ERROR_UNSUPPORED_TAG_VERSION    0x02
#define DFU_APP_ERROR_INVALID_SIZE              0x03
#define DFU_APP_ERROR_CRC_CALC_FAILED           0x04
#define DFU_APP_ERROR_CRC_APP_TAG_MISMATCH      0x05
#define DFU_APP_ERROR_CRC_APP_DATA_MISMATCH     0x06
#define DFU_APP_ERROR_DEV_ID_MISMATCH           0x07

/**
 * struct dfu_app_tag - SuperDFU bootloader application header
 * @tag_magic: must contain DFU_APP_TAG_MAGIC_STRING, initialized by the application
 * @tag_version: must contain DFU_APP_TAG_VERSION, initialized by the application
 * @tag_flags: typically 0, initialized by the application
 * @tag_dev_id: Unique 32 bit value that must be identical the for bootloader and the app.
 *               This field aims to prevent flashing an application built for another device.
 *               Initialized by the application.
 * @tag_crc: CRC32 of header, filled by superdfu-patch.py.
 * @app_size: size in bytes of the app. Filled in by superdfu-patch.py.
 * @app_crc: CRC32 of the application code (excluding struct dfu_app_tag).
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
 * static struct dfu_app_tag dfu_app_tag __attribute__((used,section(DFU_APP_HDR_SECTION_NAME))) = {
 *   .tag_magic = DFU_APP_TAG_MAGIC_STRING,
 *   .tag_version = DFU_APP_TAG_VERSION,
 *   .tag_flags = 0,
 *   .tag_dev_id = 0xdeadbeef,
 *   .app_version_major = 0,
 *   .app_version_minor = 1,
 *   .app_version_patch = 0,
 *   .app_watchdog_timeout_s = 1,
 *   .app_name = "my app",
 * };
 *
 * static struct dfu_app_tag* dfu_app_tag_ptr __attribute__((used,section(DFU_APP_TAG_PTR_SECTION_NAME))) = &dfu_app_tag;
 */
struct dfu_app_tag {
	uint8_t tag_magic[16];
	uint8_t tag_version;
	uint8_t tag_flags;
	uint16_t tag_bom;
	uint32_t tag_dev_id;
	uint32_t tag_crc;
	uint32_t app_size;
	uint32_t app_crc;
	uint8_t app_version_major;
	uint8_t app_version_minor;
	uint8_t app_version_patch;
	uint8_t app_watchdog_timeout_s;
	uint8_t app_name[24];
} __packed;

_Static_assert((sizeof(struct dfu_app_tag) & 3) == 0, "structure size must be a multiple of 4");
_Static_assert(DFU_APP_TAG_SIZE == sizeof(struct dfu_app_tag), "structure size mismatches define");


/**
 * Validate the dfu app tag
 */
int dfu_app_tag_validate_tag(struct dfu_app_tag const *tag);

/**
 * Validate the dfu app tag and the application
 */
int dfu_app_tag_validate_app(struct dfu_app_tag const *tag);

/**
 * Disables the bootloader watchdog
 *
 * The application must call this in time to disable the bootloader watchdog.
 * Else the device will reset.
 */
static inline void dfu_app_watchdog_disable(void)
{
	/* Don't write into clear without the watchdog being active else a system reset ensues. */
	if (WDT->CTRLA.bit.ENABLE) {
		/* Without this AND without LOG=2 (=> TU_LOG2), the watchdog will never be cleared.
		 * Any takers as to why?
	 	*/
		WDT->CLEAR.reg = 0xa5;
		while (WDT->SYNCBUSY.bit.CLEAR);
		WDT->CTRLA.bit.ENABLE = 0;
	}
}

