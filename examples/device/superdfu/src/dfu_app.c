/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2020-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include <dfu_app.h>
#include <dfu_debug.h>
#include <string.h>
#include <mcu.h>
#include <sam_crc32.h>

#ifndef SUPERDFU_DEV_ID
	#error Define SUPERDFU_DEV_ID
#endif



DFU_APP_FUNC int dfu_app_tag_validate_tag(struct dfu_app_tag const *tag, uint32_t *tag_crc)
{
	int error;

	if (memcmp(DFU_APP_TAG_MAGIC_STRING, tag->tag_magic, sizeof(tag->tag_magic))) {
		// magic mismatch
		return DFU_APP_ERROR_MAGIC_MISMATCH;
	}

	if (!tag->tag_version || tag->tag_version > DFU_APP_TAG_VERSION) {
		return DFU_APP_ERROR_UNSUPPORED_TAG_VERSION;
	}

	if (tag->tag_dev_id != SUPERDFU_DEV_ID) {
		return DFU_APP_ERROR_DEV_ID_MISMATCH;
	}

	// check header checksum
	struct dfu_app_tag check_tag;
	memcpy(&check_tag, tag, sizeof(check_tag));
	check_tag.tag_crc = 0;

	error = sam_crc32((uint32_t)&check_tag, sizeof(check_tag), tag_crc);
	if (error) {
		return DFU_APP_ERROR_CRC_CALC_FAILED;
	}

	if (*tag_crc != tag->tag_crc) {
		return DFU_APP_ERROR_CRC_APP_TAG_MISMATCH;
	}

	return DFU_APP_ERROR_NONE;
}

int dfu_app_tag_validate_app(struct dfu_app_tag const *tag, uint32_t *tag_crc, uint32_t *app_crc)
{
	int error;

	error = dfu_app_tag_validate_tag(tag, tag_crc);
	if (error) {
		return error;
	}

	// app size within reasonably limits
	if (0 == (tag->app_size / 4) ||
		0 != (tag->app_size % 4) ||
		SUPERDFU_BOOTLOADER_SIZE + tag->app_size >= MCU_NVM_SIZE) {
		return DFU_APP_ERROR_INVALID_SIZE;
	}

	error = sam_crc32(SUPERDFU_BOOTLOADER_SIZE, tag->app_size, app_crc);
	if (error) {
		return DFU_APP_ERROR_CRC_CALC_FAILED;
	}

	if (*app_crc != tag->app_crc) {
		return DFU_APP_ERROR_CRC_APP_DATA_MISMATCH;
	}

	return DFU_APP_ERROR_NONE;
}
