/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2020-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include <dfu_app.h>
#include <dfu_debug.h>
#include <string.h>
#include <sam.h>
#include <mcu.h>



int crc32(uint32_t addr, uint32_t bytes, uint32_t *result)
{
	// // p. 112 of 60001507E.pdf
	// disable DSU protection
	PAC->WRCTRL.reg = PAC_WRCTRL_KEY_CLR | PAC_WRCTRL_PERID(33);
	// LOG("PAC->STATUSB.reg %#08lx\n", PAC->STATUSB.reg);

	if (PAC->STATUSB.bit.DSU_) {
		return DFU_APP_ERROR_CRC_CALC_FAILED;
	}

	DSU->DATA.bit.DATA = 0xffffffff;
	// yes, both need to be divided by 4 (word size)
	DSU->ADDR.bit.ADDR = addr / 4;
	DSU->LENGTH.bit.LENGTH = bytes / 4;

	// LOG("DSU->ADDR.bit.ADDR %#08lx\n", DSU->ADDR.bit.ADDR);


	// start computation
	DSU->CTRL.bit.CRC = 1;

	while (!DSU->STATUSA.bit.DONE);
	DSU->STATUSA.bit.DONE = 1;

	PAC->WRCTRL.reg = PAC_WRCTRL_KEY_SET | PAC_WRCTRL_PERID(33);
	// LOG("PAC->STATUSB.reg %#08lx\n", PAC->STATUSB.reg);

	if (DSU->STATUSA.bit.BERR) {
		DSU->STATUSA.bit.BERR = DSU->STATUSA.bit.BERR;
		return DFU_APP_ERROR_CRC_CALC_FAILED;
	}

	// fetch computed value
	*result = ~DSU->DATA.reg;
	// LOG("CRC %#08lx\n", *result);

	return DFU_APP_ERROR_NONE;
}

int dfu_app_tag_validate_tag(struct dfu_app_tag const *tag)
{
	int error;
	uint32_t crc;

	if (memcmp(DFU_APP_TAG_MAGIC_STRING, tag->tag_magic, sizeof(tag->tag_magic))) {
		// magic mismatch
		return DFU_APP_ERROR_MAGIC_MISMATCH;
	}

	if (!tag->tag_version || tag->tag_version > DFU_APP_TAG_VERSION) {
		return DFU_APP_ERROR_CRC_APP_HEADER_MISMATCH;
	}

	if (tag->tag_dev_id != SUPERDFU_DEVID) {
		return DFU_APP_ERROR_DEV_ID_MISMATCH;
	}

	// check header checksum
	struct dfu_app_tag check_tag;
	memcpy(&check_tag, tag, sizeof(check_tag));
	check_tag.tag_crc = 0;

	error = crc32((uint32_t)&check_tag, sizeof(check_tag), &crc);
	if (error) {
		return error;
	}

	if (crc != tag->tag_crc) {
		return DFU_APP_ERROR_CRC_APP_HEADER_MISMATCH;
	}

	return DFU_APP_ERROR_NONE;
}

int dfu_app_tag_validate_app(struct dfu_app_tag const *tag)
{
	int error;
	uint32_t crc;

	error = dfu_app_tag_validate_tag(tag);
	if (error) {
		return error;
	}

	// app size within reasonably limits
	if (0 == (tag->app_size / 4) ||
		0 != (tag->app_size % 4) ||
		MCU_BOOTLOADER_SIZE + tag->app_size >= MCU_NVM_SIZE) {
		return DFU_APP_ERROR_INVALID_SIZE;
	}

	error = crc32(MCU_BOOTLOADER_SIZE, tag->app_size, &crc);
	if (error) {
		return error;
	}

	if (crc != tag->app_crc) {
		return DFU_APP_ERROR_CRC_APP_DATA_MISMATCH;
	}

	return DFU_APP_ERROR_NONE;
}
