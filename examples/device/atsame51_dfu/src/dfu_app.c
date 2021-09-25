/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020-2021 Jean Gressmann <jean@0x42.de>
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

#include <dfu_app.h>
#include <dfu_debug.h>
#include <string.h>
#include <sam.h>
#include <mcu.h>

#ifndef MCU_NVM_SIZE
#error Define MCU_NVM_SIZE
#endif



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

int dfu_app_hdr_validate_hdr(struct dfu_app_hdr const *hdr)
{
	int error;
	uint32_t crc;

	if (memcmp(DFU_APP_HDR_MAGIC_STRING, hdr->hdr_magic, sizeof(hdr->hdr_magic))) {
		// magic mismatch
		return DFU_APP_ERROR_MAGIC_MISMATCH;
	}

	if (!hdr->hdr_version || hdr->hdr_version > DFU_APP_HDR_VERSION) {
		return DFU_APP_ERROR_CRC_APP_HEADER_MISMATCH;
	}

	// check header checksum
	struct dfu_app_hdr check_hdr;
	memcpy(&check_hdr, hdr, sizeof(check_hdr));
	check_hdr.hdr_crc = 0;

	error = crc32((uint32_t)&check_hdr, sizeof(check_hdr), &crc);
	if (error) {
		return error;
	}

	if (crc != hdr->hdr_crc) {
		return DFU_APP_ERROR_CRC_APP_HEADER_MISMATCH;
	}

	return DFU_APP_ERROR_NONE;
}

int dfu_app_hdr_validate_app(struct dfu_app_hdr const *hdr)
{
	int error;
	uint32_t crc;

	error = dfu_app_hdr_validate_hdr(hdr);
	if (error) {
		return error;
	}

	// app size within reasonably limits
	uint32_t app_start = (uint32_t)(((uintptr_t)hdr) + MCU_VECTOR_TABLE_ALIGNMENT);
	if (0 == (hdr->app_size / 4) ||
		0 != (hdr->app_size % 4) ||
		app_start + hdr->app_size >= MCU_NVM_SIZE) {
		return DFU_APP_ERROR_INVALID_SIZE;
	}

	error = crc32(app_start, hdr->app_size, &crc);
	if (error) {
		return error;
	}

	if (crc != hdr->app_crc) {
		return DFU_APP_ERROR_CRC_APP_DATA_MISMATCH;
	}

	return DFU_APP_ERROR_NONE;
}
