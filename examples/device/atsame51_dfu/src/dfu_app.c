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

#include <dfu_app.h>
#include <dfu_debug.h>
#include <string.h>
#include <sam.h>
#include <mcu.h>
#include <tusb.h>

#ifndef MCU_NVM_SIZE
#error Define MCU_NVM_SIZE
#endif

int dfu_app_hdr_validate(struct dfu_app_hdr const *hdr)
{
	if (memcmp(DFU_APP_HDR_MAGIC_STRING, hdr->magic, sizeof(hdr->magic))) {
		// magic mismatch
		return DFU_APP_ERROR_MAGIC_MISMATCH;
	}

	if (!hdr->dfu_app_hdr_version || hdr->dfu_app_hdr_version > DFU_APP_HDR_VERSION) {
		return DFU_APP_ERROR_UNSUPPORED_HDR_VERSION;
	}

	if (0 == (hdr->app_size / 4) ||
		0 != (hdr->app_size % 4) ||
		hdr->app_size >= MCU_NVM_SIZE) {
		return DFU_APP_ERROR_INVALID_SIZE;
	}

	// p. 112 of 60001507E.pdf
	uint32_t length = hdr->app_size;
	uint32_t addr = (((uintptr_t)hdr) + MCU_VECTOR_TABLE_ALIGNMENT);

	// LOG("addr %#08lx len %#08lx\n", addr, length);
	// LOG("PAC->STATUSB.reg %#08lx\n", PAC->STATUSB.reg);

	// boot default
	// MCLK->AHBMASK.bit.DSU_ = 1;
	// MCLK->APBBMASK.bit.DSU_ = 1;


	// disable DSU protection
	PAC->WRCTRL.reg = PAC_WRCTRL_KEY_CLR | PAC_WRCTRL_PERID(33);
	// LOG("PAC->STATUSB.reg %#08lx\n", PAC->STATUSB.reg);

	// yes, both need to be divided by 4 (word size)
	DSU->LENGTH.bit.LENGTH = length / 4;
	DSU->ADDR.bit.ADDR = addr / 4;
	// LOG("DSU->ADDR.bit.ADDR %#08lx\n", DSU->ADDR.bit.ADDR);
	DSU->DATA.bit.DATA = 0xffffffff;

	// start computation
	DSU->CTRL.bit.CRC = 1;

	while (!DSU->STATUSA.bit.DONE);

	PAC->WRCTRL.reg = PAC_WRCTRL_KEY_SET | PAC_WRCTRL_PERID(33);
	// LOG("PAC->STATUSB.reg %#08lx\n", PAC->STATUSB.reg);

	if (DSU->STATUSA.bit.BERR) {
		return DFU_APP_ERROR_CRC_CALC_FAILED;
	}

	// fetch computed value
	uint32_t crc = ~DSU->DATA.reg;
	// LOG("CRC32 computed %#08lx\n", crc);

	if (crc != hdr->app_crc) {
		return DFU_APP_ERROR_CRC_VERIFICATION_FAILED;
	}

	return DFU_APP_ERROR_NONE;
}