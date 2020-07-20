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
#include <string.h>
#include <sam.h>
#include <mcu.h>
#include <tusb.h>

#ifndef MCU_NVM_SIZE
#error Define MCU_NVM_SIZE
#endif

int dfu_app_validate(struct dfu_app_hdr const *hdr)
{
	if (memcmp(DFU_APP_HDR_MAGIC_STRING, hdr->magic, sizeof(hdr->magic))) {
		// magic mismatch
		return DFU_APP_ERROR_MAGIC_MISMATCH;
	}

	if (!hdr->dfu_app_hdr_version || hdr->dfu_app_hdr_version > DFU_APP_HDR_VERSION) {
		return DFU_APP_ERROR_UNSUPPORED_HDR_VERSION;
	}

	if (0 == (hdr->app_size / 4) || hdr->app_size >= MCU_NVM_SIZE) {
		return DFU_APP_ERROR_INVALID_SIZE;
	}

	// p. 112 of 60001507E.pdf
	uint32_t crc = 0;
	uint32_t length = hdr->app_size / 4;
	uint32_t addr = ((uintptr_t)(hdr + 1));

	TU_LOG2("addr %08lx\n", addr);
	TU_LOG2("len %08lx [words]\n", length);
	TU_LOG2("NVMCTRL->CTRLA.reg %04x\n", NVMCTRL->CTRLA.reg);

	MCLK->AHBMASK.bit.DSU_ = 1;
	MCLK->APBBMASK.bit.DSU_ = 1;
	// GCLK->PCHCTRL[DSU_CLK_AHB_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN1 to use GLCK0 -> 120MHz
	// GCLK->PCHCTRL[35].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN; /* setup SERCOM to use GLCK2 -> 60MHz */


	// for (int i = 0; i < 3; ++i) {
	// 	// while (!DSU->STATUSA.bit.DONE);
	// 	DSU->CTRL.bit.SWRST = 1;
	// 	TU_LOG2("DSU CTRL %#02x STATUSA %#02x\n", DSU->CTRL.reg, DSU->STATUSA.reg);
	// 	// while (!DSU->STATUSA.bit.DONE);

	// 	TU_LOG2("start crc\n");

	// 	DSU->LENGTH.bit.LENGTH = DSU_LENGTH_LENGTH(length);
	// 	DSU->ADDR.bit.ADDR = DSU_ADDR_ADDR(addr);
	// 	DSU->DATA.bit.DATA = DSU_DATA_DATA(0xffffffff);

	// 	// start computation
	// 	DSU->CTRL.bit.CRC = 1;

	// 	while (!DSU->STATUSA.bit.DONE);

	// 	TU_LOG2("end crc\n");

	// 	if (!DSU->STATUSA.bit.BERR) {
	// 		crc = 1;
	// 		break;
	// 	}

	// }

	// if (!crc) {
	// 	return DFU_APP_ERROR_CRC_CALC_FAILED;
	// }

	// // fetch computed value
	// crc = DSU->DATA.reg;
	// if (crc != hdr->app_crc) {
	// 	return DFU_APP_ERROR_CRC_VERIFICATION_FAILED;
	// }

	return DFU_APP_ERROR_NONE;
}

