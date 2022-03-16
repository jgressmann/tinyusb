/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#if defined(D5035_01) || defined(SAME54XPLAINEDPRO) || defined(FEATHER_M4_CAN_EXPRESS)

#include <crc32.h>
#include <sam.h>

CRC32_FUNC int crc32f(uint32_t addr, uint32_t bytes, uint32_t flags, uint32_t *result)
{
	int error = CRC32E_NONE;

	if (flags & CRC32E_FLAG_UNLOCK) {
		// // p. 112 of 60001507E.pdf
		// disable DSU protection
		PAC->WRCTRL.reg = PAC_WRCTRL_KEY_CLR | PAC_WRCTRL_PERID(33);
		// LOG("PAC->STATUSB.reg %#08lx\n", PAC->STATUSB.reg);

		if (PAC->STATUSB.bit.DSU_) {
			return CRC32E_ACCESS;
		}
	}

	error = crc32(addr, bytes, result);

	if (flags & CRC32E_FLAG_UNLOCK) {
		PAC->WRCTRL.reg = PAC_WRCTRL_KEY_SET | PAC_WRCTRL_PERID(33);
		// LOG("PAC->STATUSB.reg %#08lx\n", PAC->STATUSB.reg);
	}

	return error;
}


CRC32_FUNC int crc32(uint32_t addr, uint32_t bytes,  uint32_t *result)
{
	DSU->DATA.bit.DATA = 0xffffffff;
	// yes, both need to be divided by 4 (word size)
	DSU->ADDR.bit.ADDR = addr / 4;
	DSU->LENGTH.bit.LENGTH = bytes / 4;

	// LOG("DSU->ADDR.bit.ADDR %#08lx\n", DSU->ADDR.bit.ADDR);


	// start computation
	DSU->CTRL.bit.CRC = 1;

	while (!DSU->STATUSA.bit.DONE);
	DSU->STATUSA.bit.DONE = 1;

	if (DSU->STATUSA.bit.BERR) {
		DSU->STATUSA.bit.BERR = DSU->STATUSA.bit.BERR;
		return CRC32E_FAILED;
	}

	// fetch computed value
	*result = ~DSU->DATA.reg;
	// LOG("CRC %#08lx\n", *result);

	return CRC32E_NONE;
}

#endif
