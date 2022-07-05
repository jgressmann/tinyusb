/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2020-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include <sam_crc32.h>
#include <sam.h>


CRC32_FUNC void sam_crc32_lock(void)
{
#if defined(__SAME51J18A__) || defined(__SAME51J19A__) || defined(__SAME51J20A__) || \
	defined(__SAME54P20A__)
	PAC->WRCTRL.reg = PAC_WRCTRL_KEY_SET | PAC_WRCTRL_PERID(33);
#else
	PAC1->WPSET.reg = 0x2;
#endif
}

CRC32_FUNC int sam_crc32_unlock(void)
{
#if defined(__SAME51J18A__) || defined(__SAME51J19A__) || defined(__SAME51J20A__) || \
	defined(__SAME54P20A__)
	if (!PAC->STATUSB.bit.DSU_) {
		return CRC32E_UNLOCKED;
	}

	// p. 112 of 60001507E.pdf
	// disable DSU protection
	PAC->WRCTRL.reg = PAC_WRCTRL_KEY_CLR | PAC_WRCTRL_PERID(33);

	if (PAC->STATUSB.bit.DSU_) {
		return CRC32E_ACCESS;
	}
#else
	PAC1->WPCLR.reg = 0x2;
#endif

	return CRC32E_NONE;
}

CRC32_FUNC int sam_crc32_update(uint32_t addr, uint32_t bytes,  uint32_t *inout_crc)
{
	DSU->DATA.reg = *inout_crc;

	// yes, both need to be divided by 4 (word size)
	DSU->ADDR.bit.ADDR = addr / 4;
	DSU->LENGTH.bit.LENGTH = bytes / 4;

	// LOG("DSU->ADDR.bit.ADDR %#08lx\n", DSU->ADDR.bit.ADDR);

#if defined(__SAMD21G16A__)
	// Revision A-D can´t compute from RAM without workaround,
	// see DS80000760D-page 15
	if (DSU->DID.bit.REVISION < 4) {
		*((volatile unsigned int*) 0x41007058) &= ~0x30000UL;
	}
#endif

	// start computation
	DSU->CTRL.bit.CRC = 1;

	while (!DSU->STATUSA.bit.DONE);
	DSU->STATUSA.bit.DONE = 1;

#if defined(__SAMD21G16A__)
	// Revision A-D can´t compute from RAM without workaround,
	// see DS80000760D-page 15
	if (DSU->DID.bit.REVISION < 4) {
		*((volatile unsigned int*) 0x41007058) |= 0x20000UL;
	}
#endif

	if (DSU->STATUSA.bit.BERR) {
		DSU->STATUSA.bit.BERR = DSU->STATUSA.bit.BERR;
		return CRC32E_FAILED;
	}

	// fetch computed value
	*inout_crc = DSU->DATA.reg;
	// LOG("CRC %#08lx\n", *result);

	return CRC32E_NONE;
}

