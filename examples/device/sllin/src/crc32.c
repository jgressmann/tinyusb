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
 */

#include <crc32.h>
#include <sam.h>

CRC32_FUNC int crc32f(uint32_t addr, uint32_t bytes, uint32_t flags, uint32_t *result)
{
	int error = CRC32E_NONE;

	if (flags & CRC32E_FLAG_UNLOCK) {
		// disable DSU protection
#if defined(__SAME51J18A__) || defined(__SAME51J19A__) || defined(__SAME51J20A__) || \
	defined(__SAME54P20A__)
		// // p. 112 of 60001507E.pdf
		// disable DSU protection
		PAC->WRCTRL.reg = PAC_WRCTRL_KEY_CLR | PAC_WRCTRL_PERID(33);
		// LOG("PAC->STATUSB.reg %#08lx\n", PAC->STATUSB.reg);

		if (PAC->STATUSB.bit.DSU_) {
			return CRC32E_ACCESS;
		}
#else
		PAC1->WPCLR.reg = 0x2;
#endif
	}

	error = crc32(addr, bytes, result);

	if (flags & CRC32E_FLAG_UNLOCK) {
		// enable DSU protection
#if defined(__SAME51J18A__) || defined(__SAME51J19A__) || defined(__SAME51J20A__) || \
	defined(__SAME54P20A__)
		PAC->WRCTRL.reg = PAC_WRCTRL_KEY_SET | PAC_WRCTRL_PERID(33);
#else
		PAC1->WPSET.reg = 0x2;
#endif
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

