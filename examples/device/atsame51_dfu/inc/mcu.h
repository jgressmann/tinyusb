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

#pragma once

#include <stdint.h>

#if defined(__SAME51J18A__) || defined(__SAME51J19A__) || defined(__SAME51J20A__) || \
	defined(__SAME54P20A__)


#include <sam.h>

#define MCU_BOOTLOADER_SIZE 0x4000
#define MCU_NVM_PAGE_SIZE NVMCTRL_PAGE_SIZE // Atmel
#define MCU_NVM_BLOCK_SIZE NVMCTRL_BLOCK_SIZE // Atmel
// 138 interrupts, DS60001507E-page 62
// https://interrupt.memfault.com/blog/how-to-write-a-bootloader-from-scratch
#define MCU_VECTOR_TABLE_ALIGNMENT 1024

static inline void same51_get_serial_number(uint32_t serial[4])
{
	// DS60001507E-page 60
	serial[0] = *(uint32_t const *)0x008061FC;
	serial[1] = *(uint32_t const *)0x00806010;
	serial[2] = *(uint32_t const *)0x00806014;
	serial[3] = *(uint32_t const *)0x00806018;
}


static inline int mcu_nvm_boot_bank_index(void)
{
	return NVMCTRL->STATUS.bit.AFIRST ? 0 : 1;
}

#if defined(__SAME51J18A__)
	#define MCU_NVM_SIZE (1ul<<18)
#elif defined(__SAME51J19A__)
	#define MCU_NVM_SIZE (1ul<<19)
#elif defined(__SAME51J20A__) || defined(__SAME54P20A__ )
	#define MCU_NVM_SIZE (1ul<<20)
#else
	#error Unknown SAME51J chip
#endif

#endif // defined(__SAME51J*A__)

