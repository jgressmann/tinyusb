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

#pragma once

#include <stdint.h>

#if defined(__SAME51J19A__)

#include <sam.h>


#define MCU_NVM_SIZE (1ul<<19)
#define MCU_NVM_PAGE_SIZE NVMCTRL_PAGE_SIZE // Atmel
#define MCU_NVM_BLOCK_SIZE NVMCTRL_BLOCK_SIZE // Atmel
#define MCU_VECTOR_TABLE_ALIGNMENT 1024 // https://interrupt.memfault.com/blog/how-to-write-a-bootloader-from-scratch

static inline void same51_get_serial_number(uint32_t serial[4])
{
	serial[0] = *(uint32_t const *)0x008061FC;
	serial[1] = *(uint32_t const *)0x00806010;
	serial[2] = *(uint32_t const *)0x00806014;
	serial[3] = *(uint32_t const *)0x00806018;
}




#endif // defined(__SAME51J19A__)

