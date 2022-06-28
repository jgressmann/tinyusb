/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2020-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#include <stdint.h>

#include <sam.h>

#if defined(__SAME51J18A__) || defined(__SAME51J19A__) || defined(__SAME51J20A__) || \
	defined(__SAME54P20A__)

#define MCU_NVM_PAGE_SIZE NVMCTRL_PAGE_SIZE // Atmel
#define MCU_NVM_BLOCK_SIZE NVMCTRL_BLOCK_SIZE // Atmel
// 138 interrupts, DS60001507E-page 62
// https://interrupt.memfault.com/blog/how-to-write-a-bootloader-from-scratch

static inline void sam_get_serial_number(uint32_t serial[4])
{
	// DS60001507E-page 60
	serial[0] = *(uint32_t const *)0x008061FC;
	serial[1] = *(uint32_t const *)0x00806010;
	serial[2] = *(uint32_t const *)0x00806014;
	serial[3] = *(uint32_t const *)0x00806018;
}

/* _MUST_ be a macro or a function in RAM */
#define mcu_wdt_set(per) \
	do { \
		WDT->CONFIG.bit.PER = per; \
		WDT->CTRLA.bit.ENABLE = 1; \
	} while (0)

#if defined(__SAME51J18A__)
	#define MCU_NVM_SIZE (1ul<<18)
#elif defined(__SAME51J19A__)
	#define MCU_NVM_SIZE (1ul<<19)
#elif defined(__SAME51J20A__) || defined(__SAME54P20A__ )
	#define MCU_NVM_SIZE (1ul<<20)
#else
	#error Unknown SAME51J chip
#endif

#elif defined(__SAMD21G16A__)

#define MCU_NVM_PAGE_SIZE NVMCTRL_PAGE_SIZE // Atmel
#define MCU_NVM_BLOCK_SIZE NVMCTRL_ROW_SIZE


static inline void sam_get_serial_number(uint32_t serial[4])
{
	// DS60001507E-page 60
	serial[0] = *(uint32_t const *)0x0080A00C;
	serial[1] = *(uint32_t const *)0x0080A040;
	serial[2] = *(uint32_t const *)0x0080A044;
	serial[3] = *(uint32_t const *)0x0080A048;
}


/* _MUST_ be a macro or a function in RAM */
#define mcu_wdt_set(per) \
	do { \
		WDT->CONFIG.bit.PER = per; \
		WDT->CTRL.bit.ENABLE = 1; \
	} while (0)


#if defined(__SAMD21G16A__)
	#define MCU_NVM_SIZE (1ul<<16)
#else
	#error Unknown SAMD21 chip
#endif

#else
	#error "Unsupported chip"
#endif

