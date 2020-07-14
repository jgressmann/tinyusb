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
#include <inttypes.h>
#include <sam.h>

#if !defined(EEPROM_SIZE)
#   error Define EEPROM_SIZE
#endif

#define EEPROM_START ((uint8_t *const)SEEPROM_ADDR)
#define EEPROM_END ((uint8_t *const)(SEEPROM_ADDR + EEPROM_SIZE))

#define DFU_EEPROM_SIZE 16
#define DFU_EEPROM_START EEPROM_START
#define DFU_EEPROM_END (EEPROM_START + DFU_EEPROM_SIZE)

#define DFU_EEPROM_MAGIC \
	((((uint64_t)'S') << 56) \
	| (((uint64_t)'u') << 48) \
	| (((uint64_t)'p') << 40) \
	| (((uint64_t)'e') << 32) \
	| (((uint64_t)'r') << 24) \
	| (((uint64_t)'D') << 16) \
	| (((uint64_t)'F') << 8) \
	| (((uint64_t)'U') << 0))

typedef union {
	struct {
		uint8_t BTL:1;         /*!< bit:  0 want DFU mode                           */
	} bit;                     /*!< Structure used for bit  access                  */
	uint8_t reg;               /*!< Type      used for register access              */
} dfu_eeprom_reg;

typedef struct dfu_eeprom {
	union {
		uint64_t magic64;
		uint8_t magic8[8];
	} magic;
	dfu_eeprom_reg dfu;
	uint8_t reserved[7];
} dfu_eeprom;

#define DFU_EEPROM ((dfu_eeprom*)DFU_EEPROM_START)





