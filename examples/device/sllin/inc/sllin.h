/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Jean Gressmann <jean@0x42.de>
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

enum {
	// use CAN ID free bits to encode information about the frame
	SLLIN_ID_FLAG_ENHANCED_CHECKSUM = 0x040, // enhanced checksum
	SLLIN_ID_FLAG_FOREIGN =           0x080, // use CAN ID free bits to encode an frame unknown to the node
	SLLIN_ID_FLAG_BUS_SLEEP =         0x100, // use CAN ID free bits to encode bus sleep
	SLLIN_ID_FLAG_BUS_WAKE_UP =       0x200, // use CAN ID free bits to encode bus wake up
	SLLIN_ID_FLAG_MASTER_TX =         0x400, // use CAN ID free bits to master data frame

};

static inline uint8_t sllin_id_to_pid(uint8_t id)
{
	static const uint8_t map[] = {
		0x80, 0xC1, 0x42, 0x03, 0xC4, 0x85, 0x06, 0x47,
		0x08, 0x49, 0xCA, 0x8B, 0x4C, 0x0D, 0x8E, 0xCF,
		0x50, 0x11, 0x92, 0xD3, 0x14, 0x55, 0xD6, 0x97,
		0xD8, 0x99, 0x1A, 0x5B, 0x9C, 0xDD, 0x5E, 0x1F,
		0x20, 0x61, 0xE2, 0xA3, 0x64, 0x25, 0xA6, 0xE7,
		0xA8, 0xE9, 0x6A, 0x2B, 0xEC, 0xAD, 0x2E, 0x6F,
		0xF0, 0xB1, 0x32, 0x73, 0xB4, 0xF5, 0x76, 0x37,
		0x78, 0x39, 0xBA, 0xFB, 0x3C, 0x7D, 0xFE, 0xBF,
	};

	return map[id & 0x3f];
}

static inline uint8_t sllin_pid_to_id(uint8_t pid)
{
	return pid & 0x3f;
}

#define sllin_crc_start() 0
#define sllin_crc_update1(crc, byte) (crc + byte)


static inline uint_least16_t sllin_crc_update(uint_least16_t crc, uint8_t const * data, unsigned bytes)
{
	for (unsigned i = 0; i < bytes; ++i) {
		crc += data[i];
	}

	return crc;
}

static inline uint8_t sllin_crc_finalize(uint_least16_t crc)
{
	unsigned factor = crc / 256;
	crc -= factor * 255;

	return (~crc) & 0xff;
}
