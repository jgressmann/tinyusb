/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
 *
 */


#pragma once

#include <stdint.h>

enum {
	// device <-> host, EFF data frame
	SLLIN_ID_FLAG_CRC_SHIFT = 0x08,
	SLLIN_ID_FLAG_CRC_MASK = 0xff,

	// device -> host, EFF data frame
	SLLIN_ID_FLAG_LIN_ERROR_SYNC =          0x00010000, //< bad sync field (not 0x55)
	SLLIN_ID_FLAG_LIN_ERROR_PID =           0x00020000, //< bad PID received, CAN ID carries recovered ID
	SLLIN_ID_FLAG_LIN_ERROR_FORM =          0x00040000, //< bit error in some fixed part of the frame e.g. start bit wasn't zero, stop bit wasn't 1, ...
	SLLIN_ID_FLAG_LIN_ERROR_TRAILING =      0x00080000, //< more than 8 byte plus checksum on bus
	SLLIN_ID_FLAG_FRAME_FOREIGN =           0x00100000, //< foreign node answered this frame

	// host -> device, EFF data frame
	SLLIN_ID_FLAG_FRAME_ENABLE =            0x00100000,
	SLLIN_ID_FLAG_FRAME_STORE =             0x00200000,
	SLLIN_ID_FLAG_FRAME_CRC_COMP_SHIFT =    0x10,
	SLLIN_ID_FLAG_FRAME_CRC_COMP_MASK =     0x03,
	SLLIN_ID_FLAG_FRAME_CRC_COMP_NONE =     0x00,
	SLLIN_ID_FLAG_FRAME_CRC_COMP_CLASSIC =  0x01,
	SLLIN_ID_FLAG_FRAME_CRC_COMP_ENHANCED = 0x02,


	// device -> host, EFF data frame
	// bus status
	SLLIN_ID_FLAG_BUS_STATE_FLAG =     0x10000000, //< bus mode
	SLLIN_ID_FLAG_BUS_STATE_MASK =     0x03, //<
	SLLIN_ID_FLAG_BUS_STATE_SHIFT =    0x00, //<
	SLLIN_ID_FLAG_BUS_STATE_ASLEEP =   0x00, //< bus is asleep
	SLLIN_ID_FLAG_BUS_STATE_AWAKE =    0x01, //< bus is awake
	SLLIN_ID_FLAG_BUS_STATE_ERROR =    0x02, //< bus is in error state

	// bus error (permanent)
	SLLIN_ID_FLAG_BUS_ERROR_FLAG =        	0x08000000, //< permanaent error on the bus
	SLLIN_ID_FLAG_BUS_ERROR_MASK =        	0x03, //<
	SLLIN_ID_FLAG_BUS_ERROR_SHIFT =        	0x02, //<
	SLLIN_ID_FLAG_BUS_ERROR_NONE =  	    0x00, //< no error
	SLLIN_ID_FLAG_BUS_ERROR_SHORT_TO_GND =  0x01, //< LIN data line shorted to GND
	SLLIN_ID_FLAG_BUS_ERROR_SHORT_TO_VBAT = 0x02, //< LIN data line shorted to VBAT

	// host -> device, SFF RTR frame
	SLLIN_ID_FLAG_BUS_BREAK = 0x100, //< send BREAK w/o SYNC, PID
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
	uint_least16_t factor = crc / 256;

	crc -= factor * 255;

	return (~crc) & 0xff;
}
