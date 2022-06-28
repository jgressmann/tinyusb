/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#include <stdint.h>

#define CRC32E_NONE          0
#define CRC32E_ACCESS       -1
#define CRC32E_FAILED       -2
#define CRC32E_UNLOCKED     -3

#ifndef CRC32_FUNC
	#define CRC32_FUNC
#endif


/* Locks CRC32 function in DSU */
CRC32_FUNC void sam_crc32_lock(void);
/* Unlocks CRC32 function in DSU */
CRC32_FUNC int sam_crc32_unlock(void);

#define sam_crc32_init() (0xffffffff)
#define sam_crc32_finalize(crc) ~(crc)

CRC32_FUNC int sam_crc32_update(uint32_t addr, uint32_t bytes, uint32_t* inout_crc);

CRC32_FUNC static inline int sam_crc32(uint32_t addr, uint32_t bytes, uint32_t* out_crc)
{
	int error;

	*out_crc = sam_crc32_init();
	error = sam_crc32_update(addr, bytes, out_crc);
	*out_crc = sam_crc32_finalize(*out_crc);

	return error;
}