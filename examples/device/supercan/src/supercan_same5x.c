/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2023 Jean Gressmann <jean@0x42.de>
 *
 */

#include <supercan_board.h>

#if D5035_01 || SAME54XPLAINEDPRO || FEATHER_M4_CAN_EXPRESS || LONGAN_CANBED_M4

#include <supercan_debug.h>

#include <sam_crc32.h>
#include <mcu.h>
#include <tusb.h>

static uint32_t device_identifier;

extern uint32_t sc_board_identifier(void)
{
	return device_identifier;
}

void same5x_init_device_identifier(void)
{
	uint32_t serial_number[4];
	int error = CRC32E_NONE;

	sam_get_serial_number(serial_number);

#if SUPERCAN_DEBUG
	LOG("SAM serial number %08x%08x%08x%08x\n", serial_number[0], serial_number[1], serial_number[2], serial_number[3]);
#endif

#if TU_LITTLE_ENDIAN == TU_BYTE_ORDER
	// swap integers so they have printf layout
	serial_number[0] = __builtin_bswap32(serial_number[0]);
	serial_number[1] = __builtin_bswap32(serial_number[1]);
	serial_number[2] = __builtin_bswap32(serial_number[2]);
	serial_number[3] = __builtin_bswap32(serial_number[3]);
#endif

	error = sam_crc32((uint32_t)serial_number, 16, &device_identifier);
	if (unlikely(error)) {
		device_identifier = serial_number[0];
		LOG("ERROR: failed to compute CRC32: %d. Using fallback device identifier\n", error);
	}

#if SUPERCAN_DEBUG
	LOG("device identifier %08x\n", device_identifier);
#endif
}

__attribute__((noreturn)) extern void sc_board_reset(void)
{
	NVIC_SystemReset();
	__unreachable();
}


#endif // supported board
