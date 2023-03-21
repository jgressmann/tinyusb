/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2023 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#define SUPERCAN_MCAN 1
#define MCAN_MESSAGE_RAM_CONFIGURABLE 1
#define MCAN_HW_RX_FIFO_SIZE 64
#define MCAN_HW_TX_FIFO_SIZE 32

#define SKIP_INTEGER_LITERALS

#include <supercan_mcan.h>


SC_RAMFUNC static inline void sc_board_can_ts_request(uint8_t index)
{
	uint8_t reg;
	(void)index;

	reg = __atomic_load_n(&TC0->COUNT32.CTRLBSET.reg, __ATOMIC_ACQUIRE);

	while (1) {
		uint8_t cmd = reg & TC_CTRLBSET_CMD_Msk;

		SC_DEBUG_ASSERT(cmd == TC_CTRLBSET_CMD_READSYNC || cmd == TC_CTRLBSET_CMD_NONE);
		if (cmd == TC_CTRLBSET_CMD_READSYNC) {
			break;
		}

		if (likely(__atomic_compare_exchange_n(
			&TC0->COUNT32.CTRLBSET.reg,
			&reg,
			TC_CTRLBSET_CMD_READSYNC,
			false, /* weak? */
			__ATOMIC_RELEASE,
			__ATOMIC_ACQUIRE))) {
				break;
			}

	}
}

SC_RAMFUNC extern uint32_t sc_board_can_ts_wait(uint8_t index);

extern void same5x_init_device_identifier(void);
extern void same5x_can_init(void);

static inline void same5x_enable_cache(void)
{
	// DS60001507E-page 83
	if (!CMCC->SR.bit.CSTS) {
		CMCC->CTRL.bit.CEN = 1;
	}
}

#define same5x_counter_1MHz_request_current_value() do { TC0->COUNT32.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_READSYNC_Val; } while (0)
#define same5x_counter_1MHz_is_current_value_ready() ((__atomic_load_n(&TC0->COUNT32.CTRLBSET.reg, __ATOMIC_ACQUIRE) & TC_CTRLBSET_CMD_Msk) == TC_CTRLBSET_CMD_NONE)
#define same5x_counter_1MHz_read_unsafe() (TC0->COUNT32.COUNT.reg & SC_TS_MAX)



#define same5x_counter_1MHz_wait_for_current_value() \
	({ \
		while (!same5x_counter_1MHz_is_current_value_ready()); \
		uint32_t counter = same5x_counter_1MHz_read_unsafe(); \
		counter; \
	})


#define sc_board_can_ts_request(index) same5x_counter_1MHz_request_current_value()
#define sc_board_can_ts_wait(index) same5x_counter_1MHz_wait_for_current_value()