/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#include <m_can.h>


#define SAME5X_DEBUG_TXR 0

enum {
	SC_BOARD_CAN_TX_FIFO_SIZE = 32,
	SC_BOARD_CAN_RX_FIFO_SIZE = 64,
};



#define TS_LO_MASK ((UINT32_C(1) << M_CAN_TS_COUNTER_BITS) - 1)
#define TS_HI(ts) ((((uint32_t)(ts)) >> (32 - M_CAN_TS_COUNTER_BITS)) & TS_LO_MASK)
#define TS_LO(ts) (((uint32_t)(ts)) & TS_LO_MASK)

#define SPAM 0

#define CAN_ELEMENT_DATA_SIZE 64

enum {
	CAN_FEAT_PERM = SC_FEATURE_FLAG_TXR,
	CAN_FEAT_CONF = (MSG_BUFFER_SIZE >= 128 ? SC_FEATURE_FLAG_FDF : 0)
					| SC_FEATURE_FLAG_TXP
					| SC_FEATURE_FLAG_EHD
					// not yet implemented
					// | SC_FEATURE_FLAG_DAR
					| SC_FEATURE_FLAG_MON_MODE
					| SC_FEATURE_FLAG_RES_MODE
					| SC_FEATURE_FLAG_EXT_LOOP_MODE,
};


struct can_tx_fifo_element {
	volatile CAN_TXBE_0_Type T0;
	volatile CAN_TXBE_1_Type T1;
	uint8_t data[CAN_ELEMENT_DATA_SIZE];
};

struct can_tx_event_fifo_element {
	volatile CAN_TXEFE_0_Type T0;
	volatile CAN_TXEFE_1_Type T1;
};

struct can_rx_fifo_element {
	volatile CAN_RXF0E_0_Type R0;
	volatile CAN_RXF0E_1_Type R1;
	uint8_t data[CAN_ELEMENT_DATA_SIZE];
};

struct rx_frame {
	volatile CAN_RXF0E_0_Type R0;
	volatile CAN_RXF0E_1_Type R1;
	volatile uint32_t ts;
	uint8_t data[CAN_ELEMENT_DATA_SIZE];
};

struct tx_frame {
	volatile CAN_TXEFE_0_Type T0;
	volatile CAN_TXEFE_1_Type T1;
	volatile uint32_t ts;
};



struct same5x_can {
	CFG_TUSB_MEM_ALIGN struct can_tx_fifo_element tx_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];
	CFG_TUSB_MEM_ALIGN struct can_tx_event_fifo_element tx_event_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];
	CFG_TUSB_MEM_ALIGN struct can_rx_fifo_element rx_fifo[SC_BOARD_CAN_RX_FIFO_SIZE];
	struct rx_frame rx_frames[SC_BOARD_CAN_RX_FIFO_SIZE];
	struct tx_frame tx_frames[SC_BOARD_CAN_TX_FIFO_SIZE];
	sc_can_bit_timing nm;
	sc_can_bit_timing dt;

	Can *m_can;
	IRQn_Type interrupt_id;
	uint32_t nm_us_per_bit;
	uint32_t dt_us_per_bit_factor_shift8;
	uint32_t int_prev_error_ts;
	uint16_t features;
	uint8_t int_prev_bus_state;
	uint8_t int_prev_rx_errors;
	uint8_t int_prev_tx_errors;
	uint8_t int_init_rx_errors;
	uint8_t int_init_tx_errors;
	uint8_t led_status_green;
	uint8_t led_status_red;
	uint8_t led_traffic;
	uint8_t rx_get_index; // NOT an index, uses full range of type
	uint8_t rx_put_index; // NOT an index, uses full range of type
	uint8_t tx_get_index; // NOT an index, uses full range of type
	uint8_t tx_put_index; // NOT an index, uses full range of type
#if SUPERCAN_DEBUG && SAME5X_DEBUG_TXR
	uint32_t txr;
#endif
};

extern struct same5x_can same5x_cans[SC_BOARD_CAN_COUNT];


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

extern void same5x_can_init(void);
extern void same5x_can_configure(uint8_t index);
extern void same5x_init_device_identifier(void);
SC_RAMFUNC extern void same5x_can_int(uint8_t index);

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