/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#include <supercan_mcanx.h>


/* need this as macro for #ifdef */
#define _SC_BOARD_CAN_TX_FIFO_SIZE 32


enum {
	// software fifo sizes (set to largest known M_CAN hardware fifo sizes)
	SC_BOARD_CAN_TX_FIFO_SIZE = _SC_BOARD_CAN_TX_FIFO_SIZE,
	SC_BOARD_CAN_RX_FIFO_SIZE = 32,
};

#ifndef MCAN_MESSAGE_RAM_CONFIGURABLE
	#error Define MCAN_MESSAGE_RAM_CONFIGURABLE
#endif

#ifndef MCAN_HW_RX_FIFO_SIZE
	#error Define MCAN_HW_RX_FIFO_SIZE
#endif

#ifndef MCAN_HW_TX_FIFO_SIZE
	#error Define MCAN_HW_TX_FIFO_SIZE
#endif

#if MCAN_HW_TX_FIFO_SIZE > MCAN_HW_RX_FIFO_SIZE
	#error HW TX fifo must not be bigger than RX fifo
#endif

#define MCAN_DEBUG_TXR 1
#define MCAN_DEBUG_GUARD 0

#define TS_LO_MASK ((UINT32_C(1) << M_CAN_TS_COUNTER_BITS) - 1)
#define TS_HI(ts) ((((uint32_t)(ts)) >> (32 - M_CAN_TS_COUNTER_BITS)) & TS_LO_MASK)
#define TS_LO(ts) (((uint32_t)(ts)) & TS_LO_MASK)

#define SPAM 0

#define CANFD_ELEMENT_DATA_SIZE 16

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


struct mcan_tx_fifo_element {
	__IO MCANX_TXBE_0_Type T0;
	__IO MCANX_TXBE_1_Type T1;
	__IO uint32_t data[CANFD_ELEMENT_DATA_SIZE];
};

struct mcan_txe_fifo_element {
	__IO MCANX_TXEFE_0_Type T0;
	__IO MCANX_TXEFE_1_Type T1;
};

struct mcan_rx_fifo_element {
	__IO MCANX_RXF0E_0_Type R0;
	__IO MCANX_RXF0E_1_Type R1;
	__IO uint32_t data[CANFD_ELEMENT_DATA_SIZE];
};

struct mcan_rx_frame {
	MCANX_RXF0E_0_Type R0;
	MCANX_RXF0E_1_Type R1;
	uint32_t ts;
	uint32_t data[CANFD_ELEMENT_DATA_SIZE];
};

struct mcan_txe_frame {
	MCANX_TXEFE_0_Type T0;
	MCANX_TXEFE_1_Type T1;
	uint32_t ts;
};

struct mcan_txq_frame {
	MCANX_TXEFE_0_Type T0;
	MCANX_TXEFE_1_Type T1;
	uint32_t data[CANFD_ELEMENT_DATA_SIZE];
};


struct mcan_can {
#if SUPERCAN_DEBUG && MCAN_DEBUG_GUARD
	uint32_t guard_hdr[16];
#endif
#if MCAN_MESSAGE_RAM_CONFIGURABLE
	CFG_TUSB_MEM_ALIGN struct mcan_tx_fifo_element hw_tx_fifo_ram[MCAN_HW_TX_FIFO_SIZE];
	CFG_TUSB_MEM_ALIGN struct mcan_txe_fifo_element hw_txe_fifo_ram[MCAN_HW_TX_FIFO_SIZE];
	CFG_TUSB_MEM_ALIGN struct mcan_rx_fifo_element hw_rx_fifo_ram[MCAN_HW_RX_FIFO_SIZE];
#else
	struct mcan_tx_fifo_element *hw_tx_fifo_ram;
	struct mcan_txe_fifo_element *hw_txe_fifo_ram;
	struct mcan_rx_fifo_element *hw_rx_fifo_ram;
#endif
	struct mcan_rx_frame rx_fifo[SC_BOARD_CAN_RX_FIFO_SIZE];
	struct mcan_txe_frame txe_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];
#if MCAN_HW_TX_FIFO_SIZE < _SC_BOARD_CAN_TX_FIFO_SIZE
	// queue frames that don't fit in hardware fifo
	struct mcan_txq_frame tx_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];
#endif
	sc_can_bit_timing nm;
	sc_can_bit_timing dt;
	MCanX *m_can;
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
	uint8_t txe_get_index; // NOT an index, uses full range of type
	uint8_t txe_put_index; // NOT an index, uses full range of type
#if MCAN_HW_TX_FIFO_SIZE < _SC_BOARD_CAN_TX_FIFO_SIZE
	uint8_t tx_get_index; // NOT an index, uses full range of type
	uint8_t tx_put_index; // NOT an index, uses full range of type
#endif
#if SUPERCAN_DEBUG && MCAN_DEBUG_TXR
	uint32_t txr; 				// requests from USB, set when in, clear when out
	volatile uint32_t int_txe;	// expected TXEs, set in USB, cleared in IRQ handler
#endif
#if SUPERCAN_DEBUG && MCAN_DEBUG_GUARD
	uint32_t guard_ftr[16];
#endif
};

extern struct mcan_can mcan_cans[SC_BOARD_CAN_COUNT];

extern void mcan_can_init(void);
extern void mcan_can_configure(uint8_t index);
SC_RAMFUNC extern void mcan_can_int(uint8_t index);
#if SUPERCAN_DEBUG && MCAN_DEBUG_GUARD
SC_RAMFUNC extern void mcan_can_verify_guard(uint8_t index);
#endif
