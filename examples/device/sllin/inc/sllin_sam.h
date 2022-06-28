/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once


#include <sam.h>


enum {
	SLAVE_PROTO_STEP_RX_BREAK = 0,
	SLAVE_PROTO_STEP_RX_SYNC,
	SLAVE_PROTO_STEP_RX_PID,
	SLAVE_PROTO_STEP_TX_DATA,
	SLAVE_PROTO_STEP_RX_DATA,

	MASTER_PROTO_STEP_TX_BREAK = 0,
	MASTER_PROTO_STEP_TX_SYNC,
	MASTER_PROTO_STEP_FINISHED,

	MASTER_PROTO_TX_BREAK_ONLY_PID = 0xff, // any non-valid PID will do

	TIMER_TYPE_SLEEP = 0,
	TIMER_TYPE_BREAK,
	TIMER_TYPE_HIGH,
	TIMER_TYPE_SOF,
	TIMER_TYPE_DATA,
};
struct slave {
	volatile uint64_t slave_frame_enabled;
	sllin_queue_element elem;
	uint32_t sleep_timeout_us;
	uint32_t sleep_elapsed_us;
	uint16_t data_timeout_us;
	uint8_t slave_proto_step;
	uint8_t slave_tx_offset;
	uint8_t slave_rx_offset;
#if SLLIN_DEBUG
	uint8_t rx_byte;
#endif
};
struct master {
	uint16_t break_timeout_us;
	uint16_t high_timeout_us;
	uint8_t busy;
	uint8_t proto_step;
	uint8_t pid;
};
struct sam_lin {
	Sercom* const sercom;
	Tc* const timer;
	struct slave slave;
	struct master master;
	IRQn_Type const timer_irq;
	uint16_t sof_timeout_us;
	uint16_t baud;
	uint8_t const rx_port_pin_mux;            // (GROUP << 5) | PIN
	uint8_t const master_slave_port_pin_mux;  // set for master, clear for slave
	uint8_t const led_status_green;
	uint8_t const led_status_red;
	uint8_t bus_state;
	uint8_t bus_error;
	uint8_t timer_type;
};

extern struct sam_lin sam_lins[SLLIN_BOARD_LIN_COUNT];

SLLIN_RAMFUNC void sam_lin_usart_int(uint8_t index);
SLLIN_RAMFUNC void sam_lin_timer_int(uint8_t index);


uint32_t sam_init_device_identifier(uint32_t const serial_number[4]);
void sam_lin_init_once(void);

