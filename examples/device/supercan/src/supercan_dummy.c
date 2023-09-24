/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#include <supercan_board.h>

#ifndef SUPERCAN_DUMMY
#	define SUPERCAN_DUMMY 0
#endif

#if SUPERCAN_DUMMY

#include <supercan_debug.h>
#include <tusb.h>
#include <leds.h>
#include <bsp/board.h>

#define M_CAN_NMBT_TQ_MIN            0x0004
#define M_CAN_NMBT_TQ_MAX            0x0181
#define M_CAN_NMBT_BRP_MIN           0x0001
#define M_CAN_NMBT_BRP_MAX           0x0200
#define M_CAN_NMBT_SJW_MIN           0x0001
#define M_CAN_NMBT_SJW_MAX           0x0080
#define M_CAN_NMBT_TSEG1_MIN         0x0002
#define M_CAN_NMBT_TSEG1_MAX         0x0100
#define M_CAN_NMBT_TSEG2_MIN         0x0002
#define M_CAN_NMBT_TSEG2_MAX         0x0080

#define M_CAN_DTBT_TQ_MIN            0x04
#define M_CAN_DTBT_TQ_MAX            0x31
#define M_CAN_DTBT_BRP_MIN           0x01
#define M_CAN_DTBT_BRP_MAX           0x20
#define M_CAN_DTBT_SJW_MIN           0x01
#define M_CAN_DTBT_SJW_MAX           0x10
#define M_CAN_DTBT_TSEG1_MIN         0x01
#define M_CAN_DTBT_TSEG1_MAX         0x20
#define M_CAN_DTBT_TSEG2_MIN         0x01
#define M_CAN_DTBT_TSEG2_MAX         0x10
#define M_CAN_TDCR_TDCO_MAX          0x7f



// NOTE: If you are using CMSIS, the registers can also be
// accessed through CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk
#define HALT_IF_DEBUGGING()                              \
  do {                                                   \
    if ((*(volatile uint32_t *)0xE000EDF0) & (1 << 0)) { \
      __asm("bkpt 1");                                   \
    }                                                    \
} while (0)

typedef struct __attribute__((packed)) ContextStateFrame {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t return_address;
  uint32_t xpsr;
} sContextStateFrame;

#define HARDFAULT_HANDLING_ASM(_x)               \
  __asm volatile(                                \
      "tst lr, #4 \n"                            \
      "ite eq \n"                                \
      "mrseq r0, msp \n"                         \
      "mrsne r0, psp \n"                         \
      "b my_fault_handler_c \n"                  \
                                                 )

// Disable optimizations for this function so "frame" argument
// does not get optimized away
__attribute__((optimize("O0")))
void my_fault_handler_c(sContextStateFrame *frame)
{
	(void)frame;
  // If and only if a debugger is attached, execute a breakpoint
  // instruction so we can take a look at what triggered the fault
  HALT_IF_DEBUGGING();

  // Logic for dealing with the exception. Typically:
  //  - log the fault which occurred for postmortem analysis
  //  - If the fault is recoverable,
  //    - clear errors and return back to Thread Mode
  //  - else
  //    - reboot system
  while (1);
}

void HardFault_Handler(void)
{
  HARDFAULT_HANDLING_ASM();
}

void BusFault_Handler(void)
{
	HARDFAULT_HANDLING_ASM();
}

void MemManage_Handler(void)
{
	HARDFAULT_HANDLING_ASM();
}



static volatile uint8_t *null_ptr_ = NULL;

static const sc_can_bit_timing_range nm_range = {
	.min = {
		.brp = M_CAN_NMBT_BRP_MIN,
		.tseg1 = M_CAN_NMBT_TSEG1_MIN,
		.tseg2 = M_CAN_NMBT_TSEG2_MIN,
		.sjw = M_CAN_NMBT_SJW_MIN,
	},
	.max = {
		.brp = M_CAN_NMBT_BRP_MAX,
		.tseg1 = M_CAN_NMBT_TSEG1_MAX,
		.tseg2 = M_CAN_NMBT_TSEG2_MAX,
		.sjw = M_CAN_NMBT_SJW_MAX,
	},
};

static const sc_can_bit_timing_range dt_range = {
	.min = {
		.brp = M_CAN_DTBT_BRP_MIN,
		.tseg1 = M_CAN_DTBT_TSEG1_MIN,
		.tseg2 = M_CAN_DTBT_TSEG2_MIN,
		.sjw = M_CAN_DTBT_SJW_MIN,
	},
	.max = {
		.brp = M_CAN_DTBT_BRP_MAX,
		.tseg1 = M_CAN_DTBT_TSEG1_MAX,
		.tseg2 = M_CAN_DTBT_TSEG2_MAX,
		.sjw = M_CAN_DTBT_SJW_MAX,
	},
};

static struct can {
	uint8_t txr_buffer[SC_BOARD_CAN_TX_FIFO_SIZE];
	uint8_t txr_get_index; // NOT an index, uses full range of type
	uint8_t txr_put_index; // NOT an index, uses full range of typ
} cans[SC_BOARD_CAN_COUNT];

extern void sc_board_led_set(uint8_t index, bool on)
{
	(void)index;

	board_led_write(on);
}

extern void sc_board_leds_on_unsafe(void)
{
	board_led_write(true);
}

extern void sc_board_init_begin(void)
{
	board_init();

	memset(cans, 0, sizeof(cans));
}

extern void sc_board_init_end(void)
{
	led_blink(0, 2000);
}

__attribute__((noreturn)) extern void sc_board_reset(void)
{
	while (1) {
		memset((void*)null_ptr_, 0xdeadbeef, UINT32_MAX / 2);
	}
}

extern uint16_t sc_board_can_feat_perm(uint8_t index)
{
	(void)index;
	return CAN_FEAT_PERM;
}

extern uint16_t sc_board_can_feat_conf(uint8_t index)
{
	(void)index;
	return CAN_FEAT_CONF;
}

SC_RAMFUNC extern bool sc_board_can_tx_queue(uint8_t index, struct sc_msg_can_tx const * msg)
{
	struct can *can = &cans[index];
	uint8_t pi = can->txr_put_index;
	uint8_t gi = __atomic_load_n(&can->txr_get_index, __ATOMIC_ACQUIRE);
	uint8_t used = pi - gi;
	bool available = used < TU_ARRAY_SIZE(can->txr_buffer);

	if (available) {
		uint8_t txr_put_index = pi % TU_ARRAY_SIZE(can->txr_buffer);

		// store
		can->txr_buffer[txr_put_index] = msg->track_id;

		// mark available
		__atomic_store_n(&can->txr_put_index, pi + 1, __ATOMIC_RELEASE);

		LOG("ch%u queued TXR %u\n", index, msg->track_id);

		sc_can_notify_task_def(index, 1);
	}

	return available;
}


SC_RAMFUNC extern int sc_board_can_retrieve(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	struct can *can = &cans[index];
	int result = 0;
	bool have_data_to_place = false;

	for (bool done = false; !done; ) {
		done = true;
		uint8_t txr_pi = __atomic_load_n(&can->txr_put_index, __ATOMIC_ACQUIRE);

		if (can->txr_get_index != txr_pi) {
			struct sc_msg_can_txr *txr = NULL;
			uint8_t const bytes = sizeof(*txr);

			have_data_to_place = true;


			if ((size_t)(tx_end - tx_ptr) >= bytes) {
				uint8_t const txr_get_index = can->txr_get_index % TU_ARRAY_SIZE(can->txr_buffer);
				done = false;

				txr = (struct sc_msg_can_txr *)tx_ptr;

				tx_ptr += bytes;
				result += bytes;

				txr->flags = 0;
				txr->id = SC_MSG_CAN_TXR;
				txr->len = bytes;
				txr->track_id = cans->txr_buffer[txr_get_index];
				txr->timestamp_us = sc_board_can_ts_wait(index);

				__atomic_store_n(&can->txr_get_index, can->txr_get_index+1, __ATOMIC_RELEASE);

				LOG("ch%u retrievd TXR %u\n", index, txr->track_id);
			}
		}
	}

	if (result > 0) {
		return result;
	}

	return have_data_to_place - 1;
}


extern sc_can_bit_timing_range const* sc_board_can_nm_bit_timing_range(uint8_t index)
{
	(void)index;

	return &nm_range;
}

extern sc_can_bit_timing_range const* sc_board_can_dt_bit_timing_range(uint8_t index)
{
	(void)index;

	return &dt_range;
}

extern void sc_board_can_feat_set(uint8_t index, uint16_t features)
{
	(void)index;
	(void)features;
}

extern void sc_board_can_go_bus(uint8_t index, bool on)
{
	(void)index;
	(void)on;

	if (on) {
		const sc_can_status status = {
			.type = SC_CAN_STATUS_FIFO_TYPE_BUS_STATUS,
			.timestamp_us = sc_board_can_ts_wait(index),
			.bus_state = SC_CAN_STATUS_ERROR_ACTIVE,
		};

		sc_can_status_queue(index, &status);
		sc_can_notify_task_def(index, 1);
	}
}

extern void sc_board_can_nm_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	(void)index;
	(void)bt;
}

extern void sc_board_can_dt_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	(void)index;
	(void)bt;
}

extern uint32_t sc_board_identifier(void)
{
	return 0x12345678;
}

extern void sc_board_can_reset(uint8_t index)
{
	(void)index;
}

#endif // #if SUPERCAN_DUMMY

