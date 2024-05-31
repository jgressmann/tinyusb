/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#include <supercan_board.h>

#ifndef SUPERCAN_MCAN
	#define SUPERCAN_MCAN 0
#endif

#if SUPERCAN_MCAN

#include <supercan_debug.h>

#include <m_can.h>
#include <leds.h>
#include <string.h>
#include <tusb.h>

#define MCAN_CAN_GUARD_HDR 0xa5a5a5a5
#define MCAN_CAN_GUARD_MID 0xfefefefe
#define MCAN_CAN_GUARD_FTR 0x5a5a5a5a


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


struct mcan_can mcan_cans[SC_BOARD_CAN_COUNT];

void mcan_can_init(void)
{
	memset(mcan_cans, 0, sizeof(mcan_cans));

	for (size_t i = 0; i < TU_ARRAY_SIZE(mcan_cans); ++i) {
		struct mcan_can *can = &mcan_cans[i];

#if SUPERCAN_DEBUG && MCAN_DEBUG_GUARD
		for (size_t j = 0; j < TU_ARRAY_SIZE(can->guard_hdr); ++j) {
			can->guard_hdr[j] = MCAN_CAN_GUARD_HDR;
		}

		for (size_t j = 0; j < TU_ARRAY_SIZE(can->guard_fifo); ++j) {
			can->guard_fifo[j] = MCAN_CAN_GUARD_MID;
		}

		for (size_t j = 0; j < TU_ARRAY_SIZE(can->guard_ftr); ++j) {
			can->guard_ftr[j] = MCAN_CAN_GUARD_FTR;
		}
#endif
		can->features = CAN_FEAT_PERM;

		// init bit timings so we can always safely compute the bit rate (see can_on)
		can->nm = sc_board_can_nm_bit_timing_range((uint8_t)i)->min;
		can->dt = sc_board_can_dt_bit_timing_range((uint8_t)i)->min;
	}
}

void mcan_can_configure(uint8_t index)
{
	struct mcan_can *c = &mcan_cans[index];
	MCanX *can = c->m_can;

	m_can_conf_begin(can);

	can->CCCR.bit.EFBI = 1; // enable edge filtering
	can->CCCR.bit.BRSE = 1; // enable CAN-FD bitrate switching (only effective in CAN-FD mode if configured)

	if (c->features & SC_FEATURE_FLAG_MON_MODE) {
		can->CCCR.bit.MON = 1;
		can->CCCR.bit.TEST = 0;
		can->CCCR.bit.ASM = 0;
		can->TEST.bit.LBCK = 0;
	} else if (c->features & SC_FEATURE_FLAG_RES_MODE) {
		can->CCCR.bit.MON = 0;
		can->CCCR.bit.TEST = 0;
		can->CCCR.bit.ASM = 1;
		can->TEST.bit.LBCK = 0;
	} else if (c->features & SC_FEATURE_FLAG_EXT_LOOP_MODE) {
		can->CCCR.bit.MON = 0;
		can->CCCR.bit.TEST = 1;
		can->CCCR.bit.ASM = 0;
		can->TEST.bit.LBCK = 1;
	} else {
		can->CCCR.bit.MON = 0;
		can->CCCR.bit.TEST = 0;
		can->CCCR.bit.ASM = 0;
		can->TEST.bit.LBCK = 0;
	}

	can->CCCR.bit.FDOE = (c->features & SC_FEATURE_FLAG_FDF) == SC_FEATURE_FLAG_FDF;
	can->CCCR.bit.PXHD = (c->features & SC_FEATURE_FLAG_EHD) == SC_FEATURE_FLAG_EHD;
	can->CCCR.bit.DAR = (c->features & SC_FEATURE_FLAG_DAR) == SC_FEATURE_FLAG_DAR; // disable automatic retransmission
	can->CCCR.bit.TXP = (c->features & SC_FEATURE_FLAG_TXP) == SC_FEATURE_FLAG_TXP;  // enable tx pause

	LOG("MON=%u TEST=%u ASM=%u PXHD=%u FDOE=%u BRSE=%u DAR=%u TXP=%u\n",
		can->CCCR.bit.MON, can->CCCR.bit.TEST, can->CCCR.bit.ASM,
		can->CCCR.bit.PXHD, can->CCCR.bit.FDOE, can->CCCR.bit.BRSE,
		can->CCCR.bit.DAR, can->CCCR.bit.TXP
	);

#if SUPERCAN_DEBUG
	LOG("ch%u ", index);
	sc_can_log_bit_timing(&c->nm, "NM");
	LOG("ch%u ", index);
	sc_can_log_bit_timing(&c->dt, "DT");
#endif

	// reset default TSCV.TSS = 0 (= value always 0)
	//can->TSCC.reg = MCANX_TSCC_TSS_ZERO; // time stamp counter in CAN bittime

	// NOTE: this needs to be on, else we might actually wrap TC0/1
	// which would lead to mistallying of time on the host
	can->TSCC.reg = MCANX_TSCC_TSS_INC;
	// reset default TOCC.ETOC = 0 (disabled)
	// can->TOCC.reg = MCANX_TOCC_TOP(0xffff) | MCANX_TOCC_TOS(0); // Timeout Counter disabled, Reset-default
	can->NBTP.reg = MCANX_NBTP_NSJW(c->nm.sjw-1)
			| MCANX_NBTP_NBRP(c->nm.brp-1)
			| MCANX_NBTP_NTSEG1(c->nm.tseg1-1)
			| MCANX_NBTP_NTSEG2(c->nm.tseg2-1);
	can->DBTP.reg = MCANX_DBTP_DBRP(c->dt.brp-1)
			| MCANX_DBTP_DTSEG1(c->dt.tseg1-1)
			| MCANX_DBTP_DTSEG2(c->dt.tseg2-1)
			| MCANX_DBTP_DSJW(c->dt.sjw-1)
			| (sc_bitrate(c->dt.brp, c->dt.tseg1, c->dt.tseg2) >= 1000000) * MCANX_DBTP_TDC; // enable TDC for bitrates >= 1MBit/s

	// transmitter delay compensation offset
	// can->TDCR.bit.TDCO = tu_min8((1 + c->dtbt_tseg1 + c->dtbt_tseg2) / 2, M_CAN_TDCR_TDCO_MAX);
	can->TDCR.bit.TDCO = tu_min8((1 + c->dt.tseg1 - c->dt.tseg2 / 2), M_CAN_TDCR_TDCO_MAX);
	can->TDCR.bit.TDCF = tu_min8((1 + c->dt.tseg1 + c->dt.tseg2 / 2), M_CAN_TDCR_TDCO_MAX);

#if MCAN_MESSAGE_RAM_CONFIGURABLE
	// tx fifo
	can->TXBC.reg = MCANX_TXBC_TBSA((uint32_t) c->hw_tx_fifo_ram) | MCANX_TXBC_TFQS(MCAN_HW_TX_FIFO_SIZE);
	// tx event fifo
	can->TXEFC.reg = MCANX_TXEFC_EFSA((uint32_t) c->hw_txe_fifo_ram) | MCANX_TXEFC_EFS(MCAN_HW_TX_FIFO_SIZE);
	// rx fifo0
	can->RXF0C.reg = MCANX_RXF0C_F0SA((uint32_t) c->hw_rx_fifo_ram) | MCANX_RXF0C_F0S(MCAN_HW_RX_FIFO_SIZE);

	// configure for max message size
	can->TXESC.reg = MCANX_TXESC_TBDS_DATA64;
	//  | MCANX_RXF0C_F0OM; // FIFO 0 overwrite mode
	can->RXESC.reg = MCANX_RXESC_RBDS_DATA64 + MCANX_RXESC_F0DS_DATA64;
#endif

	// wanted interrupts
	can->IE = 0
		| MCANX_IR_TSW    // time stamp counter wrap
		| MCANX_IR_BO     // bus off
		| MCANX_IR_EW     // error warning
		| MCANX_IR_EP     // error passive
		| MCANX_IR_RF0N   // new message in rx fifo0
		| MCANX_IR_RF0L   // message lost b/c fifo0 was full
		| MCANX_IR_PEA    // proto error in arbitration phase
		| MCANX_IR_PED    // proto error in data phase
		// | MCANX_IE_ELOE   // error logging overflow
	 	| MCANX_IR_TEFN   // new message in tx event fifo
		| MCANX_IR_MRAF   // message RAM access failure
#if defined(MCANX_IR_BEU) && MCANX_IR_BEU
		| MCANX_IR_BEU    // bit error uncorrected, sets CCCR.INIT
#endif
#if defined(MCANX_IR_BEC) && MCANX_IR_BEC
		| MCANX_IR_BEC    // bit error corrected
#endif
	;

	m_can_conf_end(can);
}

SC_RAMFUNC static inline uint8_t map_error_code(uint8_t value)
{
	static const uint8_t table[8] = {
		SC_CAN_ERROR_NONE,
		SC_CAN_ERROR_STUFF,
		SC_CAN_ERROR_FORM,
		SC_CAN_ERROR_ACK,
		SC_CAN_ERROR_BIT1,
		SC_CAN_ERROR_BIT0,
		SC_CAN_ERROR_CRC,
		0xff
	};

	return table[value & 7];
}

SC_RAMFUNC static void can_poll(uint8_t index, uint32_t * const events, uint32_t tsc);



SC_RAMFUNC static void can_int_update_status(uint8_t index, uint32_t* const events, uint32_t tsc)
{
	SC_DEBUG_ASSERT(events);

	struct mcan_can *can = &mcan_cans[index];
	uint8_t current_bus_state = 0;
	MCANX_PSR_Type current_psr = can->m_can->PSR; // always read, sets NC
	MCANX_ECR_Type current_ecr = can->m_can->ECR; // always read, clears CEL
	sc_can_status status;
	uint8_t rec = current_ecr.bit.REC;
	uint8_t tec = current_ecr.bit.TEC;

	// M_CAN keeps REC and TEC over go off bus go on bus sequence
	// This is fairly anoying as the device can appear to be in error passive
	// when in fact it is working just fine.
	// The lines below attempt to work around this quirk.
	if (unlikely((can->int_init_rx_errors | can->int_init_tx_errors) != 0)) {
		if ((rec > can->int_init_rx_errors) | (tec > can->int_init_tx_errors)) {
			// the situation is becoming worse, stop the sharade
			LOG("ch%u rec/tec init=%u/%u now %u/%u\n", index, can->int_init_rx_errors, can->int_init_tx_errors, rec, tec);
			can->int_init_tx_errors = 0;
			can->int_init_rx_errors = 0;
		} else if ((rec < 96) & (tec < 96)) {
			// we have reached faked error active state, stop the sharade
			can->int_init_tx_errors = 0;
			can->int_init_rx_errors = 0;
			LOG("ch%u end error active fake\n", index);
		} else {
			// clear all errors except for bus off
			current_psr.reg &= MCANX_PSR_BO;
		}
	}


	if (unlikely((tec != can->int_prev_tx_errors) | (rec != can->int_prev_rx_errors))) {
		can->int_prev_tx_errors = tec;
		can->int_prev_rx_errors = rec;

		// LOG("ch%u REC=%u TEC=%u\n", index, can->int_prev_rx_errors, can->int_prev_tx_errors);

		status.type = SC_CAN_STATUS_FIFO_TYPE_RXTX_ERRORS;
		status.timestamp_us = tsc;
		status.counts.rx = rec;
		status.counts.tx = tec;

		sc_can_status_queue(index, &status);
		++*events;
	}

	if (current_psr.bit.BO) {
		current_bus_state = SC_CAN_STATUS_BUS_OFF;
	} else if (current_psr.bit.EP) {
		current_bus_state = SC_CAN_STATUS_ERROR_PASSIVE;
	} else if (current_psr.bit.EW) {
		current_bus_state = SC_CAN_STATUS_ERROR_WARNING;
	} else {
		current_bus_state = SC_CAN_STATUS_ERROR_ACTIVE;
	}

	if (unlikely(can->int_prev_bus_state != current_bus_state)) {
		can->int_prev_bus_state = current_bus_state;

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_STATUS;
		status.timestamp_us = tsc;
		status.bus_state = current_bus_state;

		sc_can_status_queue(index, &status);
		++*events;


		// LOG("ch%u PSR=%x ECR=%x\n", index, current_psr.reg, current_ecr.reg);

		switch (current_bus_state) {
		case SC_CAN_STATUS_BUS_OFF:
			LOG("ch%u bus off\n", index);
			break;
		case SC_CAN_STATUS_ERROR_PASSIVE:
			LOG("ch%u error passive\n", index);
			break;
		case SC_CAN_STATUS_ERROR_WARNING:
			LOG("ch%u error warning\n", index);
			break;
		case SC_CAN_STATUS_ERROR_ACTIVE:
			LOG("ch%u error active\n", index);
			break;
		default:
			LOG("ch%u unhandled bus state\n", index);
			break;
		}
	}

	uint8_t lec = current_psr.bit.LEC;
	uint8_t dlec = current_psr.bit.DLEC;

	if (unlikely(lec >= MCANX_PSR_LEC_STUFF_Val && lec <= MCANX_PSR_LEC_CRC_Val)) {
		const bool is_tx_error = current_psr.bit.ACT == MCANX_PSR_ACT_TX_Val;
		LOG("ch%u PSR=%08lx prev lec=%x dlec=%x\n", index, current_psr.reg, lec, dlec);

		can->int_prev_error_ts = tsc;

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		status.timestamp_us = tsc;
		status.bus_error.tx = is_tx_error;
		status.bus_error.code = map_error_code(lec),
		status.bus_error.data_part = 0;

		sc_can_status_queue(index, &status);
		++*events;
	}

	if (unlikely(dlec >= MCANX_PSR_DLEC_STUFF_Val && dlec <= MCANX_PSR_DLEC_CRC_Val)) {
		const bool is_tx_error = current_psr.bit.ACT == MCANX_PSR_ACT_TX_Val;
		LOG("ch%u PSR=%08lx prev lec=%x dlec=%x\n", index, current_psr.reg, lec, dlec);

		can->int_prev_error_ts = tsc;

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		status.timestamp_us = tsc;
		status.bus_error.tx = is_tx_error;
		status.bus_error.code = map_error_code(dlec),
		status.bus_error.data_part = 1;

		sc_can_status_queue(index, &status);
		++*events;
	}
}

#if SUPERCAN_DEBUG
static int entry_count[SC_BOARD_CAN_COUNT];
#endif

SC_RAMFUNC void mcan_can_int(uint8_t index)
{
#if SUPERCAN_DEBUG
	int c;
#if defined(HAVE_ATOMIC_COMPARE_EXCHANGE) && HAVE_ATOMIC_COMPARE_EXCHANGE
	c = __atomic_add_fetch(&entry_count[index], 1, __ATOMIC_ACQ_REL);
#else
	taskENTER_CRITICAL();
	c = ++entry_count[index];
	taskEXIT_CRITICAL();
#endif
	SC_ISR_ASSERT(c == 1);
#endif // #if SUPERCAN_DEBUG

	sc_board_can_ts_request(index);

	struct mcan_can *can = &mcan_cans[index];

	uint32_t events = 0;

	// LOG("IE=%08lx IR=%08lx\n", can->m_can->IE, can->m_can->IR);
	// bool notify_usb = false;

	// LOG(".");

	uint32_t ir = can->m_can->IR;

	// clear all interrupts
	can->m_can->IR = ir;

	if (ir & MCANX_IR_TSW) {
		// always notify here to enable the host to keep track of CAN bus time
		++events;
	}


	if (unlikely(ir & MCANX_IR_MRAF)) {
		/* Happens on STM32H7A3 if
		 * system clock (64 MHz) is
		 * lower than the CAN clock
		 * (80 MHz) during receive.
		 */
		LOG("ch%u MRAF\n", index);
		SC_ASSERT(false && "MRAF");
		NVIC_SystemReset();
	}

#if defined(MCANX_IR_BEU) && MCANX_IR_BEU
	if (unlikely(ir & MCANX_IR_BEU)) {
		LOG("ch%u BEU\n", index);
		SC_ISR_ASSERT(false && "BEU");
		NVIC_SystemReset();
	}
#endif

#if defined(MCANX_IR_BEC) && MCANX_IR_BEC
	if (unlikely(ir & MCANX_IR_BEC)) {
		LOG("ch%u BEC\n", index);
	}
#endif

	// Do this late to increase likelyhood that the counter
	// is ready right away.
	const uint32_t tsc = sc_board_can_ts_wait(index);

	if (unlikely(ir & MCANX_IR_RF0L)) {
		sc_can_status status;

		status.type = SC_CAN_STATUS_FIFO_TYPE_RX_LOST;
		status.timestamp_us = tsc & SC_TS_MAX;
		status.rx_lost = 1;

		sc_can_status_queue(index, &status);
		++events;
	}

	can_int_update_status(index, &events, tsc);

	if (ir & (MCANX_IR_TEFN | MCANX_IR_RF0N)) {
		// LOG("ch%u RX/TX\n", index);
		can_poll(index, &events, tsc);
	}

	if (likely(events)) {
		// LOG(">");
		sc_can_notify_task_isr(index, events);
	}
#if SUPERCAN_DEBUG
#if defined(HAVE_ATOMIC_COMPARE_EXCHANGE) && HAVE_ATOMIC_COMPARE_EXCHANGE
	__atomic_add_fetch(&entry_count[index], -1, __ATOMIC_ACQ_REL);
#else
	taskENTER_CRITICAL();
	--entry_count[index];
	taskEXIT_CRITICAL();
#endif
#endif
}

static inline void can_set_state1(MCanX* can, IRQn_Type interrupt_id, bool enabled)
{
	if (enabled) {
		// enable interrupt
		NVIC_EnableIRQ(interrupt_id);
		// disable initialization
		m_can_init_end(can);
	} else {
		m_can_init_begin(can);
		NVIC_DisableIRQ(interrupt_id);
		// clear any old interrupts
		can->IR = can->IR;
		// read out ECR, PSR to reset state
		(void)can->ECR;
		(void)can->PSR;
		// TXFQS, RXF0S are reset when CCCR.CCE is set (read as 0), DS60001507E-page 1207
	}
}

static inline void can_reset_task_state_unsafe(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(mcan_cans));

	struct mcan_can *can = &mcan_cans[index];


#if SUPERCAN_DEBUG
	memset(can->rx_fifo, 0, sizeof(can->rx_fifo));
	memset(can->txe_fifo, 0, sizeof(can->txe_fifo));
#if MCAN_HW_TX_FIFO_SIZE < _SC_BOARD_CAN_TX_FIFO_SIZE
	memset(can->tx_fifo, 0, sizeof(can->tx_fifo));
#endif
#endif

	can->int_prev_bus_state = SC_CAN_STATUS_ERROR_ACTIVE;
	can->int_prev_rx_errors = 0;
	can->int_prev_tx_errors = 0;
	can->int_init_rx_errors = 0;
	can->int_init_tx_errors = 0;
	can->int_prev_error_ts = 0;

	// call this here to timestamp / last rx/tx values
	// since we won't get any further interrupts
	can->rx_get_index = 0;
	can->rx_put_index = 0;
	can->txe_get_index = 0;
	can->txe_put_index = 0;

#if MCAN_HW_TX_FIFO_SIZE < _SC_BOARD_CAN_TX_FIFO_SIZE
	can->tx_get_index = 0;
	can->tx_put_index = 0;
#endif

#if SUPERCAN_DEBUG && MCAN_DEBUG_TXR
	can->txr = 0;
	can->int_txe = 0;
#endif

#if SUPERCAN_DEBUG && MCAN_DEBUG_TX_SEQ
	can->tx_last_value = -1;
#endif

	__atomic_thread_fence(__ATOMIC_RELEASE); // int_*
}

static inline void can_off(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(mcan_cans));

	struct mcan_can *can = &mcan_cans[index];

	// go bus off
	can_set_state1(can->m_can, can->interrupt_id, false);

	can_reset_task_state_unsafe(index);
}


static void can_on(uint8_t index)
{
	struct mcan_can *can = &mcan_cans[index];
	MCANX_ECR_Type current_ecr;
	uint32_t dtbr = 0;

	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(mcan_cans));

	can->nm_us_per_bit = UINT32_C(1000000) / sc_bitrate(can->nm.brp, can->nm.tseg1, can->nm.tseg2);
	dtbr = SC_BOARD_CAN_CLK_HZ / sc_bitrate(can->dt.brp, can->dt.tseg1, can->dt.tseg2);

	if (likely(dtbr > 0)) {
		can->dt_us_per_bit_factor_shift8 = (UINT32_C(1000000) << 8) / dtbr;
	} else {
		can->dt_us_per_bit_factor_shift8 = 32;
	}

	mcan_can_configure(index);

	current_ecr = can->m_can->ECR;

	/*
	 * If we have errors from prior use, try to get rid of them.
	 *
	 * Unfortunately M_CAN can't be reset on its own wich means
	 * we need to tx/rx messages for the error counters to go down.
	 */
	if (current_ecr.bit.REC | current_ecr.bit.TEC) {
		MCANX_ECR_Type previous_ecr;

		LOG("ch%u PSR=%x ECR=%x\n", index, can->m_can->PSR.reg, can->m_can->ECR.reg);
		// LOG("ch%u RXF0S=%x TXFQS=%x TXEFS=%x\n", index, can->m_can->RXF0S.reg, can->m_can->TXFQS.reg, can->m_can->TXEFS.reg);

		m_can_conf_begin(can->m_can);

		can->m_can->CCCR.bit.MON = 1;
		can->m_can->CCCR.bit.TEST = 1;
		can->m_can->CCCR.bit.ASM = 0;
		can->m_can->TEST.bit.LBCK = 1; // needs test set

		m_can_conf_end(can->m_can);

		LOG("ch%u TEST=%x CCCR=%x\n", index, can->m_can->TEST.reg, can->m_can->CCCR.reg);

		m_can_init_end(can->m_can);

		SC_DEBUG_ASSERT(!m_can_tx_event_fifo_avail(can->m_can));


		do {
			previous_ecr = current_ecr;

			uint8_t put_index = can->m_can->TXFQS.bit.TFQPI;

			can->hw_tx_fifo_ram[put_index].T0.reg = UINT32_C(1) << 18; // CAN-ID 1, ESI=0, XTD=0, RTR=0
			can->hw_tx_fifo_ram[put_index].T1.reg = 0; // DLC=0, MM=0, no FDF, no BSR, no TXEF event, see EFC field, DS60001507E-page 1305

			can->m_can->TXBAR.reg = UINT32_C(1) << put_index;

			while ((can->m_can->IR & (MCANX_IR_RF0N)) != (MCANX_IR_RF0N));

			// clear interrupts
			can->m_can->IR = ~0;

			// LOG("ch%u RXF0S=%x TXFQS=%x TXEFS=%x\n", index, can->m_can->RXF0S.reg, can->m_can->TXFQS.reg, can->m_can->TXEFS.reg);

			// throw away rx msg, tx event
			can->m_can->RXF0A.reg = MCANX_RXF0A_F0AI(can->m_can->RXF0S.bit.F0GI);

			current_ecr = can->m_can->ECR;

			// LOG("ch%u PSR=%x ECR=%x\n", index, can->m_can->PSR.reg, can->m_can->ECR.reg);
			// LOG("ch%u RXF0S=%x TXFQS=%x TXEFS=%x\n", index, can->m_can->RXF0S.reg, can->m_can->TXFQS.reg, can->m_can->TXEFS.reg);
		} while (current_ecr.reg != previous_ecr.reg);

		LOG("ch%u PSR=%x ECR=%x\n", index, can->m_can->PSR.reg, can->m_can->ECR.reg);

		m_can_init_begin(can->m_can);

		mcan_can_configure(index);
	}

	can->int_init_rx_errors = current_ecr.bit.REC;
	can->int_init_tx_errors = current_ecr.bit.TEC;
	__atomic_thread_fence(__ATOMIC_RELEASE); // int_*

	can_set_state1(can->m_can, can->interrupt_id, true);

	SC_DEBUG_ASSERT(!m_can_tx_event_fifo_avail(can->m_can));

	LOG("ch%u PSR=%x ECR=%x\n", index, can->m_can->PSR.reg, can->m_can->ECR.reg);
	// LOG("ch%u RXF0S=%x TXFQS=%x TXEFS=%x\n", index, can->m_can->RXF0S.reg, can->m_can->TXFQS.reg, can->m_can->TXEFS.reg);
}

static inline void can_reset(uint8_t index)
{
	struct mcan_can *can = &mcan_cans[index];

	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(mcan_cans));

	// disable CAN units, reset configuration & status
	can_off(index);

	can->features = CAN_FEAT_PERM;
	can->nm = nm_range.min;
	can->dt = dt_range.min;
}

extern void sc_board_can_reset(uint8_t index)
{
	// reset
	can_reset(index);
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
	// NOTE: there can't be situation (other than a bug) in
	// which the host sends more TX requests that we have fifo space.


	struct mcan_can *can = &mcan_cans[index];
	bool available = (can->m_can->TXFQS.reg & MCANX_TXFQS_TFQF) == 0;
	struct mcan_tx_fifo_element* txe = NULL;
	bool notify_hardware = true;
	uint8_t tx_pi_mod = can->m_can->TXFQS.bit.TFQPI;

	SC_DEBUG_ASSERT(msg->track_id < SC_BOARD_CAN_TX_FIFO_SIZE);

#if MCAN_HW_TX_FIFO_SIZE < _SC_BOARD_CAN_TX_FIFO_SIZE
	uint8_t const used = can->tx_put_index - can->tx_get_index;
	// Don´t sneak past frames waiting in RAM based fifo
	bool const queue = !available || used;

	if (queue) {
		available = used < SC_BOARD_CAN_TX_FIFO_SIZE;

		SC_DEBUG_ASSERT(available && "TX request accounting bug");

		if (likely(available)) {
			tx_pi_mod =  can->tx_put_index++ & (SC_BOARD_CAN_TX_FIFO_SIZE-1);
			txe = (struct mcan_tx_fifo_element*)&can->tx_fifo[tx_pi_mod];
			notify_hardware = false;
		}
	} else {
		txe = &can->hw_tx_fifo_ram[tx_pi_mod];
	}
#else
	SC_DEBUG_ASSERT(available && "TX request accounting bug");

	if (likely(available)) {
		txe = &can->hw_tx_fifo_ram[can->m_can->TXFQS.bit.TFQPI];
	}
#endif

	if (likely(available)) {
		MCANX_TXBE_0_Type t0;
		MCANX_TXBE_1_Type t1;

		t0.reg = (((msg->flags & SC_CAN_FRAME_FLAG_ESI) == SC_CAN_FRAME_FLAG_ESI) << MCANX_TXBE_0_ESI_Pos)
			| (((msg->flags & SC_CAN_FRAME_FLAG_RTR) == SC_CAN_FRAME_FLAG_RTR) << MCANX_TXBE_0_RTR_Pos)
			| (((msg->flags & SC_CAN_FRAME_FLAG_EXT) == SC_CAN_FRAME_FLAG_EXT) << MCANX_TXBE_0_XTD_Pos)
			;

		if (msg->flags & SC_CAN_FRAME_FLAG_EXT) {
			t0.reg |= MCANX_TXBE_0_ID(msg->can_id);
		} else {
			t0.reg |= MCANX_TXBE_0_ID(msg->can_id << 18);
		}

		t1.reg = MCANX_TXBE_1_EFC
			| MCANX_TXBE_1_DLC(msg->dlc)
			| MCANX_TXBE_1_MM(msg->track_id)
			| (((msg->flags & SC_CAN_FRAME_FLAG_FDF) == SC_CAN_FRAME_FLAG_FDF) << MCANX_TXBE_1_FDF_Pos)
			| (((msg->flags & SC_CAN_FRAME_FLAG_BRS) == SC_CAN_FRAME_FLAG_BRS) << MCANX_TXBE_1_BRS_Pos)
			;

		txe->T0 = t0;
		txe->T1 = t1;

		if (likely(!(msg->flags & SC_CAN_FRAME_FLAG_RTR))) {
			const unsigned can_frame_len = dlc_to_len(msg->dlc);

			if (likely(can_frame_len)) {
				if (unlikely(msg->flags & SC_CAN_FRAME_FLAG_TX4)) {
					uint32_t const *src = (void const *)&msg->data[3];

					for (unsigned i = 0, e = (can_frame_len + 3) / 4; i < e; ++i) {
						txe->data[i] = src[i];
					}
				} else {
#if MCAN_MESSAGE_RAM_CONFIGURABLE
					memcpy((void*)txe->data, msg->data, can_frame_len);
#else
					// too bad, we have to align :/
					uint32_t aligned[16];
					__IO uint32_t* dst = txe->data;
					uint32_t const* src = aligned;

					memcpy(aligned, msg->data, can_frame_len);

					for (unsigned i = 0, e = (can_frame_len + 3) / 4; i < e; ++i) {
						*dst++ = *src++;
					}
#endif
				}
			}
		}

#if SUPERCAN_DEBUG && MCAN_DEBUG_TXR
		{
			bool track_id_present = can->txr & (UINT32_C(1) << msg->track_id);

			if (unlikely(track_id_present)) {
				LOG("ch%u track id %u in TXR bitset %08x\n", index, msg->track_id, can->txr);
			}

			SC_DEBUG_ASSERT(!track_id_present);

			// LOG("ch%u TXR q %08x %02x\n", index, can->txr, msg->track_id);

			can->txr |= (UINT32_C(1) << msg->track_id);

#if defined(HAVE_ATOMIC_COMPARE_EXCHANGE) && HAVE_ATOMIC_COMPARE_EXCHANGE
			__atomic_or_fetch(&can->int_txe, UINT32_C(1) << msg->track_id, __ATOMIC_ACQ_REL);
#else
			taskENTER_CRITICAL();
			can->int_txe |= UINT32_C(1) << msg->track_id;
			taskEXIT_CRITICAL();
#endif
		}
#endif

#if SUPERCAN_DEBUG && MCAN_DEBUG_TX_SEQ
		uint8_t curr_value = txe->data[0] & 0xff;

		if (can->tx_last_value >= 0) {
			uint8_t last = can->tx_last_value;
			uint8_t next = last + 1;

			if (unlikely(curr_value != next)) {
				LOG("ch%u tx seq last=%02x curr=%02x\n", index, last, curr_value);
				SC_DEBUG_ASSERT(curr_value == next);
			}
		}

		can->tx_last_value = curr_value;
#endif
		if (notify_hardware) {
			// mark transmission for hardware
			can->m_can->TXBAR.reg = UINT32_C(1) << tx_pi_mod;
		}
	} else {
		LOG("ch%u TX no space\n", index);
	}

	return available;
}

SC_RAMFUNC static inline bool mcan_tx_try_queue(uint8_t index, struct mcan_txq_frame const * txq)
{
	struct mcan_can *can = &mcan_cans[index];
	bool available = (can->m_can->TXFQS.reg & MCANX_TXFQS_TFQF) == 0;

	if (available) {
		uint8_t tx_pi_mod = can->m_can->TXFQS.bit.TFQPI;

		memcpy(&can->hw_tx_fifo_ram[tx_pi_mod], txq, sizeof(*txq));

		// mark transmission for hardware
		can->m_can->TXBAR.reg = UINT32_C(1) << tx_pi_mod;
	}

	return available;
}

static inline void sc_dump_rx_fifo_ts(uint8_t index)
{
	struct mcan_can *can = &mcan_cans[index];

	for (size_t i = 0; i < TU_ARRAY_SIZE(can->rx_fifo); ++i) {
		LOG("ch%u RX i=%02x ts=%08x\n", index, (unsigned)i, can->rx_fifo[i].ts);
	}
}


static inline void sc_dump_txe_fifo_ts(uint8_t index)
{
	struct mcan_can *can = &mcan_cans[index];

	for (size_t i = 0; i < TU_ARRAY_SIZE(can->txe_fifo); ++i) {
		LOG("ch%u TXE i=%02x ts=%08x\n", index, (unsigned)i, can->txe_fifo[i].ts);
	}
}



SC_RAMFUNC extern int sc_board_can_retrieve(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	struct mcan_can *can = &mcan_cans[index];
	int result = 0;
	bool have_data_to_place = false;
#if SUPERCAN_DEBUG
	uint32_t rx_ts_last = 0;
	uint32_t txe_ts_last = 0;
#endif

	for (bool done = false; !done; ) {
		done = true;

		uint8_t const rx_pi = __atomic_load_n(&can->rx_put_index, __ATOMIC_ACQUIRE);

		if (can->rx_get_index != rx_pi) {
			uint8_t const rx_gi = can->rx_get_index;
			uint8_t const rx_gi_mod = rx_gi % SC_BOARD_CAN_RX_FIFO_SIZE;
			uint8_t bytes = sizeof(struct sc_msg_can_rx);
			MCANX_RXF0E_0_Type const r0 = can->rx_fifo[rx_gi_mod].R0;
			MCANX_RXF0E_1_Type const r1 = can->rx_fifo[rx_gi_mod].R1;
			uint8_t const can_frame_len = dlc_to_len(r1.bit.DLC);

			have_data_to_place = true;

			SC_DEBUG_ASSERT(rx_pi - can->rx_get_index <= SC_BOARD_CAN_RX_FIFO_SIZE);

			if (!r0.bit.RTR) {
				bytes += can_frame_len;
			}

			// align
			if (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1)) {
				bytes += SC_MSG_CAN_LEN_MULTIPLE - (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1));
			}

			if ((size_t)(tx_end - tx_ptr) >= bytes) {
				// LOG("rx %u bytes\n", bytes);
				struct sc_msg_can_rx *msg = (struct sc_msg_can_rx *)tx_ptr;
				uint32_t id;

				done = false;

				tx_ptr += bytes;
				result += bytes;


				msg->id = SC_MSG_CAN_RX;
				msg->len = bytes;
				msg->dlc = r1.bit.DLC;
				msg->flags = 0;

				id = r0.bit.ID;
				if (r0.bit.XTD) {
					msg->flags |= SC_CAN_FRAME_FLAG_EXT;
				} else {
					id >>= 18;
				}
				msg->can_id = id;

				msg->timestamp_us = can->rx_fifo[rx_gi_mod].ts;

#if SUPERCAN_DEBUG
				if (rx_ts_last) {
					uint32_t delta = (can->rx_fifo[rx_gi_mod].ts - rx_ts_last) & SC_TS_MAX;
					bool rx_ts_ok = delta <= SC_TS_MAX / 4;
					if (unlikely(!rx_ts_ok)) {
						LOG("ch%u RX gi=%02x\n", index, rx_gi_mod);
						sc_dump_rx_fifo_ts(index);
						SC_DEBUG_ASSERT(rx_ts_ok);
						__asm("bkpt 1");
					}


				}

				rx_ts_last = can->rx_fifo[rx_gi_mod].ts;
#endif

				// LOG("ch%u rx ts %lu\n", index, msg->timestamp_us);

				if (r1.bit.FDF) {
					msg->flags |= SC_CAN_FRAME_FLAG_FDF;

					if (r1.bit.BRS) {
						msg->flags |= SC_CAN_FRAME_FLAG_BRS;
					}

					memcpy(msg->data, can->rx_fifo[rx_gi_mod].data, can_frame_len);
				} else {
					if (r0.bit.RTR) {
						msg->flags |= SC_CAN_FRAME_FLAG_RTR;
					} else {
						memcpy(msg->data, can->rx_fifo[rx_gi_mod].data, can_frame_len);
					}
				}

				// LOG("g %02x\n", rx_gi_mod);

				__atomic_store_n(&can->rx_get_index, rx_gi+1, __ATOMIC_RELEASE);
			}
		}

		uint8_t const txe_pi = __atomic_load_n(&can->txe_put_index, __ATOMIC_ACQUIRE);

		if (can->txe_get_index != txe_pi) {
			uint8_t txe_gi = can->txe_get_index;
			uint8_t txe_gi_mod = txe_gi & (SC_BOARD_CAN_TX_FIFO_SIZE-1);
			struct sc_msg_can_txr *msg = NULL;


			have_data_to_place = true;
			SC_DEBUG_ASSERT(txe_pi - can->txe_get_index <= SC_BOARD_CAN_TX_FIFO_SIZE);

			if ((size_t)(tx_end - tx_ptr) >= sizeof(*msg)) {
				done = false;

				msg = (struct sc_msg_can_txr *)tx_ptr;

				tx_ptr += sizeof(*msg);
				result += sizeof(*msg);

				MCANX_TXEFE_0_Type t0 = can->txe_fifo[txe_gi_mod].T0;
				MCANX_TXEFE_1_Type t1 = can->txe_fifo[txe_gi_mod].T1;

				msg->id = SC_MSG_CAN_TXR;
				msg->len = sizeof(*msg);
				msg->track_id = t1.bit.MM;

				// LOG("ch%u TXR track id=%u @ index %u\n", index, msg->track_id, txe_gi_mod);

#if SUPERCAN_DEBUG && MCAN_DEBUG_TXR
				SC_DEBUG_ASSERT(can->txr & (UINT32_C(1) << msg->track_id));

				can->txr &= ~(UINT32_C(1) << msg->track_id);

				// LOG("ch%u TXR r %08x %02x\n", index, can->txr, msg->track_id);
#endif
				msg->timestamp_us = can->txe_fifo[txe_gi_mod].ts;
				msg->flags = 0;

#if SUPERCAN_DEBUG
				if (txe_ts_last) {
					uint32_t delta = (can->txe_fifo[txe_gi_mod].ts - txe_ts_last) & SC_TS_MAX;
					bool txe_ts_ok = delta <= SC_TS_MAX / 4;
					if (unlikely(!txe_ts_ok)) {
						LOG("ch%u TXE gi=%02x\n", index, txe_gi_mod);
						sc_dump_txe_fifo_ts(index);
						SC_DEBUG_ASSERT(txe_ts_ok);
						__asm("bkpt 1");
					}


				}

				txe_ts_last = can->txe_fifo[txe_gi_mod].ts;
#endif

				// Report the available flags back so host code
				// needs to store less information.
				if (t0.bit.XTD) {
					msg->flags |= SC_CAN_FRAME_FLAG_EXT;
				}

				if (t1.bit.FDF) {
					msg->flags |= SC_CAN_FRAME_FLAG_FDF;

					if (t0.bit.ESI) {
						msg->flags |= SC_CAN_FRAME_FLAG_ESI;
					}

					if (t1.bit.BRS) {
						msg->flags |= SC_CAN_FRAME_FLAG_BRS;
					}
				} else {
					if (t0.bit.RTR) {
						msg->flags |= SC_CAN_FRAME_FLAG_RTR;
					}
				}

				__atomic_store_n(&can->txe_get_index, txe_gi+1, __ATOMIC_RELEASE);

#if MCAN_HW_TX_FIFO_SIZE < _SC_BOARD_CAN_TX_FIFO_SIZE
				// try send frames to hw
				while (can->tx_get_index != can->tx_put_index) {
					uint8_t tx_gi_mod = can->tx_get_index & (SC_BOARD_CAN_TX_FIFO_SIZE-1);
					struct mcan_txq_frame* txq = &can->tx_fifo[tx_gi_mod];

					if (mcan_tx_try_queue(index, txq)) {
						++can->tx_get_index;
					} else {
						break;
					}
				}
#endif
			}
		}
	}

	if (result > 0) {
		return result;
	}

	return have_data_to_place - 1;
}


SC_RAMFUNC static inline void can_frame_bits(
	uint32_t xtd,
	uint32_t rtr,
	uint32_t fdf,
	uint32_t brs,
	uint8_t dlc,
	uint32_t* nmbr_bits,
	uint32_t* dtbr_bits)
{
	uint32_t payload_bits = dlc_to_len(dlc) * UINT32_C(8); /* payload */

	/* For SOF / interframe spacing, see ISO 11898-1:2015(E) 10.4.2.2 SOF
	 *
	 * Since the third bit (if dominant) in the interframe space marks
	 * SOF, there could be sitiuations in which the IFS is only 2 bit times
	 * long. The solution adopted here is to compute including 1 bit time SOF and shorted
	 * IFS to 2.
	 */

	if (fdf) {
		// FD frames have a 3 bit stuff count field and a 1 bit parity field prior to the actual checksum
		// There is a stuff bit at the begin of the stuff count field (always) and then at fixed positions
		// every 4 bits.
		uint32_t crc_bits = dlc <= 10 ? (17+4+5) : (21+4+6);

		if (brs) {
			*dtbr_bits =
				1 /* ESI */
				+ 4 /* DLC */
				+ payload_bits
				+ crc_bits; /* CRC */

			if (xtd) {
				*nmbr_bits =
					1 /* SOF? */
					+ 11 /* ID */
					+ 1 /* SRR */
					+ 1 /* IDE */
					+ 18 /* ID */
					+ 1 /* reserved 0 */
					+ 1 /* EDL */
					+ 1 /* reserved 0 */
					+ 1 /* BRS */
					// + 1 /* ESI */
					// + 4 /* DLC */
					// + dlc_to_len(dlc) * UINT32_C(8) /* payload */
					// + /* CRC */
					+ 1 /* CRC delimiter */
					+ 1 /* ACK slot */
					+ 1 /* ACK delimiter */
					+ 7 /* EOF */
					+ 2; /* INTERFRAME SPACE: INTERMISSION (3) + (SUSPEND TRANSMISSION)? + (BUS IDLE)? */

			} else {
				*nmbr_bits =
					1 /* SOF */
					+ 11 /* ID */
					+ 1 /* reserved 1 */
					+ 1 /* IDE */
					+ 1 /* EDL */
					+ 1 /* reserved 0 */
					+ 1 /* BRS */
					// + 1 /* ESI */
					// + 4 /* DLC */
					// + dlc_to_len(dlc) * UINT32_C(8) /* payload */
					// + /* CRC */
					+ 1 /* CRC delimiter */
					+ 1 /* ACK slot */
					+ 1 /* ACK delimiter */
					+ 7 /* EOF */
					+ 2; /* INTERFRAME SPACE: INTERMISSION (3) + (SUSPEND TRANSMISSION)? + (BUS IDLE)? */
			}
		} else {
			*dtbr_bits = 0;

			if (xtd) {
				*nmbr_bits =
					1 /* SOF */
					+ 11 /* ID */
					+ 1 /* SRR */
					+ 1 /* IDE */
					+ 18 /* ID */
					+ 1 /* reserved 0 */
					+ 1 /* EDL */
					+ 1 /* reserved 0 */
					+ 1 /* BRS */
					+ 1 /* ESI */
					+ 4 /* DLC */
					+ payload_bits
					+ crc_bits /* CRC */
					+ 1 /* CRC delimiter */
					+ 1 /* ACK slot */
					+ 1 /* ACK delimiter */
					+ 7 /* EOF */
					+ 2; /* INTERFRAME SPACE: INTERMISSION (3) + (SUSPEND TRANSMISSION)? + (BUS IDLE)? */
			} else {
				*nmbr_bits =
					1 /* SOF */
					+ 11 /* ID */
					+ 1 /* reserved 1 */
					+ 1 /* IDE */
					+ 1 /* EDL */
					+ 1 /* reserved 0 */
					+ 1 /* BRS */
					+ 1 /* ESI */
					+ 4 /* DLC */
					+ payload_bits
					+ crc_bits /* CRC */
					+ 1 /* CRC delimiter */
					+ 1 /* ACK slot */
					+ 1 /* ACK delimiter */
					+ 7 /* EOF */
					+ 2; /* INTERFRAME SPACE: INTERMISSION (3) + (SUSPEND TRANSMISSION)? + (BUS IDLE)? */
			}
		}
	} else {
		*dtbr_bits = 0;

		if (xtd) {
			*nmbr_bits =
				1 /* SOF */
				+ 11 /* non XTD identifier part */
				+ 1 /* SRR */
				+ 1 /* IDE */
				+ 18 /* XTD identifier part */
				+ 1 /* RTR */
				+ 2 /* reserved */
				+ 4 /* DLC */
				+ (!rtr) * payload_bits
				+ 15 /* CRC */
				+ 1 /* CRC delimiter */
				+ 1 /* ACK slot */
				+ 1 /* ACK delimiter */
				+ 7 /* EOF */
				+ 2; /* INTERFRAME SPACE: INTERMISSION (3) + (SUSPEND TRANSMISSION)? + (BUS IDLE)? */
		} else {
			*nmbr_bits =
				1 /* SOF */
				+ 11 /* ID */
				+ 1 /* RTR */
				+ 1 /* IDE */
				+ 1 /* reserved */
				+ 4 /* DLC */
				+ (!rtr) * payload_bits
				+ 15 /* CRC */
				+ 1 /* CRC delimiter */
				+ 1 /* ACK slot */
				+ 1 /* ACK delimiter */
				+ 7 /* EOF */
				+ 2; /* INTERFRAME SPACE: INTERMISSION (3) + (SUSPEND TRANSMISSION)? + (BUS IDLE)? */
		}
	}
}

SC_RAMFUNC static inline uint32_t can_frame_time_us(
	uint8_t index,
	uint32_t nm,
	uint32_t dt)
{
	struct mcan_can *can = &mcan_cans[index];
	return can->nm_us_per_bit * nm + ((can->dt_us_per_bit_factor_shift8 * dt) >> 8);
}

#ifdef SUPERCAN_DEBUG
static volatile uint32_t rx_lost_reported[TU_ARRAY_SIZE(mcan_cans)];
static volatile bool txe_any[TU_ARRAY_SIZE(mcan_cans)];
static volatile uint8_t txe_prev_pi[TU_ARRAY_SIZE(mcan_cans)];
#endif

SC_RAMFUNC static void can_poll(
	uint8_t index,
	uint32_t* const events,
	uint32_t tsc)
{
	struct mcan_can *can = &mcan_cans[index];

	uint32_t tsv[MCAN_HW_RX_FIFO_SIZE];
	uint8_t count = 0;
	unsigned rx_lost = 0;

	count = can->m_can->RXF0S.bit.F0FL;

	if (count) {
		// reverse loop reconstructs timestamps
		uint32_t ts = tsc;
		uint8_t hw_rx_gi_mod = 0;
		uint8_t rx_pi = 0;
		uint32_t nmbr_bits, dtbr_bits;
		struct mcan_rx_fifo_element *rx_get_ptr = NULL;

		for (uint8_t i = 0, gio = can->m_can->RXF0S.bit.F0GI; i < count; ++i) {
			uint8_t const offset = count - 1 - i;
			hw_rx_gi_mod = (gio + offset) % MCAN_HW_RX_FIFO_SIZE;
			// LOG("ch%u ts rx count=%u gi=%u\n", index, count, get_index);

			tsv[offset] = ts & SC_TS_MAX;

			rx_get_ptr = &can->hw_rx_fifo_ram[hw_rx_gi_mod];

			can_frame_bits(
				rx_get_ptr->R0.bit.XTD,
				rx_get_ptr->R0.bit.RTR,
				rx_get_ptr->R1.bit.FDF,
				rx_get_ptr->R1.bit.BRS,
				rx_get_ptr->R1.bit.DLC,
				&nmbr_bits,
				&dtbr_bits);

			// LOG("ch%u rx gi=%u xtd=%d rtr=%d fdf=%d brs=%d dlc=%d nmbr_bits=%lu dtbr_bits=%lu ts=%lx data us=%lu\n",
			// 	index, hw_rx_gi_mod, rx_get_ptr->R0.bit.XTD,
			// 	rx_get_ptr->R0.bit.RTR,
			// 	rx_get_ptr->R1.bit.FDF,
			// 	rx_get_ptr->R1.bit.BRS,
			// 	rx_get_ptr->R1.bit.DLC, nmbr_bits, dtbr_bits,
			// 	(unsigned long)ts,
			// 	(unsigned long)((can->dt_us_per_bit_factor_shift8 * dtbr_bits) >> 8));

			ts -= can_frame_time_us(index, nmbr_bits, dtbr_bits);
		}

		// forward loop stores frames and notifies usb task
		rx_pi = can->rx_put_index;

		for (uint8_t i = 0, gio = can->m_can->RXF0S.bit.F0GI; i < count; ++i) {
			uint8_t const offset = i;

			hw_rx_gi_mod = (gio + offset) % MCAN_HW_RX_FIFO_SIZE;
			rx_get_ptr = &can->hw_rx_fifo_ram[hw_rx_gi_mod];

			uint8_t rx_gi = __atomic_load_n(&can->rx_get_index, __ATOMIC_ACQUIRE);
			uint8_t const used = rx_pi - rx_gi;
			SC_DEBUG_ASSERT(used <= SC_BOARD_CAN_RX_FIFO_SIZE);

			if (unlikely(used == SC_BOARD_CAN_RX_FIFO_SIZE)) {
				++rx_lost;

#if SUPERCAN_DEBUG
				{
					if (rx_lost_reported[index] + UINT32_C(1000000) <= tsc) {
						rx_lost_reported[index] = tsc;
						LOG("ch%u rx lost %lx pi=%u gi=%u\n", index, ts, rx_pi, rx_gi);
					}
				}
#endif
			} else {
				uint8_t const rx_pi_mod = rx_pi % SC_BOARD_CAN_RX_FIFO_SIZE;
				struct mcan_rx_frame *rx_put_ptr = &can->rx_fifo[rx_pi_mod];
				MCANX_RXF0E_0_Type const r0 = rx_get_ptr->R0;
				MCANX_RXF0E_1_Type const r1 = rx_get_ptr->R1;

				// LOG("p %02x\n", rx_pi_mod);

				rx_put_ptr->R0 = r0;
				rx_put_ptr->R1 = r1;
				rx_put_ptr->ts = tsv[offset];
				if (likely(!r0.bit.RTR)) {
					uint8_t can_frame_len = dlc_to_len(r1.bit.DLC);

					if (likely(can_frame_len)) {

#if !MCAN_MESSAGE_RAM_CONFIGURABLE
						// hardware fifo must be accessed using 32bit words (STM32H7A3)
						if (can_frame_len & 3) {
							can_frame_len += 4 - (can_frame_len & 3);
						}
#endif
						memcpy(rx_put_ptr->data, (void*)rx_get_ptr->data, can_frame_len);
					}
				}

				++rx_pi;

				// NOTE: This code is too slow to have here for some reason.
				// NOTE: If called outside this function, it is fast enough.
				// NOTE: Likely because of register / cache thrashing.

				// xTaskNotifyGive(can->usb_task_hancan->rx_fifo[rx_pi_mod].dle);
				// BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				// vTaskNotifyGiveFromISR(can->usb_task_handle, &xHigherPriorityTaskWoken);
				// portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				++*events;
			}
		}

		// removes frames from rx fifo
		can->m_can->RXF0A.reg = MCANX_RXF0A_F0AI(hw_rx_gi_mod);

		// atomic update of rx put index
		__atomic_store_n(&can->rx_put_index, rx_pi, __ATOMIC_RELEASE);

		// LOG("r c=%u pi=%02x\n", count, rx_pi);
	}

	count = can->m_can->TXEFS.bit.EFFL;

	if (count) {
		// reverse loop reconstructs timestamps
		uint32_t ts = tsc;
		uint8_t hw_txe_gi_mod = 0;
		uint8_t txe_pi = 0;
		uint32_t const txp = can->m_can->CCCR.bit.TXP * 2;
		uint32_t nmbr_bits, dtbr_bits;

		for (uint8_t i = 0, gio = can->m_can->TXEFS.bit.EFGI; i < count; ++i) {
			uint8_t const offset = count - 1 - i;

			hw_txe_gi_mod = (gio + offset) % MCAN_HW_TX_FIFO_SIZE;
			SC_DEBUG_ASSERT(hw_txe_gi_mod < MCAN_HW_TX_FIFO_SIZE);
			// LOG("ch%u poll tx count=%u gi=%u\n", index, count, hw_tx_gi_mod);

			tsv[offset] = ts & SC_TS_MAX;

			can_frame_bits(
				can->hw_txe_fifo_ram[hw_txe_gi_mod].T0.bit.XTD,
				can->hw_txe_fifo_ram[hw_txe_gi_mod].T0.bit.RTR,
				can->hw_txe_fifo_ram[hw_txe_gi_mod].T1.bit.FDF,
				can->hw_txe_fifo_ram[hw_txe_gi_mod].T1.bit.BRS,
				can->hw_txe_fifo_ram[hw_txe_gi_mod].T1.bit.DLC,
				&nmbr_bits,
				&dtbr_bits);

			ts -= can_frame_time_us(index, nmbr_bits + txp, dtbr_bits);
		}

		// forward loop stores frames and notifies usb task
		txe_pi = can->txe_put_index;

		for (uint8_t i = 0, gio = can->m_can->TXEFS.bit.EFGI; i < count; ++i) {
			uint8_t const offset = i;
			uint8_t txe_pi_mod = txe_pi % SC_BOARD_CAN_TX_FIFO_SIZE;

			SC_DEBUG_ASSERT(txe_pi_mod < SC_BOARD_CAN_TX_FIFO_SIZE);
			hw_txe_gi_mod = (gio + offset) % MCAN_HW_TX_FIFO_SIZE;
			SC_DEBUG_ASSERT(hw_txe_gi_mod < MCAN_HW_TX_FIFO_SIZE);


			// We only queue to hw TX fifo iff there is space, thus there
			// is ALWAYS space for TXE.


			can->txe_fifo[txe_pi_mod].T0 = can->hw_txe_fifo_ram[hw_txe_gi_mod].T0;
			can->txe_fifo[txe_pi_mod].T1 = can->hw_txe_fifo_ram[hw_txe_gi_mod].T1;
			can->txe_fifo[txe_pi_mod].ts = tsv[offset];
			// LOG("ch%u tx place MM %u @ index %u\n", index, can->txe_fifo[tx_pi_mod].T1.bit.MM, tx_pi_mod);


			++txe_pi;

#if SUPERCAN_DEBUG && MCAN_DEBUG_TXR

#if defined(HAVE_ATOMIC_COMPARE_EXCHANGE) && HAVE_ATOMIC_COMPARE_EXCHANGE
			__atomic_and_fetch(&can->int_txe, ~(UINT32_C(1) << can->txe_fifo[txe_pi_mod].T1.bit.MM), __ATOMIC_ACQ_REL);
#else
			taskENTER_CRITICAL();
			can->int_txe &= ~(UINT32_C(1) << can->txe_fifo[txe_pi_mod].T1.bit.MM);
			taskEXIT_CRITICAL();
#endif

			// LOG("ch%u TXE %08x\n", index, can->int_txe);
#endif


			// xTaskNotifyGive(can->usb_task_handle);
			// BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			// vTaskNotifyGiveFromISR(can->usb_task_handle, &xHigherPriorityTaskWoken);
			// portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			++*events;

#if SUPERCAN_DEBUG
			if (txe_any[index]) {
				uint8_t txe_pi_mod_prev = (txe_pi_mod + (SC_BOARD_CAN_TX_FIFO_SIZE-1)) % SC_BOARD_CAN_TX_FIFO_SIZE;
				SC_DEBUG_ASSERT(txe_pi_mod_prev < SC_BOARD_CAN_TX_FIFO_SIZE);
				uint32_t delta = (can->txe_fifo[txe_pi_mod].ts - can->txe_fifo[txe_pi_mod_prev].ts) & SC_TS_MAX;
				bool txe_ts_ok = delta <= SC_TS_MAX / 4;
				if (unlikely(!txe_ts_ok)) {
					for (uint8_t j = 0; j < count; ++j) {
						LOG("ch%u TXE HW %02x %08x\n", index, j, tsv[j]);
					}
					LOG("ch%u TXE pi curr=%02x prev=%02x batch=%02x count=%u\n", index, txe_pi_mod, txe_pi_mod_prev, txe_prev_pi[index], count);
					sc_dump_txe_fifo_ts(index);
					SC_DEBUG_ASSERT(txe_ts_ok);
					__asm("bkpt 1");
				}
			}

			txe_any[index] = true;
#endif
		}



		// removes frames from tx fifo
		can->m_can->TXEFA.reg = MCANX_TXEFA_EFAI(hw_txe_gi_mod);
		// LOG("ch%u poll tx count=%u done\n", index, count);

		// atomic update of tx put index
		__atomic_store_n(&can->txe_put_index, txe_pi, __ATOMIC_RELEASE);

		txe_prev_pi[index] = txe_pi;
		// LOG("t c=%u pi=%02x\n", count, tx_pi);
	}

	if (unlikely(rx_lost)) {
		sc_can_status status;

		status.type = SC_CAN_STATUS_FIFO_TYPE_RX_LOST;
		status.timestamp_us = tsc;
		status.rx_lost = rx_lost;

		sc_can_status_queue(index, &status);
		++*events;
	}
#if SUPERCAN_DEBUG && MCAN_DEBUG_GUARD
	mcan_can_verify_guard(index);
#endif
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
	struct mcan_can *can = &mcan_cans[index];

	can->features = features;
}

extern void sc_board_can_go_bus(uint8_t index, bool on)
{
	if (on) {
		can_on(index);
	} else {
		can_off(index);
	}
}

extern void sc_board_can_nm_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	struct mcan_can *can = &mcan_cans[index];

	can->nm = *bt;
}

extern void sc_board_can_dt_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	struct mcan_can *can = &mcan_cans[index];

	can->dt = *bt;
}

#if SUPERCAN_DEBUG && MCAN_DEBUG_GUARD
SC_RAMFUNC extern void mcan_can_verify_guard(uint8_t index)
{
	struct mcan_can *can = &mcan_cans[index];

	for (size_t j = 0; j < TU_ARRAY_SIZE(can->guard_hdr); ++j) {
		SC_DEBUG_ASSERT(MCAN_CAN_GUARD_HDR == can->guard_hdr[j]);
	}

	for (size_t j = 0; j < TU_ARRAY_SIZE(can->guard_fifo); ++j) {
		SC_DEBUG_ASSERT(MCAN_CAN_GUARD_MID == can->guard_fifo[j]);
	}

	for (size_t j = 0; j < TU_ARRAY_SIZE(can->guard_ftr); ++j) {
		SC_DEBUG_ASSERT(MCAN_CAN_GUARD_FTR == can->guard_ftr[j]);
	}
}
#endif



#endif // SUPERMCANX_MCAN

