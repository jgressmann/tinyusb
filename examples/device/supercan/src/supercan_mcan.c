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
	can->IE =
		0
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
			LOG("CAN%u rec/tec init=%u/%u now %u/%u\n", index, can->int_init_rx_errors, can->int_init_tx_errors, rec, tec);
			can->int_init_tx_errors = 0;
			can->int_init_rx_errors = 0;
		} else if ((rec < 96) & (tec < 96)) {
			// we have reached faked error active state, stop the sharade
			can->int_init_tx_errors = 0;
			can->int_init_rx_errors = 0;
			LOG("CAN%u end error active fake\n", index);
		} else {
			// clear all errors except for bus off
			current_psr.reg &= MCANX_PSR_BO;
		}
	}


	if (unlikely((tec != can->int_prev_tx_errors) | (rec != can->int_prev_rx_errors))) {
		can->int_prev_tx_errors = tec;
		can->int_prev_rx_errors = rec;

		// LOG("CAN%u REC=%u TEC=%u\n", index, can->int_prev_rx_errors, can->int_prev_tx_errors);

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
			LOG("CAN%u bus off\n", index);
			break;
		case SC_CAN_STATUS_ERROR_PASSIVE:
			LOG("CAN%u error passive\n", index);
			break;
		case SC_CAN_STATUS_ERROR_WARNING:
			LOG("CAN%u error warning\n", index);
			break;
		case SC_CAN_STATUS_ERROR_ACTIVE:
			LOG("CAN%u error active\n", index);
			break;
		default:
			LOG("CAN%u unhandled bus state\n", index);
			break;
		}
	}

	uint8_t lec = current_psr.bit.LEC;
	uint8_t dlec = current_psr.bit.DLEC;

	if (unlikely(lec >= MCANX_PSR_LEC_STUFF_Val && lec <= MCANX_PSR_LEC_CRC_Val)) {
		const bool is_tx_error = current_psr.bit.ACT == MCANX_PSR_ACT_TX_Val;
		LOG("CAN%u PSR=%08lx prev lec=%x dlec=%x\n", index, current_psr.reg, lec, dlec);

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
		LOG("CAN%u PSR=%08lx prev lec=%x dlec=%x\n", index, current_psr.reg, lec, dlec);

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
#endif

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
		LOG("ch%u MRAF\n", index);
		SC_ISR_ASSERT(false && "MRAF");
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
		status.timestamp_us = tsc;
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
	memset(can->rx_frames, 0, sizeof(can->rx_frames));
	memset(can->tx_frames, 0, sizeof(can->tx_frames));
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
	can->tx_get_index = 0;
	can->tx_put_index = 0;

#if SUPERCAN_DEBUG && MCAN_DEBUG_TXR
	can->txr = 0;
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
	struct mcan_can *can = &mcan_cans[index];
#if MCAN_HW_TX_FIFO_SIZE == 32
	bool available = can->m_can->TXFQS.bit.TFQPI - can->m_can->TXFQS.bit.TFGI < MCAN_HW_TX_FIFO_SIZE;
#else
	bool available = can->m_can->TXFQS.bit.TFQPI + (can->m_can->TXFQS.bit.TFQPI >= can->m_can->TXFQS.bit.TFGI ? 0 : MCAN_HW_TX_FIFO_SIZE) - can->m_can->TXFQS.bit.TFGI < MCAN_HW_TX_FIFO_SIZE;
#endif

	// LOG("ch%u TFQPI=%02x TFGI=%02x avail=%u, track id=%u CAN ID=%08x\n", index, can->m_can->TXFQS.bit.TFQPI, can->m_can->TXFQS.bit.TFGI, available, msg->track_id, msg->can_id);
	// sc_dump_mem(msg->data, dlc_to_len(msg->dlc));

	if (available) {
		uint32_t id = msg->can_id;
		uint8_t const tx_pi_mod = can->m_can->TXFQS.bit.TFQPI;
		MCANX_TXBE_0_Type t0;
		MCANX_TXBE_1_Type t1;

		t0.reg = (((msg->flags & SC_CAN_FRAME_FLAG_ESI) == SC_CAN_FRAME_FLAG_ESI) << MCANX_TXBE_0_ESI_Pos)
			| (((msg->flags & SC_CAN_FRAME_FLAG_RTR) == SC_CAN_FRAME_FLAG_RTR) << MCANX_TXBE_0_RTR_Pos)
			| (((msg->flags & SC_CAN_FRAME_FLAG_EXT) == SC_CAN_FRAME_FLAG_EXT) << MCANX_TXBE_0_XTD_Pos)
			;

		if (msg->flags & SC_CAN_FRAME_FLAG_EXT) {
			t0.reg |= MCANX_TXBE_0_ID(id);
		} else {
			t0.reg |= MCANX_TXBE_0_ID(id << 18);
		}

		t1.reg = MCANX_TXBE_1_EFC
			| MCANX_TXBE_1_DLC(msg->dlc)
			| MCANX_TXBE_1_MM(msg->track_id)
			| (((msg->flags & SC_CAN_FRAME_FLAG_FDF) == SC_CAN_FRAME_FLAG_FDF) << MCANX_TXBE_1_FDF_Pos)
			| (((msg->flags & SC_CAN_FRAME_FLAG_BRS) == SC_CAN_FRAME_FLAG_BRS) << MCANX_TXBE_1_BRS_Pos)
			;




		if (likely(!(msg->flags & SC_CAN_FRAME_FLAG_RTR))) {
			const unsigned can_frame_len = dlc_to_len(msg->dlc);

			if (likely(can_frame_len)) {
#if MCAN_MESSAGE_RAM_CONFIGURABLE
				memcpy((void*)can->hw_tx_fifo_ram[tx_pi_mod].data, msg->data, can_frame_len);
#else
// 				uint32_t aligned[16];
// 				volatile uint32_t* dst32 = (volatile uint32_t*)can->hw_tx_fifo_ram[tx_pi_mod].data;

// 				memcpy(aligned, msg->data, can_frame_len);

// 				unsigned words = (can_frame_len + 3) / 4;
// 				LOG("words=%u\n", words);

// 				for (unsigned i = 0, e = (can_frame_len + 3) / 4; i < e; ++e) {
// 					*dst32++ = aligned[i];
// 				}
// 				// memcpy((void*)can->hw_tx_fifo_ram[tx_pi_mod].data, aligned, (can_frame_len + 3) / 4);
#endif
			}
		}

		can->hw_tx_fifo_ram[tx_pi_mod].T0 = t0;
		can->hw_tx_fifo_ram[tx_pi_mod].T1 = t1;

		can->m_can->TXBAR.reg = UINT32_C(1) << tx_pi_mod;
#if SUPERCAN_DEBUG && MCAN_DEBUG_TXR
		SC_DEBUG_ASSERT(!(can->txr & (UINT32_C(1) << msg->track_id)));

		can->txr |= UINT32_C(1) << msg->track_id;
#endif
	} else {
		LOG("ch0 no TX space\n", index);
	}

	return available;
}

SC_RAMFUNC extern int sc_board_can_retrieve(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	struct mcan_can *can = &mcan_cans[index];
	int result = 0;
	bool have_data_to_place = false;

	for (bool done = false; !done; ) {
		done = true;

		uint8_t const rx_pi = __atomic_load_n(&can->rx_put_index, __ATOMIC_ACQUIRE);

		if (can->rx_get_index != rx_pi) {
			have_data_to_place = true;
			// __atomic_thread_fence(__ATOMIC_ACQUIRE);

			uint8_t const rx_gi = can->rx_get_index;
			uint8_t const rx_gi_mod = rx_gi & (SC_BOARD_CAN_RX_FIFO_SIZE-1);
			uint8_t bytes = sizeof(struct sc_msg_can_rx);
			MCANX_RXF0E_0_Type r0 = can->rx_frames[rx_gi_mod].R0;
			MCANX_RXF0E_1_Type r1 = can->rx_frames[rx_gi_mod].R1;
			uint8_t can_frame_len = dlc_to_len(r1.bit.DLC);

			SC_DEBUG_ASSERT(rx_pi - can->rx_get_index <= SC_BOARD_CAN_RX_FIFO_SIZE);

			if (!r0.bit.RTR) {
				bytes += can_frame_len;
			}

			// align
			if (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1)) {
				bytes += SC_MSG_CAN_LEN_MULTIPLE - (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1));
			}


			if ((size_t)(tx_end - tx_ptr) >= bytes) {
				done = false;

				// LOG("rx %u bytes\n", bytes);
				struct sc_msg_can_rx *msg = (struct sc_msg_can_rx *)tx_ptr;
				// usb_can->tx_offsets[usb_can->tx_bank] += bytes;
				tx_ptr += bytes;
				result += bytes;


				msg->id = SC_MSG_CAN_RX;
				msg->len = bytes;
				msg->dlc = r1.bit.DLC;
				msg->flags = 0;
				uint32_t id = r0.bit.ID;
				if (r0.bit.XTD) {
					msg->flags |= SC_CAN_FRAME_FLAG_EXT;
				} else {
					id >>= 18;
				}
				msg->can_id = id;


				uint32_t ts = can->rx_frames[rx_gi_mod].ts;
	// #if SUPERCAN_DEBUG
	// 					uint32_t delta = (ts - rx_ts_last) & SC_TS_MAX;
	// 					bool rx_ts_ok = delta <= SC_TS_MAX / 4;
	// 					if (unlikely(!rx_ts_ok)) {
	// 						taskDISABLE_INTERRUPTS();
	// 						// SC_BOARD_MCANX_init_begin(can);
	// 						LOG("ch%u rx gi=%u ts=%lx prev=%lx\n", index, rx_gi_mod, ts, rx_ts_last);
	// 						for (unsigned i = 0; i < MCANX_RX_FIFO_SIZE; ++i) {
	// 							LOG("ch%u rx gi=%u ts=%lx\n", index, i, can->rx_frames[i].ts);
	// 						}

	// 					}
	// 					SC_ASSERT(rx_ts_ok);
	// 					// LOG("ch%u rx gi=%u d=%lx\n", index, rx_gi_mod, ts - rx_ts_last);
	// 					rx_ts_last = ts;
	// #endif

				msg->timestamp_us = ts;

				// LOG("ch%u rx ts %lu\n", index, msg->timestamp_us);

				if (r1.bit.FDF) {
					msg->flags |= SC_CAN_FRAME_FLAG_FDF;
					if (r1.bit.BRS) {
						msg->flags |= SC_CAN_FRAME_FLAG_BRS;
					}

					memcpy(msg->data, can->rx_frames[rx_gi_mod].data, can_frame_len);
				} else {
					if (r0.bit.RTR) {
						msg->flags |= SC_CAN_FRAME_FLAG_RTR;
					} else {
						memcpy(msg->data, can->rx_frames[rx_gi_mod].data, can_frame_len);
					}
				}

				// LOG("rx store %u bytes\n", bytes);
				// sc_dump_mem(msg, bytes);

				__atomic_store_n(&can->rx_get_index, rx_gi+1, __ATOMIC_RELEASE);
			}
		}

		uint8_t const tx_pi = __atomic_load_n(&can->tx_put_index, __ATOMIC_ACQUIRE);

		if (can->tx_get_index != tx_pi) {
			uint8_t tx_gi = can->tx_get_index;
			uint8_t tx_gi_mod = tx_gi & (SC_BOARD_CAN_TX_FIFO_SIZE-1);
			struct sc_msg_can_txr *msg = NULL;


			have_data_to_place = true;
			SC_DEBUG_ASSERT(tx_pi - can->tx_get_index <= SC_BOARD_CAN_TX_FIFO_SIZE);



			if ((size_t)(tx_end - tx_ptr) >= sizeof(*msg)) {
				// __atomic_thread_fence(__ATOMIC_ACQUIRE);
				done = false;

				msg = (struct sc_msg_can_txr *)tx_ptr;

				tx_ptr += sizeof(*msg);
				result += sizeof(*msg);

				MCANX_TXEFE_0_Type t0 = can->tx_frames[tx_gi_mod].T0;
				MCANX_TXEFE_1_Type t1 = can->tx_frames[tx_gi_mod].T1;

				msg->id = SC_MSG_CAN_TXR;
				msg->len = sizeof(*msg);
				msg->track_id = t1.bit.MM;

				// LOG("ch%u TXR track id=%u @ index %u\n", index, msg->track_id, tx_gi_mod);
#if SUPERCAN_DEBUG && MCAN_DEBUG_TXR
				SC_DEBUG_ASSERT(can->txr & (UINT32_C(1) << msg->track_id));

				can->txr &= ~(UINT32_C(1) << msg->track_id);
#endif
				uint32_t ts = can->tx_frames[tx_gi_mod].ts;
	// #if SUPERCAN_DEBUG
	// 					// bool tx_ts_ok = ts >= tx_ts_last || (TS_HI(tx_ts_last) == 0xffff && TS_HI(ts) == 0);
	// 					uint32_t delta = (ts - tx_ts_last) & SC_TS_MAX;
	// 					bool tx_ts_ok = delta <= SC_TS_MAX / 4;
	// 					if (unlikely(!tx_ts_ok)) {
	// 						taskDISABLE_INTERRUPTS();
	// 						// SC_BOARD_MCANX_init_begin(can);
	// 						LOG("ch%u tx gi=%u ts=%lx prev=%lx\n", index, tx_gi_mod, ts, tx_ts_last);
	// 						for (unsigned i = 0; i < SC_BOARD_MCANX_hw_tx_fifo_ram_SIZE; ++i) {
	// 							LOG("ch%u tx gi=%u ts=%lx\n", index, i, can->tx_frames[i].ts);
	// 						}

	// 					}
	// 					SC_ASSERT(tx_ts_ok);
	// 					// LOG("ch%u tx gi=%u d=%lx\n", index, tx_gi_mod, ts - tx_ts_last);
	// 					tx_ts_last = ts;
	// #endif
				msg->timestamp_us = ts;
				msg->flags = 0;

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

				__atomic_store_n(&can->tx_get_index, tx_gi+1, __ATOMIC_RELEASE);
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
// static volatile uint32_t rx_ts_last[TU_ARRAY_SIZE(mcan_cans)];
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
// #ifdef SUPERCAN_DEBUG
// 		uint32_t us = tsc - rx_ts_last[index];
// 		rx_ts_last[index] = tsc;
// 		LOG("ch%u rx dt=%lu\n", index, (unsigned long)us);
// #endif
		// reverse loop reconstructs timestamps
		uint32_t ts = tsc;
		uint8_t hw_rx_gi_mod = 0;
		uint8_t rx_pi = 0;
		uint32_t nmbr_bits, dtbr_bits;

		for (uint8_t i = 0, gio = can->m_can->RXF0S.bit.F0GI; i < count; ++i) {
			hw_rx_gi_mod = (gio + count - 1 - i) % MCAN_HW_RX_FIFO_SIZE;
			// LOG("ch%u ts rx count=%u gi=%u\n", index, count, get_index);

			tsv[hw_rx_gi_mod] = ts & SC_TS_MAX;

			can_frame_bits(
				can->hw_rx_fifo_ram[hw_rx_gi_mod].R0.bit.XTD,
				can->hw_rx_fifo_ram[hw_rx_gi_mod].R0.bit.RTR,
				can->hw_rx_fifo_ram[hw_rx_gi_mod].R1.bit.FDF,
				can->hw_rx_fifo_ram[hw_rx_gi_mod].R1.bit.BRS,
				can->hw_rx_fifo_ram[hw_rx_gi_mod].R1.bit.DLC,
				&nmbr_bits,
				&dtbr_bits);

			// LOG("ch%u rx gi=%u xtd=%d rtr=%d fdf=%d brs=%d dlc=%d nmbr_bits=%lu dtbr_bits=%lu ts=%lx data us=%lu\n",
			// 	index, hw_rx_gi_mod, can->hw_rx_fifo_ram[hw_rx_gi_mod].R0.bit.XTD,
			// 	can->hw_rx_fifo_ram[hw_rx_gi_mod].R0.bit.RTR,
			// 	can->hw_rx_fifo_ram[hw_rx_gi_mod].R1.bit.FDF,
			// 	can->hw_rx_fifo_ram[hw_rx_gi_mod].R1.bit.BRS,
			// 	can->hw_rx_fifo_ram[hw_rx_gi_mod].R1.bit.DLC, nmbr_bits, dtbr_bits,
			// 	(unsigned long)ts,
			// 	(unsigned long)((can->dt_us_per_bit_factor_shift8 * dtbr_bits) >> 8));

			ts -= can_frame_time_us(index, nmbr_bits, dtbr_bits);
		}

		// forward loop stores frames and notifies usb task
		rx_pi = can->rx_put_index;

		for (uint8_t i = 0, gio = can->m_can->RXF0S.bit.F0GI; i < count; ++i) {
			hw_rx_gi_mod = (gio + i) % MCAN_HW_RX_FIFO_SIZE;

			uint8_t rx_gi = __atomic_load_n(&can->rx_get_index, __ATOMIC_ACQUIRE);
			uint8_t used = rx_pi - rx_gi;
			SC_DEBUG_ASSERT(used <= SC_BOARD_CAN_RX_FIFO_SIZE);

			if (unlikely(used == SC_BOARD_CAN_RX_FIFO_SIZE)) {
				//__atomic_add_fetch(&can->rx_lost, 1, __ATOMIC_ACQ_REL);
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
				MCANX_RXF0E_0_Type const r0 = can->hw_rx_fifo_ram[hw_rx_gi_mod].R0;
				MCANX_RXF0E_1_Type const r1 = can->hw_rx_fifo_ram[hw_rx_gi_mod].R1;

				can->rx_frames[rx_pi_mod].R0 = r0;
				can->rx_frames[rx_pi_mod].R1 = r1;
				can->rx_frames[rx_pi_mod].ts = tsv[hw_rx_gi_mod];
				if (likely(!r0.bit.RTR)) {
					uint8_t can_frame_len = dlc_to_len(r1.bit.DLC);

					if (likely(can_frame_len)) {
						memcpy(can->rx_frames[rx_pi_mod].data, (void*)can->hw_rx_fifo_ram[hw_rx_gi_mod].data, can_frame_len);
					}
				}

				++rx_pi;

				// NOTE: This code is too slow to have here for some reason.
				// NOTE: If called outside this function, it is fast enough.
				// NOTE: Likely because of register / cache thrashing.

				// xTaskNotifyGive(can->usb_task_handle);
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
	}

	count = can->m_can->TXEFS.bit.EFFL;

	if (count) {
		// reverse loop reconstructs timestamps
		uint32_t ts = tsc;
		uint8_t hw_tx_gi_mod = 0;
		uint8_t tx_pi = 0;
		uint32_t txp = can->m_can->CCCR.bit.TXP * 2;
		uint32_t nmbr_bits, dtbr_bits;

		for (uint8_t i = 0, gio = can->m_can->TXEFS.bit.EFGI; i < count; ++i) {
			hw_tx_gi_mod = (gio + count - 1 - i) % MCAN_HW_TX_FIFO_SIZE;
			// LOG("ch%u poll tx count=%u gi=%u\n", index, count, hw_tx_gi_mod);

			tsv[hw_tx_gi_mod] = ts & SC_TS_MAX;

			can_frame_bits(
				can->hw_txe_fifo_ram[hw_tx_gi_mod].T0.bit.XTD,
				can->hw_txe_fifo_ram[hw_tx_gi_mod].T0.bit.RTR,
				can->hw_txe_fifo_ram[hw_tx_gi_mod].T1.bit.FDF,
				can->hw_txe_fifo_ram[hw_tx_gi_mod].T1.bit.BRS,
				can->hw_txe_fifo_ram[hw_tx_gi_mod].T1.bit.DLC,
				&nmbr_bits,
				&dtbr_bits);

			ts -= can_frame_time_us(index, nmbr_bits + txp, dtbr_bits);
		}

		// forward loop stores frames and notifies usb task
		tx_pi = can->tx_put_index;

		for (uint8_t i = 0, gio = can->m_can->TXEFS.bit.EFGI; i < count; ++i) {
			hw_tx_gi_mod = (gio + i) % MCAN_HW_TX_FIFO_SIZE;

			uint8_t tx_pi_mod = tx_pi % SC_BOARD_CAN_TX_FIFO_SIZE;
			// if (unlikely(target_put_index == can->rx_get_index)) {
			// 	__atomic_add_fetch(&can->rx_lost, 1, __ATOMIC_ACQ_REL);
			// } else {
				can->tx_frames[tx_pi_mod].T0 = can->hw_txe_fifo_ram[hw_tx_gi_mod].T0;
				can->tx_frames[tx_pi_mod].T1 = can->hw_txe_fifo_ram[hw_tx_gi_mod].T1;
				can->tx_frames[tx_pi_mod].ts = tsv[hw_tx_gi_mod];
				// LOG("ch%u tx place MM %u @ index %u\n", index, can->tx_frames[tx_pi_mod].T1.bit.MM, tx_pi_mod);

				//__atomic_store_n(&can->tx_put_index, target_put_index, __ATOMIC_RELEASE);
				//++can->tx_put_index;
				++tx_pi;
			// }

			// xTaskNotifyGive(can->usb_task_handle);
			// BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			// vTaskNotifyGiveFromISR(can->usb_task_handle, &xHigherPriorityTaskWoken);
			// portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			++*events;
		}

		// removes frames from tx fifo
		can->m_can->TXEFA.reg = MCANX_TXEFA_EFAI(hw_tx_gi_mod);
		// LOG("ch%u poll tx count=%u done\n", index, count);

		// atomic update of tx put index
		__atomic_store_n(&can->tx_put_index, tx_pi, __ATOMIC_RELEASE);
	}

	if (unlikely(rx_lost)) {
		sc_can_status status;

		status.type = SC_CAN_STATUS_FIFO_TYPE_RX_LOST;
		status.timestamp_us = tsc;
		status.rx_lost = rx_lost;

		sc_can_status_queue(index, &status);
		++*events;
	}
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


#endif // SUPERMCANX_MCAN

