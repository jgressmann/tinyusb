/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
 *
 */


#if defined(D5035_01) || defined(SAME54XPLAINEDPRO) || defined(FEATHER_M4_CAN_EXPRESS)

#include <supercan_debug.h>
#include <supercan_board.h>
#include <m_can.h>
#include <crc32.h>
#include <mcu.h>
#include <leds.h>

#include <hal/include/hal_gpio.h>
#include <usb_descriptors.h>

#ifndef ARRAY_SIZE
#	define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif

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


struct same5x_can same5x_cans[SC_BOARD_CAN_COUNT];

void same5x_can_init(void)
{
	memset(same5x_cans, 0, sizeof(same5x_cans));

	for (size_t i = 0; i < TU_ARRAY_SIZE(same5x_cans); ++i) {
		struct same5x_can *can = &same5x_cans[i];

		// init bit timings so we can always safelye compute the bit rate (see can_on)
		can->nm = sc_board_can_nm_bit_timing_range((uint8_t)i)->min;
		can->dt = sc_board_can_dt_bit_timing_range((uint8_t)i)->min;
	}
}

void same5x_can_configure(uint8_t index)
{
	struct same5x_can *c = &same5x_cans[index];
	Can *can = c->m_can;

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
	//can->TSCC.reg = CAN_TSCC_TSS_ZERO; // time stamp counter in CAN bittime

	// NOTE: this needs to be on, else we might actually wrap TC0/1
	// which would lead to mistallying of time on the host
	can->TSCC.reg = CAN_TSCC_TSS_INC;
	// reset default TOCC.ETOC = 0 (disabled)
	// can->TOCC.reg = CAN_TOCC_TOP(0xffff) | CAN_TOCC_TOS(0); // Timeout Counter disabled, Reset-default
	can->NBTP.reg = CAN_NBTP_NSJW(c->nm.sjw-1)
			| CAN_NBTP_NBRP(c->nm.brp-1)
			| CAN_NBTP_NTSEG1(c->nm.tseg1-1)
			| CAN_NBTP_NTSEG2(c->nm.tseg2-1);
	can->DBTP.reg = CAN_DBTP_DBRP(c->dt.brp-1)
			| CAN_DBTP_DTSEG1(c->dt.tseg1-1)
			| CAN_DBTP_DTSEG2(c->dt.tseg2-1)
			| CAN_DBTP_DSJW(c->dt.sjw-1)
			| (sc_bitrate(c->dt.brp, c->dt.tseg1, c->dt.tseg2) >= 1000000) * CAN_DBTP_TDC; // enable TDC for bitrates >= 1MBit/s

	// transmitter delay compensation offset
	// can->TDCR.bit.TDCO = tu_min8((1 + c->dtbt_tseg1 + c->dtbt_tseg2) / 2, M_CAN_TDCR_TDCO_MAX);
	can->TDCR.bit.TDCO = tu_min8((1 + c->dt.tseg1 - c->dt.tseg2 / 2), M_CAN_TDCR_TDCO_MAX);
	can->TDCR.bit.TDCF = tu_min8((1 + c->dt.tseg1 + c->dt.tseg2 / 2), M_CAN_TDCR_TDCO_MAX);


	// tx fifo
	can->TXBC.reg = CAN_TXBC_TBSA((uint32_t) c->tx_fifo) | CAN_TXBC_TFQS(SC_BOARD_CAN_TX_FIFO_SIZE);

	can->TXESC.reg = CAN_TXESC_TBDS_DATA64;

	// tx event fifo
	can->TXEFC.reg = CAN_TXEFC_EFSA((uint32_t) c->tx_event_fifo) | CAN_TXEFC_EFS(SC_BOARD_CAN_TX_FIFO_SIZE);


	// rx fifo0
	can->RXF0C.reg = CAN_RXF0C_F0SA((uint32_t) c->rx_fifo) | CAN_RXF0C_F0S(SC_BOARD_CAN_RX_FIFO_SIZE);
	//  | CAN_RXF0C_F0OM; // FIFO 0 overwrite mode
	can->RXESC.reg = CAN_RXESC_RBDS_DATA64 + CAN_RXESC_F0DS_DATA64;

	// enable interrupt line 0
	can->ILE.reg = CAN_ILE_EINT0;

	// wanted interrupts
	can->IE.reg =
		//
		0
		| CAN_IE_TSWE   // time stamp counter wrap
		| CAN_IE_BOE    // bus off
		| CAN_IE_EWE    // error warning
		| CAN_IE_EPE    // error passive
		| CAN_IE_RF0NE  // new message in rx fifo0
		| CAN_IE_RF0LE  // message lost b/c fifo0 was full
		| CAN_IE_PEAE   // proto error in arbitration phase
		| CAN_IE_PEDE   // proto error in data phase
		// | CAN_IE_ELOE   // error logging overflow
	 	| CAN_IE_TEFNE  // new message in tx event fifo
		| CAN_IE_MRAFE  // message RAM access failure
		| CAN_IE_BEUE   // bit error uncorrected, sets CCCR.INIT
		| CAN_IE_BECE   // bit error corrected
		// | CAN_IE_RF0WE
	;

	m_can_conf_end(can);
}

SC_RAMFUNC static inline uint8_t can_map_m_can_ec(uint8_t value)
{
	static const uint8_t can_map_m_can_ec_table[8] = {
		SC_CAN_ERROR_NONE,
		SC_CAN_ERROR_STUFF,
		SC_CAN_ERROR_FORM,
		SC_CAN_ERROR_ACK,
		SC_CAN_ERROR_BIT1,
		SC_CAN_ERROR_BIT0,
		SC_CAN_ERROR_CRC,
		0xff
	};

	return can_map_m_can_ec_table[value & 7];
}

SC_RAMFUNC static bool can_poll(uint8_t index, uint32_t * const events, uint32_t tsc);



SC_RAMFUNC static void can_int_update_status(uint8_t index, uint32_t* const events, uint32_t tsc)
{
	SC_DEBUG_ASSERT(events);

	struct same5x_can *can = &same5x_cans[index];
	uint8_t current_bus_state = 0;
	CAN_PSR_Type current_psr = can->m_can->PSR; // always read, sets NC
	CAN_ECR_Type current_ecr = can->m_can->ECR; // always read, clears CEL
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
			current_psr.reg &= CAN_PSR_BO;
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

	if (likely(CAN_PSR_LEC_NC_Val == lec)) {
		lec = can->int_prev_lec_no_nc;
	}

	if (likely(CAN_PSR_DLEC_NC_Val == dlec)) {
		dlec = can->int_prev_dlec_no_nc;
	}



	const bool is_tx_error = current_psr.bit.ACT == CAN_PSR_ACT_TX_Val;
	const bool report_error = (tsc - can->int_prev_error_ts) >= 1000; // once per millisecond, USB poll interval
	// bool is_rx_tx_error = current_psr.bit.ACT == CAN_PSR_ACT_RX_Val || is_tx_error;


	if ((lec != CAN_PSR_LEC_NONE_Val) &&
		((lec != can->int_prev_lec_no_nc) || report_error)) {
		// LOG("CAN%u lec %x\n", index, current_psr.bit.LEC);

		can->int_prev_error_ts = tsc;

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		status.timestamp_us = tsc;
		status.bus_error.tx = is_tx_error;
		status.bus_error.code = can_map_m_can_ec(lec),
		status.bus_error.data_part = 0;

		sc_can_status_queue(index, &status);
		++*events;
	}

	if ((dlec != CAN_PSR_DLEC_NONE_Val) &&
		((dlec != can->int_prev_dlec_no_nc) || report_error)) {
		// LOG("CAN%u dlec %x\n", index, current_psr.bit.DLEC);

		can->int_prev_error_ts = tsc;

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		status.timestamp_us = tsc;
		status.bus_error.tx = is_tx_error;
		status.bus_error.code = can_map_m_can_ec(dlec),
		status.bus_error.data_part = 1;

		sc_can_status_queue(index, &status);
		++*events;
	}

	// store updated lec, dlec
	can->int_prev_lec_no_nc = lec;
	can->int_prev_dlec_no_nc = dlec;
}

#if SUPERCAN_DEBUG
static int entry_count[SC_BOARD_CAN_COUNT];
#endif

SC_RAMFUNC void same5x_can_int(uint8_t index)
{
#if SUPERCAN_DEBUG
	int c = __atomic_add_fetch(&entry_count[index], 1, __ATOMIC_ACQ_REL);
	SC_ISR_ASSERT(c == 1);
#endif
	same5x_counter_1MHz_request_current_value();

	struct same5x_can *can = &same5x_cans[index];

	uint32_t events = 0;

	// LOG("IE=%08lx IR=%08lx\n", can->m_can->IE.reg, can->m_can->IR.reg);
	// bool notify_usb = false;

	// LOG(".");

	CAN_IR_Type ir = can->m_can->IR;

	// clear all interrupts
	can->m_can->IR = ir;

	if (ir.bit.TSW) {
		// always notify here to enable the host to keep track of CAN bus time
		++events;
	}


	if (unlikely(ir.bit.MRAF)) {
		LOG("CAN%u MRAF\n", index);
		SC_ISR_ASSERT(false && "MRAF");
		NVIC_SystemReset();
	}

	if (unlikely(ir.bit.BEU)) {
		LOG("CAN%u BEU\n", index);
		SC_ISR_ASSERT(false && "BEU");
		NVIC_SystemReset();
	}

	if (unlikely(ir.bit.BEC)) {
		LOG("CAN%u BEC\n", index);
	}

	// Do this late to increase likelyhood that the counter
	// is ready right away.
	const uint32_t tsc = same5x_counter_1MHz_wait_for_current_value();

	if (unlikely(ir.bit.RF0L)) {
		sc_can_status status;

		status.type = SC_CAN_STATUS_FIFO_TYPE_RX_LOST;
		status.timestamp_us = tsc;
		status.rx_lost = 1;

		sc_can_status_queue(index, &status);
		++events;
	}

	can_int_update_status(index, &events, tsc);

	if (ir.reg & (CAN_IR_TEFN | CAN_IR_RF0N)) {

		// LOG("CAN%u RX/TX\n", index);
		can_poll(index, &events, tsc);
	}

	// msg->tx_fifo_size = SC_BOARD_CAN_TX_FIFO_SIZE - can->m_can->TXFQS.bit.TFFL;
	// msg->rx_fifo_size = can->m_can->RXF0S.bit.F0FL;

	if (likely(events)) {
		// LOG(">");
		sc_can_notify_task_isr(index, events);
	}
#if SUPERCAN_DEBUG
	__atomic_add_fetch(&entry_count[index], -1, __ATOMIC_ACQ_REL);
#endif
}


static inline void can_set_state1(Can *can, IRQn_Type interrupt_id, bool enabled)
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
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(same5x_cans));

	struct same5x_can *can = &same5x_cans[index];


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
	can->int_prev_lec_no_nc = CAN_PSR_LEC_NONE_Val;
	can->int_prev_dlec_no_nc = CAN_PSR_DLEC_NONE_Val;

	// call this here to timestamp / last rx/tx values
	// since we won't get any further interrupts
	can->rx_get_index = 0;
	can->rx_put_index = 0;
	can->tx_get_index = 0;
	can->tx_put_index = 0;

#if SUPERCAN_DEBUG && SAME5X_DEBUG_TXR
	can->txr = 0;
#endif

	__atomic_thread_fence(__ATOMIC_RELEASE); // int_*
}

static inline void can_off(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(same5x_cans));

	struct same5x_can *can = &same5x_cans[index];

	// go bus off
	can_set_state1(can->m_can, can->interrupt_id, false);

	can_reset_task_state_unsafe(index);
}


static void can_on(uint8_t index)
{
	struct same5x_can *can = &same5x_cans[index];
	CAN_ECR_Type current_ecr;
	uint32_t dtbr = 0;

	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(same5x_cans));

	can->nm_us_per_bit = UINT32_C(1000000) / sc_bitrate(can->nm.brp, can->nm.tseg1, can->nm.tseg2);
	dtbr = SC_BOARD_CAN_CLK_HZ / sc_bitrate(can->dt.brp, can->dt.tseg1, can->dt.tseg2);

	if (likely(dtbr > 0)) {
		can->dt_us_per_bit_factor_shift8 = (UINT32_C(1000000) << 8) / dtbr;
	} else {
		can->dt_us_per_bit_factor_shift8 = 32;
	}

	same5x_can_configure(index);

	current_ecr = can->m_can->ECR;

	/*
	 * If we have errors from prior use, try to get rid of them.
	 *
	 * Unfortunately M_CAN can't be reset on its own wich means
	 * we need to tx/rx messages for the error counters to go down.
	 */
	if (current_ecr.bit.REC | current_ecr.bit.TEC) {
		CAN_ECR_Type previous_ecr;

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

			can->tx_fifo[put_index].T0.reg = UINT32_C(1) << 18; // CAN-ID 1, ESI=0, XTD=0, RTR=0
			can->tx_fifo[put_index].T1.reg = 0; // DLC=0, MM=0, no FDF, no BSR, no TXEF event, see EFC field, DS60001507E-page 1305

			can->m_can->TXBAR.reg = UINT32_C(1) << put_index;

			while ((can->m_can->IR.reg & (CAN_IR_RF0N)) != (CAN_IR_RF0N));

			// clear interrupts
			can->m_can->IR.reg = ~0;

			// LOG("ch%u RXF0S=%x TXFQS=%x TXEFS=%x\n", index, can->m_can->RXF0S.reg, can->m_can->TXFQS.reg, can->m_can->TXEFS.reg);

			// throw away rx msg, tx event
			can->m_can->RXF0A.reg = CAN_RXF0A_F0AI(can->m_can->RXF0S.bit.F0GI);

			current_ecr = can->m_can->ECR;

			// LOG("ch%u PSR=%x ECR=%x\n", index, can->m_can->PSR.reg, can->m_can->ECR.reg);
			// LOG("ch%u RXF0S=%x TXFQS=%x TXEFS=%x\n", index, can->m_can->RXF0S.reg, can->m_can->TXFQS.reg, can->m_can->TXEFS.reg);
		} while (current_ecr.reg != previous_ecr.reg);

		LOG("ch%u PSR=%x ECR=%x\n", index, can->m_can->PSR.reg, can->m_can->ECR.reg);

		m_can_init_begin(can->m_can);

		same5x_can_configure(index);
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
	struct same5x_can *can = &same5x_cans[index];

	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(same5x_cans));

	// disable CAN units, reset configuration & status
	can_off(index);

	can->features = CAN_FEAT_PERM;
	can->nm = nm_range.min;
	can->dt = dt_range.min;
}

static uint32_t device_identifier;

extern uint32_t sc_board_identifier(void)
{
	return device_identifier;
}


void same5x_init_device_identifier(void)
{
	uint32_t serial_number[4];
	int error = CRC32E_NONE;

	same51_get_serial_number(serial_number);

#if SUPERCAN_DEBUG
	char serial_buffer[64];
	memset(serial_buffer, '0', 32);
	char hex_buffer[16];
	int chars = usnprintf(hex_buffer, sizeof(hex_buffer), "%x", serial_number[0]);
	memcpy(&serial_buffer[8-chars], hex_buffer, chars);
	chars = usnprintf(hex_buffer, sizeof(hex_buffer), "%x", serial_number[1]);
	memcpy(&serial_buffer[16-chars], hex_buffer, chars);
	chars = usnprintf(hex_buffer, sizeof(hex_buffer), "%x", serial_number[2]);
	memcpy(&serial_buffer[24-chars], hex_buffer, chars);
	chars = usnprintf(hex_buffer, sizeof(hex_buffer), "%x", serial_number[3]);
	memcpy(&serial_buffer[32-chars], hex_buffer, chars);
	serial_buffer[32] = 0;
	LOG("SAM serial number %s\n", serial_buffer);
#endif

#if TU_LITTLE_ENDIAN == TU_BYTE_ORDER
	// swap integers so they have printf layout
	serial_number[0] = __builtin_bswap32(serial_number[0]);
	serial_number[1] = __builtin_bswap32(serial_number[1]);
	serial_number[2] = __builtin_bswap32(serial_number[2]);
	serial_number[3] = __builtin_bswap32(serial_number[3]);
#endif

	error = crc32f((uint32_t)serial_number, 16, CRC32E_FLAG_UNLOCK, &device_identifier);
	if (unlikely(error)) {
		device_identifier = serial_number[0];
		LOG("ERROR: failed to compute CRC32: %d. Using fallback device identifier\n", error);
	}

#if SUPERCAN_DEBUG
	memset(serial_buffer, '0', 8);
	chars = usnprintf(hex_buffer, sizeof(hex_buffer), "%x", device_identifier);
	memcpy(&serial_buffer[8-chars], hex_buffer, chars);
	serial_buffer[8] = 0;
	LOG("device identifier %s\n", serial_buffer);
#endif
}

extern void sc_board_can_reset(uint8_t index)
{
	// reset
	can_reset(index);
}

__attribute__((noreturn)) extern void sc_board_reset(void)
{
	NVIC_SystemReset();
	__unreachable();
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
	struct same5x_can *can = &same5x_cans[index];
	bool available = can->m_can->TXFQS.bit.TFQPI - can->m_can->TXFQS.bit.TFGI < SC_BOARD_CAN_TX_FIFO_SIZE;

	if (available) {
		uint32_t id = msg->can_id;
		uint8_t put_index = can->m_can->TXFQS.bit.TFQPI;
		CAN_TXBE_0_Type t0;
		CAN_TXBE_1_Type t1;

		t0.reg = (((msg->flags & SC_CAN_FRAME_FLAG_ESI) == SC_CAN_FRAME_FLAG_ESI) << CAN_TXBE_0_ESI_Pos)
			| (((msg->flags & SC_CAN_FRAME_FLAG_RTR) == SC_CAN_FRAME_FLAG_RTR) << CAN_TXBE_0_RTR_Pos)
			| (((msg->flags & SC_CAN_FRAME_FLAG_EXT) == SC_CAN_FRAME_FLAG_EXT) << CAN_TXBE_0_XTD_Pos)
			;

		if (msg->flags & SC_CAN_FRAME_FLAG_EXT) {
			t0.reg |= CAN_TXBE_0_ID(id);
		} else {
			t0.reg |= CAN_TXBE_0_ID(id << 18);
		}

		t1.reg = CAN_TXBE_1_EFC
			| CAN_TXBE_1_DLC(msg->dlc)
			| CAN_TXBE_1_MM(msg->track_id)
			| (((msg->flags & SC_CAN_FRAME_FLAG_FDF) == SC_CAN_FRAME_FLAG_FDF) << CAN_TXBE_1_FDF_Pos)
			| (((msg->flags & SC_CAN_FRAME_FLAG_BRS) == SC_CAN_FRAME_FLAG_BRS) << CAN_TXBE_1_BRS_Pos)
			;


		can->tx_fifo[put_index].T0 = t0;
		can->tx_fifo[put_index].T1 = t1;

		if (likely(!(msg->flags & SC_CAN_FRAME_FLAG_RTR))) {
			const unsigned can_frame_len = dlc_to_len(msg->dlc);

			if (likely(can_frame_len)) {
				memcpy(can->tx_fifo[put_index].data, msg->data, can_frame_len);
			}
		}

		can->m_can->TXBAR.reg = UINT32_C(1) << put_index;
#if SUPERCAN_DEBUG && SAME5X_DEBUG_TXR
		SC_DEBUG_ASSERT(!(can->txr & (UINT32_C(1) << msg->track_id)));

		can->txr |= UINT32_C(1) << msg->track_id;
#endif
	}

	return available;
}

SC_RAMFUNC extern int sc_board_can_retrieve(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	struct same5x_can *can = &same5x_cans[index];
	int result = 0;
	bool have_data_to_place = false;

	for (bool done = false; !done; ) {
		done = true;

		uint8_t rx_put_index = __atomic_load_n(&can->rx_put_index, __ATOMIC_ACQUIRE);

		if (can->rx_get_index != rx_put_index) {
			have_data_to_place = true;
			__atomic_thread_fence(__ATOMIC_ACQUIRE);


			uint8_t get_index = can->rx_get_index & (SC_BOARD_CAN_RX_FIFO_SIZE-1);
			uint8_t bytes = sizeof(struct sc_msg_can_rx);
			CAN_RXF0E_0_Type r0 = can->rx_frames[get_index].R0;
			CAN_RXF0E_1_Type r1 = can->rx_frames[get_index].R1;
			uint8_t can_frame_len = dlc_to_len(r1.bit.DLC);

			SC_DEBUG_ASSERT(rx_put_index - can->rx_get_index <= SC_BOARD_CAN_RX_FIFO_SIZE);

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


				uint32_t ts = can->rx_frames[get_index].ts;
	// #if SUPERCAN_DEBUG
	// 					uint32_t delta = (ts - rx_ts_last) & SC_TS_MAX;
	// 					bool rx_ts_ok = delta <= SC_TS_MAX / 4;
	// 					if (unlikely(!rx_ts_ok)) {
	// 						taskDISABLE_INTERRUPTS();
	// 						// SC_BOARD_CAN_init_begin(can);
	// 						LOG("ch%u rx gi=%u ts=%lx prev=%lx\n", index, get_index, ts, rx_ts_last);
	// 						for (unsigned i = 0; i < CAN_RX_FIFO_SIZE; ++i) {
	// 							LOG("ch%u rx gi=%u ts=%lx\n", index, i, can->rx_frames[i].ts);
	// 						}

	// 					}
	// 					SC_ASSERT(rx_ts_ok);
	// 					// LOG("ch%u rx gi=%u d=%lx\n", index, get_index, ts - rx_ts_last);
	// 					rx_ts_last = ts;
	// #endif

				msg->timestamp_us = ts;

				// LOG("ch%u rx ts %lu\n", index, msg->timestamp_us);

				if (r1.bit.FDF) {
					msg->flags |= SC_CAN_FRAME_FLAG_FDF;
					if (r1.bit.BRS) {
						msg->flags |= SC_CAN_FRAME_FLAG_BRS;
					}

					memcpy(msg->data, can->rx_frames[get_index].data, can_frame_len);
				} else {
					if (r0.bit.RTR) {
						msg->flags |= SC_CAN_FRAME_FLAG_RTR;
					} else {
						memcpy(msg->data, can->rx_frames[get_index].data, can_frame_len);
					}
				}

				// LOG("rx store %u bytes\n", bytes);
				// sc_dump_mem(msg, bytes);

				__atomic_store_n(&can->rx_get_index, can->rx_get_index+1, __ATOMIC_RELEASE);
			}
		}

		uint8_t tx_put_index = __atomic_load_n(&can->tx_put_index, __ATOMIC_ACQUIRE);
		if (can->tx_get_index != tx_put_index) {
			have_data_to_place = true;
			SC_DEBUG_ASSERT(tx_put_index - can->tx_get_index <= SC_BOARD_CAN_TX_FIFO_SIZE);

			uint8_t get_index = can->tx_get_index & (SC_BOARD_CAN_TX_FIFO_SIZE-1);
			struct sc_msg_can_txr *msg = NULL;
			if ((size_t)(tx_end - tx_ptr) >= sizeof(*msg)) {
				// LOG("1\n");
				__atomic_thread_fence(__ATOMIC_ACQUIRE);
				done = false;


				msg = (struct sc_msg_can_txr *)tx_ptr;
				// usb_can->tx_offsets[usb_can->tx_bank] += sizeof(*msg);
				tx_ptr += sizeof(*msg);
				result += sizeof(*msg);

				CAN_TXEFE_0_Type t0 = can->tx_frames[get_index].T0;
				CAN_TXEFE_1_Type t1 = can->tx_frames[get_index].T1;

				msg->id = SC_MSG_CAN_TXR;
				msg->len = sizeof(*msg);
				msg->track_id = t1.bit.MM;
#if SUPERCAN_DEBUG && SAME5X_DEBUG_TXR
				SC_DEBUG_ASSERT(can->txr & (UINT32_C(1) << msg->track_id));

				can->txr &= ~(UINT32_C(1) << msg->track_id);
#endif
				uint32_t ts = can->tx_frames[get_index].ts;
	// #if SUPERCAN_DEBUG
	// 					// bool tx_ts_ok = ts >= tx_ts_last || (TS_HI(tx_ts_last) == 0xffff && TS_HI(ts) == 0);
	// 					uint32_t delta = (ts - tx_ts_last) & SC_TS_MAX;
	// 					bool tx_ts_ok = delta <= SC_TS_MAX / 4;
	// 					if (unlikely(!tx_ts_ok)) {
	// 						taskDISABLE_INTERRUPTS();
	// 						// SC_BOARD_CAN_init_begin(can);
	// 						LOG("ch%u tx gi=%u ts=%lx prev=%lx\n", index, get_index, ts, tx_ts_last);
	// 						for (unsigned i = 0; i < SC_BOARD_CAN_TX_FIFO_SIZE; ++i) {
	// 							LOG("ch%u tx gi=%u ts=%lx\n", index, i, can->tx_frames[i].ts);
	// 						}

	// 					}
	// 					SC_ASSERT(tx_ts_ok);
	// 					// LOG("ch%u tx gi=%u d=%lx\n", index, get_index, ts - tx_ts_last);
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

				// LOG("2\n");

				__atomic_store_n(&can->tx_get_index, can->tx_get_index+1, __ATOMIC_RELEASE);
				// LOG("3\n");
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
	struct same5x_can *can = &same5x_cans[index];
	return can->nm_us_per_bit * nm + ((can->dt_us_per_bit_factor_shift8 * dt) >> 8);
}

#ifdef SUPERCAN_DEBUG
static volatile uint32_t rx_lost_reported[TU_ARRAY_SIZE(same5x_cans)];
// static volatile uint32_t rx_ts_last[TU_ARRAY_SIZE(same5x_cans)];
#endif

SC_RAMFUNC static bool can_poll(
	uint8_t index,
	uint32_t* const events,
	uint32_t tsc)
{
	struct same5x_can *can = &same5x_cans[index];

	uint32_t tsv[SC_BOARD_CAN_RX_FIFO_SIZE];
	uint8_t count = 0;
	unsigned rx_lost = 0;
	bool more = false;
	uint8_t pi = 0;

	count = can->m_can->RXF0S.bit.F0FL;

	if (count) {
		more = true;
// #ifdef SUPERCAN_DEBUG
// 		uint32_t us = tsc - rx_ts_last[index];
// 		rx_ts_last[index] = tsc;
// 		LOG("ch%u rx dt=%lu\n", index, (unsigned long)us);
// #endif
		// reverse loop reconstructs timestamps
		uint32_t ts = tsc;
		uint8_t get_index;
		for (uint8_t i = 0, gio = can->m_can->RXF0S.bit.F0GI; i < count; ++i) {
			get_index = (gio + count - 1 - i) & (SC_BOARD_CAN_RX_FIFO_SIZE-1);
			// LOG("ch%u ts rx count=%u gi=%u\n", index, count, get_index);

			tsv[get_index] = ts & SC_TS_MAX;

			uint32_t nmbr_bits, dtbr_bits;
			can_frame_bits(
				can->rx_fifo[get_index].R0.bit.XTD,
				can->rx_fifo[get_index].R0.bit.RTR,
				can->rx_fifo[get_index].R1.bit.FDF,
				can->rx_fifo[get_index].R1.bit.BRS,
				can->rx_fifo[get_index].R1.bit.DLC,
				&nmbr_bits,
				&dtbr_bits);

			// LOG("ch%u rx gi=%u xtd=%d rtr=%d fdf=%d brs=%d dlc=%d nmbr_bits=%lu dtbr_bits=%lu ts=%lx data us=%lu\n",
			// 	index, get_index, can->rx_fifo[get_index].R0.bit.XTD,
			// 	can->rx_fifo[get_index].R0.bit.RTR,
			// 	can->rx_fifo[get_index].R1.bit.FDF,
			// 	can->rx_fifo[get_index].R1.bit.BRS,
			// 	can->rx_fifo[get_index].R1.bit.DLC, nmbr_bits, dtbr_bits,
			// 	(unsigned long)ts,
			// 	(unsigned long)((can->dt_us_per_bit_factor_shift8 * dtbr_bits) >> 8));

			ts -= can_frame_time_us(index, nmbr_bits, dtbr_bits);
		}

		// forward loop stores frames and notifies usb task
		pi = can->rx_put_index;

		for (uint8_t i = 0, gio = can->m_can->RXF0S.bit.F0GI; i < count; ++i) {
			get_index = (gio + i) & (SC_BOARD_CAN_RX_FIFO_SIZE-1);

			uint8_t rx_get_index = __atomic_load_n(&can->rx_get_index, __ATOMIC_ACQUIRE);
			uint8_t used = pi - rx_get_index;
			SC_ASSERT(used <= SC_BOARD_CAN_RX_FIFO_SIZE);

			if (unlikely(used == SC_BOARD_CAN_RX_FIFO_SIZE)) {
				//__atomic_add_fetch(&can->rx_lost, 1, __ATOMIC_ACQ_REL);
				++rx_lost;

#if SUPERCAN_DEBUG
				{
					if (rx_lost_reported[index] + UINT32_C(1000000) <= tsc) {
						rx_lost_reported[index] = tsc;
						LOG("ch%u rx lost %lx pi=%u gi=%u\n", index, ts, pi, rx_get_index);
					}
				}
#endif
			} else {
				uint8_t const put_index = pi & (SC_BOARD_CAN_RX_FIFO_SIZE-1);
				CAN_RXF0E_0_Type const r0 = can->rx_fifo[get_index].R0;
				CAN_RXF0E_1_Type const r1 = can->rx_fifo[get_index].R1;

				can->rx_frames[put_index].R0 = r0;
				can->rx_frames[put_index].R1 = r1;
				can->rx_frames[put_index].ts = tsv[get_index];
				if (likely(!r0.bit.RTR)) {
					uint8_t can_frame_len = dlc_to_len(r1.bit.DLC);
					if (likely(can_frame_len)) {
						memcpy(can->rx_frames[put_index].data, can->rx_fifo[get_index].data, can_frame_len);
					}
				}

				++pi;

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
		can->m_can->RXF0A.reg = CAN_RXF0A_F0AI(get_index);

		// atomic update of rx put index
		__atomic_store_n(&can->rx_put_index, pi, __ATOMIC_RELEASE);
	}

	count = can->m_can->TXEFS.bit.EFFL;
	if (count) {
		more = true;

		// reverse loop reconstructs timestamps
		uint32_t ts = tsc;
		uint8_t get_index;
		uint32_t txp = can->m_can->CCCR.bit.TXP * 2;
		for (uint8_t i = 0, gio = can->m_can->TXEFS.bit.EFGI; i < count; ++i) {
			get_index = (gio + count - 1 - i) & (SC_BOARD_CAN_TX_FIFO_SIZE-1);
			// LOG("ch%u poll tx count=%u gi=%u\n", index, count, get_index);

			tsv[get_index] = ts & SC_TS_MAX;

			uint32_t nmbr_bits, dtbr_bits;
			can_frame_bits(
				can->tx_event_fifo[get_index].T0.bit.XTD,
				can->tx_event_fifo[get_index].T0.bit.RTR,
				can->tx_event_fifo[get_index].T1.bit.FDF,
				can->tx_event_fifo[get_index].T1.bit.BRS,
				can->tx_event_fifo[get_index].T1.bit.DLC,
				&nmbr_bits,
				&dtbr_bits);

			ts -= can_frame_time_us(index, nmbr_bits + txp, dtbr_bits);
		}

		// forward loop stores frames and notifies usb task
		pi = can->tx_put_index;

		for (uint8_t i = 0, gio = can->m_can->TXEFS.bit.EFGI; i < count; ++i) {
			get_index = (gio + i) & (SC_BOARD_CAN_TX_FIFO_SIZE-1);

			uint8_t put_index = pi & (SC_BOARD_CAN_TX_FIFO_SIZE-1);
			// if (unlikely(target_put_index == can->rx_get_index)) {
			// 	__atomic_add_fetch(&can->rx_lost, 1, __ATOMIC_ACQ_REL);
			// } else {
				can->tx_frames[put_index].T0 = can->tx_event_fifo[get_index].T0;
				can->tx_frames[put_index].T1 = can->tx_event_fifo[get_index].T1;
				can->tx_frames[put_index].ts = tsv[get_index];
				// LOG("ch%u tx place MM %u @ index %u\n", index, can->tx_frames[put_index].T1.bit.MM, put_index);

				//__atomic_store_n(&can->tx_put_index, target_put_index, __ATOMIC_RELEASE);
				//++can->tx_put_index;
				++pi;
			// }

			// xTaskNotifyGive(can->usb_task_handle);
			// BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			// vTaskNotifyGiveFromISR(can->usb_task_handle, &xHigherPriorityTaskWoken);
			// portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			++*events;
		}

		// removes frames from tx fifo
		can->m_can->TXEFA.reg = CAN_TXEFA_EFAI(get_index);
        // LOG("ch%u poll tx count=%u done\n", index, count);

		// atomic update of tx put index
		__atomic_store_n(&can->tx_put_index, pi, __ATOMIC_RELEASE);
	}

	if (unlikely(rx_lost)) {
		sc_can_status status;

		status.type = SC_CAN_STATUS_FIFO_TYPE_RX_LOST;
		status.timestamp_us = tsc;
		status.rx_lost = rx_lost;

		sc_can_status_queue(index, &status);
		++*events;
	}

	return more;
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
	struct same5x_can *can = &same5x_cans[index];

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
	struct same5x_can *can = &same5x_cans[index];

	can->nm = *bt;
}

extern void sc_board_can_dt_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	struct same5x_can *can = &same5x_cans[index];

	can->dt = *bt;
}




#endif
