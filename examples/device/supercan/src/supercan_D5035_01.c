/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Jean Gressmann <jean@0x42.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */



#ifdef D5035_01

#include <supercan_debug.h>
#include <supercan_board.h>
#include <m_can.h>
#include <crc32.h>

#include <hal/include/hal_gpio.h>
#include <mcu.h>
#include <dfu_ram.h>
#include <dfu_app.h>

#ifndef ARRAY_SIZE
#	define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif


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

	CAN_STATUS_FIFO_SIZE = 256,

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



#if SUPERDFU_APP
struct dfu_hdr dfu_hdr __attribute__((section(DFU_RAM_HDR_SECTION_NAME)));
static struct dfu_app_hdr dfu_app_hdr __attribute__((used,section(DFU_APP_HDR_SECTION_NAME))) = {
	.hdr_magic = DFU_APP_HDR_MAGIC_STRING,
	.hdr_version = DFU_APP_HDR_VERSION,
	.app_version_major = SUPERCAN_VERSION_MAJOR,
	.app_version_minor = SUPERCAN_VERSION_MINOR,
	.app_version_patch = SUPERCAN_VERSION_PATCH,
	.app_watchdog_timeout_s = 1,
	.app_name = SC_NAME,
};

static struct dfu_app_ftr dfu_app_ftr __attribute__((used,section(DFU_APP_FTR_SECTION_NAME))) = {
	.magic = DFU_APP_FTR_MAGIC_STRING,
};

static struct dfu {
	StaticTimer_t timer_mem;
	TimerHandle_t timer_handle;
} dfu;

static void dfu_timer_expired(TimerHandle_t t);
static void dfu_timer_expired(TimerHandle_t t)
{
	(void)t;

	LOG("DFU detach timer expired\n");

	/*
	 * This is wrong. DFU 1.1. specification wants us to wait for USB reset.
	 * However, without these lines, the device can't be updated on Windows using
	 * dfu-util. Updating from Linux works either way. All hail compatibility (sigh).
	 */
	dfu_request_dfu(1);
	NVIC_SystemReset();
}
#endif




struct can {
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
	CAN_PSR_Type int_prev_psr_reg;
	uint16_t rx_lost;
	uint16_t features;
	uint16_t tx_dropped;
	uint8_t int_comm_flags;
	uint8_t int_prev_bus_state;
	uint8_t led_status_green;
	uint8_t led_status_red;
	uint8_t led_traffic;
	uint8_t tx_available;
	uint8_t rx_get_index; // NOT an index, uses full range of type
	uint8_t rx_put_index; // NOT an index, uses full range of type
	uint8_t tx_get_index; // NOT an index, uses full range of type
	uint8_t tx_put_index; // NOT an index, uses full range of type

	bool enabled;
	bool desync;
};

struct can cans[SC_BOARD_CAN_COUNT];




 // controller and hardware specific setup of i/o pins for CAN
static inline void can_init_pins(void)
{
	// CAN0 port
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_HWSEL |           // upper half
		PORT_WRCONFIG_PINMASK(0x00c0) | // PA22/23
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(8) |         // I, CAN0, DS60001507E page 32, 910
		PORT_WRCONFIG_PMUXEN;

	// CAN1 port
	PORT->Group[1].WRCONFIG.reg =
		PORT_WRCONFIG_PINMASK(0xc000) | // PB14/15 = 0xc000, PB12/13 = 0x3000
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(7) |         // H, CAN1, DS60001507E page 32, 910
		PORT_WRCONFIG_PMUXEN;
}

static inline void can_init_clock(void) // controller and hardware specific setup of clock for the m_can module
{
	MCLK->AHBMASK.bit.CAN0_ = 1;
	MCLK->AHBMASK.bit.CAN1_ = 1;
	GCLK->PCHCTRL[CAN0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN0 to use GLCK0
	GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN1 to use GLCK0
}


static void can_configure(struct can *c)
{
	Can *can = c->m_can;

	m_can_conf_begin(can);

	can->CCCR.bit.EFBI = 1; // enable edge filtering
	can->CCCR.bit.BRSE = 1; // enable CAN-FD bitrate switching (only effective on CAN-FD mode if configured)

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


	// can_log_nominal_bit_timing(c);
	// can_log_data_bit_timing(c);

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
			| CAN_DBTP_TDC;

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


static void can_init_module(void)
{
	memset(&cans, 0, sizeof(cans));

	cans[0].m_can = CAN0;
	cans[1].m_can = CAN1;
	cans[0].interrupt_id = CAN0_IRQn;
	cans[1].interrupt_id = CAN1_IRQn;

	for (size_t j = 0; j < TU_ARRAY_SIZE(cans); ++j) {
		struct can *can = &cans[j];
		can->features = CAN_FEAT_PERM;

		for (size_t i = 0; i < TU_ARRAY_SIZE(cans[0].rx_fifo); ++i) {
			SC_DEBUG_ASSERT(can->rx_frames[i].ts == 0);
		}

		for (size_t i = 0; i < TU_ARRAY_SIZE(cans[0].tx_fifo); ++i) {
			can->tx_fifo[i].T1.bit.EFC = 1; // store tx events

			SC_DEBUG_ASSERT(can->tx_frames[i].ts == 0);
		}
	}

	m_can_init_begin(CAN0);
	m_can_init_begin(CAN1);

	NVIC_SetPriority(CAN0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(CAN1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
}

static inline char const *m_can_psr_act_str(uint8_t act)
{
	switch (act) {
	case CAN_PSR_ACT_SYNC_Val:
		return "sync";
	case CAN_PSR_ACT_IDLE_Val:
		return "idle";
	case CAN_PSR_ACT_RX_Val:
		return "rx";
	default:
		return "tx";
	}
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

SC_RAMFUNC static bool can_poll(uint8_t index, uint32_t *events, uint32_t tsc);

SC_RAMFUNC static void sat_u16(volatile uint16_t* sat)
{
	uint16_t prev = 0;
	uint16_t curr = 0;

	do {
		curr = __atomic_load_n(sat, __ATOMIC_ACQUIRE);

		if (curr == 0xffff) {
			break;
		}

		prev = curr;
		++curr;
	} while (!__atomic_compare_exchange_n(sat, &prev, curr, 0 /* weak */, __ATOMIC_RELEASE, __ATOMIC_RELAXED));
}

SC_RAMFUNC static inline void can_inc_sat_rx_lost(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans));

	struct can *can = &cans[index];

	sat_u16(&can->rx_lost);
}



static inline void counter_1MHz_init_clock(void)
{
	// TC0 and TC1 pair to form a single 32 bit counter
	// TC1 is enslaved to TC0 and doesn't need to be configured.
	// DS60001507E-page 1716, 48.6.2.4 Counter Mode
	MCLK->APBAMASK.bit.TC0_ = 1;
	MCLK->APBAMASK.bit.TC1_ = 1;
	GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN; /* setup TC0 to use GLCK2 */
	GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN; /* setup TC1 to use GLCK2 */
}

// SC_RAMFUNC static inline void counter_1MHz_request_current_value(void)
// {
// 	TC0->COUNT32.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_READSYNC_Val;
// }

#define counter_1MHz_request_current_value() do { TC0->COUNT32.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_READSYNC_Val; } while (0)




// SC_RAMFUNC static inline bool counter_1MHz_is_current_value_ready(void)
// {
// 	return (__atomic_load_n(&TC0->COUNT32.CTRLBSET.reg, __ATOMIC_ACQUIRE) & TC_CTRLBSET_CMD_Msk) == TC_CTRLBSET_CMD_NONE;
// 	//return TC0->COUNT32.CTRLBSET.bit.CMD == TC_CTRLBSET_CMD_NONE_Val;
// }

#define counter_1MHz_is_current_value_ready() ((__atomic_load_n(&TC0->COUNT32.CTRLBSET.reg, __ATOMIC_ACQUIRE) & TC_CTRLBSET_CMD_Msk) == TC_CTRLBSET_CMD_NONE)


// SC_RAMFUNC static inline uint32_t counter_1MHz_read_unsafe(void)
// {
// 	return TC0->COUNT32.COUNT.reg & SC_TS_MAX;
// }

#define counter_1MHz_read_unsafe() (TC0->COUNT32.COUNT.reg & SC_TS_MAX)


static inline void counter_1MHz_reset(void)
{
	TC0->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
	while(1 == TC0->COUNT32.SYNCBUSY.bit.SWRST);

	// 16MHz -> 1MHz
	TC0->COUNT32.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV16;
	while(1 == TC0->COUNT32.SYNCBUSY.bit.ENABLE);
}


// SC_RAMFUNC static inline uint32_t counter_1MHz_wait_for_current_value(void)
// {
// 	while (!counter_1MHz_is_current_value_ready()) {
// 		;
// 	}

// 	return counter_1MHz_read_unsafe();
// }

#define counter_1MHz_wait_for_current_value() \
	({ \
		while (!counter_1MHz_is_current_value_ready()); \
		uint32_t counter = counter_1MHz_read_unsafe(); \
		counter; \
	})

SC_RAMFUNC static void can_int_update_status(uint8_t index, uint32_t* events, uint32_t tsc)
{
	// SC_ISR_ASSERT(index < TU_ARRAY_SIZE(cans));
	SC_DEBUG_ASSERT(events);

	struct can *can = &cans[index];
	uint8_t current_bus_state = 0;
	CAN_PSR_Type current_psr = can->m_can->PSR; // always read, sets NC
	CAN_PSR_Type prev_psr = can->int_prev_psr_reg;
	CAN_ECR_Type current_ecr = can->m_can->ECR; // always read, clears CEL
	(void)current_ecr;
	sc_can_status status;

	// update NC (no change) from last value (which might also be NC)
	if (CAN_PSR_LEC_NC_Val == current_psr.bit.LEC) {
		current_psr.bit.LEC = prev_psr.bit.LEC;
	}
	if (CAN_PSR_DLEC_NC_Val == current_psr.bit.DLEC) {
		current_psr.bit.DLEC = prev_psr.bit.DLEC;
	}

	// store updated psr reg
	can->int_prev_psr_reg = current_psr;

	if (current_psr.bit.BO) {
		current_bus_state = SC_CAN_STATUS_BUS_OFF;
		if (!prev_psr.bit.BO) {
			LOG("CAN%u bus off\n", index);
		}
	} else if (current_psr.bit.EP) {
		current_bus_state = SC_CAN_STATUS_ERROR_PASSIVE;
		if (!prev_psr.bit.EP) {
			LOG("CAN%u error passive\n", index);
		}
	} else if (current_psr.bit.EW) {
		current_bus_state = SC_CAN_STATUS_ERROR_WARNING;
		if (!prev_psr.bit.EW) {
			LOG("CAN%u error warning\n", index);
		}
	} else {
		current_bus_state = SC_CAN_STATUS_ERROR_ACTIVE;
	 	if (can->int_prev_bus_state != SC_CAN_STATUS_ERROR_ACTIVE) {
			LOG("CAN%u error active\n", index);
		}
	}

	if (unlikely(can->int_prev_bus_state != current_bus_state)) {
		can->int_prev_bus_state = current_bus_state;

		// uint16_t pi = can->status_put_index;
		// uint16_t gi = __atomic_load_n(&can->status_get_index, __ATOMIC_ACQUIRE);
		// uint16_t used = pi - gi;

		// LOG("CAN%u bus status update\n", index);


		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_STATUS;
		status.payload = current_bus_state;
		status.timestamp_us = tsc;
		sc_can_status_queue(index, &status);



		// if (likely(used < CAN_STATUS_FIFO_SIZE)) {
		// 	uint16_t fifo_index = pi & (CAN_STATUS_FIFO_SIZE-1);
		// 	struct can_status *s = &can->status_fifo[fifo_index];

		// 	s->type = SC_CAN_STATUS_FIFO_TYPE_BUS_STATUS;
		// 	s->payload = current_bus_state;
		// 	s->ts = tsc;

		// 	__atomic_store_n(&can->status_put_index, pi + 1, __ATOMIC_RELEASE);
		// } else {
		// 	__sync_or_and_fetch(&can->int_comm_flags, SC_CAN_STATUS_FLAG_IRQ_QUEUE_FULL);
		// }

		++*events;
	}

	bool is_tx_error = current_psr.bit.ACT == CAN_PSR_ACT_TX_Val;
	// bool is_rx_tx_error = current_psr.bit.ACT == CAN_PSR_ACT_RX_Val || is_tx_error;
	// bool had_nm_error = prev_psr.bit.LEC != CAN_PSR_LEC_NONE_Val && prev_psr.bit.LEC != CAN_PSR_LEC_NC_Val;


	// if (current_psr.bit.LEC != prev_psr.bit.LEC &&
	// 	current_psr.bit.LEC != CAN_PSR_LEC_NONE_Val &&
	// 	current_psr.bit.LEC != CAN_PSR_LEC_NC_Val &&
	// 	/* is_rx_tx_error */ true) {
	if (current_psr.bit.LEC != CAN_PSR_LEC_NONE_Val && current_psr.bit.LEC != CAN_PSR_LEC_NC_Val) {
		// uint16_t pi = can->status_put_index;
		// uint16_t gi = __atomic_load_n(&can->status_get_index, __ATOMIC_ACQUIRE);
		// uint16_t used = pi - gi;

		// LOG("CAN%u lec %x\n", index, current_psr.bit.LEC);

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		status.tx = is_tx_error;
		status.timestamp_us = tsc;
		status.payload = can_map_m_can_ec(current_psr.bit.LEC),
		status.data_part = 0;
		sc_can_status_queue(index, &status);

		// if (likely(used < CAN_STATUS_FIFO_SIZE)) {
		// 	uint16_t fifo_index = pi & (CAN_STATUS_FIFO_SIZE-1);
		// 	struct can_status *s = &can->status_fifo[fifo_index];

		// 	s->type = SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		// 	s->tx = is_tx_error;
		// 	s->data_part = 0;
		// 	s->payload = can_map_m_can_ec(current_psr.bit.LEC),
		// 	s->ts = tsc;

		// 	__atomic_store_n(&can->status_put_index, pi + 1, __ATOMIC_RELEASE);
		// } else {
		// 	__sync_or_and_fetch(&can->int_comm_flags, SC_CAN_STATUS_FLAG_IRQ_QUEUE_FULL);
		// }

		++*events;
	}

	// bool had_dt_error = prev_psr.bit.DLEC != CAN_PSR_DLEC_NONE_Val && prev_psr.bit.DLEC != CAN_PSR_DLEC_NC_Val;
	// if (current_psr.bit.DLEC != prev_psr.bit.DLEC &&
	// 	current_psr.bit.DLEC != CAN_PSR_DLEC_NONE_Val &&
	// 	current_psr.bit.DLEC != CAN_PSR_DLEC_NC_Val &&
	// 	/*is_rx_tx_error*/ true) {
	if (current_psr.bit.DLEC != CAN_PSR_DLEC_NONE_Val && current_psr.bit.DLEC != CAN_PSR_DLEC_NC_Val) {
		// uint16_t pi = can->status_put_index;
		// uint16_t gi = __atomic_load_n(&can->status_get_index, __ATOMIC_ACQUIRE);
		// uint16_t used = pi - gi;

		// LOG("CAN%u dlec %x\n", index, current_psr.bit.DLEC);

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		status.tx = is_tx_error;
		status.timestamp_us = tsc;
		status.payload = can_map_m_can_ec(current_psr.bit.LEC),
		status.data_part = 1;
		sc_can_status_queue(index, &status);

		// if (likely(used < CAN_STATUS_FIFO_SIZE)) {
		// 	uint16_t fifo_index = pi & (CAN_STATUS_FIFO_SIZE-1);
		// 	struct can_status *s = &can->status_fifo[fifo_index];

		// 	s->type = CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		// 	s->tx = is_tx_error;
		// 	s->data_part = 1;
		// 	s->payload = can_map_m_can_ec(current_psr.bit.DLEC),
		// 	s->ts = tsc;

		// 	__atomic_store_n(&can->status_put_index, pi + 1, __ATOMIC_RELEASE);
		// } else {
		// 	__sync_or_and_fetch(&can->int_comm_flags, SC_CAN_STATUS_FLAG_IRQ_QUEUE_FULL);
		// }

		++*events;
	}

	// if ((had_nm_error || had_dt_error) &&
	// 	(current_psr.bit.LEC == CAN_PSR_LEC_NONE_Val && current_psr.bit.DLEC == CAN_PSR_DLEC_NONE_Val)) {

	// 	uint16_t pi = can->status_put_index;
	// 	uint16_t gi = __atomic_load_n(&can->status_get_index, __ATOMIC_ACQUIRE);
	// 	uint16_t used = pi - gi;

	// 	LOG("CAN%u no error\n", index);

	// 	if (likely(used < TU_ARRAY_SIZE(can->status_fifo))) {
	// 		uint16_t fifo_index = pi & (CAN_STATUS_FIFO_SIZE-1);
	// 		struct can_status *s = &can->status_fifo[fifo_index];

	// 		s->type = CAN_STATUS_FIFO_TYPE_BUS_ERROR;
	// 		s->tx = 0;
	// 		s->data_part = 0;
	// 		s->payload = SC_CAN_ERROR_NONE,
	// 		s->ts = counter_1MHz_wait_for_current_value();

	// 		__atomic_store_n(&can->status_put_index, pi + 1, __ATOMIC_RELEASE);
	// 	} else {
	// 		__sync_or_and_fetch(&can->int_comm_flags, SC_CAN_STATUS_FLAG_IRQ_QUEUE_FULL);
	// 	}
	//
	//  ++*events;
	// }
}

SC_RAMFUNC static void can_int(uint8_t index)
{
	counter_1MHz_request_current_value();

	// SC_ISR_ASSERT(index < TU_ARRAY_SIZE(cans));
	struct can *can = &cans[index];

	uint32_t events = 0;



	// if (can->m_can->CCCR.reg & CAN_CCCR_CCE) { // config mode sentinal
	// 	LOG("CAN%u CCE\n", index);
	// 	return;
	// }

	// if (can->m_can->CCCR.reg & CAN_CCCR_INIT) {
	// 	LOG("CAN%u INIT\n", index);
	// 	return;
	// }

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



	if (ir.bit.RF0L) {
		LOG("CAN%u msg lost\n", index);
		//__atomic_add_fetch(&can->rx_lost, 1, __ATOMIC_ACQ_REL);
		can_inc_sat_rx_lost(index);
		// notify = true;
	}

	// Do this late to increase likelyhood that the counter
	// is ready right away.
	uint32_t tsc = counter_1MHz_wait_for_current_value();

	can_int_update_status(index, &events, tsc);

	if (ir.reg & (CAN_IR_TEFN | CAN_IR_RF0N)) {

		// LOG("CAN%u RX/TX\n", index);
		can_poll(index, &events, tsc);

	}

	sc_can_notify_task_isr(index, events);
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
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans));

	struct can *can = &cans[index];


#if SUPERCAN_DEBUG
	memset(can->rx_frames, 0, sizeof(can->rx_frames));
	memset(can->tx_frames, 0, sizeof(can->tx_frames));
	// memset(can->status_fifo, 0, sizeof(can->status_fifo));
#endif

	can->rx_lost = 0;
	can->tx_dropped = 0;
	can->tx_available = SC_BOARD_CAN_TX_FIFO_SIZE;
	can->desync = false;
	can->int_prev_bus_state = 0;
	can->int_comm_flags = 0;
	can->int_prev_psr_reg.reg = 0;

	// call this here to timestamp / last rx/tx values
	// since we won't get any further interrupts
	can->rx_get_index = 0;
	can->rx_put_index = 0;
	can->tx_get_index = 0;
	can->tx_put_index = 0;
	// can->status_put_index = 0;
	// can->status_get_index = 0;

	__atomic_thread_fence(__ATOMIC_RELEASE); // rx_lost
}

static inline void can_off(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans));

	struct can *can = &cans[index];

	// go bus off
	can_set_state1(can->m_can, can->interrupt_id, false);

	// // _With_ usb lock, since this isn't the interrupt handler
	// // and neither a higher priority task.
	// while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));

	// Call this with the lock held so the tasks
	// don't read an inconsistent state
	can_reset_task_state_unsafe(index);

	// mark CAN as disabled
	can->enabled = false;

	sc_board_led_can_status_set(index, SC_CAN_LED_STATUS_ENABLED_BUS_OFF);

	// xSemaphoreGive(usb_can->mutex_handle);

	// // notify task to make sure its knows
	// xTaskNotifyGive(can->usb_task_handle);
}


static inline void can_on(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans));


	struct can *can = &cans[index];

	// // _With_ usb lock, since this isn't the interrupt handler
	// // and neither a higher priority task.
	// while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));

	can->nm_us_per_bit = UINT32_C(1000000) / (SC_BOARD_CAN_CLK_HZ / ((uint32_t)can->nm.brp * (1 + can->nm.tseg1 + can->nm.tseg2)));
	uint32_t dtbr = SC_BOARD_CAN_CLK_HZ / ((uint32_t)can->dt.brp * (1 + can->dt.tseg1 + can->dt.tseg2));
	can->dt_us_per_bit_factor_shift8 = (UINT32_C(1000000) << 8) / dtbr;

	// mark CAN as enabled
	can->enabled = true;

	can_configure(can);

	can_set_state1(can->m_can, can->interrupt_id, can->enabled);

	SC_ASSERT(!m_can_tx_event_fifo_avail(can->m_can));

	sc_board_led_can_status_set(index, SC_CAN_LED_STATUS_ENABLED_BUS_ON_PASSIVE);

	// xSemaphoreGive(usb_can->mutex_handle);

	// // notify task to make sure its knows
	// xTaskNotifyGive(can->usb_task_handle);
}

static inline void can_reset(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans));
	// SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));

	// disable CAN units, reset configuration & status
	can_off(index);

	struct can *can = &cans[index];

	can->features = CAN_FEAT_PERM;
	memset(&can->nm, 0, sizeof(can->nm));
	memset(&can->dt, 0, sizeof(can->dt));
}


struct led {
	uint8_t pin;
};

// struct can {
// 	Can *m_can;
// 	IRQn_Type interrupt_id;
// 	uint8_t led_status_green;
// 	uint8_t led_status_red;
// 	uint8_t led_traffic;
// };

#define LED_STATIC_INITIALIZER(name, pin) \
	{ pin }


static const struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PA02), // board led
	LED_STATIC_INITIALIZER("red", PIN_PA18),
	LED_STATIC_INITIALIZER("orange", PIN_PA19),
	LED_STATIC_INITIALIZER("green", PIN_PB16),
	LED_STATIC_INITIALIZER("blue", PIN_PB17),
	LED_STATIC_INITIALIZER("can1_red", PIN_PB00),
	LED_STATIC_INITIALIZER("can1_green", PIN_PB01),
	LED_STATIC_INITIALIZER("can0_red", PIN_PB02),
	LED_STATIC_INITIALIZER("can0_green", PIN_PB03),
};

extern void sc_board_led_init(void)
{
	PORT->Group[0].DIRSET.reg = PORT_PA18 | PORT_PA19;
	PORT->Group[1].DIRSET.reg = PORT_PB16 | PORT_PB17 | PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03;
}

#define POWER_LED LED_DEBUG_0
#define CAN0_TRAFFIC_LED LED_DEBUG_1
#define CAN1_TRAFFIC_LED LED_DEBUG_2
#define USB_LED LED_DEBUG_3


extern void sc_board_led_set(uint8_t index, bool on)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(leds));

	gpio_set_pin_level(leds[index].pin, on);
}


extern void sc_board_leds_on_unsafe(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(leds); ++i) {
		gpio_set_pin_level(leds[i].pin, 1);
	}
}



extern void sc_board_can_init_module(void)
{
	cans[0].interrupt_id = CAN0_IRQn;
	cans[0].m_can = CAN0;
	NVIC_SetPriority(CAN0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	cans[0].led_traffic = CAN0_TRAFFIC_LED;
	cans[1].interrupt_id = CAN1_IRQn;
	cans[1].m_can = CAN1;
	NVIC_SetPriority(CAN1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	cans[1].led_traffic = CAN1_TRAFFIC_LED;

	cans[0].led_status_green = LED_CAN0_STATUS_GREEN;
	cans[0].led_status_red = LED_CAN0_STATUS_RED;
	cans[1].led_status_green = LED_CAN1_STATUS_GREEN;
	cans[1].led_status_red = LED_CAN1_STATUS_RED;
}


// controller and hardware specific setup of i/o pins for CAN
extern void sc_board_can_init_pins(void)
{
	// CAN0 port
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_HWSEL |           // upper half
		PORT_WRCONFIG_PINMASK(0x00c0) | // PA22/23
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(8) |         // I, CAN0, DS60001507E page 32, 910
		PORT_WRCONFIG_PMUXEN;
#if SC_BOARD_CAN_COUNT > 1
	// CAN1 port
	PORT->Group[1].WRCONFIG.reg =
		PORT_WRCONFIG_PINMASK(0xc000) | // PB14/15 = 0xc000, PB12/13 = 0x3000
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(7) |         // H, CAN1, DS60001507E page 32, 910
		PORT_WRCONFIG_PMUXEN;
#endif
}

extern void sc_board_can_init_clock(void) // controller and hardware specific setup of clock for the m_can module
{
	MCLK->AHBMASK.bit.CAN0_ = 1;
	GCLK->PCHCTRL[CAN0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN1 to use GLCK0
#if SC_BOARD_CAN_COUNT > 1
	MCLK->AHBMASK.bit.CAN1_ = 1;
	GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN1 to use GLCK0
#endif
}

extern void sc_board_can_interrupt_enable(uint8_t index, bool on)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(cans));

	if (on) {
		NVIC_EnableIRQ(cans[index].interrupt_id);
	} else {
		NVIC_DisableIRQ(cans[index].interrupt_id);
	}
}

extern void* sc_board_can_m_can(uint8_t index)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(cans));
	return cans[index].m_can;
}

extern void sc_board_can_burst_led(uint8_t index, uint16_t duration_ms)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(cans));

	led_burst(cans[index].led_traffic, duration_ms);
}


extern void sc_board_can_led_set_status(uint8_t index, int status)
{
	const uint16_t BLINK_DELAY_PASSIVE_MS = 512;
	const uint16_t BLINK_DELAY_ACTIVE_MS = 128;
	struct can* can = &cans[index];

	SC_DEBUG_ASSERT(index < ARRAY_SIZE(cans));

	switch (status) {
	case SC_CAN_LED_STATUS_DISABLED:
		led_set(can->led_status_green, 0);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_BUS_OFF:
		led_set(can->led_status_green, 1);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_BUS_ON_PASSIVE:
		led_blink(can->led_status_green, BLINK_DELAY_PASSIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_BUS_ON_ACTIVE:
		led_blink(can->led_status_green, BLINK_DELAY_ACTIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ERROR_PASSIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, BLINK_DELAY_ACTIVE_MS);
		break;
	case SC_CAN_LED_STATUS_ERROR_ACTIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, BLINK_DELAY_ACTIVE_MS);
		break;
	}
}




extern void sc_board_power_led_on(void)
{
	led_set(POWER_LED, 1);
}

extern void sc_board_usb_led_burst(uint16_t duration_ms)
{
	led_burst(USB_LED, duration_ms);
}



extern void sc_board_counter_1MHz_init(void)
{
	// TC0 and TC1 pair to form a single 32 bit counter
	// TC1 is enslaved to TC0 and doesn't need to be configured.
	// DS60001507E-page 1716, 48.6.2.4 Counter Mode
	MCLK->APBAMASK.bit.TC0_ = 1;
	MCLK->APBAMASK.bit.TC1_ = 1;
	GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN; /* setup TC0 to use GLCK2 */
	GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN; /* setup TC1 to use GLCK2 */

	TC0->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
	while(1 == TC0->COUNT32.SYNCBUSY.bit.SWRST);

	// 16MHz -> 1MHz
	TC0->COUNT32.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV16;
	while(1 == TC0->COUNT32.SYNCBUSY.bit.ENABLE);
}

static uint32_t device_identifier;

extern uint32_t sc_board_identifier(void)
{
	return device_identifier;
}

static void init_device_identifier(void)
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

extern uint32_t _svectors;
extern uint32_t _evectors;

static void move_vector_table_to_ram(void)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
	void* vectors_ram = (void*)(uint32_t)&_svectors;

	memcpy(vectors_ram, (void*)SCB->VTOR, MCU_VECTOR_TABLE_ALIGNMENT);
	SCB->VTOR = (uint32_t)vectors_ram;
#pragma GCC diagnostic pop
}

static inline void enable_cache(void)
{
	// DS60001507E-page 83
	if (!CMCC->SR.bit.CSTS) {
		CMCC->CTRL.bit.CEN = 1;
	}
}

extern void sc_board_init_begin(void)
{
	board_init();

	LOG("Vectors ROM @ %p\n", (void*)SCB->VTOR);
	move_vector_table_to_ram();
	LOG("Vectors RAM @ %p\n", (void*)SCB->VTOR);

	LOG("Enabling cache\n");
	enable_cache();

	init_device_identifier();


#if SUPERDFU_APP
	LOG(
		"%s v%u.%u.%u starting...\n",
		dfu_app_hdr.app_name,
		dfu_app_hdr.app_version_major,
		dfu_app_hdr.app_version_minor,
		dfu_app_hdr.app_version_patch);

	dfu_request_dfu(0); // no bootloader request

	dfu.timer_handle = xTimerCreateStatic("dfu", pdMS_TO_TICKS(DFU_USB_RESET_TIMEOUT_MS), pdFALSE, NULL, &dfu_timer_expired, &dfu.timer_mem);
#endif

	can_init_pins();
	can_init_clock();
	can_init_module();

	counter_1MHz_init_clock();
	counter_1MHz_reset();
}

extern void sc_board_init_end(void)
{
	led_blink(0, 2000);
	led_set(POWER_LED, 1);

#if SUPERDFU_APP
	dfu_app_watchdog_disable();
#endif
}

extern void sc_board_can_reset(uint8_t index)
{
	// reset
	can_reset(index);
	sc_board_led_can_status_set(index, SC_CAN_LED_STATUS_ENABLED_BUS_OFF);


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
	struct can *can = &cans[index];
	bool queued = false;

	if (can->tx_available) {
		--can->tx_available;

		uint32_t id = msg->can_id;
		uint8_t put_index = can->m_can->TXFQS.bit.TFQPI;

		CAN_TXBE_0_Type t0;
		t0.reg = (((msg->flags & SC_CAN_FRAME_FLAG_ESI) == SC_CAN_FRAME_FLAG_ESI) << CAN_TXBE_0_ESI_Pos)
			| (((msg->flags & SC_CAN_FRAME_FLAG_RTR) == SC_CAN_FRAME_FLAG_RTR) << CAN_TXBE_0_RTR_Pos)
			| (((msg->flags & SC_CAN_FRAME_FLAG_EXT) == SC_CAN_FRAME_FLAG_EXT) << CAN_TXBE_0_XTD_Pos)
			;

		if (msg->flags & SC_CAN_FRAME_FLAG_EXT) {
			t0.reg |= CAN_TXBE_0_ID(id);
		} else {
			t0.reg |= CAN_TXBE_0_ID(id << 18);
		}

		can->tx_fifo[put_index].T0 = t0;
		can->tx_fifo[put_index].T1.bit.DLC = msg->dlc;
		can->tx_fifo[put_index].T1.bit.FDF = (msg->flags & SC_CAN_FRAME_FLAG_FDF) == SC_CAN_FRAME_FLAG_FDF;
		can->tx_fifo[put_index].T1.bit.BRS = (msg->flags & SC_CAN_FRAME_FLAG_BRS) == SC_CAN_FRAME_FLAG_BRS;
		can->tx_fifo[put_index].T1.bit.MM = msg->track_id;

		if (likely(!(msg->flags & SC_CAN_FRAME_FLAG_RTR))) {
			const unsigned can_frame_len = dlc_to_len(msg->dlc);

			if (likely(can_frame_len)) {
				memcpy(can->tx_fifo[put_index].data, msg->data, can_frame_len);
			}
		}

		can->m_can->TXBAR.reg = UINT32_C(1) << put_index;

		queued = true;
	}

	return queued;
}

SC_RAMFUNC extern void sc_board_can_status_fill(uint8_t index, struct sc_msg_can_status *msg)
{
	struct can *can = &cans[index];

	uint16_t rx_lost = __sync_fetch_and_and(&can->rx_lost, 0);

	// LOG("status ts %lu\n", ts);
	CAN_ECR_Type ecr = can->m_can->ECR;


	msg->rx_lost = rx_lost;
	msg->flags = __sync_or_and_fetch(&can->int_comm_flags, 0);
	msg->bus_status = 0; // FIX ME
	msg->tx_errors = ecr.bit.TEC;
	msg->rx_errors = ecr.bit.REC;
	msg->tx_fifo_size = SC_BOARD_CAN_TX_FIFO_SIZE - can->m_can->TXFQS.bit.TFFL;
	msg->rx_fifo_size = can->m_can->RXF0S.bit.F0FL;
}

SC_RAMFUNC extern int sc_board_can_place_msgs(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	struct can *can = &cans[index];
	int result = 0;
	bool have_data_to_place = false;


	for (bool done = false; !done; ) {
		done = true;

		uint8_t rx_put_index = __atomic_load_n(&can->rx_put_index, __ATOMIC_ACQUIRE);

		if (can->rx_get_index != rx_put_index) {
			have_data_to_place = true;
			__atomic_thread_fence(__ATOMIC_ACQUIRE);
			uint8_t rx_count = rx_put_index - can->rx_get_index;
			if (unlikely(rx_count > SC_BOARD_CAN_RX_FIFO_SIZE)) {
				LOG("ch%u rx count %u\n", index, rx_count);
				SC_ASSERT(rx_put_index - can->rx_get_index <= SC_BOARD_CAN_RX_FIFO_SIZE);
			}

			// has_bus_error = false;
			// bus_activity_tc = xTaskGetTickCount();
			uint8_t get_index = can->rx_get_index & (SC_BOARD_CAN_RX_FIFO_SIZE-1);
			uint8_t bytes = sizeof(struct sc_msg_can_rx);
			CAN_RXF0E_0_Type r0 = can->rx_frames[get_index].R0;
			CAN_RXF0E_1_Type r1 = can->rx_frames[get_index].R1;
			uint8_t can_frame_len = dlc_to_len(r1.bit.DLC);
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

				// LOG("ch%u rx i=%u rx hi=%x lo=%x tscv hi=%x lo=%x\n", index, get_index, rx_high, rx_low, tscv_high, tscv_low);
				// LOG("ch%u rx i=%u rx=%lx\n", index, get_index, ts);


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

			// has_bus_error = false;
			// bus_activity_tc = xTaskGetTickCount();
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
				SC_ASSERT(can->tx_available < SC_BOARD_CAN_TX_FIFO_SIZE);
				++can->tx_available;
				// LOG("3\n");
			}
		}
	}

	if (result > 0) {
		return result;
	}

	return have_data_to_place * -1;
}

SC_RAMFUNC extern void sc_board_led_can_status_set(uint8_t index, int status)
{
	struct can *can = &cans[index];

	switch (status) {
	case SC_CAN_LED_STATUS_DISABLED:
		led_set(can->led_status_green, 0);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_BUS_OFF:
		led_set(can->led_status_green, 1);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_BUS_ON_PASSIVE:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_PASSIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_BUS_ON_ACTIVE:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ERROR_ACTIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		break;
	case SC_CAN_LED_STATUS_ERROR_PASSIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		break;
	default:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
		break;
	}
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
	struct can *can = &cans[index];
	return can->nm_us_per_bit * nm + ((can->dt_us_per_bit_factor_shift8 * dt) >> 8);
}

#ifdef SUPERCAN_DEBUG
static volatile uint32_t rx_lost_reported[TU_ARRAY_SIZE(cans)];
// static volatile uint32_t rx_ts_last[TU_ARRAY_SIZE(cans)];
#endif

SC_RAMFUNC static bool can_poll(
	uint8_t index,
	uint32_t* events,
	uint32_t tsc)
{
	SC_DEBUG_ASSERT(events);

	struct can *can = &cans[index];

	bool more = false;
	uint32_t tsv[SC_BOARD_CAN_RX_FIFO_SIZE];
	uint8_t count = 0;
	uint8_t pi = 0;

	count = can->m_can->RXF0S.bit.F0FL;
	// static volatile uint32_t c = 0;
	// tsc = c++;
	// if (!counter_1MHz_is_current_value_ready()) {
	// 	LOG("ch%u counter not ready\n", index);
	// }
	//tsc = counter_1MHz_wait_for_current_value(index);
	// counter_1MHz_request_current_value();
	//tsc = counter_1MHz_read_unsafe(index);



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
				can_inc_sat_rx_lost(index);

#ifdef SUPERCAN_DEBUG
				{
					if (rx_lost_reported[index] + UINT32_C(1000000) <= tsc) {
						rx_lost_reported[index] = tsc;
						LOG("ch%u rx lost %lx\n", index, ts);
					}
				}
#endif
			} else {
				uint8_t put_index = pi & (SC_BOARD_CAN_RX_FIFO_SIZE-1);
				can->rx_frames[put_index].R0 = can->rx_fifo[get_index].R0;
				can->rx_frames[put_index].R1 = can->rx_fifo[get_index].R1;
				can->rx_frames[put_index].ts = tsv[get_index];
				if (likely(!can->rx_frames[put_index].R0.bit.RTR)) {
					uint8_t can_frame_len = dlc_to_len(can->rx_frames[put_index].R1.bit.DLC);
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

	// Because this task runs at a higher priority, it is ok to do this last
	//__atomic_thread_fence(__ATOMIC_RELEASE);

	return more;
}

SC_RAMFUNC extern uint32_t sc_board_can_ts_wait(uint8_t index)
{
	(void)index;
	while (!counter_1MHz_is_current_value_ready()); \
	return counter_1MHz_read_unsafe();
}


SC_RAMFUNC void CAN0_Handler(void)
{
	// LOG("CAN0 int\n");

	can_int(0);
}

#if SC_BOARD_CAN_COUNT > 1
SC_RAMFUNC void CAN1_Handler(void)
{
	// LOG("CAN1 int\n");

	can_int(1);
}
#endif


extern sc_can_bit_timing_range const* sc_board_can_nm_bit_timing_range(uint8_t index)
{
	(void)index;

	static const sc_can_bit_timing_range range = {

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

	return &range;
}

extern sc_can_bit_timing_range const* sc_board_can_dt_bit_timing_range(uint8_t index)
{
	(void)index;

	static const sc_can_bit_timing_range range = {

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

	return &range;
}

extern void sc_board_can_feat_set(uint8_t index, uint16_t features)
{
	struct can *can = &cans[index];

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
	struct can *can = &cans[index];

	can->nm = *bt;
}

extern void sc_board_can_dt_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	struct can *can = &cans[index];

	can->dt = *bt;
}

#endif // #ifdef D5035_01
