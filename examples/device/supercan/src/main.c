/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jean Gressmann <jean@0x42.de>
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

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>


#include <bsp/board.h>
#include <tusb.h>
#include <dcd.h>

#include <sam.h>

#include <hal/include/hal_gpio.h>

#define SC_PACKED __packed

#include <supercan.h>
#include <supercan_m1.h>
#include <supercan_debug.h>
#include <usb_descriptors.h>
#include <m_can.h>
#include <mcu.h>
#include <usb_dfu_1_1.h>
#include <dfu_ram.h>
#include <dfu_app.h>
#include <leds.h>


#if TU_BIG_ENDIAN == TU_BYTE_ORDER
static inline uint16_t le16_to_cpu(uint16_t value) { return __builtin_bswap16(value); }
static inline uint32_t le32_to_cpu(uint32_t value) { return __builtin_bswap32(value); }
static inline uint16_t cpu_to_le16(uint16_t value) { return __builtin_bswap16(value); }
static inline uint32_t cpu_to_le32(uint32_t value) { return __builtin_bswap32(value); }
static inline uint16_t be16_to_cpu(uint16_t value) { return value; }
static inline uint32_t be32_to_cpu(uint32_t value) { return value; }
static inline uint16_t cpu_to_be16(uint16_t value) { return value; }
static inline uint32_t cpu_to_be32(uint32_t value) { return value; }
#else
static inline uint16_t le16_to_cpu(uint16_t value) { return value; }
static inline uint32_t le32_to_cpu(uint32_t value) { return value; }
static inline uint16_t cpu_to_le16(uint16_t value) { return value; }
static inline uint32_t cpu_to_le32(uint32_t value) { return value; }
static inline uint16_t be16_to_cpu(uint16_t value) { return __builtin_bswap16(value); }
static inline uint32_t be32_to_cpu(uint32_t value) { return __builtin_bswap32(value); }
static inline uint16_t cpu_to_be16(uint16_t value) { return __builtin_bswap16(value); }
static inline uint32_t cpu_to_be32(uint32_t value) { return __builtin_bswap32(value); }
#endif

#ifndef D5035_01
# error Only D5035-01 boards supported
#endif



#ifndef likely
#define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x),0)
#endif

#define CAN_TX_FIFO_SIZE 32
#define CAN_RX_FIFO_SIZE 64
#define CAN_ELEMENT_DATA_SIZE 64
#define CAN_CLK_HZ CONF_CPU_FREQUENCY

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


enum {
	CAN_FEAT_PERM = SC_FEATURE_FLAG_TXR,
	CAN_FEAT_CONF = SC_FEATURE_FLAG_FDF |
					SC_FEATURE_FLAG_EHD |
					// not yet implemented
					// SC_FEATURE_FLAG_DAR |
					SC_FEATURE_FLAG_MON_MODE |
					SC_FEATURE_FLAG_RES_MODE |
					SC_FEATURE_FLAG_EXT_LOOP_MODE,

	CAN_QUEUE_SIZE = CAN_RX_FIFO_SIZE,
	CAN_QUEUE_ITEM_TYPE_BUS_STATUS = 0,
	CAN_QUEUE_ITEM_TYPE_BUS_ERROR = 1,
};

struct can_queue_item {
	uint8_t type : 1;
	uint8_t tx : 1;
	uint8_t data_part : 1;
	uint8_t payload : 5;
};


struct can {
	CFG_TUSB_MEM_ALIGN struct can_tx_fifo_element tx_fifo[CAN_TX_FIFO_SIZE];
	CFG_TUSB_MEM_ALIGN struct can_tx_event_fifo_element tx_event_fifo[CAN_TX_FIFO_SIZE];
	CFG_TUSB_MEM_ALIGN struct can_rx_fifo_element rx_fifo[CAN_RX_FIFO_SIZE];
	StackType_t stack_mem[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t task_mem;
	TaskHandle_t task_handle;
	uint8_t queue_storage[CAN_QUEUE_SIZE];
	StaticQueue_t queue_mem;
	QueueHandle_t queue_handle;
	Can *m_can;
	IRQn_Type interrupt_id;
	uint32_t nm_bittime_us;
	volatile uint32_t int_ts;
	uint32_t int_prev_psr_reg;
	uint16_t nmbt_brp;
	uint16_t nmbt_tseg1;
	volatile uint16_t rx_lost;
	uint16_t features;
	uint16_t tx_dropped;
	uint16_t int_ts_high;
	uint16_t int_tsc_last;
	uint8_t nmbt_sjw;
	uint8_t nmbt_tseg2;
	uint8_t dtbt_brp;
	uint8_t dtbt_sjw;
	uint8_t dtbt_tseg1;
	uint8_t dtbt_tseg2;
	volatile uint8_t int_comm_flags;
	uint8_t int_prev_bus_state;
	uint8_t led_status_green;
	uint8_t led_status_red;
	uint8_t led_traffic;
	bool enabled;
	bool desync;
};

static struct {
	struct can can[2];
} cans;




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
	GCLK->PCHCTRL[CAN0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN1 to use GLCK0 -> 120MHz
	GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN1 to use GLCK0 -> 120MHz
}

static inline void can_log_nominal_bittiming(struct can *c)
{
	(void)c;

	LOG("nominal brp=%u sjw=%u tseg1=%u tseg2=%u bitrate=%lu sp=%u/1000\n",
		c->nmbt_brp, c->nmbt_sjw, c->nmbt_tseg1, c->nmbt_tseg2,
		CAN_CLK_HZ / ((uint32_t)c->nmbt_brp * (1 + c->nmbt_tseg1 + c->nmbt_tseg2)),
		((c->nmbt_tseg1) * 1000) / (c->nmbt_tseg1 + c->nmbt_tseg2)
	);
}

static inline void can_log_data_bittiming(struct can *c)
{
	(void)c;

	LOG("data brp=%u sjw=%u tseg1=%u tseg2=%u bitrate=%lu sp=%u/1000\n",
		c->dtbt_brp, c->dtbt_sjw, c->dtbt_tseg1, c->dtbt_tseg2,
		CAN_CLK_HZ / ((uint32_t)c->dtbt_brp * (1 + c->dtbt_tseg1 + c->dtbt_tseg2)),
		((c->dtbt_tseg1) * 1000) / (c->dtbt_tseg1 + c->dtbt_tseg2)
	);
}

static void can_configure(struct can *c)
{
	Can *can = c->m_can;

	m_can_conf_begin(can);

	can->CCCR.bit.TXP = 1;  // enable tx pause
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

	LOG("MON=%u TEST=%u ASM=%u PXHD=%u FDOE=%u BRSE=%u DAR=%u\n",
		can->CCCR.bit.MON, can->CCCR.bit.TEST, can->CCCR.bit.ASM,
		can->CCCR.bit.PXHD, can->CCCR.bit.FDOE, can->CCCR.bit.BRSE,
		can->CCCR.bit.DAR
	);


	can_log_nominal_bittiming(c);
	can_log_data_bittiming(c);

	can->TSCC.reg = CAN_TSCC_TCP(0) | CAN_TSCC_TSS(1); // time stamp counter in CAN bittime
	can->TOCC.reg = CAN_TOCC_TOP(0xffff) | CAN_TOCC_TOS(0); // Timeout Counter disabled, Reset-default
	can->NBTP.reg = CAN_NBTP_NSJW(c->nmbt_sjw-1)
			| CAN_NBTP_NBRP(c->nmbt_brp-1)
			| CAN_NBTP_NTSEG1(c->nmbt_tseg1-1)
			| CAN_NBTP_NTSEG2(c->nmbt_tseg2-1);
	can->DBTP.reg = CAN_DBTP_DBRP(c->dtbt_brp-1)
			| CAN_DBTP_DTSEG1(c->dtbt_tseg1-1)
			| CAN_DBTP_DTSEG2(c->dtbt_tseg2-1)
			| CAN_DBTP_DSJW(c->dtbt_sjw-1)
			| CAN_DBTP_TDC;

	// transmitter delay compensation offset
	can->TDCR.bit.TDCO = tu_min8((1 + c->dtbt_tseg1 + c->dtbt_tseg2) / 2, M_CAN_TDCR_TDCO_MAX);

	// tx fifo
	can->TXBC.reg = CAN_TXBC_TBSA((uint32_t) c->tx_fifo) | CAN_TXBC_TFQS(CAN_TX_FIFO_SIZE);

	can->TXESC.reg = CAN_TXESC_TBDS_DATA64;

	// tx event fifo
	can->TXEFC.reg = CAN_TXEFC_EFSA((uint32_t) c->tx_event_fifo) | CAN_TXEFC_EFS(CAN_TX_FIFO_SIZE);


	// rx fifo0
	can->RXF0C.reg = CAN_RXF0C_F0SA((uint32_t) c->rx_fifo) | CAN_RXF0C_F0S(CAN_RX_FIFO_SIZE);
	//  | CAN_RXF0C_F0OM; // FIFO 0 overwrite mode
	can->RXESC.reg = CAN_RXESC_RBDS_DATA64 + CAN_RXESC_F0DS_DATA64;

	// enable interrupt line 0
	can->ILE.reg = CAN_ILE_EINT0;

	// wanted interrupts
	can->IE.reg =
		CAN_IE_TSWE     // time stamp counter wrap
		| CAN_IE_BOE    // bus off
		| CAN_IE_EWE    // error warning
		| CAN_IE_EPE    // error passive
		| CAN_IE_RF0NE  // new message in rx fifo0
		| CAN_IE_RF0LE  // message lost b/c fifo0 was full
		| CAN_IE_PEAE   // proto error in arbitration phase
		| CAN_IE_PEDE   // proto error in data phase
	 	| CAN_IE_TEFNE  // new message in tx event fifo
		| CAN_IE_MRAFE  // message RAM access failure
		| CAN_IE_BEUE   // bit error uncorrected, sets CCCR.INIT
		| CAN_IE_BECE   // bit error corrected
	;

	m_can_conf_end(can);
}


static void can_init_module(void)
{
	memset(&cans, 0, sizeof(cans));

	cans.can[0].m_can = CAN0;
	cans.can[1].m_can = CAN1;
	cans.can[0].interrupt_id = CAN0_IRQn;
	cans.can[1].interrupt_id = CAN1_IRQn;

	for (size_t j = 0; j < TU_ARRAY_SIZE(cans.can); ++j) {
		struct can *can = &cans.can[j];
		can->features = CAN_FEAT_PERM;
		for (size_t i = 0; i < TU_ARRAY_SIZE(cans.can[0].tx_fifo); ++i) {
			can->tx_fifo[i].T1.bit.EFC = 1; // store tx events
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

static inline uint8_t can_map_m_can_ec(uint8_t lec, uint8_t previous)
{
	switch (lec) {
	case 0x0: return SC_CAN_ERROR_NONE;
	case 0x1: return SC_CAN_ERROR_STUFF;
	case 0x2: return SC_CAN_ERROR_FORM;
	case 0x3: return SC_CAN_ERROR_ACK;
	case 0x4: return SC_CAN_ERROR_BIT1;
	case 0x5: return SC_CAN_ERROR_BIT0;
	case 0x6: return SC_CAN_ERROR_CRC;
	case 0x7: return 0xf & previous;
	default: return 0xff;
	}
}

static void can_int(uint8_t index)
{

	// SC_ASSERT(index < TU_ARRAY_SIZE(cans.can));
	struct can *can = &cans.can[index];

	// LOG("IE=%08lx IR=%08lx\n", can->m_can->IE.reg, can->m_can->IR.reg);
	bool notify = false;
	uint8_t current_bus_state = 0;
	uint16_t ts_high = can->int_ts_high;
	uint16_t tscv = can->m_can->TSCV.bit.TSC;


	if (tscv < can->int_tsc_last) {
		++ts_high;
		can->int_ts_high = ts_high;
		// LOG("CAN%u ts_high=%08lx\n", index, ts_high);
	}

	can->int_tsc_last = tscv;

	// update last non-interrupted timestamp
	can->int_ts = ((uint32_t)ts_high << M_CAN_TS_COUNTER_BITS) | tscv;


	CAN_IR_Type ir = can->m_can->IR;
	if (ir.bit.TSW) {
		// always notify here to enable the host to keep track of CAN bus time
		notify = true;
	}

	if (ir.bit.RF0N) {
		/* F0PI is incremented prior to interrupt call
		 *
		 * This interrupt can be late in the sense that F0PI will already
		 * have incremted again by the time this code runs. Conversely,
		 * we cannot rely on this interrupt to track the high part of the
		 * receive timestamp.
		 *
		 * Timestamp tracking now happens in the CAN task which does its own
		 * tracking of the high part.
		 */

		notify = true;
	}

	if (ir.bit.TEFN) {
		/* see comment for ir.bit.RF0N */
		notify = true;
	}

	if (unlikely(ir.bit.MRAF)) {
		LOG("CAN%u MRAF\n", index);
		SC_ISR_ASSERT(false && "MRAF");
	}

	if (unlikely(ir.bit.BEU)) {
		LOG("CAN%u BEU\n", index);
		SC_ISR_ASSERT(false && "BEU");
	}

	if (unlikely(ir.bit.BEC)) {
		LOG("CAN%u BEC\n", index);
	}

	CAN_PSR_Type current_psr = can->m_can->PSR; // always read, sets NC
	CAN_PSR_Type prev_psr;
	prev_psr.reg = can->int_prev_psr_reg;

	// update NC (no change) from last value (which might also be NC)
	if (CAN_PSR_LEC_NC_Val == current_psr.bit.LEC) {
		current_psr.bit.LEC = prev_psr.bit.LEC;
	}
	if (CAN_PSR_DLEC_NC_Val == current_psr.bit.DLEC) {
		current_psr.bit.DLEC = prev_psr.bit.DLEC;
	}

	// store updated psr reg
	can->int_prev_psr_reg = current_psr.reg;

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
		notify = true;
		struct can_queue_item msg = {
			.type = CAN_QUEUE_ITEM_TYPE_BUS_STATUS,
			.payload = current_bus_state,
		};

		if (unlikely(pdTRUE != xQueueSendFromISR(can->queue_handle, &msg, NULL))) {
			__sync_or_and_fetch(&can->int_comm_flags, SC_CAN_STATUS_FLAG_IRQ_QUEUE_FULL);
			LOG("CAN%u queue full\n", index);
		}
	}



	bool is_tx_error = current_psr.bit.ACT == CAN_PSR_ACT_TX_Val;
	bool is_rx_tx_error = current_psr.bit.ACT == CAN_PSR_ACT_RX_Val || is_tx_error;
	bool had_nm_error = prev_psr.bit.LEC != CAN_PSR_LEC_NONE_Val && prev_psr.bit.LEC != CAN_PSR_LEC_NC_Val;
	if (current_psr.bit.LEC != prev_psr.bit.LEC &&
		current_psr.bit.LEC != CAN_PSR_LEC_NONE_Val &&
		current_psr.bit.LEC != CAN_PSR_LEC_NC_Val &&
		is_rx_tx_error) {

		notify = true;
		struct can_queue_item msg = {
			.type = CAN_QUEUE_ITEM_TYPE_BUS_ERROR,
			.tx = is_tx_error,
			.data_part = 0,
			.payload = can_map_m_can_ec(current_psr.bit.LEC, 0),
		};

		if (unlikely(pdTRUE != xQueueSendFromISR(can->queue_handle, &msg, NULL))) {
			__sync_or_and_fetch(&can->int_comm_flags, SC_CAN_STATUS_FLAG_IRQ_QUEUE_FULL);
			LOG("CAN%u queue full\n", index);
		} else {
			// LOG("CAN%u LEC %1x\n", index, current_psr.bit.LEC);
		}
	} else {
		// LOG("CAN%u LEC %1x\n", index, current_psr.bit.LEC);
	}

	bool had_dt_error = prev_psr.bit.DLEC != CAN_PSR_LEC_NONE_Val && prev_psr.bit.DLEC != CAN_PSR_LEC_NC_Val;
	if (current_psr.bit.DLEC != prev_psr.bit.DLEC &&
		current_psr.bit.DLEC != CAN_PSR_DLEC_NONE_Val &&
		current_psr.bit.DLEC != CAN_PSR_DLEC_NC_Val &&
		is_rx_tx_error) {

		notify = true;
		struct can_queue_item msg = {
			.type = CAN_QUEUE_ITEM_TYPE_BUS_ERROR,
			.tx = is_tx_error,
			.data_part = 1,
			.payload = can_map_m_can_ec(current_psr.bit.DLEC, 0),
		};

		if (unlikely(pdTRUE != xQueueSendFromISR(can->queue_handle, &msg, NULL))) {
			__sync_or_and_fetch(&can->int_comm_flags, SC_CAN_STATUS_FLAG_IRQ_QUEUE_FULL);
			LOG("CAN%u queue full\n", index);
		} else {
			// LOG("CAN%u DLEC %1x\n", index, current_psr.bit.DLEC);
		}
	} else {
		// LOG("CAN%u DLEC %1x\n", index, current_psr.bit.DLEC);
	}

	if ((had_nm_error || had_dt_error) &&
		(current_psr.bit.LEC == CAN_PSR_DLEC_NONE_Val && current_psr.bit.DLEC == CAN_PSR_DLEC_NONE_Val)) {

		notify = true;
		struct can_queue_item msg = {
			.type = CAN_QUEUE_ITEM_TYPE_BUS_ERROR,
			.tx = 0,
			.data_part = 0,
			.payload = SC_CAN_ERROR_NONE,
		};

		if (unlikely(pdTRUE != xQueueSendFromISR(can->queue_handle, &msg, NULL))) {
			__sync_or_and_fetch(&can->int_comm_flags, SC_CAN_STATUS_FLAG_IRQ_QUEUE_FULL);
			LOG("CAN%u queue full\n", index);
		} else {
			// LOG("CAN%u good\n", index);
		}
	}

	if (ir.bit.RF0L) {
		// LOG("CAN%u msg lost\n", index);
		__sync_add_and_fetch(&can->rx_lost, 1);
		// notify = true;
	}

	// clear all interrupts
	can->m_can->IR = ir;

	if (notify) {
		// LOG("CAN%u notify\n", index);
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(can->task_handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void CAN0_Handler(void)
{
	// LOG("CAN0 int\n");
	can_int(0);
}

void CAN1_Handler(void)
{
	// LOG("CAN1 int\n");
	can_int(1);
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
		// TXFQS, RXF0S are reset when CCCR.CCE is set (read as 0), DS60001507E-page 1207
	}
}

static inline uint8_t dlc_to_len(uint8_t dlc)
{
	static const uint8_t map[16] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
	};
	return map[dlc & 0xf];
}

static inline uint32_t can_bittime_to_us(struct can const *can, uint32_t can_time)
{
	return can->nm_bittime_us * can_time;
}

static StackType_t usb_device_stack[configMINIMAL_SECURE_STACK_SIZE];
static StaticTask_t usb_device_stack_mem;

static void tusb_device_task(void* param);
static void can_task(void* param);


#if HWREV == 1
#define USB_TRAFFIC_LED LED_DEBUG
#define USB_TRAFFIC_DO_LED led_burst(LED_ORANGE1, 8)
#define POWER_LED LED_RED1
#define CAN0_TRAFFIC_LED LED_GREEN1
#define CAN1_TRAFFIC_LED LED_GREEN2

#else // HWREV > 1
#define USB_TRAFFIC_DO_LED led_burst(LED_DEBUG_3, 8)
#define POWER_LED LED_DEBUG_0
#define CAN0_TRAFFIC_LED LED_DEBUG_1
#define CAN1_TRAFFIC_LED LED_DEBUG_2
#endif // HWREV > 1



enum {
	CANLED_STATUS_DISABLED,
	CANLED_STATUS_ENABLED_BUS_OFF,
	CANLED_STATUS_ENABLED_BUS_ON_PASSIVE,
	CANLED_STATUS_ENABLED_BUS_ON_ACTIVE,
	CANLED_STATUS_ERROR_ACTIVE,
	CANLED_STATUS_ERROR_PASSIVE,
};

static inline void canled_set_status(struct can *can, int status)
{
#if HWREV >= 3
	const uint16_t BLINK_DELAY_PASSIVE_MS = 512;
	const uint16_t BLINK_DELAY_ACTIVE_MS = 128;
	switch (status) {
	case CANLED_STATUS_DISABLED:
		led_set(can->led_status_green, 0);
		led_set(can->led_status_red, 0);
		break;
	case CANLED_STATUS_ENABLED_BUS_OFF:
		led_set(can->led_status_green, 1);
		led_set(can->led_status_red, 0);
		break;
	case CANLED_STATUS_ENABLED_BUS_ON_PASSIVE:
		led_blink(can->led_status_green, BLINK_DELAY_PASSIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case CANLED_STATUS_ENABLED_BUS_ON_ACTIVE:
		led_blink(can->led_status_green, BLINK_DELAY_ACTIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case CANLED_STATUS_ERROR_PASSIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, BLINK_DELAY_PASSIVE_MS);
		break;
	case CANLED_STATUS_ERROR_ACTIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, BLINK_DELAY_ACTIVE_MS);
		break;
	}
#else
	(void)can;
	(void)status;
#endif
}

static inline void cans_led_status_set(int status)
{
	for (uint8_t i = 0; i < TU_ARRAY_SIZE(cans.can); ++i) {
		canled_set_status(&cans.can[i], status);
	}
}

#define MAJOR 0
#define MINOR 2
#define PATCH 6


#if SUPERDFU_APP
struct dfu_hdr dfu_hdr __attribute__((section(DFU_RAM_HDR_SECTION_NAME)));
static struct dfu_app_hdr dfu_app_hdr __attribute__((used,section(DFU_APP_HDR_SECTION_NAME))) = {
	.hdr_magic = DFU_APP_HDR_MAGIC_STRING,
	.hdr_version = DFU_APP_HDR_VERSION,
	.app_version_major = MAJOR,
	.app_version_minor = MINOR,
	.app_version_patch = PATCH,
	.app_watchdog_timeout_s = 1,
	.app_name = SC_NAME,
};

static struct dfu_app_ftr dfu_app_ftr __attribute__((used,section(DFU_APP_FTR_SECTION_NAME))) = {
	.magic = DFU_APP_FTR_MAGIC_STRING,
};

static struct dfu {
	struct dfu_get_status_reply status;
	StaticTimer_t timer_mem;
	TimerHandle_t timer_handle;
} dfu;

static void dfu_timer_expired(TimerHandle_t t);
static void dfu_timer_expired(TimerHandle_t t)
{
	(void)t;

	LOG("DFU detach timeer expired\n");

	dfu.status.bState = DFU_STATE_APP_IDLE;
}

#endif

#define CMD_BUFFER_SIZE 64
#define MSG_BUFFER_SIZE 512

struct usb_can {
	CFG_TUSB_MEM_ALIGN uint8_t tx_buffers[2][MSG_BUFFER_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t rx_buffers[2][MSG_BUFFER_SIZE];
	StaticSemaphore_t mutex_mem;
	SemaphoreHandle_t mutex_handle;
	uint16_t tx_offsets[2];
	uint8_t tx_bank;
	uint8_t rx_bank;
	uint8_t pipe;
};

struct usb_cmd {
	CFG_TUSB_MEM_ALIGN uint8_t tx_buffers[2][CMD_BUFFER_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t rx_buffers[2][CMD_BUFFER_SIZE];
	uint16_t tx_offsets[2];
	uint8_t tx_bank;
	uint8_t rx_bank;
	uint8_t pipe;
};


static struct usb {
	struct usb_cmd cmd[2];
	struct usb_can can[2];
	uint8_t port;
	bool mounted;
} usb;

static inline void can_off(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans.can));
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));

	struct can *can = &cans.can[index];
	struct usb_can *usb_can = &usb.can[index];

	// go bus off
	can_set_state1(can->m_can, can->interrupt_id, false);

	// reset can bus state
	can->enabled = false;
	can->rx_lost = 0;
	can->tx_dropped = 0;
	can->desync = false;
	can->int_ts_high = 0;
	can->int_prev_bus_state = 0;
	can->int_comm_flags = 0;
	can->int_prev_psr_reg = 0;
	can->int_tsc_last = 0;

	// clear tx buffers
	xSemaphoreTake(usb_can->mutex_handle, ~0);
	for (size_t j = 0; j < TU_ARRAY_SIZE(usb_can->tx_offsets); ++j) {
		usb_can->tx_offsets[j] = 0;
		memset(usb_can->tx_buffers[j], 0, sizeof(usb_can->tx_buffers[j]));
	}
	xSemaphoreGive(usb_can->mutex_handle);

	// clear interrupt handler queue
	xQueueReset(can->queue_handle);

	// notify task to make sure its knows
	xTaskNotifyGive(can->task_handle);
}

static inline void can_reset(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans.can));
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));

	// disable CAN units, reset configuration & status
	can_off(index);

	struct can *can = &cans.can[index];

	can->features = CAN_FEAT_PERM;
	can->nm_bittime_us = 0;
	can->nmbt_brp = 0;
	can->nmbt_sjw = 0;
	can->nmbt_tseg1 = 0;
	can->dtbt_brp = 0;
	can->dtbt_sjw = 0;
	can->dtbt_tseg1 = 0;
	can->dtbt_tseg2 = 0;
}

static inline void cans_reset(void)
{
	for (size_t i = 0; i < TU_ARRAY_SIZE(cans.can); ++i) {
		can_reset((uint8_t)i);
	}
}

static inline bool sc_cmd_bulk_in_ep_ready(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.cmd));
	struct usb_cmd *cmd = &usb.cmd[index];
	return 0 == cmd->tx_offsets[!cmd->tx_bank];
}

static inline void sc_cmd_bulk_in_submit(uint8_t index)
{
	SC_DEBUG_ASSERT(sc_cmd_bulk_in_ep_ready(index));
	struct usb_cmd *cmd = &usb.cmd[index];
	SC_DEBUG_ASSERT(cmd->tx_offsets[cmd->tx_bank] > 0);
	SC_DEBUG_ASSERT(cmd->tx_offsets[cmd->tx_bank] <= CMD_BUFFER_SIZE);
	(void)dcd_edpt_xfer(usb.port, 0x80 | cmd->pipe, cmd->tx_buffers[cmd->tx_bank], cmd->tx_offsets[cmd->tx_bank]);
	cmd->tx_bank = !cmd->tx_bank;
}

static inline bool sc_can_bulk_in_ep_ready(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));
	struct usb_can *can = &usb.can[index];
	return 0 == can->tx_offsets[!can->tx_bank];
}

static void sc_can_bulk_in_submit(uint8_t index, char const *func)
{
	SC_DEBUG_ASSERT(sc_can_bulk_in_ep_ready(index));
	struct usb_can *can = &usb.can[index];
	SC_DEBUG_ASSERT(can->tx_bank < 2);
	SC_DEBUG_ASSERT(can->tx_offsets[can->tx_bank] > 0);
	SC_DEBUG_ASSERT(can->tx_offsets[can->tx_bank] <= MSG_BUFFER_SIZE);
	(void)func;

#if SUPERCAN_DEBUG
	// LOG("ch%u %s: send %u bytes\n", index, func, can->tx_offsets[can->tx_bank]);
	if (can->tx_offsets[can->tx_bank] > MSG_BUFFER_SIZE) {
		LOG("ch%u %s: msg buffer size %u out of bounds\n", index, func, can->tx_offsets[can->tx_bank]);
		can->tx_offsets[can->tx_bank] = 0;
		return;
	}

	if (can->tx_offsets[can->tx_bank] & 3) {
		LOG("ch%u %s: msg buffer size %u not multiple of 4\n", index, func, can->tx_offsets[can->tx_bank]);
		can->tx_offsets[can->tx_bank] = 0;
		return;
	}

	uint8_t const *sptr = can->tx_buffers[can->tx_bank];
	uint8_t const *eptr = sptr + can->tx_offsets[can->tx_bank];
	uint8_t const *ptr = sptr;
	for (; ptr + SC_MSG_HEADER_LEN <= eptr; ) {
		struct sc_msg_header *hdr = (struct sc_msg_header *)ptr;
		if (!hdr->id || !hdr->len) {
			break;
		}

		if (ptr + hdr->len > eptr) {
			LOG("ch%u %s: msg offset %u out of bounds\n", index, func, ptr - sptr);
			can->tx_offsets[can->tx_bank] = 0;
			return;
		}

		switch (hdr->id) {
		case SC_MSG_EOF:
		case SC_MSG_HELLO_HOST:
		case SC_MSG_DEVICE_INFO:
		case SC_MSG_CAN_INFO:
		case SC_MSG_ERROR:
		case SC_MSG_CAN_STATUS:
		case SC_MSG_CAN_RX:
		case SC_MSG_CAN_TXR:
		case SC_MSG_CAN_ERROR:
			break;
		default:
			LOG("ch%u %s msg offset %u non-device msg id %#02x\n", index, func, ptr - sptr, hdr->id);
			can->tx_offsets[can->tx_bank] = 0;
			return;
		}

		ptr += hdr->len;
	}
#endif

	(void)dcd_edpt_xfer(usb.port, 0x80 | can->pipe, can->tx_buffers[can->tx_bank], can->tx_offsets[can->tx_bank]);
	can->tx_bank = !can->tx_bank;
	SC_DEBUG_ASSERT(!can->tx_offsets[can->tx_bank]);
	// memset(can->tx_buffers[can->tx_bank], 0, MSG_BUFFER_SIZE);
}

static void sc_cmd_bulk_out(uint8_t index, uint32_t xferred_bytes);
static void sc_cmd_bulk_in(uint8_t index);
static void sc_can_bulk_out(uint8_t index, uint32_t xferred_bytes);
static void sc_can_bulk_in(uint8_t index);
static void sc_cmd_place_error_reply(uint8_t index, int8_t error);

static void sc_cmd_bulk_out(uint8_t index, uint32_t xferred_bytes)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.cmd));
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans.can));

	struct can *can = &cans.can[index];
	struct usb_cmd *usb_cmd = &usb.cmd[index];
	struct usb_can *usb_can = &usb.can[index];


	uint8_t const *in_ptr = usb_cmd->rx_buffers[usb_cmd->rx_bank];
	uint8_t const * const in_end = in_ptr + xferred_bytes;

	// setup next transfer
	usb_cmd->rx_bank = !usb_cmd->rx_bank;
	(void)dcd_edpt_xfer(usb.port, usb_cmd->pipe, usb_cmd->rx_buffers[usb_cmd->rx_bank], CMD_BUFFER_SIZE);

	// process messages
	while (in_ptr + SC_MSG_HEADER_LEN <= in_end) {
		struct sc_msg_header const *msg = (struct sc_msg_header const *)in_ptr;
		if (in_ptr + msg->len > in_end) {
			LOG("ch%u malformed msg\n", index);
			break;
		}

		if (!msg->len) {
			break;
		}

		in_ptr += msg->len;

		switch (msg->id) {
		case SC_MSG_EOF: {
			LOG("ch%u SC_MSG_EOF\n", index);
			in_ptr = in_end;
		} break;

		case SC_MSG_HELLO_DEVICE: {
			LOG("ch%u SC_MSG_HELLO_DEVICE\n", index);

			// reset
			can_reset(index);
			canled_set_status(can, CANLED_STATUS_ENABLED_BUS_OFF);

			// transmit empty buffers (clear whatever was in there before)
			(void)dcd_edpt_xfer(usb.port, 0x80 | usb_can->pipe, usb_can->tx_buffers[usb_can->tx_bank], usb_can->tx_offsets[usb_can->tx_bank]);

			// reset tx buffer
			uint8_t len = sizeof(struct sc_msg_hello);
			usb_cmd->tx_offsets[usb_cmd->tx_bank] = len;
			struct sc_msg_hello *rep = (struct sc_msg_hello *)&usb_cmd->tx_buffers[usb_cmd->tx_bank][0];
			rep->id = SC_MSG_HELLO_HOST;
			rep->len = len;
			rep->proto_version = SC_VERSION;
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
			rep->byte_order = SC_BYTE_ORDER_BE;
#else
			rep->byte_order = SC_BYTE_ORDER_LE;
#endif
			rep->cmd_buffer_size = cpu_to_be16(CMD_BUFFER_SIZE);

			// don't process any more messages
			in_ptr = in_end;

			// assume in token is available
		} break;
		case SC_MSG_DEVICE_INFO: {
			LOG("ch%u SC_MSG_DEVICE_INFO\n", index);
			uint8_t bytes = sizeof(struct sc_msg_dev_info);

			uint8_t *out_ptr;
			uint8_t *out_end;

send_dev_info:
			out_ptr = usb_cmd->tx_buffers[usb_cmd->tx_bank] + usb_cmd->tx_offsets[usb_cmd->tx_bank];
			out_end = usb_cmd->tx_buffers[usb_cmd->tx_bank] + CMD_BUFFER_SIZE;
			if (out_end - out_ptr >= bytes) {
				usb_cmd->tx_offsets[usb_cmd->tx_bank] += bytes;
				struct sc_msg_dev_info *rep = (struct sc_msg_dev_info *)out_ptr;
				rep->id = SC_MSG_DEVICE_INFO;
				rep->len = bytes;
				rep->feat_perm = CAN_FEAT_PERM;
				rep->feat_conf = CAN_FEAT_CONF;
				rep->fw_ver_major = MAJOR;
				rep->fw_ver_minor = MINOR;
				rep->fw_ver_patch = PATCH;
				static const char dev_name[] = BOARD_NAME " " SC_NAME " chX";
				rep->name_len = tu_min8(sizeof(dev_name)-1, sizeof(rep->name_bytes));
				memcpy(rep->name_bytes, dev_name, rep->name_len);
				if (rep->name_len <= TU_ARRAY_SIZE(rep->name_bytes)) {
					rep->name_bytes[rep->name_len-1] = '0' + index;
				}
				rep->sn_len = 16;
				static_assert(sizeof(rep->sn_bytes) >= 16, "expect at least 16 of buffer for serial number");
				uint32_t serial[4];
				same51_get_serial_number(serial);
				for (unsigned i = 0, j = 0; i < 4; ++i) {
					uint32_t w = serial[i];
					rep->sn_bytes[j++] = (w >> 24) & 0xff;
					rep->sn_bytes[j++] = (w >> 16) & 0xff;
					rep->sn_bytes[j++] = (w >> 8) & 0xff;
					rep->sn_bytes[j++] = (w >> 0) & 0xff;
				}
			} else {
				if (sc_cmd_bulk_in_ep_ready(index)) {
					sc_cmd_bulk_in_submit(index);
					goto send_dev_info;
				} else {
					LOG("no space for device info reply\n");
				}
			}
		} break;
		case SC_MSG_CAN_INFO: {
			LOG("ch%u SC_MSG_CAN_INFO\n", index);
			uint8_t bytes = sizeof(struct sc_msg_can_info);

			uint8_t *out_ptr;
			uint8_t *out_end;

send_can_info:
			out_ptr = usb_cmd->tx_buffers[usb_cmd->tx_bank] + usb_cmd->tx_offsets[usb_cmd->tx_bank];
			out_end = usb_cmd->tx_buffers[usb_cmd->tx_bank] + CMD_BUFFER_SIZE;
			if (out_end - out_ptr >= bytes) {
				usb_cmd->tx_offsets[usb_cmd->tx_bank] += bytes;
				struct sc_msg_can_info *rep = (struct sc_msg_can_info *)out_ptr;
				rep->id = SC_MSG_CAN_INFO;
				rep->len = bytes;
				rep->can_clk_hz = CAN_CLK_HZ;
				rep->nmbt_brp_min = M_CAN_NMBT_BRP_MIN;
				rep->nmbt_brp_max = M_CAN_NMBT_BRP_MAX;
				rep->nmbt_sjw_max = M_CAN_NMBT_SJW_MAX;
				rep->nmbt_tseg1_min = M_CAN_NMBT_TSEG1_MIN;
				rep->nmbt_tseg1_max = M_CAN_NMBT_TSEG1_MAX;
				rep->nmbt_tseg2_min = M_CAN_NMBT_TSEG2_MIN;
				rep->nmbt_tseg2_max = M_CAN_NMBT_TSEG2_MAX;
				rep->dtbt_brp_min = M_CAN_DTBT_BRP_MIN;
				rep->dtbt_brp_max = M_CAN_DTBT_BRP_MAX;
				rep->dtbt_sjw_max = M_CAN_DTBT_SJW_MAX;
				rep->dtbt_tseg1_min = M_CAN_DTBT_TSEG1_MIN;
				rep->dtbt_tseg1_max = M_CAN_DTBT_TSEG1_MAX;
				rep->dtbt_tseg2_min = M_CAN_DTBT_TSEG2_MIN;
				rep->dtbt_tseg2_max = M_CAN_DTBT_TSEG2_MAX;
				rep->tx_fifo_size = CAN_TX_FIFO_SIZE;
				rep->rx_fifo_size = CAN_RX_FIFO_SIZE;
				rep->msg_buffer_size = MSG_BUFFER_SIZE;
			} else {
				if (sc_cmd_bulk_in_ep_ready(index)) {
					sc_cmd_bulk_in_submit(index);
					goto send_can_info;
				} else {
					LOG("no space for can info reply\n");
				}
			}
		} break;
		case SC_MSG_NM_BITTIMING: {
			LOG("ch%u SC_MSG_NM_BITTIMING\n", index);
			int8_t error = SC_CAN_ERROR_NONE;
			struct sc_msg_bittiming const *tmsg = (struct sc_msg_bittiming const *)msg;
			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ch%u ERROR: msg too short\n", index);
				error = SC_ERROR_SHORT;
			} else {
				// clamp
				can->nmbt_brp = tu_max16(M_CAN_NMBT_BRP_MIN, tu_min16(tmsg->brp, M_CAN_NMBT_BRP_MAX));
				can->nmbt_sjw = tu_max8(M_CAN_NMBT_SJW_MIN, tu_min8(tmsg->sjw, M_CAN_NMBT_SJW_MAX));
				can->nmbt_tseg1 = tu_max16(M_CAN_NMBT_TSEG1_MIN, tu_min16(tmsg->tseg1, M_CAN_NMBT_TSEG1_MAX));
				can->nmbt_tseg2 = tu_max8(M_CAN_NMBT_TSEG2_MIN, tu_min8(tmsg->tseg2, M_CAN_NMBT_TSEG2_MAX));

				// set nominal bittime for timestamp calculation
				can->nm_bittime_us = UINT32_C(1000000) / (CAN_CLK_HZ / ((uint32_t)can->nmbt_brp * (1 + can->nmbt_tseg1 + can->nmbt_tseg2)));
				can_log_nominal_bittiming(can);
			}

			sc_cmd_place_error_reply(index, error);
		} break;
		case SC_MSG_DT_BITTIMING: {
			LOG("ch%u SC_MSG_DT_BITTIMING\n", index);
			int8_t error = SC_CAN_ERROR_NONE;
			struct sc_msg_bittiming const *tmsg = (struct sc_msg_bittiming const *)msg;
			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ch%u ERROR: msg too short\n", index);
				error = SC_ERROR_SHORT;
			} else {
				// clamp
				can->dtbt_brp = tu_max16(M_CAN_DTBT_BRP_MIN, tu_min16(tmsg->brp, M_CAN_DTBT_BRP_MAX));
				can->dtbt_sjw = tu_max8(M_CAN_DTBT_SJW_MIN, tu_min8(tmsg->sjw, M_CAN_DTBT_SJW_MAX));
				can->dtbt_tseg1 = tu_max16(M_CAN_DTBT_TSEG1_MIN, tu_min16(tmsg->tseg1, M_CAN_DTBT_TSEG1_MAX));
				can->dtbt_tseg2 = tu_max8(M_CAN_DTBT_TSEG2_MIN, tu_min8(tmsg->tseg2, M_CAN_DTBT_TSEG2_MAX));
				can_log_data_bittiming(can);
			}

			sc_cmd_place_error_reply(index, error);
		} break;
		case SC_MSG_FEATURES: {
			LOG("ch%u SC_MSG_FEATURES\n", index);
			struct sc_msg_features const *tmsg = (struct sc_msg_features const *)msg;
			int8_t error = SC_ERROR_NONE;
			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ch%u ERROR: msg too short\n", index);
				error = SC_ERROR_SHORT;
			} else {
				switch (tmsg->op) {
				case SC_FEAT_OP_CLEAR:
					can->features = CAN_FEAT_PERM;
					LOG("ch%u CLEAR features to %#x\n", index, can->features);
					break;
				case SC_FEAT_OP_OR: {
					uint32_t mode_bits = tmsg->arg & (SC_FEATURE_FLAG_MON_MODE | SC_FEATURE_FLAG_RES_MODE | SC_FEATURE_FLAG_EXT_LOOP_MODE);
					if (__builtin_popcount(mode_bits) > 1) {
						error = SC_ERROR_PARAM;
						LOG("ch%u ERROR: attempt to activate more than one mode %08lx\n", index, mode_bits);
					} else if (tmsg->arg & ~(CAN_FEAT_PERM | CAN_FEAT_CONF)) {
						error = SC_ERROR_UNSUPPORTED;
						LOG("ch%u ERROR: unsupported features %08lx\n", index, tmsg->arg);
					} else {
						can->features |= tmsg->arg;
						LOG("ch%u OR features to %#x\n", index, can->features);
					}
				} break;
				}
			}
			sc_cmd_place_error_reply(index, error);
		} break;
		case SC_MSG_BUS: {
			LOG("ch%u SC_MSG_BUS\n", index);
			struct sc_msg_config const *tmsg = (struct sc_msg_config const *)msg;
			int8_t error = SC_ERROR_NONE;
			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ERROR: msg too short\n");
				error = SC_ERROR_SHORT;
			} else {
				bool was_enabled = can->enabled;
				can->enabled = tmsg->arg != 0;
				if (was_enabled != can->enabled) {
					LOG("ch%u enabled=%u\n", index, can->enabled);
					if (can->enabled) {
						can_configure(can);
						can_set_state1(can->m_can, can->interrupt_id, can->enabled);
						canled_set_status(can, CANLED_STATUS_ENABLED_BUS_ON_PASSIVE);
					} else {
						can_off(index);
						canled_set_status(can, CANLED_STATUS_ENABLED_BUS_OFF);
					}
				}
			}

			sc_cmd_place_error_reply(index, error);
		} break;
		default:
			TU_LOG2_MEM(msg, msg->len, 2);
			sc_cmd_place_error_reply(index, SC_ERROR_UNSUPPORTED);
			break;
		}
	}

	if (usb_cmd->tx_offsets[usb_cmd->tx_bank] > 0 && sc_cmd_bulk_in_ep_ready(index)) {
		sc_cmd_bulk_in_submit(index);
	}
}

static void sc_can_bulk_out(uint8_t index, uint32_t xferred_bytes)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans.can));

	struct can *can = &cans.can[index];
	struct usb_can *usb_can = &usb.can[index];
	led_burst(can->led_traffic, 8);


	const uint8_t rx_bank = usb_can->rx_bank;
	(void)rx_bank;
	uint8_t const * const in_beg = usb_can->rx_buffers[usb_can->rx_bank];
	uint8_t const *in_ptr = in_beg;
	uint8_t const * const in_end = in_ptr + xferred_bytes;

	// start new transfer right away
	usb_can->rx_bank = !usb_can->rx_bank;
	(void)dcd_edpt_xfer(usb.port, usb_can->pipe, usb_can->rx_buffers[usb_can->rx_bank], MSG_BUFFER_SIZE);

	xSemaphoreTake(usb_can->mutex_handle, ~0);

	// process messages
	while (in_ptr + SC_MSG_HEADER_LEN <= in_end) {
		struct sc_msg_header const *msg = (struct sc_msg_header const *)in_ptr;
		if (in_ptr + msg->len > in_end) {
			LOG("ch%u malformed msg\n", index);
			break;
		}

		if (!msg->len) {
			break;
		}

		in_ptr += msg->len;

		switch (msg->id) {
		case SC_MSG_EOF: {
			// LOG("SC_MSG_EOF\n");
			in_ptr = in_end;
		} break;
		case SC_MSG_CAN_TX: {
			// LOG("SC_MSG_CAN_TX\n");
			struct sc_msg_can_tx const *tmsg = (struct sc_msg_can_tx const *)msg;
			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ch%u ERROR: msg too short\n", index);
				continue;
			}

			const uint8_t can_frame_len = dlc_to_len(tmsg->dlc);
			if (!(tmsg->flags & SC_CAN_FRAME_FLAG_RTR)) {
				if (msg->len < sizeof(*tmsg) + can_frame_len) {
					LOG("ch%u ERROR: msg too short\n", index);
					continue;
				}
			}

			if (can->m_can->TXFQS.bit.TFQF) {
				++can->tx_dropped;

				uint8_t *out_ptr;
				uint8_t *out_end;
send_txr:
				out_ptr = usb_can->tx_buffers[usb_can->tx_bank] + usb_can->tx_offsets[usb_can->tx_bank];
				out_end = usb_can->tx_buffers[usb_can->tx_bank] + MSG_BUFFER_SIZE;
				uint8_t bytes = sizeof(struct sc_msg_can_txr);
				if (out_end - out_ptr >= bytes) {
					usb_can->tx_offsets[usb_can->tx_bank] += bytes;

					struct sc_msg_can_txr *rep = (struct sc_msg_can_txr *)out_ptr;
					rep->id = SC_MSG_CAN_TXR;
					rep->len = bytes;
					rep->track_id = tmsg->track_id;
					uint32_t ts = __sync_fetch_and_or(&can->int_ts, 0);
					rep->timestamp_us = can_bittime_to_us(can, ts);
					rep->flags = SC_CAN_FRAME_FLAG_DRP;
				} else {
					if (sc_can_bulk_in_ep_ready(index)) {
						sc_can_bulk_in_submit(index, __func__);
						goto send_txr;
					} else {
						LOG("ch%u: desync\n", index);
						can->desync = true;
					}
				}
			} else {
				uint32_t id = tmsg->can_id;
				uint8_t put_index = can->m_can->TXFQS.bit.TFQPI;

				CAN_TXBE_0_Type t0;
				t0.reg = (((tmsg->flags & SC_CAN_FRAME_FLAG_ESI) == SC_CAN_FRAME_FLAG_ESI) << CAN_TXBE_0_ESI_Pos)
					| (((tmsg->flags & SC_CAN_FRAME_FLAG_RTR) == SC_CAN_FRAME_FLAG_RTR) << CAN_TXBE_0_RTR_Pos)
					| (((tmsg->flags & SC_CAN_FRAME_FLAG_EXT) == SC_CAN_FRAME_FLAG_EXT) << CAN_TXBE_0_XTD_Pos)
					;



				if (tmsg->flags & SC_CAN_FRAME_FLAG_EXT) {
					t0.reg |= CAN_TXBE_0_ID(id);
				} else {
					t0.reg |= CAN_TXBE_0_ID(id << 18);
				}

				can->tx_fifo[put_index].T0 = t0;
				can->tx_fifo[put_index].T1.bit.DLC = tmsg->dlc;
				can->tx_fifo[put_index].T1.bit.FDF = (tmsg->flags & SC_CAN_FRAME_FLAG_FDF) == SC_CAN_FRAME_FLAG_FDF;
				can->tx_fifo[put_index].T1.bit.BRS = (tmsg->flags & SC_CAN_FRAME_FLAG_BRS) == SC_CAN_FRAME_FLAG_BRS;
				can->tx_fifo[put_index].T1.bit.MM = tmsg->track_id;

				if (!(tmsg->flags & SC_CAN_FRAME_FLAG_RTR)) {
					if (can_frame_len) {
						memcpy(can->tx_fifo[put_index].data, tmsg->data, can_frame_len);
					}
				}

				can->m_can->TXBAR.reg = 1UL << put_index;
			}
		} break;

		default:
			TU_LOG2_MEM(msg, msg->len, 2);
			break;
		}
	}

	if (usb_can->tx_offsets[usb_can->tx_bank] > 0 && sc_can_bulk_in_ep_ready(index)) {
		sc_can_bulk_in_submit(index, __func__);
	}

	xSemaphoreGive(usb_can->mutex_handle);

	// // notify CAN task on bus-off we don't get bittime ticks
	// vTaskNotifyGiveFromISR(can->task_handle, NULL);
}

static void sc_cmd_bulk_in(uint8_t index)
{
	// LOG("< cmd%u IN token\n", index);

	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.cmd));

	struct usb_cmd *usb_cmd = &usb.cmd[index];

	usb_cmd->tx_offsets[!usb_cmd->tx_bank] = 0;

	if (usb_cmd->tx_offsets[usb_cmd->tx_bank]) {
		sc_cmd_bulk_in_submit(index);
	}
}

static void sc_can_bulk_in(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));

	struct usb_can *usb_can = &usb.can[index];

	xSemaphoreTake(usb_can->mutex_handle, ~0);

	usb_can->tx_offsets[!usb_can->tx_bank] = 0;

	if (usb_can->tx_offsets[usb_can->tx_bank]) {
		sc_can_bulk_in_submit(index, __func__);
	}

	xSemaphoreGive(usb_can->mutex_handle);
}

static void sc_cmd_place_error_reply(uint8_t index, int8_t error)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.cmd));

	struct usb_cmd *usb_cmd = &usb.cmd[index];
	uint8_t bytes = sizeof(struct sc_msg_error);
	uint8_t *out_ptr;
	uint8_t *out_end;

send:
	out_ptr = usb_cmd->tx_buffers[usb_cmd->tx_bank] + usb_cmd->tx_offsets[usb_cmd->tx_bank];
	out_end = usb_cmd->tx_buffers[usb_cmd->tx_bank] + CMD_BUFFER_SIZE;
	if (out_end - out_ptr >= bytes) {
		usb_cmd->tx_offsets[usb_cmd->tx_bank] += bytes;
		struct sc_msg_error *rep = (struct sc_msg_error *)out_ptr;
		rep->id = SC_MSG_ERROR;
		rep->len = sizeof(*rep);
		rep->error = error;
	} else {
		if (sc_cmd_bulk_in_ep_ready(index)) {
			sc_cmd_bulk_in_submit(index);
			goto send;
		} else {
			LOG("ch%u: no space for error reply\n", index);
		}
	}
}



int main(void)
{
	board_init();

#if SUPERDFU_APP
	LOG(
		"%s v%u.%u.%u starting...\n",
		dfu_app_hdr.app_name,
		dfu_app_hdr.app_version_major,
		dfu_app_hdr.app_version_minor,
		dfu_app_hdr.app_version_patch);

	dfu_request_dfu(0); // no bootloader request

	dfu.status.bStatus = DFU_ERROR_OK;
	dfu.status.bwPollTimeout = 100; // ms
	dfu.status.bState = DFU_STATE_APP_IDLE;
	dfu.timer_handle = xTimerCreateStatic("dfu", pdMS_TO_TICKS(DFU_USB_RESET_TIMEOUT_MS), pdFALSE, NULL, &dfu_timer_expired, &dfu.timer_mem);
#endif

	led_init();

	tusb_init();

	can_init_pins();
	can_init_clock();
	can_init_module();

	(void) xTaskCreateStatic(&tusb_device_task, "tusb", TU_ARRAY_SIZE(usb_device_stack), NULL, configMAX_PRIORITIES-1, usb_device_stack, &usb_device_stack_mem);
	(void) xTaskCreateStatic(&led_task, "led", TU_ARRAY_SIZE(led_task_stack), NULL, configMAX_PRIORITIES-1, led_task_stack, &led_task_mem);

	usb.can[0].mutex_handle = xSemaphoreCreateMutexStatic(&usb.can[0].mutex_mem);
	usb.can[1].mutex_handle = xSemaphoreCreateMutexStatic(&usb.can[1].mutex_mem);
#if HWREV >= 3
	cans.can[0].led_status_green = LED_CAN0_STATUS_GREEN;
	cans.can[0].led_status_red = LED_CAN0_STATUS_RED;
	cans.can[1].led_status_green = LED_CAN1_STATUS_GREEN;
	cans.can[1].led_status_red = LED_CAN1_STATUS_RED;
#endif
	cans.can[0].led_traffic = CAN0_TRAFFIC_LED;
	cans.can[1].led_traffic = CAN1_TRAFFIC_LED;
	cans.can[0].queue_handle = xQueueCreateStatic(CAN_QUEUE_SIZE, sizeof(cans.can[0].queue_storage) / CAN_QUEUE_SIZE, cans.can[0].queue_storage, &cans.can[0].queue_mem);
	cans.can[1].queue_handle = xQueueCreateStatic(CAN_QUEUE_SIZE, sizeof(cans.can[1].queue_storage) / CAN_QUEUE_SIZE, cans.can[0].queue_storage, &cans.can[1].queue_mem);


	cans.can[0].task_handle = xTaskCreateStatic(&can_task, "can0", TU_ARRAY_SIZE(cans.can[0].stack_mem), (void*)(uintptr_t)0, configMAX_PRIORITIES-1, cans.can[0].stack_mem, &cans.can[0].task_mem);
	cans.can[1].task_handle = xTaskCreateStatic(&can_task, "can1", TU_ARRAY_SIZE(cans.can[1].stack_mem), (void*)(uintptr_t)1, configMAX_PRIORITIES-1, cans.can[1].stack_mem, &cans.can[1].task_mem);

	led_blink(0, 2000);
	led_set(POWER_LED, 1);



#if SUPERDFU_APP
	dfu_app_watchdog_disable();
#endif


	vTaskStartScheduler();
	NVIC_SystemReset();
	return 0;
}


//--------------------------------------------------------------------+
// USB DEVICE TASK
//--------------------------------------------------------------------+
static void tusb_device_task(void* param)
{
	(void) param;

	while (1) {
		LOG("tud_task\n");
		tud_task();
	}
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
	LOG("mounted\n");
	led_blink(0, 250);
	usb.mounted = true;

	cans_led_status_set(CANLED_STATUS_DISABLED);
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	LOG("unmounted\n");
	led_blink(0, 1000);
	usb.mounted = false;

	cans_reset();
	cans_led_status_set(CANLED_STATUS_DISABLED);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	LOG("suspend\n");
	usb.mounted = false;
	led_blink(0, 500);

	cans_reset();
	cans_led_status_set(CANLED_STATUS_DISABLED);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	LOG("resume\n");
	usb.mounted = true;
	led_blink(0, 250);
}


//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+
void tud_custom_init_cb(void)
{
	LOG("init\n");
}

void tud_custom_reset_cb(uint8_t rhport)
{
	LOG("port %u reset\n", rhport);
	usb.mounted = false;
	usb.port = rhport;
	usb.cmd[0].pipe = SC_M1_EP_CMD0_BULK_OUT;
	usb.cmd[0].tx_offsets[0] = 0;
	usb.cmd[0].tx_offsets[1] = 0;
	usb.cmd[1].pipe = SC_M1_EP_CMD1_BULK_OUT;
	usb.cmd[1].tx_offsets[0] = 0;
	usb.cmd[1].tx_offsets[1] = 0;
	usb.can[0].pipe = SC_M1_EP_MSG0_BULK_OUT;
	usb.can[0].tx_offsets[0] = 0;
	usb.can[0].tx_offsets[1] = 0;
	usb.can[1].pipe = SC_M1_EP_MSG1_BULK_OUT;
	usb.can[1].tx_offsets[0] = 0;
	usb.can[1].tx_offsets[1] = 0;
}

bool tud_custom_open_cb(uint8_t rhport, tusb_desc_interface_t const * desc_intf, uint16_t* p_length)
{
	LOG("port %u open\n", rhport);

	if (unlikely(rhport != usb.port)) {
		return false;
	}

	TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == desc_intf->bInterfaceClass);

	if (unlikely(desc_intf->bInterfaceNumber >= TU_ARRAY_SIZE(usb.can))) {
		return false;
	}


	struct usb_cmd *usb_cmd = &usb.cmd[desc_intf->bInterfaceNumber];
	struct usb_can *usb_can = &usb.can[desc_intf->bInterfaceNumber];

	uint8_t const *ptr = (void const *)desc_intf;

	ptr += 9;

	const uint8_t eps = 4;

	for (uint8_t i = 0; i < eps; ++i) {
		tusb_desc_endpoint_t const *ep_desc = (tusb_desc_endpoint_t const *)(ptr + i * 7);
		LOG("! ep %02x open\n", ep_desc->bEndpointAddress);
		bool success = dcd_edpt_open(rhport, ep_desc);
		SC_ASSERT(success);
	}

	bool success_cmd = dcd_edpt_xfer(rhport, usb_cmd->pipe, usb_cmd->rx_buffers[usb_cmd->rx_bank], CMD_BUFFER_SIZE);
	bool success_can = dcd_edpt_xfer(rhport, usb_can->pipe, usb_can->rx_buffers[usb_can->rx_bank], MSG_BUFFER_SIZE);
	SC_ASSERT(success_cmd);
	SC_ASSERT(success_can);

	*p_length = 9+eps*7;

	return true;
}

bool tud_custom_xfer_cb(
	uint8_t rhport,
	uint8_t ep_addr,
	xfer_result_t event,
	uint32_t xferred_bytes)
{
	(void)event; // always success

	if (unlikely(rhport != usb.port)) {
		return false;
	}

	USB_TRAFFIC_DO_LED;



	switch (ep_addr) {
	case SC_M1_EP_CMD0_BULK_OUT:
		sc_cmd_bulk_out(0, xferred_bytes);
		break;
	case SC_M1_EP_CMD1_BULK_OUT:
		sc_cmd_bulk_out(1, xferred_bytes);
		break;
	case SC_M1_EP_CMD0_BULK_IN:
		sc_cmd_bulk_in(0);
		break;
	case SC_M1_EP_CMD1_BULK_IN:
		sc_cmd_bulk_in(1);
		break;
	case SC_M1_EP_MSG0_BULK_OUT:
		sc_can_bulk_out(0, xferred_bytes);
		break;
	case SC_M1_EP_MSG1_BULK_OUT:
		sc_can_bulk_out(1, xferred_bytes);
		break;
	case SC_M1_EP_MSG0_BULK_IN:
		sc_can_bulk_in(0);
		break;
	case SC_M1_EP_MSG1_BULK_IN:
		sc_can_bulk_in(1);
		break;
	default:
		LOG("port %u ep %02x event %d bytes %u\n", rhport, ep_addr, event, (unsigned)xferred_bytes);
		return false;
	}

	return true;
}


static inline const char* recipient_str(tusb_request_recipient_t r)
{
	switch (r) {
	case TUSB_REQ_RCPT_DEVICE:
		return "device (0)";
	case TUSB_REQ_RCPT_INTERFACE:
		return "interface (1)";
	case TUSB_REQ_RCPT_ENDPOINT:
		return "endpoint (2)";
	case TUSB_REQ_RCPT_OTHER:
		return "other (3)";
	default:
		return "???";
	}
}

static inline const char* type_str(tusb_request_type_t value)
{
	switch (value) {
	case TUSB_REQ_TYPE_STANDARD:
		return "standard (0)";
	case TUSB_REQ_TYPE_CLASS:
		return "class (1)";
	case TUSB_REQ_TYPE_VENDOR:
		return "vendor (2)";
	case TUSB_REQ_TYPE_INVALID:
		return "invalid (3)";
	default:
		return "???";
	}
}

static inline const char* dir_str(tusb_dir_t value)
{
	switch (value) {
	case TUSB_DIR_OUT:
		return "out (0)";
	case TUSB_DIR_IN:
		return "in (1)";
	default:
		return "???";
	}
}


bool tud_vendor_control_request_cb(uint8_t rhport, tusb_control_request_t const * request)
{
	// LOG("port %u req\n", rhport);

	if (unlikely(rhport != usb.port)) {
		return false;
	}

	USB_TRAFFIC_DO_LED;

	switch (request->bRequest) {
	case VENDOR_REQUEST_MICROSOFT:
		if (request->wIndex == 7) {
			// Get Microsoft OS 2.0 compatible descriptor
			uint16_t total_len;
			memcpy(&total_len, desc_ms_os_20+8, 2);
			total_len = le16_to_cpu(total_len);
			return tud_control_xfer(rhport, request, (void*)desc_ms_os_20, total_len);
		}
		break;
	default:
		LOG("req type 0x%02x (reci %s type %s dir %s) req 0x%02x, value 0x%04x index 0x%04x reqlen %u\n",
			request->bmRequestType,
			recipient_str(request->bmRequestType_bit.recipient),
			type_str(request->bmRequestType_bit.type),
			dir_str(request->bmRequestType_bit.direction),
			request->bRequest, request->wValue, request->wIndex,
			request->wLength);
		break;
	}

	// stall unknown request
	return false;
}

bool tud_vendor_control_complete_cb(uint8_t rhport, tusb_control_request_t const *request)
{
	(void) rhport;
	(void) request;

	return true;
}

#if defined(CFG_TUD_VENDOR_CUSTOM) && CFG_TUD_VENDOR_CUSTOM
void vendord_init(void)
{
	tud_custom_init_cb();
}

void vendord_reset(uint8_t rhport)
{
	tud_custom_reset_cb(rhport);
}

bool vendord_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t *p_len)
{
	return tud_custom_open_cb(rhport, itf_desc, p_len);
}

bool vendord_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
	return tud_custom_xfer_cb(rhport, ep_addr, result, xferred_bytes);
}
#endif

#if CFG_TUD_DFU_RT
void dfu_rtd_init(void)
{
}

void dfu_rtd_reset(uint8_t rhport)
{
	(void) rhport;
	LOG("dfu_rtd_reset\n");

	if (DFU_STATE_APP_DETACH == dfu.status.bState) {
		LOG("Detected USB reset while DFU detach timer is running\n");
		dfu_request_dfu(1);
		NVIC_SystemReset();
	} else {
		dfu.status.bState = DFU_STATE_APP_IDLE;
	}
}

bool dfu_rtd_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t *p_length)
{
	(void) rhport;

	// Ensure this is DFU Runtime
	TU_VERIFY(itf_desc->bInterfaceSubClass == TUD_DFU_APP_SUBCLASS);
	TU_VERIFY(itf_desc->bInterfaceProtocol == DFU_PROTOCOL_RT);

	uint8_t const * p_desc = tu_desc_next( itf_desc );
	(*p_length) = sizeof(tusb_desc_interface_t);

	if (TUSB_DESC_FUNCTIONAL == tu_desc_type(p_desc)) {
		(*p_length) += p_desc[DESC_OFFSET_LEN];
		p_desc = tu_desc_next(p_desc);
	}

	return true;
}

bool dfu_rtd_control_complete(uint8_t rhport, tusb_control_request_t const * request)
{
	(void) rhport;
	(void) request;

	// nothing to do
	return true;
}

bool dfu_rtd_control_request(uint8_t rhport, tusb_control_request_t const * request)
{
	// Handle class request only
	// TU_VERIFY(request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);
	// TU_VERIFY(request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE);




	switch (request->bRequest) {
	case DFU_REQUEST_GETSTATUS:
		return tud_control_xfer(rhport, request, &dfu.status, sizeof(dfu.status));
	case DFU_REQUEST_DETACH:
		LOG("detach request, timeout %u [ms]\n", request->wValue);
		if (!dfu_signature_compatible()) {
			TU_LOG1("Bootloader runtime signature incompatible, not starting detach\n");
		} else {
			dfu.status.bState = DFU_STATE_APP_DETACH;
			xTimerStart(dfu.timer_handle, 0);
			/*
			 * This is wrong. DFU 1.1. specification wants us to wait for USB reset.
			 * However, without these lines, the device can't be updated on Windows using
			 * dfu-util. Updating from Linux works either way. All hail compatibility (sigh).
			 */
			dfu_request_dfu(1);
			NVIC_SystemReset();
		}
		return tud_control_xfer(rhport, request, NULL, 0);
	default:
		LOG("req type 0x%02x (reci %s type %s dir %s) req 0x%02x, value 0x%04x index 0x%04x reqlen %u\n",
			request->bmRequestType,
			recipient_str(request->bmRequestType_bit.recipient),
			type_str(request->bmRequestType_bit.type),
			dir_str(request->bmRequestType_bit.direction),
			request->bRequest, request->wValue, request->wIndex,
			request->wLength);
		break;
	}

	return false;
}

bool dfu_rtd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
	(void) rhport;
	(void) ep_addr;
	(void) result;
	(void) xferred_bytes;
	return true;
}
#endif

#define LATCH_STATE_UNLATCHED  0
#define LATCH_STATE_LATCHED    1


//--------------------------------------------------------------------+
// CAN TASK
//--------------------------------------------------------------------+
static void can_task(void *param)
{
	const unsigned BUS_ACTIVITY_TIMEOUT_MS = 256;

	const uint8_t index = (uint8_t)(uintptr_t)param;
	SC_ASSERT(index < TU_ARRAY_SIZE(cans.can));
	SC_ASSERT(index < TU_ARRAY_SIZE(usb.can));

	LOG("CAN%u task start\n", index);

	struct can *can = &cans.can[index];
	struct usb_can *usb_can = &usb.can[index];

	uint32_t rx_ts_last = 0;
	uint32_t tx_ts_last = 0;
	uint8_t previous_bus_status = 0;
	uint8_t current_bus_status = 0;
	TickType_t bus_activity_tc = 0;
	bool had_bus_activity = false;
	bool has_bus_error = false;
	bool had_bus_error = false;
	bool send_can_status = 0;
	struct can_queue_item queue_item;
	uint16_t tscv_high = 0;
	uint16_t tscv_low = 0;
	uint16_t rx_high = 0;
	uint16_t rx_low = 0;
	uint8_t rx_latch_state = LATCH_STATE_UNLATCHED;
	uint16_t tx_high = 0;
	uint16_t tx_low = 0;
	uint8_t tx_latch_state = LATCH_STATE_UNLATCHED;


	while (42) {
		// LOG("CAN%u task wait\n", index);
		(void)ulTaskNotifyTake(pdFALSE, ~0);

		// LOG("CAN%u task loop\n", index);
		if (unlikely(!usb.mounted)) {
			continue;
		}

		if (unlikely(!can->enabled)) {
			current_bus_status = 0;
			bus_activity_tc = xTaskGetTickCount() - pdMS_TO_TICKS(BUS_ACTIVITY_TIMEOUT_MS);
			had_bus_activity = false;
			has_bus_error = false;
			had_bus_error = false;
			rx_ts_last = 0;
			tx_ts_last = 0;
			tscv_high = 0;
			tscv_low = 0;
			rx_high = 0;
			rx_low = 0;
			rx_latch_state = LATCH_STATE_UNLATCHED;
			tx_high = 0;
			tx_low = 0;
			tx_latch_state = LATCH_STATE_UNLATCHED;
			continue;
		}

		led_burst(can->led_traffic, 8);
		send_can_status = 1;

		xSemaphoreTake(usb_can->mutex_handle, ~0);

		for (bool done = false; !done; ) {
			done = true;

			// timestamp
			uint16_t tscv = can->m_can->TSCV.bit.TSC;

			if (tscv < tscv_low) {
				++tscv_high;
				// LOG("ch%u ts_high=%08lx\n", index, tscv_high);
			}

			tscv_low = tscv;

			if (LATCH_STATE_LATCHED == rx_latch_state) {
				uint16_t diff = tscv_high - rx_high;
				if (diff && diff < UINT16_MAX / 2) {
					rx_latch_state = LATCH_STATE_UNLATCHED;
					// LOG("ch%u rx ts unlatch\n", index);
				}
			}

			if (LATCH_STATE_LATCHED == tx_latch_state) {
				uint16_t diff = tscv_high - tx_high;
				if (diff && diff < UINT16_MAX / 2) {
					tx_latch_state = LATCH_STATE_UNLATCHED;
					// LOG("ch%u tx ts unlatch\n", index);
				}
			}

			// loop


			uint8_t *out_beg;
			uint8_t *out_end;
			uint8_t *out_ptr;

			out_beg = usb_can->tx_buffers[usb_can->tx_bank];
			out_end = out_beg + MSG_BUFFER_SIZE;
			SC_ASSERT(usb_can->tx_offsets[usb_can->tx_bank] <= MSG_BUFFER_SIZE);
			out_ptr = out_beg + usb_can->tx_offsets[usb_can->tx_bank];
			SC_ASSERT(out_ptr <= out_end);



			if (send_can_status) {
				uint8_t bytes = sizeof(struct sc_msg_can_status);
				if (out_end - out_ptr >= bytes) {
					done = false;
					send_can_status = 0;
					struct sc_msg_can_status *msg = (struct sc_msg_can_status *)out_ptr;
					out_ptr += bytes;

					uint16_t rx_lost = __sync_fetch_and_and(&can->rx_lost, 0);
					uint16_t tx_dropped = can->tx_dropped;
					can->tx_dropped = 0;
					uint32_t ts = ((uint32_t)tscv_high << M_CAN_TS_COUNTER_BITS) | tscv_low;
					// LOG("status ts %lu\n", ts);
					uint32_t us = can_bittime_to_us(can, ts);
					CAN_ECR_Type ecr = can->m_can->ECR;


					msg->id = SC_MSG_CAN_STATUS;
					msg->len = sizeof(*msg);
					msg->timestamp_us = us;
					msg->rx_lost = rx_lost;
					msg->tx_dropped = tx_dropped;
					msg->flags = __sync_or_and_fetch(&can->int_comm_flags, 0);
					if (can->desync) {
						msg->flags |= SC_CAN_STATUS_FLAG_TXR_DESYNC;
					}

					msg->bus_status = current_bus_status;
					msg->tx_errors = ecr.bit.TEC;
					msg->rx_errors = ecr.bit.REC;
					msg->tx_fifo_size = CAN_TX_FIFO_SIZE - can->m_can->TXFQS.bit.TFFL;
					msg->rx_fifo_size = can->m_can->RXF0S.bit.F0FL;

					SC_ASSERT(out_ptr <= out_end);
					usb_can->tx_offsets[usb_can->tx_bank] = out_ptr - out_beg;
				} else {
					if (sc_can_bulk_in_ep_ready(index)) {
						done = false;
						sc_can_bulk_in_submit(index, __func__);
						continue;
					} else {
						break;
					}
				}
			}

			if (pdTRUE == xQueueReceive(can->queue_handle, &queue_item, 0)) {
				done = false;
				bus_activity_tc = xTaskGetTickCount();

				switch (queue_item.type) {
				case CAN_QUEUE_ITEM_TYPE_BUS_STATUS: {
					current_bus_status = queue_item.payload;
					LOG("ch%u bus status %#x\n", index, current_bus_status);
					send_can_status = 1;
				} break;
				case CAN_QUEUE_ITEM_TYPE_BUS_ERROR: {
					has_bus_error = SC_CAN_ERROR_NONE != queue_item.payload;
					uint8_t bytes = sizeof(struct sc_msg_can_error);
					if (out_end - out_ptr >= bytes) {
						struct sc_msg_can_error *msg = (struct sc_msg_can_error *)out_ptr;
						out_ptr += bytes;

						// uint32_t ts = __sync_or_and_fetch(&can->int_ts, 0);
						uint32_t ts = (uint32_t)tscv_high << M_CAN_TS_COUNTER_BITS | tscv_low;
						uint32_t us = can_bittime_to_us(can, ts);

						msg->id = SC_MSG_CAN_ERROR;
						msg->len = sizeof(*msg);
						msg->timestamp_us = us;
						msg->error = queue_item.payload;
						msg->flags = 0;
						if (queue_item.tx) {
							msg->flags |= SC_CAN_ERROR_FLAG_RXTX_TX;
						}
						if (queue_item.data_part) {
							msg->flags |= SC_CAN_ERROR_FLAG_NMDT_DT;
						}

						SC_ASSERT(out_ptr <= out_end);
						usb_can->tx_offsets[usb_can->tx_bank] = out_ptr - out_beg;
					} else {
						if (sc_can_bulk_in_ep_ready(index)) {
							sc_can_bulk_in_submit(index, __func__);
							continue;
						} else {
							// LOG("ch%u dropped CAN bus error msg\n", index);
							// break;
						}
					}
				} break;
				default:
					LOG("ch%u unhandled message type %#02x\n", index, queue_item.type);
					break;
				}
			}

			if (m_can_rx0_msg_fifo_avail(can->m_can)) {
				bus_activity_tc = xTaskGetTickCount();
				uint8_t get_index = can->m_can->RXF0S.bit.F0GI;
				uint8_t bytes = sizeof(struct sc_msg_can_rx);
				CAN_RXF0E_0_Type r0 = can->rx_fifo[get_index].R0;
				CAN_RXF0E_1_Type r1 = can->rx_fifo[get_index].R1;
				uint8_t can_frame_len = dlc_to_len(r1.bit.DLC);
				if (!r0.bit.RTR) {
					bytes += can_frame_len;
				}

				// align to 4 bytes
				if (bytes & 3) {
					bytes += 4 - (bytes & 3);
				}

				if (out_end - out_ptr >= bytes) {
					done = false;
					struct sc_msg_can_rx *msg = (struct sc_msg_can_rx *)out_ptr;
					out_ptr += bytes;

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

					uint16_t msg_low = r1.bit.RXTS;
					if (LATCH_STATE_UNLATCHED == rx_latch_state) {
						rx_latch_state = LATCH_STATE_LATCHED;
						rx_high = tscv_high;
					} else {
						if (msg_low < rx_low) {
							++rx_high;
							// LOG("ch%u rx_high=%x tscv_high=%x\n", index, rx_high, tscv_high);
						}
					}

					rx_low = msg_low;

					uint32_t ts = ((uint32_t)rx_high << M_CAN_TS_COUNTER_BITS) | msg_low;
					if (ts < rx_ts_last) {
						LOG("ch%u rx gi=%u ts=%lx prev=%lx\n", index, get_index, ts, rx_ts_last);
					}
					SC_ASSERT(ts >= rx_ts_last);
					rx_ts_last = ts;

					msg->timestamp_us = can_bittime_to_us(can, ts);

					// LOG("ch%u rx ts %lu\n", index, msg->timestamp_us);

					if (r1.bit.FDF) {
						msg->flags |= SC_CAN_FRAME_FLAG_FDF;
					}
					if (r1.bit.BRS) {
						msg->flags |= SC_CAN_FRAME_FLAG_BRS;
					}
					if (r0.bit.RTR) {
						msg->flags |= SC_CAN_FRAME_FLAG_RTR;
					} else {
						memcpy(msg->data, can->rx_fifo[get_index].data, can_frame_len);
					}

					SC_ASSERT(out_ptr <= out_end);
					usb_can->tx_offsets[usb_can->tx_bank] = out_ptr - out_beg;

					can->m_can->RXF0A.reg = CAN_RXF0A_F0AI(get_index);
				} else {
					if (sc_can_bulk_in_ep_ready(index)) {
						done = false;
						sc_can_bulk_in_submit(index, __func__);
						continue;
					} else {
						break;
					}
				}
			}

			if (m_can_tx_event_fifo_avail(can->m_can)) {
				done = false;
				bus_activity_tc = xTaskGetTickCount();
				uint8_t bytes = sizeof(struct sc_msg_can_txr);


				if (out_end - out_ptr >= bytes) {
					done = false;
					struct sc_msg_can_txr *msg = (struct sc_msg_can_txr *)out_ptr;
					out_ptr += bytes;

					uint8_t get_index = can->m_can->TXEFS.bit.EFGI;
					CAN_TXEFE_0_Type t0 = can->tx_event_fifo[get_index].T0;
					CAN_TXEFE_1_Type t1 = can->tx_event_fifo[get_index].T1;

					msg->id = SC_MSG_CAN_TXR;
					msg->len = bytes;
					msg->track_id = t1.bit.MM;
					uint16_t msg_low = t1.bit.TXTS;
					if (LATCH_STATE_UNLATCHED == tx_latch_state) {
						tx_latch_state = LATCH_STATE_LATCHED;
						tx_high = tscv_high;
					} else {
						if (msg_low < tx_low) {
							++tx_high;
							// LOG("ch%u tx_high=%x tscv_high=%x\n", index, tx_high, tscv_high);
						}
					}

					tx_low = msg_low;

					uint32_t ts = ((uint32_t)tx_high << M_CAN_TS_COUNTER_BITS) | msg_low;
					// if (ts < tx_ts_last) {
					// 	LOG("tx gi=%u ts=%lx prev=%lx\n", get_index, ts, tx_ts_last);
					// }
					SC_ASSERT(ts >= tx_ts_last);
					tx_ts_last = ts;
					msg->timestamp_us = can_bittime_to_us(can, ts);
					msg->flags = 0;
					if (t0.bit.ESI) {
						msg->flags |= SC_CAN_FRAME_FLAG_ESI;
					}

					if (t1.bit.FDF) {
						msg->flags |= SC_CAN_FRAME_FLAG_FDF;
					}

					if (t1.bit.BRS) {
						msg->flags |= SC_CAN_FRAME_FLAG_BRS;
					}

					SC_ASSERT(out_ptr <= out_end);
					usb_can->tx_offsets[usb_can->tx_bank] = out_ptr - out_beg;

					can->m_can->TXEFA.reg = CAN_TXEFA_EFAI(get_index);
				} else {
					if (sc_can_bulk_in_ep_ready(index)) {
						done = false;
						sc_can_bulk_in_submit(index, __func__);
						continue;
					} else {
						// LOG("ch%u no space for txr\n");
						// can->desync = true;
						break;
					}
				}
			}
		}

		if (usb_can->tx_offsets[usb_can->tx_bank] > 0 && sc_can_bulk_in_ep_ready(index)) {
			sc_can_bulk_in_submit(index, __func__);
		}

		const bool has_bus_activity = xTaskGetTickCount() - bus_activity_tc < pdMS_TO_TICKS(BUS_ACTIVITY_TIMEOUT_MS);
		bool led_change =
			has_bus_activity != had_bus_activity ||
			has_bus_error != had_bus_error;
		if (!led_change) {
			if (previous_bus_status >= SC_CAN_STATUS_ERROR_PASSIVE &&
				current_bus_status < SC_CAN_STATUS_ERROR_PASSIVE) {
				led_change = true;
			} else if (previous_bus_status < SC_CAN_STATUS_ERROR_PASSIVE &&
				current_bus_status >= SC_CAN_STATUS_ERROR_PASSIVE) {
				led_change = true;
			}
		}

		if (led_change) {
			if (has_bus_error || current_bus_status >= SC_CAN_STATUS_ERROR_PASSIVE) {
				canled_set_status(can, has_bus_activity ? CANLED_STATUS_ERROR_ACTIVE : CANLED_STATUS_ERROR_PASSIVE);
			} else {
				canled_set_status(can, has_bus_activity ? CANLED_STATUS_ENABLED_BUS_ON_ACTIVE : CANLED_STATUS_ENABLED_BUS_ON_PASSIVE);
			}
		}

		had_bus_activity = has_bus_activity;
		had_bus_error = has_bus_error;
		previous_bus_status = current_bus_status;

		xSemaphoreGive(usb_can->mutex_handle);
	}
}
