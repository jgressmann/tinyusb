/*
 * Copyright (c) 2020 Jean Gressmann <jean@0x42.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <bsp/board.h>
#include <tusb.h>
#include <dcd.h>
#include <usbd_pvt.h>

#include <sam.h>

#include <hal/include/hal_gpio.h>

#define SC_PACKED __packed

#include <supercan.h>


#include <m_can.h>



#include <linux.h>
#include <peak_canfd.h>
#include <peak_usb_fd.h>

#if CONF_CPU_FREQUENCY != 120000000
# error "CONF_CPU_FREQUENCY" must 120000000
#endif


#define CAN_CLK_HZ CONF_CPU_FREQUENCY

// #define PCAN_USBPROFD_CHANNEL_COUNT	2
// #define PCAN_USBFD_CHANNEL_COUNT	1

/* PCAN-USB Pro FD adapter internal clock (Hz) */
#define PCAN_UFD_CRYSTAL_HZ		80000000

#define PCAN_UFD_CMD_BUFFER_SIZE	512
#define PCAN_UFD_LOSPD_PKT_SIZE		64

/* PCAN-USB Pro FD command timeout (ms.) */
#define PCAN_UFD_CMD_TIMEOUT_MS		1000

/* PCAN-USB Pro FD rx/tx buffers size */
#define PCAN_UFD_RX_BUFFER_SIZE		2048
#define PCAN_UFD_TX_BUFFER_SIZE		512



/*
 * USB Vendor request data types
 */
#define PCAN_USBPRO_REQ_INFO		0
#define PCAN_USBPRO_REQ_FCT		2

/* Vendor Request value for XXX_INFO */
#define PCAN_USBPRO_INFO_BL		0
#define PCAN_USBPRO_INFO_FW		1

typedef uint16_t __le16;
typedef uint32_t __le32;
typedef uint8_t u8;

struct __packed pcan_ufd_fw_info {
	__le16	size_of;	/* sizeof this */
	__le16	type;		/* type of this structure */
	u8	hw_type;	/* Type of hardware (HW_TYPE_xxx) */
	u8	bl_version[3];	/* Bootloader version */
	u8	hw_version;	/* Hardware version (PCB) */
	u8	fw_version[3];	/* Firmware version */
	__le32	dev_id[2];	/* "device id" per CAN */
	__le32	ser_no;		/* S/N */
	__le32	flags;		/* special functions */
};


/* Extended USB commands (non uCAN commands) */

/* Clock Modes command */
#define PCAN_UFD_CMD_CLK_SET		0x80

#define PCAN_UFD_CLK_80MHZ		0x0
#define PCAN_UFD_CLK_60MHZ		0x1
#define PCAN_UFD_CLK_40MHZ		0x2
#define PCAN_UFD_CLK_30MHZ		0x3
#define PCAN_UFD_CLK_24MHZ		0x4
#define PCAN_UFD_CLK_20MHZ		0x5
#define PCAN_UFD_CLK_DEF		PCAN_UFD_CLK_80MHZ

struct __packed pcan_ufd_clock {
	__le16	opcode_channel;

	u8	mode;
	u8	unused[5];
};

/* LED control command */
#define PCAN_UFD_CMD_LED_SET		0x86

#define PCAN_UFD_LED_DEV		0x00
#define PCAN_UFD_LED_FAST		0x01
#define PCAN_UFD_LED_SLOW		0x02
#define PCAN_UFD_LED_ON			0x03
#define PCAN_UFD_LED_OFF		0x04
#define PCAN_UFD_LED_DEF		PCAN_UFD_LED_DEV

struct __packed pcan_ufd_led {
	__le16	opcode_channel;

	u8	mode;
	u8	unused[5];
};

/* Extended usage of uCAN messages for PCAN-USB Pro FD */
#define PCAN_UFD_MSG_CALIBRATION	0x100

struct __packed pcan_ufd_ts_msg {
	__le16	size;
	__le16	type;
	__le32	ts_low;
	__le32	ts_high;
	__le16	usb_frame_index;
	u16	unused;
};

/* Extended usage of uCAN commands CMD_xxx_xx_OPTION for PCAN-USB Pro FD */
#define PCAN_UFD_FLTEXT_CALIBRATION	0x8000

struct __packed pcan_ufd_options {
	__le16	opcode_channel;

	__le16	ucan_mask;
	u16	unused;
	__le16	usb_mask;
};


#define PCAN_UFD_MSG_OVERRUN		0x101

#define PCAN_UFD_OVMSG_CHANNEL(o)	((o)->channel & 0xf)

struct __packed pcan_ufd_ovr_msg {
	__le16	size;
	__le16	type;
	__le32	ts_low;
	__le32	ts_high;
	u8	channel;
	u8	unused[3];
};


static struct peakfd {
	uint8_t clock_mode;
} peakfd[2];


static const uint32_t peak_clock_mhz[] = {
	80 * 1000 * 1000,
	60 * 1000 * 1000,
	40 * 1000 * 1000,
	30 * 1000 * 1000,
	24 * 1000 * 1000,
	20 * 1000 * 1000,
};

struct m_can_reg {
	uint16_t brp;
	uint16_t tseg1;
	uint8_t tseg2;
	uint8_t sjw;
};

static void peak_to_m_can(
	uint8_t peak_brp,
	uint8_t peak_sjw,
	uint8_t peak_tseg1,
	uint8_t peak_tseg2,
	uint32_t peak_clock_hz,
	uint16_t max_tqs,
	struct m_can_reg const *limits_min,
	struct m_can_reg const *limits_max,
	struct m_can_reg *result)
{
	uint16_t peak_tqs = peak_sjw + peak_tseg1 + peak_tseg2;

	uint32_t bitrate_bps = peak_clock_hz / (peak_brp * peak_tqs);
	uint16_t tqs = max_tqs;

	result->brp = CAN_CLK_HZ / (tqs * bitrate_bps);
	result->sjw = (peak_sjw * tqs) / peak_tqs;
	result->sjw = tu_max8(limits_min->sjw, tu_min8(result->sjw, limits_max->sjw));
	result->tseg2 = (peak_tseg2 * tqs) / peak_tqs;
	result->tseg2 = tu_max8(limits_min->tseg2, tu_min8(result->tseg2, limits_max->tseg2));
	result->tseg1 = tqs - result->sjw - result->tseg2;

	while (result->brp < limits_max->brp &&
			result->tseg1 > limits_max->tseg1) {
		++result->brp;
		tqs = CAN_CLK_HZ / (result->brp * bitrate_bps);
		result->sjw = (peak_sjw * tqs) / peak_tqs;
		result->sjw = tu_max8(limits_min->sjw, tu_min8(result->sjw, limits_max->sjw));
		result->tseg2 = (peak_tseg2 * tqs) / peak_tqs;
		result->tseg2 = tu_max8(limits_min->tseg2, tu_min8(result->tseg2, limits_max->tseg2));
		result->tseg1 = tqs - result->sjw - result->tseg2;
	}

	// result->brp = limits_max->brp;
	// result->sjw = tu_max8(limits_min->sjw, tu_min8(peak_sjw, limits_max->sjw));
	// uint16_t m_can_tqs;
	// bool acceptable = false;
	// while (result->brp > limits_min->brp) {
	// 	m_can_tqs = CAN_CLK_HZ / (result->brp * bitrate_bps);
	// 	if (m_can_tqs < result->sjw + limits_min->tseg1 + limits_min->tseg2) {
	// 		--result->brp;
	// 	} else {
	// 		result->tseg2 = m_can_tqs - (m_can_tqs * ratio01) / 255;
	// 		result->tseg1 = m_can_tqs - result->sjw - result->tseg2;
	// 		if (result->tseg1 > limits_max->tseg1 ||
	// 			result->tseg2 > limits_max->tseg2) {
	// 			break;
	// 		}

	// 		acceptable = true;
	// 		--result->brp;
	// 	}
	// }

	// // finaly round
	// result->brp += acceptable;
	// m_can_tqs = CAN_CLK_HZ / (result->brp * bitrate_bps);
	// result->tseg2 = m_can_tqs - (m_can_tqs * ratio01) / 255;
	// result->tseg1 = m_can_tqs - result->sjw - result->tseg2;

}


#ifndef likely
#define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x),0)
#endif

#define CAN_TX_FIFO_SIZE 32
#define CAN_RX_FIFO_SIZE 64
#define CAN_ELEMENT_DATA_SIZE 64


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



struct can {
	CFG_TUSB_MEM_ALIGN struct can_tx_fifo_element tx_fifo[CAN_TX_FIFO_SIZE];
	// CFG_TUSB_MEM_ALIGN struct can_tx_event_fifo_element tx_event_fifo[CAN_TX_FIFO_SIZE];
	CFG_TUSB_MEM_ALIGN struct can_rx_fifo_element rx_fifo[CAN_RX_FIFO_SIZE];
	uint16_t message_marker_map[CAN_TX_FIFO_SIZE];
	TaskHandle_t task;
	Can *m_can;
	IRQn_Type interrupt_id;
	volatile uint32_t ts_high;
	uint32_t nm_bitrate_bps;
    uint16_t nmbt_brp;
    uint16_t nmbt_tseg1;
	uint8_t nmbt_sjw;
    uint8_t nmbt_tseg2;
    uint8_t dtbt_brp;
    uint8_t dtbt_sjw;
    uint8_t dtbt_tseg1;
    uint8_t dtbt_tseg2;
	uint8_t mode_flags;
	uint16_t rx_lost;
	uint16_t tx_dropped;
	volatile uint8_t status_flags;
	uint8_t led;
	bool enabled;
	// bool desync;
};

static struct {
	struct can can[2];
} cans;





static inline void can_init_pins(void) // controller and hardware specific setup of i/o pins for CAN
{
	// CAN ports
	REG_PORT_DIRSET0 = PORT_PA20; /* CAN_EN_1 */
	REG_PORT_DIRSET0 = PORT_PA21; /* CAN_STB_1 */
	REG_PORT_OUTSET0 = PORT_PA20; /* CAN_EN_1 */
	REG_PORT_OUTSET0 = PORT_PA21; /* CAN_STB_1 */

	REG_PORT_WRCONFIG0 =	PORT_WRCONFIG_HWSEL |			// upper half
	PORT_WRCONFIG_PINMASK(0x00c0) |	// 21 + 22
	PORT_WRCONFIG_WRPINCFG |
	PORT_WRCONFIG_WRPMUX |
	PORT_WRCONFIG_PMUX(8) |			// CAN
	PORT_WRCONFIG_PMUXEN;
}


static inline void can_init_clock(void) // controller and hardware specific setup of clock for the m_can module
{
	REG_MCLK_AHBMASK |= MCLK_AHBMASK_CAN0 | MCLK_AHBMASK_CAN1;
	REG_GCLK_PCHCTRL27 = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN0/1 to use GLCK0 -> 120MHz
}

static inline void can_configure(struct can *c)
{
	Can *can = c->m_can;
	// m_can_init_begin(can);
	m_can_conf_begin(can);

	can->CCCR.bit.TXP = 1;  // Enable Transmit Pause
	can->CCCR.bit.EFBI = 1; // Enable Edge Filtering
	can->CCCR.bit.MON = (c->mode_flags & SC_MODE_FLAG_TX) != SC_MODE_FLAG_TX;
	can->CCCR.bit.FDOE = (c->mode_flags & SC_MODE_FLAG_FD) == SC_MODE_FLAG_FD;
	can->CCCR.bit.BRSE = (c->mode_flags & SC_MODE_FLAG_BRS) == SC_MODE_FLAG_BRS;
	can->CCCR.bit.DAR = (c->mode_flags & SC_MODE_FLAG_AUTO_RE) != SC_MODE_FLAG_AUTO_RE;
	can->CCCR.bit.PXHD = (c->mode_flags & SC_MODE_FLAG_EH) != SC_MODE_FLAG_EH;


	TU_LOG2("nominal brp=%u sjw=%u tseg1=%u tseg2=%u\n",
		c->nmbt_brp, c->nmbt_sjw, c->nmbt_tseg1, c->nmbt_tseg2
	);

	TU_LOG2("data brp=%u sjw=%u tseg1=%u tseg2=%u\n",
		c->dtbt_brp, c->dtbt_sjw, c->dtbt_tseg1, c->dtbt_tseg2
	);

	can->TSCC.reg = CAN_TSCC_TCP(0) | CAN_TSCC_TSS(1);
	can->TOCC.reg = CAN_TOCC_TOP(0xffff) | CAN_TOCC_TOS(0); // Timeout Counter disabled, Reset-default
	can->NBTP.reg = CAN_NBTP_NSJW(c->nmbt_sjw-1)
			| CAN_NBTP_NBRP(c->nmbt_brp-1)
			| CAN_NBTP_NTSEG1(c->nmbt_tseg1-1)
			| CAN_NBTP_NTSEG2(c->nmbt_tseg2-1);
	can->DBTP.reg = CAN_DBTP_DBRP(c->dtbt_brp-1)
			| CAN_DBTP_DTSEG1(c->dtbt_tseg1-1)
			| CAN_DBTP_DTSEG2(c->dtbt_tseg2-1)
			| CAN_DBTP_DSJW(c->dtbt_sjw-1);

	// can->NBTP.reg = CAN_NBTP_NBRP(2) | CAN_NBTP_NTSEG1(62) | CAN_NBTP_NTSEG2(15) | CAN_NBTP_NSJW(15); /* 500kBit @ 120 / 3 = 40MHz, 80% */
    // can->DBTP.reg = CAN_DBTP_DBRP(2) | CAN_DBTP_DTSEG1(12) | CAN_DBTP_DTSEG2(5) | CAN_DBTP_DSJW(5); /* 2MBit @ 120 / 3 = 40MHz, 70% */

	// tx fifo
	can->TXBC.reg = CAN_TXBC_TBSA((uint32_t) c->tx_fifo) | CAN_TXBC_TFQS(CAN_TX_FIFO_SIZE);
	//	REG_CAN0_TXBC |= CAN_TXBC_TFQM; // reset default
	can->TXESC.reg = CAN_TXESC_TBDS_DATA64;

	//can->TXEFC.reg = CAN_TXEFC_EFSA((uint32_t) c->tx_event_fifo) | CAN_TXEFC_EFS(CAN_TX_FIFO_SIZE);


	// rx fifo0
	can->RXF0C.reg = CAN_RXF0C_F0SA((uint32_t) c->rx_fifo) | CAN_RXF0C_F0S(CAN_RX_FIFO_SIZE);
	//  | CAN_RXF0C_F0OM; // FIFO 0 overwrite mode
	can->RXESC.reg = CAN_RXESC_RBDS_DATA64 + CAN_RXESC_F0DS_DATA64;

	// clear existing interrupt flags
	can->IR.reg = can->IR.reg;
	// enable interrupt line 0
	can->ILE.reg = CAN_ILE_EINT0;

	// select interrupt line 0
	// can->ILS.reg = 0; // reset default

	// wanted interrupts
	can->IE.reg =
		CAN_IE_TSWE     // time stamp counter wrap
		| CAN_IE_BOE    // bus off
		| CAN_IE_EWE    // error warning
		| CAN_IE_EPE    // error passive
		//| CAN_IE_TEFNE  // new message in tx event fifo
		| CAN_IE_RF0NE  // new message in rx fifo0
		| CAN_IE_RF0LE  // message lost b/c fifo0 was full
		;


	m_can_conf_end(can);
	// m_can_init_end(can);
}

static void can_init_module(void);
static void can_init_module(void)
{
	memset(&cans, 0, sizeof(cans));

	cans.can[0].m_can = CAN0;
	cans.can[1].m_can = CAN1;
	cans.can[0].interrupt_id = CAN0_IRQn;
	cans.can[1].interrupt_id = CAN1_IRQn;

	for (size_t j = 0; j < TU_ARRAY_SIZE(cans.can); ++j) {
		struct can *can = &cans.can[j];
		for (size_t i = 0; i < TU_ARRAY_SIZE(cans.can[0].tx_fifo); ++i) {
			can->tx_fifo[i].T1.bit.EFC = 1; // store tx events
			can->tx_fifo[i].T1.bit.MM = i; 	// direct mapping of message markers
		}
	}

	m_can_init_begin(CAN0);
	m_can_init_begin(CAN1);

	NVIC_SetPriority(CAN0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(CAN1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
}

static inline void can_int(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(cans.can), );
	struct can *can = &cans.can[index];

	// if (m_can_rx0_msg_fifo_avail(can->m_can)) {
	// 	TU_LOG2("CAN%u has can msg\n", index);
	// }

	// TU_LOG2("CAN%u CCCR=%08lx NBTP=%08lx DBTP=%08lx ECR=%08lx PSR=%08lx RXF0S=%08lx\n",
	// 	index,
	// 	can->m_can->CCCR.reg,
	// 	can->m_can->NBTP.reg,
	// 	can->m_can->DBTP.reg,
	// 	can->m_can->ECR.reg,
	// 	can->m_can->PSR.reg,
	// 	can->m_can->RXF0S.reg);

	bool notify = false;
	CAN_IR_Type ir = can->m_can->IR;
	if (ir.bit.TSW) {
		uint32_t ts_high = __sync_add_and_fetch(&can->ts_high, 1u << M_CAN_TS_COUNTER_BITS);
		(void)ts_high;
		// TU_LOG2("CAN%u ts_high %08lx\n", index, ts_high);
	}

	uint8_t prev_status = can->status_flags;
	uint8_t curr_status = 0;

	bool had_ep = (prev_status & SC_STATUS_FLAG_ERROR_PASSIVE) == SC_STATUS_FLAG_ERROR_PASSIVE;
	notify |= had_ep != ir.bit.EP;
	if (ir.bit.EP) {
		TU_LOG2("CAN%u error passive\n", index);
		curr_status |= SC_STATUS_FLAG_ERROR_PASSIVE;
	}

	bool had_ew = (prev_status & SC_STATUS_FLAG_ERROR_WARNING) == SC_STATUS_FLAG_ERROR_WARNING;
	notify |= had_ew != ir.bit.EW;
	if (ir.bit.EW) {
		TU_LOG2("CAN%u error warning\n", index);
		curr_status |= SC_STATUS_FLAG_ERROR_WARNING;
	}

	bool had_bo = (prev_status & SC_STATUS_FLAG_BUS_OFF) == SC_STATUS_FLAG_BUS_OFF;
	notify |= had_bo != ir.bit.BO;
	if (ir.bit.BO) {
		TU_LOG2("CAN%u bus off\n", index);
		curr_status |= SC_STATUS_FLAG_BUS_OFF;
	}

	bool had_rx_fifo0_msg_lost = (prev_status & SC_STATUS_FLAG_RX_FULL) == SC_STATUS_FLAG_RX_FULL;
	notify |= had_rx_fifo0_msg_lost != ir.bit.RF0L;
	if (ir.bit.RF0L) {
		TU_LOG2("CAN%u msg lost\n", index);
		curr_status |= SC_STATUS_FLAG_RX_FULL;
	}

	can->status_flags = curr_status;

	notify |= ir.bit.RF0N;

	// clear all interrupts
	can->m_can->IR = ir;

	if (notify) {
		// TU_LOG2("CAN%u notify\n", index);
		BaseType_t woken = pdFALSE;
		vTaskNotifyGiveFromISR(can->task, &woken);
		portYIELD_FROM_ISR(woken);
	}
}

void CAN0_Handler(void)
{
	// TU_LOG2("CAN0 int\n");
	can_int(0);
}

void CAN1_Handler(void)
{
	// TU_LOG2("CAN1 int\n");
	can_int(1);
}

static inline void can_set_state1(Can *can, IRQn_Type interrupt_id, bool enabled)
{
	if (enabled) {
		m_can_init_end(can);
		NVIC_EnableIRQ(interrupt_id);
	} else {
		NVIC_DisableIRQ(interrupt_id);
		m_can_init_begin(can);
	}
}

static void can_engage(void)
{
	for (size_t i = 0; i < TU_ARRAY_SIZE(cans.can); ++i) {
		struct can *can = &cans.can[i];
		can_set_state1(can->m_can, can->interrupt_id, can->enabled);
	}
}

static void can_disengage(void)
{
	for (size_t i = 0; i < TU_ARRAY_SIZE(cans.can); ++i) {
		struct can *can = &cans.can[i];
		// can->desync = false;
		can_set_state1(can->m_can, can->interrupt_id, false);
	}
}

static inline uint8_t dlc_to_len(uint8_t dlc)
{
	static const uint8_t map[16] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
	};
	return map[dlc & 0xf];
}

static inline uint32_t can_time_to_us(struct can const *can, uint32_t can_time)
{
	uint64_t r = can_time;
	r *= can->nm_bitrate_bps;
	r /= 1000000;
	return r;
}

// static task for usbd
// Increase stack size when debug log is enabled
#if CFG_TUSB_DEBUG
	#define USBD_STACK_SIZE (3*configMINIMAL_STACK_SIZE)
#else
	#define USBD_STACK_SIZE (3*configMINIMAL_STACK_SIZE/2)
#endif

static StackType_t usb_device_stack[USBD_STACK_SIZE];
static StaticTask_t usb_device_stack_mem;

static StackType_t led_task_stack[configMINIMAL_STACK_SIZE];
static StaticTask_t led_task_mem;

static StackType_t can_task_stack[2][configMINIMAL_STACK_SIZE];
static StaticTask_t can_task_mem[2];


static void tusb_device_task(void* param);
static void led_task(void* param);
static void can_task(void* param);



struct led {
#if CFG_TUSB_DEBUG > 0
	const char* name;
#endif
	volatile uint16_t time_ms	: 15;
	volatile uint16_t blink		: 1;
	uint8_t pin;
};

#if CFG_TUSB_DEBUG > 0
#define LED_STATIC_INITIALIZER(name, pin) \
	{ name, 0, 0, pin }
#else
#define LED_STATIC_INITIALIZER(name, pin) \
	{ 0, 0, pin }
#endif

static struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PA02),
	LED_STATIC_INITIALIZER("red1", PIN_PB14),
	LED_STATIC_INITIALIZER("orange1", PIN_PB15),
	LED_STATIC_INITIALIZER("green1", PIN_PA12),
	LED_STATIC_INITIALIZER("red2", PIN_PA13),
	LED_STATIC_INITIALIZER("orange2", PIN_PA14),
	LED_STATIC_INITIALIZER("green2", PIN_PA15),
};

enum {
	LED_DEBUG,
	LED_RED1,
	LED_ORANGE1,
	LED_GREEN1,
	LED_RED2,
	LED_ORANGE2,
	LED_GREEN2
};

#define USB_TRAFFIC_LED LED_ORANGE1
#define USB_TRAFFIC_BURST_DURATION_MS 8
#define USB_TRAFFIC_DO_LED led_burst(USB_TRAFFIC_LED, USB_TRAFFIC_BURST_DURATION_MS)

#define CAN_TX_LED LED_GREEN1
#define CAN_TX_BURST_DURATION_MS 8

#define CAN_RX_LED LED_GREEN2
#define CAN_RX_BURST_DURATION_MS CAN_TX_BURST_DURATION_MS


static void led_init(void);
static inline void led_set(uint8_t index, bool on)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].time_ms = 0;
	gpio_set_pin_level(leds[index].pin, on);
}

static inline void led_toggle(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].time_ms = 0;
	gpio_toggle_pin_level(leds[index].pin);
}

static inline void led_blink(uint8_t index, uint16_t delay_ms)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].time_ms = delay_ms;
	leds[index].blink = 1;
}

static inline void led_burst(uint8_t index, uint16_t duration_ms)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].time_ms = duration_ms;
	leds[index].blink = 0;
	gpio_set_pin_level(leds[index].pin, 1);
}

//#define BUFFER_SIZE PEAK_USB_FD_EP_SIZE
#define PCAN_UFD_CMD_BUFFER_SIZE	512
#define PCAN_UFD_RX_BUFFER_SIZE		2048
#define PCAN_UFD_TX_BUFFER_SIZE		512

#define CMD_BUFFER_SIZE PCAN_UFD_LOSPD_PKT_SIZE
#define MSG_TX_BUFFER_SIZE PCAN_UFD_RX_BUFFER_SIZE
#define MSG_RX_BUFFER_SIZE 512 // can't be smaller otherwise we get reset of 512 byte message on next rx

struct usb_can {
	CFG_TUSB_MEM_ALIGN uint8_t msg_tx_buffers[2][MSG_TX_BUFFER_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t msg_rx_buffers[2][MSG_RX_BUFFER_SIZE];
	uint16_t msg_tx_offsets[2];
	uint8_t msg_tx_bank;
	uint8_t msg_rx_bank;
	uint8_t pipe;
};

static struct usb {
	CFG_TUSB_MEM_ALIGN uint8_t cmd_rx_buffer[CMD_BUFFER_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t cmd_tx_buffers[2][CMD_BUFFER_SIZE];
	uint16_t cmd_tx_offsets[2];
	struct usb_can can[2];
	uint8_t cmd_tx_bank;
	uint8_t port;
	bool mounted;
} usb;

static inline bool usb_cmd_bulk_in_ep_ready(void)
{
	return 0 == usb.cmd_tx_offsets[!usb.cmd_tx_bank];
}

static inline void usb_cmd_bulk_in_submit(void)
{
	TU_ASSERT(usb_cmd_bulk_in_ep_ready(), );
	TU_ASSERT(usb.cmd_tx_offsets[usb.cmd_tx_bank] > 0, );
	(void)dcd_edpt_xfer(usb.port, PEAK_USB_FD_EP_BULK_IN_CMD, usb.cmd_tx_buffers[usb.cmd_tx_bank], usb.cmd_tx_offsets[usb.cmd_tx_bank]);
	usb.cmd_tx_bank = !usb.cmd_tx_bank;
}

static inline bool usb_can_bulk_in_ep_ready(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.can));
	struct usb_can *can = &usb.can[index];
	return 0 == can->msg_tx_offsets[!can->msg_tx_bank];
}

static inline void usb_can_bulk_in_submit(uint8_t index)
{
	TU_ASSERT(usb_can_bulk_in_ep_ready(index), );
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.can), );
	struct usb_can *can = &usb.can[index];
	TU_ASSERT(can->msg_tx_offsets[can->msg_tx_bank] > 0, );
	(void)dcd_edpt_xfer(usb.port, 0x80 | can->pipe, can->msg_tx_buffers[can->msg_tx_bank], can->msg_tx_offsets[can->msg_tx_bank]);
	can->msg_tx_bank = !can->msg_tx_bank;
}

static inline void peak_seal_msg_buffer(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.can), );
	struct usb_can *can_usb = &usb.can[index];
	const uint8_t bytes_to_add = 4;
	if (likely(can_usb->msg_tx_offsets[can_usb->msg_tx_bank] + bytes_to_add <= MSG_TX_BUFFER_SIZE)) {
		memset(&can_usb->msg_tx_buffers[can_usb->msg_tx_bank][can_usb->msg_tx_offsets[can_usb->msg_tx_bank]], 0, bytes_to_add);
		can_usb->msg_tx_offsets[can_usb->msg_tx_bank] += bytes_to_add;
	}
}

static void peak_cmd_bulk_out(uint32_t xferred_bytes);
static void peak_can_bulk_out(uint8_t index, uint32_t xferred_bytes);
static void peak_can_bulk_in(uint8_t index);


static void peak_cmd_bulk_out(uint32_t xferred_bytes)
{
	uint8_t const *in_ptr = usb.cmd_rx_buffer;
	uint8_t const * const in_end = in_ptr + xferred_bytes;
	while (in_ptr + sizeof(struct pucan_command) <= in_end) {
		struct pucan_command const *cmd = (struct pucan_command const *)in_ptr;

		uint16_t opcode_channel = le16_to_cpu(cmd->opcode_channel);
		uint16_t opcode = opcode_channel & 0x3ff;

		if (PUCAN_CMD_END_OF_COLLECTION == opcode) {
			break;
		}

		in_ptr += sizeof(*cmd);

		uint8_t channel = (opcode_channel >> 12); // some bits unused?

		if (unlikely(channel >= TU_ARRAY_SIZE(cans.can))) {
			TU_LOG1("channel %u out of bounds\n", channel);
			continue;
		}

		// TU_LOG2("channel %u opcode %04x %04x %04x %04x\n",
		// 	channel, opcode, cmd->args[0], cmd->args[1], cmd->args[2]);

		switch (opcode) {
		case PUCAN_CMD_NOP:
			break;
		case PUCAN_CMD_RESET_MODE: {// the mode after device reset, => everything off
			TU_LOG2("CAN%u PUCAN_CMD_RESET_MODE\n", channel);

			struct can *can = &cans.can[channel];
			can->enabled = false;
			can_set_state1(can->m_can, can->interrupt_id, can->enabled);

			can->mode_flags = SC_MODE_FLAG_AUTO_RE | SC_MODE_FLAG_RX;

		} break;
		case PUCAN_CMD_NORMAL_MODE: {
			TU_LOG2("CAN%u PUCAN_CMD_NORMAL_MODE\n", channel);
			if (channel >= 2) {
				TU_LOG1("channel %u out of bounds\n", channel);
				continue;
			}

			struct can *can = &cans.can[channel];


			can->mode_flags |= SC_MODE_FLAG_EH | SC_MODE_FLAG_TX;
			can_configure(can);

			can->enabled = true;
			can_set_state1(can->m_can, can->interrupt_id, can->enabled);

			// memset(usb.msg_tx_buffer, 0, sizeof(usb.msg_tx_buffer));
			// struct pucan_rx_msg *rm = (struct pucan_rx_msg *)usb.msg_tx_buffer;
			// rm->size = cpu_to_le16(sizeof(*rm) + 1);
			// rm->type = cpu_to_le16(PUCAN_MSG_CAN_RX);
			// rm->flags = cpu_to_le16(0);
			// rm->can_id = cpu_to_le32(0x1);
			// rm->channel_dlc = 1u << 4;
			// rm->d[0] = 0xfe;
			// usb.msg_tx_offset = sizeof(*rm) + 4;
			// usbd_edpt_xfer(usb.port, PEAK_USB_FD_EP_BULK_IN_MSG, usb.msg_tx_buffer, usb.msg_tx_offset);

		} break;
		case PUCAN_CMD_LISTEN_ONLY_MODE: {
			TU_LOG2("CAN%u PUCAN_CMD_LISTEN_ONLY_MODE\n", channel);

			struct can *can = &cans.can[channel];

			can->mode_flags &= ~(SC_MODE_FLAG_EH | SC_MODE_FLAG_TX);
			can_configure(can);

			can->enabled = true;
			can_set_state1(can->m_can, can->interrupt_id, can->enabled);
		} break;
		case PUCAN_CMD_TIMING_SLOW: { // CAN & CAN-FD -> regular can bus
			TU_LOG2("CAN%u PUCAN_CMD_TIMING_SLOW\n", channel);


			struct can *can = &cans.can[channel];
			struct pucan_timing_slow *x = (struct pucan_timing_slow *)cmd;

			uint16_t brp = x->brp + 1;
			uint8_t sjw = (x->sjw_t & PUCAN_TSLOW_SJW_MASK) + 1;
			uint8_t seg1 = x->tseg1 + 1;
			uint8_t seg2 = x->tseg2 + 1;
			bool tsample = (x->sjw_t & 0x80) == 0x80;
			TU_LOG2("PEAK brp=%u sjw=%u seg1=%u seg2=%u tsample=%u\n", brp, sjw, seg1, seg2, tsample);
			// const uint16_t max_tq = (1 << PUCAN_TSLOW_TSGEG1_BITS) + (1 << PUCAN_TSLOW_TSGEG2_BITS) + (1 << PUCAN_TSLOW_SJW_BITS);
			uint16_t tqs = sjw + seg1 + seg2;
			uint32_t bitrate_bps = peak_clock_mhz[peakfd[channel].clock_mode] / (brp * tqs);
			TU_LOG2("bitrate %lu [bps]\n", bitrate_bps);
			uint8_t ratio01 = ((sjw + seg1) * 255) / tqs;
			TU_LOG2("sp %u [1/1000]\n", (ratio01 * 1000) / 255);


			const struct m_can_reg lim_min = {
				.brp = M_CAN_NMBT_BRP_MIN,
				.sjw = M_CAN_NMBT_SJW_MIN,
				.tseg1 = M_CAN_NMBT_TSEG1_MIN,
				.tseg2 = M_CAN_NMBT_TSEG2_MIN,
			};

			const struct m_can_reg lim_max = {
				.brp = M_CAN_NMBT_BRP_MAX,
				.sjw = M_CAN_NMBT_SJW_MAX,
				.tseg1 = M_CAN_NMBT_TSEG1_MAX,
				.tseg2 = M_CAN_NMBT_TSEG2_MAX,
			};
			struct m_can_reg result;

	// 		static void peak_to_m_can(
	// uint8_t peak_brp,
	// uint8_t peak_sjw,
	// uint8_t peak_tseg1,
	// uint8_t peak_tseg2,
	// uint32_t peak_clock_hz,
	// uint16_t max_tqs,
	// struct m_can_reg const *limits_min,
	// struct m_can_reg const *limits_max,
	// struct m_can_reg *result)


			peak_to_m_can(
				brp,
				sjw,
				seg1,
				seg2,
				peak_clock_mhz[peakfd[channel].clock_mode],
				M_CAN_NMBT_TQ_MAX,
				&lim_min,
				&lim_max,
				&result);

			TU_LOG2("M_CAN brp=%u sjw=%u seg1=%u seg2=%u\n", result.brp, result.sjw, result.tseg1, result.tseg2);
			uint8_t m_can_ratio01 = ((result.sjw + result.tseg1) * 255) / (result.sjw + result.tseg1 + result.tseg2);
			TU_LOG2("M_CAN sp %u [1/1000]\n", (m_can_ratio01 * 1000) / 255);


			// no CAN-FD
			can->mode_flags &= ~(SC_MODE_FLAG_FD | SC_MODE_FLAG_BRS);

			can->nmbt_brp = result.brp;
			can->nmbt_sjw = result.sjw;
			can->nmbt_tseg1 = result.tseg1;
			can->nmbt_tseg2 = result.tseg2;

			can->nm_bitrate_bps = bitrate_bps;


		} break;
		case PUCAN_CMD_TIMING_FAST: {
			TU_LOG2("PUCAN_CMD_TIMING_FAST\n");
			if (channel >= 2) {
				TU_LOG1("channel %u out of bounds\n", channel);
				continue;
			}

			struct can *can = &cans.can[channel];
			struct pucan_timing_fast *x = (struct pucan_timing_fast *)cmd;

			uint8_t brp = x->brp + 1;
			uint8_t sjw = x->sjw + 1;
			uint8_t seg1 = x->tseg1 + 1;
			uint8_t seg2 = x->tseg2 + 1;
			TU_LOG2("PEAK FD brp=%u sjw=%u seg1=%u seg2=%u\n", brp, sjw, seg1, seg2);
			uint16_t tqs = sjw + seg1 + seg2;
			uint32_t bitrate_bps = peak_clock_mhz[peakfd[channel].clock_mode] / (brp * tqs);
			TU_LOG2("bitrate %lu [bps]\n", bitrate_bps);
			uint8_t ratio01 = ((sjw + seg1) * 255) / tqs;
			TU_LOG2("sp %u [1/1000]\n", (ratio01 * 1000) / 255);

			const struct m_can_reg lim_min = {
				.brp = M_CAN_DTBT_BRP_MIN,
				.sjw = M_CAN_DTBT_SJW_MIN,
				.tseg1 = M_CAN_DTBT_TSEG1_MIN,
				.tseg2 = M_CAN_DTBT_TSEG2_MIN,
			};

			const struct m_can_reg lim_max = {
				.brp = M_CAN_DTBT_BRP_MAX,
				.sjw = M_CAN_DTBT_SJW_MAX,
				.tseg1 = M_CAN_DTBT_TSEG1_MAX,
				.tseg2 = M_CAN_DTBT_TSEG2_MAX,
			};
			struct m_can_reg result;
			// peak_to_m_can(sjw, ratio01, bitrate_bps, &lim_min, &lim_max, &result);


	// struct m_can_reg *result)


			peak_to_m_can(
				brp,
				sjw,
				seg1,
				seg2,
				peak_clock_mhz[peakfd[channel].clock_mode],
				M_CAN_DTBT_TQ_MAX,
				&lim_min,
				&lim_max,
				&result);


			TU_LOG2("M_CAN FD brp=%u sjw=%u seg1=%u seg2=%u\n", result.brp, result.sjw, result.tseg1, result.tseg2);
			uint8_t m_can_ratio01 = ((result.sjw + result.tseg1) * 255) / (result.sjw + result.tseg1 + result.tseg2);
			TU_LOG2("M_CAN sp %u [1/1000]\n", (m_can_ratio01 * 1000) / 255);

			// CAN FD
			can->mode_flags |= SC_MODE_FLAG_FD | SC_MODE_FLAG_BRS;

			can->dtbt_brp = result.brp;
			can->dtbt_sjw = result.sjw;
			can->dtbt_tseg1 = result.tseg1;
			can->dtbt_tseg2 = result.tseg2;

		} break;
		case PUCAN_CMD_SET_STD_FILTER:
			break;
		// case PUCAN_CMD_RESERVED2:
		// 	break;
		case PUCAN_CMD_FILTER_STD:
		break;
		case PUCAN_CMD_TX_ABORT:
			break;
		case PUCAN_CMD_WR_ERR_CNT: {
			TU_LOG2("PUCAN_CMD_WR_ERR_CNT\n");
			if (channel >= 2) {
				TU_LOG1("channel %u out of bounds\n", channel);
				continue;
			}

			struct pucan_wr_err_cnt *x = (struct pucan_wr_err_cnt *)cmd;
			struct can *can = &cans.can[channel];
			uint16_t flags = le16_to_cpu(x->sel_mask);
			if (flags & PUCAN_WRERRCNT_RE) {
				can->rx_lost = x->rx_counter;
			}
			if (flags & PUCAN_WRERRCNT_TE) {
				can->tx_dropped = x->tx_counter;
			}
		} break;
		case PUCAN_CMD_CLR_DIS_OPTION:
		case PUCAN_CMD_SET_EN_OPTION: {
			TU_LOG2("%s\n", PUCAN_CMD_CLR_DIS_OPTION == opcode ? "PUCAN_CMD_CLR_DIS_OPTION" : "PUCAN_CMD_SET_EN_OPTION");
			if (channel >= 2) {
				TU_LOG1("channel %u out of bounds\n", channel);
				continue;
			}
			struct pucan_options *puo = (struct pucan_options *)cmd;
			uint16_t options = le16_to_cpu(puo->options);
// #define PUCAN_OPTION_ERROR		0x0001
// #define PUCAN_OPTION_BUSLOAD		0x0002
// #define PUCAN_OPTION_CANDFDISO		0x0004
		} break;
		case PUCAN_CMD_RX_BARRIER:
			break;
		case PCAN_UFD_CMD_LED_SET: {
			TU_LOG2("PCAN_UFD_CMD_LED_SET\n");
			if (channel >= 2) {
				TU_LOG1("channel %u out of bounds\n", channel);
				continue;
			}
		} break;
		case PCAN_UFD_CMD_CLK_SET: {
			TU_LOG2("PCAN_UFD_CMD_CLK_SET\n");
			if (channel >= 2) {
				TU_LOG1("channel %u out of bounds\n", channel);
				continue;
			}

			struct pcan_ufd_clock *x = (struct pcan_ufd_clock *)cmd;

			if (x->mode > PCAN_UFD_CLK_20MHZ) {
				TU_LOG1("clock mode %u out of bounds\n", x->mode);
				continue;
			}

			peakfd[channel].clock_mode = x->mode;
		} break;

		default:
			TU_LOG2("opcode %04x not implemented\n", opcode);
			break;
		}
	}

	if (usb.cmd_tx_offsets[usb.cmd_tx_bank] > 0 && usb_cmd_bulk_in_ep_ready()) {
		// TU_LOG2("usb tx %u bytes\n", usb.cmd_tx_offsets[usb.cmd_tx_bank]);
		usb_cmd_bulk_in_submit();
	}

	// start new transaction
	(void)usbd_edpt_xfer(usb.port, 0x01, usb.cmd_rx_buffer, TU_ARRAY_SIZE(usb.cmd_rx_buffer));
}


static void peak_can_bulk_out(uint8_t index, uint32_t xferred_bytes)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.can), );

	struct can *can = &cans.can[index];
	struct usb_can *usb_can = &usb.can[index];
	led_burst(can->led, 8);


	const uint8_t rx_bank = usb_can->msg_rx_bank;
	TU_LOG2("CAN%u rx bank %u\n", index, rx_bank);
	uint8_t const * const in_beg = usb_can->msg_rx_buffers[usb_can->msg_rx_bank];
	uint8_t const *in_ptr = in_beg;
	uint8_t const * const in_end = in_ptr + xferred_bytes;

	// start new transfer right away
	usb_can->msg_rx_bank = !usb_can->msg_rx_bank;
	(void)dcd_edpt_xfer(usb.port, usb_can->pipe, usb_can->msg_rx_buffers[usb_can->msg_rx_bank], MSG_RX_BUFFER_SIZE);

	// now process messages
	while (in_ptr + sizeof(struct pucan_msg) <= in_end) {
		struct pucan_msg const *msg = (struct pucan_msg const *)in_ptr;

		if (!msg->type || !msg->size) {
			break;
		}

		uint16_t len = le16_to_cpu(msg->size);
		uint16_t type = le16_to_cpu(msg->type);

		TU_LOG2("CAN%u offset %u\n", index, (size_t)(in_ptr - in_beg));

		in_ptr += len;

		switch (type) {
		case PUCAN_MSG_CAN_TX: {
			struct pucan_tx_msg const *x = (struct pucan_tx_msg const *)msg;
			uint8_t channel = x->channel_dlc & 0xf;
			uint8_t dlc = (x->channel_dlc >> 4) & 0xf;
			uint16_t flags = le16_to_cpu(x->flags);
			uint32_t id = le32_to_cpu(x->can_id);
			const uint8_t can_frame_len = dlc_to_len(dlc);

			TU_LOG2("CAN%u dlc=%u len=%u id=%lx\n", index, dlc, can_frame_len, id);

			if (unlikely(channel != index)) {
				TU_LOG1("channel %u for CAN%u\n", channel, index);
				continue;
			}

			// if (unlikely(channel >= 2)) {
			// 	TU_LOG1("channel %u out of bounds\n", channel);
			// 	continue;
			// }


			if (!(flags & PUCAN_MSG_RTR)) {
				if (unlikely(len < sizeof(struct pucan_tx_msg) + can_frame_len)) {
					TU_LOG1("ERROR: PUCAN_MSG_CAN_TX msg too short\n");
					continue;
				}
			}



			if (can->m_can->TXFQS.bit.TFQF) {
				++can->tx_dropped;
// 					uint8_t *ptr;
// 					uint8_t *end;
// send_txr:
// 					ptr = usb.tx_buffers[usb.tx_bank] + usb.tx_offsets[usb.tx_bank];
// 					end = ptr + BUFFER_SIZE;

// 					uint8_t bytes = SC_HEADER_LEN + sizeof(struct sc_msg_can_txr);
// 					if (end - ptr >= bytes) {
// 						usb.tx_offsets[usb.tx_bank] += bytes;

// 						struct sc_msg *rep = (struct sc_msg *)ptr;
// 						rep->id = SC_MSG_CAN_TXR;
// 						rep->len = bytes;
// 						rep->u.txr.channel = tx->channel;
// 						memcpy(&rep->u.txr.track_id, &tx->track_id, sizeof(rep->u.txr.track_id));
// 						uint32_t ts = can->ts_high | can->m_can->TSCV.bit.TSC;
// 						memcpy(&rep->u.txr.timestamp, &ts, sizeof(rep->u.txr.timestamp));
// 						rep->u.txr.flags = SC_CAN_FLAG_DRP;
// 					} else {
// 						if (usb_bulk_in_ep_ready()) {
// 							usb_bulk_in_submit();
// 							// usb.has_in_token = false;
// 							// usbd_edpt_xfer(rhport, SC_M1_EP_BULK_IN, usb.tx_buffers[usb.tx_bank], usb.tx_offsets[usb.tx_bank]);
// 							// usb.tx_bank = !usb.tx_bank;
// 							goto send_txr;
// 						} else {
// 							TU_LOG1("ch %u: desync\n", tx->channel);
// 							can->desync = true;
// 						}
// 					}
			} else {
				uint8_t put_index = can->m_can->TXFQS.bit.TFQPI;
				// uint8_t marker_index = can->tx_fifo[index].T1.bit.MM;
				// can->message_marker_map[marker_index] = tid; // save message marker

				CAN_TXBE_0_Type t0;
				t0.reg = (((flags & PUCAN_MSG_ERROR_STATE_IND) == PUCAN_MSG_ERROR_STATE_IND) << CAN_TXBE_0_ESI_Pos)
					| (((flags & PUCAN_MSG_RTR) == PUCAN_MSG_RTR) << CAN_TXBE_0_RTR_Pos)
					| (((flags & PUCAN_MSG_EXT_ID) == PUCAN_MSG_EXT_ID) << CAN_TXBE_0_XTD_Pos)
					;


				if (flags & PUCAN_MSG_EXT_ID) {
					t0.reg |= CAN_TXBE_0_ID(id);
				} else {
					t0.reg |= CAN_TXBE_0_ID(id << 18);
				}

				can->tx_fifo[put_index].T0 = t0;
				can->tx_fifo[put_index].T1.bit.DLC = dlc;
				can->tx_fifo[put_index].T1.bit.FDF = (flags & PUCAN_MSG_EXT_DATA_LEN) == PUCAN_MSG_EXT_DATA_LEN;
				can->tx_fifo[put_index].T1.bit.BRS = (flags & PUCAN_MSG_BITRATE_SWITCH) == PUCAN_MSG_BITRATE_SWITCH;

				if (!(flags & PUCAN_MSG_RTR)) {
					if (can_frame_len) {
						memcpy(can->tx_fifo[put_index].data, x->d, can_frame_len);
					}
				}

				can->m_can->TXBAR.reg = 1UL << put_index;
			}


		} break;
		default:
			TU_LOG2("!CAN%u rx bank=%u unhandled msg type=%04x len=%04x\n", index, rx_bank, type, len);
			break;
		}
	}
}

static void peak_can_bulk_in(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.can), );

	struct usb_can *usb_can = &usb.can[index];

	// TU_ASSERT(!usbd_edpt_busy(rhport, ep_addr), false);
	// TU_LOG2("< msg1 IN token\n");
	// mark previous bank as free
	// memset(usb_can->msg_tx_buffers[!usb_can->msg_tx_bank], 0, MSG_TX_BUFFER_SIZE);
	usb_can->msg_tx_offsets[!usb_can->msg_tx_bank] = 0;

	if (usb_can->msg_tx_offsets[usb_can->msg_tx_bank]) {
		peak_seal_msg_buffer(index);
		// send off current bank data
		// TU_LOG2("usb msg tx %u bytes\n", usb.msg_tx_offsets[usb.msg_tx_bank]);

		usb_can_bulk_in_submit(index);
	}
}


int main(void)
{
	board_init();
	led_init();

	tusb_init();

	can_init_pins();
	can_init_clock();
	can_init_module();

	for (uint8_t i = 0; i < TU_ARRAY_SIZE(peakfd); ++i) {
		peakfd[i].clock_mode = PCAN_UFD_CLK_DEF;
	}

	cans.can[0].led = LED_GREEN1;
	cans.can[1].led = LED_GREEN2;
	cans.can[0].task = xTaskCreateStatic(&can_task, "can0", TU_ARRAY_SIZE(can_task_stack[0]), (void*)0, configMAX_PRIORITIES-1, can_task_stack[0], &can_task_mem[0]);
	cans.can[1].task = xTaskCreateStatic(&can_task, "can1", TU_ARRAY_SIZE(can_task_stack[1]), (void*)1, configMAX_PRIORITIES-1, can_task_stack[1], &can_task_mem[1]);

	(void) xTaskCreateStatic(&tusb_device_task, "tusb", TU_ARRAY_SIZE(usb_device_stack), NULL, configMAX_PRIORITIES-1, usb_device_stack, &usb_device_stack_mem);
	(void) xTaskCreateStatic(&led_task, "led", TU_ARRAY_SIZE(led_task_stack), NULL, configMAX_PRIORITIES-1, led_task_stack, &led_task_mem);


	led_blink(0, 2000);
	led_set(LED_RED1, 1);

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
		tud_task();
		// TU_LOG2("usb\n");
	}
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
	TU_LOG2("mounted\n");
	led_blink(0, 250);
	usb.mounted = true;

}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	TU_LOG2("unmounted\n");
	led_blink(0, 1000);
	usb.mounted = false;

	can_disengage();

	// disable cans until explicitly enabled
	for (size_t i = 0; i < TU_ARRAY_SIZE(cans.can); ++i) {
		struct can *can = &cans.can[i];
		can->enabled = false;
	}
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	TU_LOG2("suspend\n");
	usb.mounted = false;
	led_blink(0, 500);

	can_disengage();
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	TU_LOG2("resume\n");
	usb.mounted = true;
	led_blink(0, 250);

	can_engage();
}


//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+
void tud_custom_init_cb(void)
{
	TU_LOG2("init\n");
	memset(&usb, 0, sizeof(usb));
}

void tud_custom_reset_cb(uint8_t rhport)
{
	(void) rhport;
	TU_LOG2("port %u reset\n", rhport);
	memset(&usb, 0, sizeof(usb));
	usb.can[0].pipe = 0x2;
	usb.can[1].pipe = 0x3;
	usb.port = rhport;
}

bool tud_custom_open_cb(uint8_t rhport, tusb_desc_interface_t const * desc_intf, uint16_t* p_length)
{
	(void)desc_intf;
	TU_LOG2("port %u open\n", rhport);

	if (rhport != usb.port) {
		return false;
	}

	uint8_t const *ptr = (void const *)desc_intf;
	//ptr += 2*9+2*7;
	ptr += 9;

	for (uint8_t i = 0; i < 6; ++i) {
		TU_ASSERT(dcd_edpt_open(rhport, (tusb_desc_endpoint_t const *)(ptr + i * 7)));
	}
	// TU_ASSERT(dcd_edpt_open(rhport, (tusb_desc_endpoint_t const *)(ptr + )));
	// TU_ASSERT(dcd_edpt_open(rhport, (tusb_desc_endpoint_t const *)(ptr + 2*7)));

	TU_ASSERT(usbd_edpt_xfer(rhport, PEAK_USB_FD_EP_BULK_OUT_CMD, usb.cmd_rx_buffer, TU_ARRAY_SIZE(usb.cmd_rx_buffer)));
	for (size_t i = 0; i < TU_ARRAY_SIZE(usb.can); ++i) {
		struct usb_can *usb_can = &usb.can[i];
		TU_ASSERT(usbd_edpt_xfer(rhport, usb_can->pipe, usb_can->msg_rx_buffers[usb_can->msg_rx_bank], MSG_RX_BUFFER_SIZE));
	}

	//*p_length = 2*9+6*7;
	*p_length = 9+6*7;

	return true;
}

static const char* recipient_str(tusb_request_recipient_t r)
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

static const char* type_str(tusb_request_type_t value)
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

static const char* dir_str(tusb_dir_t value)
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

static uint8_t blak[16];

bool tud_custom_control_request_cb(uint8_t rhport, tusb_control_request_t const * request)
{
	(void)request;
	TU_LOG2("port %u req\n", rhport);

	if (rhport != usb.port) {
		return false;
	}

	// uint8_t const *data_ptr = (uint8_t const *)(request+1);
	// uint16_t data_len = request->wLength - sizeof(*request);



	TU_LOG2("req type 0x%02x (reci %s type %s dir %s) req 0x%02x, value 0x%04x index 0x%04x reqlen %u\n",
		request->bmRequestType,
		recipient_str(request->bmRequestType_bit.recipient),
		type_str(request->bmRequestType_bit.type),
		dir_str(request->bmRequestType_bit.direction),
		request->bRequest, request->wValue, request->wIndex,
		request->wLength);


	switch (request->bmRequestType) {

	case 0xc3:
		switch (request->bRequest) {
		case 0: // request info
			switch (request->wValue) {
			case 1: {
				// index == 0;
				CFG_TUSB_MEM_ALIGN
				static const struct pcan_ufd_fw_info rep = {
					//.size_of = sizeof(struct pcan_ufd_fw_info),
					.size_of = 0x24,
					.type = 2,
					.hw_type = 1,
					.bl_version = {2, 1, 0},
					.hw_version = 1,
					.fw_version = {3, 2, 0},
					.dev_id = {0, ~0},
					.flags = 2
				};
				TU_LOG2_MEM(&rep, sizeof(rep), 2);
				return tud_control_xfer(rhport, request, (void*)&rep, sizeof(rep));
			} break;


				// struct __packed pcan_ufd_fw_info {
			// CONTROL response data:
// 					2400 // sizeof
					// 0200 type
					//01 hw type
					//02 bl[0]
					//01 bl[1]
					//00 bl[2]
					//01 hw version
					//03 fw[0]
					//03 fw[1]
					//00 fw[2]
					// 00000000 deviceid[0]
					// ffffffff deviceid[1]
					// 00000000 ser no
					// 02000000 flags
				break;
			}
		}
		break;
	case 0x43:
		switch (request->bRequest) {
		case 2: // request function
			switch (request->wValue) {
			case 5: { // driver load

			return tud_control_xfer(rhport, request, blak, sizeof(blak));
			// usbd_control_xfer_cb(event.rhport, ep_addr, event.xfer_complete.result, event.xfer_complete.len);
			// 	if (usbd_control_xfer_cb(rhport, 0x00, )
			// 	// if (data_len == 8) {
			// 	// 	TU_LOG2("host driver loaded %02x\n", data_ptr[1]);
			// 	// 	return tud_control_status(rhport, request);
			// 	// }
			// 	// index == 0;
			// 	return true;
				// break;
			} break;
			}
		}
	}

	return false;
}
bool tud_custom_control_complete_cb(uint8_t rhport, tusb_control_request_t const * request)
{
	(void)request;
	TU_LOG2("port %u req com\n", rhport);

	if (rhport != usb.port) {
		return false;
	}

	switch (request->bmRequestType) {


	case 0x43:
		switch (request->bRequest) {
		case 2: // request function
			switch (request->wValue) {
			case 5: { // driver load
			TU_LOG2_MEM(blak, sizeof(blak), 2);

			} break;
			}
		}
	}

	return true;
}



bool tud_custom_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
	USB_TRAFFIC_DO_LED;
	(void)event; // always success

	if (rhport != usb.port) {
		return false;
	}

	switch (ep_addr) {
	case PEAK_USB_FD_EP_BULK_OUT_CMD: {
		peak_cmd_bulk_out(xferred_bytes);
	} break;
	case PEAK_USB_FD_EP_BULK_IN_CMD:
		TU_ASSERT(!usbd_edpt_busy(rhport, ep_addr), false);
		TU_LOG2("< cmd IN token\n");
		// mark previous bank as free
		usb.cmd_tx_offsets[!usb.cmd_tx_bank] = 0;

		if (usb.cmd_tx_offsets[usb.cmd_tx_bank]) {
			// send off current bank data
			// TU_LOG2("usb tx %u bytes\n", usb.cmd_tx_offsets[usb.cmd_tx_bank]);

			usb_cmd_bulk_in_submit();
		}
		break;
	case PEAK_USB_FD_EP_BULK_OUT_MSG_CH0: {
		peak_can_bulk_out(0, xferred_bytes);
	} break;
	case PEAK_USB_FD_EP_BULK_OUT_MSG_CH1: {
		peak_can_bulk_out(1, xferred_bytes);
	} break;
	case PEAK_USB_FD_EP_BULK_IN_MSG_CH0: {
		peak_can_bulk_in(0);
	} break;
	case PEAK_USB_FD_EP_BULK_IN_MSG_CH1: {
		peak_can_bulk_in(1);
	} break;
	default:
		TU_LOG2("port %u ep %02x event %d bytes %u\n", rhport, ep_addr, event, (unsigned)xferred_bytes);
		return false;
	}

	return true;
}

bool tud_vendor_control_request_cb(uint8_t rhport, tusb_control_request_t const * request)
{
	return tud_custom_control_request_cb(rhport, request);
}

bool tud_vendor_control_complete_cb(uint8_t rhport, tusb_control_request_t const * request)
{
	return tud_custom_control_complete_cb(rhport, request);
}


//--------------------------------------------------------------------+
// LED TASK
//--------------------------------------------------------------------+
static void led_init(void)
{
	PORT->Group[1].DIRSET.reg = PORT_PB14; /* Debug-LED */
	PORT->Group[1].DIRSET.reg = PORT_PB15; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA12; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA13; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA14; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA15; /* Debug-LED */
}



static void led_task(void *param)
{
	(void) param;

	uint16_t left[TU_ARRAY_SIZE(leds)];
	memset(&left, 0, sizeof(left));
	const uint8_t TICK_MS = 8;

	while (42) {
		for (uint8_t i = 0; i < TU_ARRAY_SIZE(leds); ++i) {
			uint16_t t = leds[i].time_ms;
			if (t) {
				if (leds[i].blink) {
					if (0 == left[i]) {
						// TU_LOG2("led %s (%u) toggle\n", leds[i].name, i);
						gpio_toggle_pin_level(leds[i].pin);
						left[i] = t / TICK_MS;
					} else {
						--left[i];
					}
				} else {
					// burst
					t -= tu_min16(t, TICK_MS);
					leds[i].time_ms = t;
					if (!t) {
						gpio_set_pin_level(leds[i].pin, 0);
					}
				}
			}
		}

		vTaskDelay(pdMS_TO_TICKS(TICK_MS));
	}
}

//--------------------------------------------------------------------+
// CAN TASK
//--------------------------------------------------------------------+
static void can_task(void *param)
{
	const uint8_t index = (uint8_t)(uintptr_t)param;
	TU_ASSERT(index < TU_ARRAY_SIZE(cans.can), );
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.can), );

	TU_LOG2("CAN%u task start\n", index);

	struct can *can = &cans.can[index];
	struct usb_can *usb_can = &usb.can[index];

	uint8_t last_status = 0;

	while (42) {
		(void)ulTaskNotifyTake(pdFALSE, ~0);

		// TU_LOG2("can task loop\n");
		if (unlikely(!usb.mounted)) {
			continue;
		}

		if (unlikely(!can->enabled)) {
			continue;
		}

		led_burst(can->led, 8);




		for (bool done = false; !done; ) {
			done = true;
			uint8_t *ptr;
			uint8_t *end;
start:
			ptr = usb_can->msg_tx_buffers[usb_can->msg_tx_bank] + usb_can->msg_tx_offsets[usb_can->msg_tx_bank];
			end = ptr + MSG_TX_BUFFER_SIZE;

			if (!usb_can->msg_tx_offsets[usb_can->msg_tx_bank]) {

				// place status messages
				done = false;

				uint32_t ts = can->ts_high | can->m_can->TSCV.bit.TSC;
				uint32_t us = can_time_to_us(can, ts);

				struct pucan_error_msg *error_msg = (struct pucan_error_msg *)ptr;
				error_msg->size = cpu_to_le16(sizeof(*error_msg));
				error_msg->type = cpu_to_le16(PUCAN_MSG_ERROR);
				error_msg->ts_low = cpu_to_le32(us);
				error_msg->ts_high = 0;
				error_msg->channel_type_d = index;
				error_msg->code_g = 0;
				error_msg->tx_err_cnt = can->m_can->ECR.bit.TEC;
				error_msg->rx_err_cnt = can->m_can->ECR.bit.REC;

				ptr += sizeof(*error_msg);
			}

			// check if status changed
			uint8_t curr_status = can->status_flags;

			if (curr_status != last_status) {
				bool had_rx_lost = (last_status & SC_STATUS_FLAG_RX_FULL) == SC_STATUS_FLAG_RX_FULL;
				bool has_rx_lost = (curr_status & SC_STATUS_FLAG_RX_FULL) == SC_STATUS_FLAG_RX_FULL;
				if (has_rx_lost && !had_rx_lost) {
					if (ptr + sizeof(struct pcan_ufd_ovr_msg) <= end) {
						done = false;

						uint32_t ts = can->ts_high | can->m_can->TSCV.bit.TSC;
						uint32_t us = can_time_to_us(can, ts);
						struct pcan_ufd_ovr_msg *ovr_msg = (struct pcan_ufd_ovr_msg *)ptr;
						ovr_msg->size = cpu_to_le16(sizeof(*ovr_msg));
						ovr_msg->type = cpu_to_le16(PCAN_UFD_MSG_OVERRUN);
						ovr_msg->ts_low = cpu_to_le32(us);
						ovr_msg->ts_high = 0;
						ovr_msg->channel = index;

						ptr += sizeof(*ovr_msg);
					} else {
						if (usb_can_bulk_in_ep_ready(index)) {
							usb_can->msg_tx_offsets[usb_can->msg_tx_bank] = ptr - usb_can->msg_tx_buffers[usb_can->msg_tx_bank];
							peak_seal_msg_buffer(index);
							usb_can_bulk_in_submit(index);
							goto start;
						} else {
							TU_LOG1("!CAN%u: no space for rx ovr msg\n", index);
						}
					}
				}

				uint8_t other_last_status = last_status & ~SC_STATUS_FLAG_RX_FULL;
				uint8_t other_curr_status = curr_status & ~SC_STATUS_FLAG_RX_FULL;

				if (other_last_status != other_curr_status) {
					if (ptr + sizeof(struct pucan_status_msg) <= end) {
						done = false;

						uint32_t ts = can->ts_high | can->m_can->TSCV.bit.TSC;
						uint32_t us = can_time_to_us(can, ts);
						struct pucan_status_msg *status_msg = (struct pucan_status_msg *)ptr;
						status_msg->size = cpu_to_le16(sizeof(*status_msg));
						status_msg->type = cpu_to_le16(PUCAN_MSG_STATUS);
						status_msg->ts_low = cpu_to_le32(us);
						status_msg->ts_high = 0;
						status_msg->channel_p_w_b = index;

						if (other_curr_status & SC_STATUS_FLAG_BUS_OFF) {
							status_msg->channel_p_w_b |= PUCAN_BUS_BUSOFF;
						}
						if (other_curr_status & SC_STATUS_FLAG_ERROR_PASSIVE) {
							status_msg->channel_p_w_b |= PUCAN_BUS_PASSIVE;
						}
						if (other_curr_status & SC_STATUS_FLAG_ERROR_WARNING) {
							status_msg->channel_p_w_b |= PUCAN_BUS_WARNING;
						}

						ptr += sizeof(*status_msg);
					} else {
						if (usb_can_bulk_in_ep_ready(index)) {
							usb_can->msg_tx_offsets[usb_can->msg_tx_bank] = ptr - usb_can->msg_tx_buffers[usb_can->msg_tx_bank];
							peak_seal_msg_buffer(index);
							usb_can_bulk_in_submit(index);
							goto start;
						} else {
							TU_LOG1("!CAN%u: no space for status msg\n", index);
						}
					}
				}

				last_status = curr_status;
			}

			if (m_can_rx0_msg_fifo_avail(can->m_can)) {
				done = false;

				uint8_t get_index = can->m_can->RXF0S.bit.F0GI;
				uint8_t bytes = sizeof(struct pucan_rx_msg);
				CAN_RXF0E_0_Type r0 = can->rx_fifo[get_index].R0;
				CAN_RXF0E_1_Type r1 = can->rx_fifo[get_index].R1;
				uint8_t can_frame_len = dlc_to_len(r1.bit.DLC);
				if (!r0.bit.RTR) {
					bytes += can_frame_len;
				}

				// ensure data is 4 byte aligned
				if (bytes & 3) {
					bytes += 4 - (bytes & 3);
				}

				// TU_LOG2("CAN%u msg @ i%u\n", i, get_index);



				if (end - ptr >= bytes) {
					struct pucan_rx_msg *msg = (struct pucan_rx_msg *)ptr;
					ptr += bytes;

					uint16_t flags = 0;
					msg->size = cpu_to_le16(bytes);
					msg->type = cpu_to_le16(PUCAN_MSG_CAN_RX);
					msg->tag_low = 0;
					msg->tag_high = 0;
					msg->channel_dlc = PUCAN_MSG_CHANNEL_DLC(index, r1.bit.DLC);
					msg->client = 0;

					uint32_t id = r0.bit.ID;
					if (r0.bit.XTD) {
						flags |= PUCAN_MSG_EXT_ID;
					} else {
						id >>= 18;
					}
					msg->can_id = cpu_to_le32(id);
					msg->ts_high = 0;
					uint32_t ts = can->ts_high | r1.bit.RXTS;
					uint32_t us = can_time_to_us(can, ts);
					msg->ts_low = cpu_to_le32(us);

					if (r1.bit.FDF) {
						flags |= PUCAN_MSG_EXT_DATA_LEN;
					}
					if (r1.bit.BRS) {
						flags |= PUCAN_MSG_BITRATE_SWITCH;
					}
					if (r0.bit.ESI) {
						flags |= PUCAN_MSG_ERROR_STATE_IND;
					}
					if (r0.bit.RTR) {
						flags |= PUCAN_MSG_RTR;
					} else {
						memcpy(msg->d, can->rx_fifo[get_index].data, can_frame_len);
					}

					msg->flags = cpu_to_le16(flags);
				} else {
					if (usb_can_bulk_in_ep_ready(index)) {
						usb_can->msg_tx_offsets[usb_can->msg_tx_bank] = ptr - usb_can->msg_tx_buffers[usb_can->msg_tx_bank];
						peak_seal_msg_buffer(index);
						usb_can_bulk_in_submit(index);
						goto start;
					} else {
						++can->rx_lost;
					}
				}

				can->m_can->RXF0A.reg = CAN_RXF0A_F0AI(get_index);
			}

			if (m_can_tx_event_fifo_avail(can->m_can)) {
				done = false;
				uint8_t get_index = can->m_can->TXEFS.bit.EFGI;
				// uint8_t bytes = SC_HEADER_LEN + sizeof(struct sc_msg_can_txr);
				// CAN_TXEFE_0_Type t0 = can->tx_event_fifo[get_index].T0;
				// CAN_TXEFE_1_Type t1 = can->tx_event_fifo[get_index].T1;

				// if (end - ptr >= bytes) {
				// 	struct sc_msg *msg = (struct sc_msg *)ptr;
				// 	ptr += bytes;

				// 	uint8_t marker_index = t1.bit.MM;

				// 	msg->id = SC_MSG_CAN_TXR;
				// 	msg->len = bytes;
				// 	msg->u.txr.channel = i;
				// 	memcpy(&msg->u.txr.track_id, &can->message_marker_map[marker_index], sizeof(msg->u.txr.track_id));
				// 	uint32_t ts = can->ts_high | t1.bit.TXTS;
				// 	memcpy(&msg->u.txr.timestamp, &ts, sizeof(msg->u.txr.timestamp));
				// 	msg->u.txr.flags = 0;
				// 	if (t0.bit.ESI) {
				// 		msg->u.txr.flags |= SC_CAN_FLAG_ESI;
				// 	}

				// 	if (t1.bit.FDF) {
				// 		msg->u.txr.flags |= SC_CAN_FLAG_FDF;
				// 	}

				// 	if (t1.bit.BRS) {
				// 		msg->u.txr.flags |= SC_CAN_FLAG_BRS;
				// 	}
				// } else {
				// 	if (usb_msg_bulk_in_ep_ready()) {
				// 		usb.msg_tx_offsets[usb.msg_tx_bank] = ptr - usb.msg_tx_buffers[usb.msg_tx_bank];
				// 		peak_seal_msg_buffer();
				// 		usb_msg_bulk_in_submit();
				// 	// if (usb.has_in_token) {
				// 	// 	usb.has_in_token = false;
				// 	// 	usb.tx_offsets[usb.tx_bank] = ptr - usb.tx_buffers[usb.tx_bank];
				// 	// 	usbd_edpt_xfer(usb.port, SC_M1_EP_BULK_IN, usb.tx_buffers[usb.tx_bank], usb.tx_offsets[usb.tx_bank]);
				// 	// 	usb.tx_bank = !usb.tx_bank;
				// 		goto start;
				// 	} else {
				// 		can->desync = true;
				// 	}
				// }

				can->m_can->TXEFA.reg = CAN_TXEFA_EFAI(get_index);
			}

			usb_can->msg_tx_offsets[usb_can->msg_tx_bank] = ptr - usb_can->msg_tx_buffers[usb_can->msg_tx_bank];
		}

		if (usb_can->msg_tx_offsets[usb_can->msg_tx_bank] > 0 && usb_can_bulk_in_ep_ready(index)) {
			peak_seal_msg_buffer(index);

			// TU_LOG2("usb tx %u bytes\n", usb.msg_tx_offsets[usb.msg_tx_bank]);
			usb_can_bulk_in_submit(index);
		}
	}
}
