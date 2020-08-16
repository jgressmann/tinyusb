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



struct can {
	CFG_TUSB_MEM_ALIGN struct can_tx_fifo_element tx_fifo[CAN_TX_FIFO_SIZE];
	CFG_TUSB_MEM_ALIGN struct can_tx_event_fifo_element tx_event_fifo[CAN_TX_FIFO_SIZE];
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
	uint8_t mode;
	uint16_t features;
	uint16_t rx_lost;
	uint16_t tx_dropped;
	volatile uint8_t status_flags;
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
		PORT_WRCONFIG_PMUX(8) |         // I, CAN0, DS60001507E-page 32, 910
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

static void can_configure(struct can *c)
{
	Can *can = c->m_can;

	m_can_conf_begin(can);

	can->CCCR.bit.TXP = 1;  // Enable Transmit Pause
	can->CCCR.bit.EFBI = 1; // Enable Edge Filtering
	can->CCCR.bit.DAR = 0; // Automatic retransmission enabled

	uint8_t mode = SC_MODE_MASK & c->mode;
	switch (mode) {
	case SC_MODE_NORMAL:
		can->CCCR.bit.MON = 0;
		can->CCCR.bit.TEST = 0;
		can->CCCR.bit.ASM = 0;
		break;
	case SC_MODE_MONITORING:
		can->CCCR.bit.MON = 1;
		can->CCCR.bit.TEST = 0;
		can->CCCR.bit.ASM = 0;
		break;
	case SC_MODE_RESTRICTED:
		can->CCCR.bit.MON = 0;
		can->CCCR.bit.TEST = 0;
		can->CCCR.bit.ASM = 1;
		break;
	case SC_MODE_EXT_LOOPBACK:
		can->CCCR.bit.MON = 0;
		can->CCCR.bit.TEST = 1;
		can->CCCR.bit.ASM = 0;
		can->TEST.bit.LBCK = 1;
		break;
	default:
		LOG("mode %u not implemented\n", mode);
		break;
	}

	can->CCCR.bit.FDOE = (c->mode & SC_MODE_FLAG_FDF) == SC_MODE_FLAG_FDF;
	can->CCCR.bit.BRSE = 1;
	can->CCCR.bit.PXHD = (c->features & SC_FEATURE_FLAG_EHD) == SC_FEATURE_FLAG_EHD;

	LOG("MON=%u TEST=%u ASM=%u PXHD=%u FDOE=%u BRSE=%u txr=%u\n",
		can->CCCR.bit.MON, can->CCCR.bit.TEST, can->CCCR.bit.ASM,
		can->CCCR.bit.PXHD, can->CCCR.bit.FDOE, can->CCCR.bit.BRSE,
		(c->features & SC_FEATURE_FLAG_TXR) == SC_FEATURE_FLAG_TXR
	);

	LOG("nominal brp=%u sjw=%u tseg1=%u tseg2=%u bitrate=%lu sp=%u/1000\n",
		c->nmbt_brp, c->nmbt_sjw, c->nmbt_tseg1, c->nmbt_tseg2,
		CAN_CLK_HZ / ((uint32_t)c->nmbt_brp * (1 + c->nmbt_tseg1 + c->nmbt_tseg2)),
		((c->nmbt_tseg1) * 1000) / (c->nmbt_tseg1 + c->nmbt_tseg2)
	);

	LOG("data brp=%u sjw=%u tseg1=%u tseg2=%u bitrate=%lu sp=%u/1000\n",
		c->dtbt_brp, c->dtbt_sjw, c->dtbt_tseg1, c->dtbt_tseg2,
		CAN_CLK_HZ / ((uint32_t)c->dtbt_brp * (1 + c->dtbt_tseg1 + c->dtbt_tseg2)),
		((c->dtbt_tseg1) * 1000) / (c->dtbt_tseg1 + c->dtbt_tseg2)
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

	// tx fifo
	can->TXBC.reg = CAN_TXBC_TBSA((uint32_t) c->tx_fifo) | CAN_TXBC_TFQS(CAN_TX_FIFO_SIZE);
	//	REG_CAN0_TXBC |= CAN_TXBC_TFQM; // reset default
	can->TXESC.reg = CAN_TXESC_TBDS_DATA64;

	//can->TXEFC.reg = CAN_TXEFC_EFSA((uint32_t) c->tx_event_fifo) | CAN_TXEFC_EFS(CAN_TX_FIFO_SIZE);


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
		;

	if ((c->features & SC_FEATURE_FLAG_TXR) == SC_FEATURE_FLAG_TXR) {
		can->IE.reg |= CAN_IE_TEFNE; 	// new message in tx event fifo
	}

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

static void can_int(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(cans.can), );
	struct can *can = &cans.can[index];

	// LOG("IE=%08lx IR=%08lx\n", can->m_can->IE.reg, can->m_can->IR.reg);

	CAN_IR_Type ir = can->m_can->IR;
	if (ir.bit.TSW) {
		uint32_t ts_high = __sync_add_and_fetch(&can->ts_high, 1u << M_CAN_TS_COUNTER_BITS);
		(void)ts_high;
		// LOG("CAN%u ts_high=%08lx\n", index, ts_high);
	}

	uint8_t curr_status = 0;

	if (ir.bit.EP) {
		LOG("CAN%u error passive\n", index);
		curr_status |= SC_CAN_STATUS_FLAG_ERROR_PASSIVE;
	}

	if (ir.bit.EW) {
		LOG("CAN%u error warning\n", index);
		curr_status |= SC_CAN_STATUS_FLAG_ERROR_WARNING;
	}

	if (ir.bit.BO) {
		LOG("CAN%u bus off\n", index);
		curr_status |= SC_CAN_STATUS_FLAG_BUS_OFF;
	}

	if (ir.bit.RF0L) {
		LOG("CAN%u msg lost\n", index);
		curr_status |= SC_CAN_STATUS_FLAG_RX_FULL;
	}

	bool notify = can->status_flags != curr_status;

	can->status_flags = curr_status;

	notify |= ir.bit.RF0N;

	// clear all interrupts
	can->m_can->IR = ir;

	if (notify) {
		// LOG("CAN%u notify\n", index);
		BaseType_t woken = pdFALSE;
		vTaskNotifyGiveFromISR(can->task, &woken);
		portYIELD_FROM_ISR(woken);
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
		// clear any old interrupts
		// can->IR = can->IR;
		// disable initialization
		m_can_init_end(can);
		// enable interrupt
		NVIC_EnableIRQ(interrupt_id);
	} else {
		NVIC_DisableIRQ(interrupt_id);
		m_can_init_begin(can);
	}
}

static inline void cans_engage(void)
{
	for (size_t i = 0; i < TU_ARRAY_SIZE(cans.can); ++i) {
		struct can *can = &cans.can[i];
		can_set_state1(can->m_can, can->interrupt_id, can->enabled);
	}
}

static inline void cans_disengage(void)
{
	for (size_t i = 0; i < TU_ARRAY_SIZE(cans.can); ++i) {
		struct can *can = &cans.can[i];
		// can->desync = false;
		can_set_state1(can->m_can, can->interrupt_id, false);
	}
}

static void cans_reset(void)
{
	// disable CAN units, reset configuration & status
	for (size_t i = 0; i < TU_ARRAY_SIZE(cans.can); ++i) {
		struct can *can = &cans.can[i];
		can_set_state1(can->m_can, can->interrupt_id, false);

		can->enabled = false;
		can->features = 0;
		can->status_flags = 0;
		can->rx_lost = 0;
		can->tx_dropped = 0;
		can->desync = false;
		can->mode = 0;
		can->nm_bitrate_bps = 0;
		can->nmbt_brp = 0;
		can->nmbt_sjw = 0;
		can->nmbt_tseg1 = 0;
		can->dtbt_brp = 0;
		can->dtbt_sjw = 0;
		can->dtbt_tseg1 = 0;
		can->dtbt_tseg2 = 0;
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
	volatile uint16_t time_ms;
	volatile uint8_t blink;
	uint8_t pin;
};

#if CFG_TUSB_DEBUG > 0
#define LED_STATIC_INITIALIZER(name, pin) \
	{ name, 0, 0, pin }
#else
#define LED_STATIC_INITIALIZER(name, pin) \
	{ 0, 0, pin }
#endif

#if HWREV == 1
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
#define POWER_LED LED_RED1
#define CAN0_LED LED_GREEN1
#define CAN1_LED LED_GREEN2

static void led_init(void)
{
	PORT->Group[1].DIRSET.reg = PORT_PB14; /* Debug-LED */
	PORT->Group[1].DIRSET.reg = PORT_PB15; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA12; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA13; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA14; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA15; /* Debug-LED */
}
#else // HWREV > 1
static struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PA02),
	LED_STATIC_INITIALIZER("red", PIN_PA18),
	LED_STATIC_INITIALIZER("orange", PIN_PA19),
	LED_STATIC_INITIALIZER("green", PIN_PB16),
	LED_STATIC_INITIALIZER("blue", PIN_PB17),
#if HWREV >= 3
	LED_STATIC_INITIALIZER("can0_green", PIN_PB02),
	LED_STATIC_INITIALIZER("can0_red", PIN_PB03),
	LED_STATIC_INITIALIZER("can1_green", PIN_PB00),
	LED_STATIC_INITIALIZER("can1_red", PIN_PB01),
#endif
};

enum {
	LED_DEBUG_DEFAULT,
	LED_DEBUG_0,
	LED_DEBUG_1,
	LED_DEBUG_2,
	LED_DEBUG_3,
#if HWREV >= 3
	LED_CAN0_STATUS_GREEN,
	LED_CAN0_STATUS_RED,
	LED_CAN1_STATUS_GREEN,
	LED_CAN1_STATUS_RED,
#endif
};

#define USB_TRAFFIC_DO_LED led_burst(LED_DEBUG_3, 8)
#define POWER_LED LED_DEBUG_0
#define CAN0_TRAFFIC_LED LED_DEBUG_1
#define CAN1_TRAFFIC_LED LED_DEBUG_2


static void led_init(void)
{
	PORT->Group[0].DIRSET.reg = PORT_PA18 | PORT_PA19;
	PORT->Group[1].DIRSET.reg =
		PORT_PB16 | PORT_PB17
#if HWREV >= 3
		| PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03
#endif
		;
}

#endif // HWREV


static void led_init(void);
static inline void led_set(uint8_t index, bool on)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].time_ms = 0;
	leds[index].blink = 0;
	gpio_set_pin_level(leds[index].pin, on);
}

static inline void led_toggle(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].time_ms = 0;
	leds[index].blink = 0;
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

static inline void canled_set_status(struct can *can)
{
#if HWREV >= 3
	const uint16_t BLINK_DELAY_MS = 200;
	if (can->enabled) {
		uint8_t status = can->status_flags;
		if (status) {
			led_set(can->led_status_green, 0);
			led_blink(can->led_status_red, BLINK_DELAY_MS);
		} else {
			led_blink(can->led_status_green, BLINK_DELAY_MS);
			led_set(can->led_status_red, 0);
		}
	} else {
		led_set(can->led_status_green, 0);
		led_set(can->led_status_red, 0);
	}
#endif
}

static inline void cans_led_status_set(void)
{
	for (uint8_t i = 0; i < TU_ARRAY_SIZE(cans.can); ++i) {
		canled_set_status(&cans.can[i]);
	}
}

#define MAJOR 0
#define MINOR 2
#define PATCH 0


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
	dfu.status.bState = DFU_STATE_APP_IDLE;
}

#endif

#define CMD_BUFFER_SIZE 64
#define MSG_BUFFER_SIZE 512

struct usb_can {
	CFG_TUSB_MEM_ALIGN uint8_t tx_buffers[2][MSG_BUFFER_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t rx_buffers[2][MSG_BUFFER_SIZE];
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

static inline bool sc_cmd_bulk_in_ep_ready(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.cmd));
	struct usb_cmd *cmd = &usb.cmd[index];
	return 0 == cmd->tx_offsets[!cmd->tx_bank];
}

static inline void sc_cmd_bulk_in_submit(uint8_t index)
{
	TU_ASSERT(sc_cmd_bulk_in_ep_ready(index), );
	struct usb_cmd *cmd = &usb.cmd[index];
	TU_ASSERT(cmd->tx_offsets[cmd->tx_bank] > 0, );
	TU_ASSERT(cmd->tx_offsets[cmd->tx_bank] <= CMD_BUFFER_SIZE, );
	(void)dcd_edpt_xfer(usb.port, 0x80 | cmd->pipe, cmd->tx_buffers[cmd->tx_bank], cmd->tx_offsets[cmd->tx_bank]);
	cmd->tx_bank = !cmd->tx_bank;
}

static inline bool sc_can_bulk_in_ep_ready(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.can));
	struct usb_can *can = &usb.can[index];
	return 0 == can->tx_offsets[!can->tx_bank];
}

static inline void sc_can_bulk_in_submit(uint8_t index)
{
	TU_ASSERT(sc_can_bulk_in_ep_ready(index), );
	struct usb_can *can = &usb.can[index];
	TU_ASSERT(can->tx_offsets[can->tx_bank] > 0, );
	TU_ASSERT(can->tx_offsets[can->tx_bank] <= MSG_BUFFER_SIZE, );
	(void)dcd_edpt_xfer(usb.port, 0x80 | can->pipe, can->tx_buffers[can->tx_bank], can->tx_offsets[can->tx_bank]);
	can->tx_bank = !can->tx_bank;
}

static void sc_cmd_bulk_out(uint8_t index, uint32_t xferred_bytes);
static void sc_cmd_bulk_in(uint8_t index);
static void sc_can_bulk_out(uint8_t index, uint32_t xferred_bytes);
static void sc_can_bulk_in(uint8_t index);

static void sc_cmd_bulk_out(uint8_t index, uint32_t xferred_bytes)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.cmd), );
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.can), );
	TU_ASSERT(index < TU_ARRAY_SIZE(cans.can), );

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
			cans_reset();
			cans_led_status_set();

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
			rep->msg_buffer_size = cpu_to_be16(MSG_BUFFER_SIZE);

			// don't process any more messages
			in_ptr = in_end;

			// assume in token is available
		} break;
		case SC_MSG_DEVICE_INFO: {
			LOG("ch%u SC_MSG_DEVICE_INFO\n", index);
			uint8_t bytes = sizeof(struct sc_msg_dev_info) + sizeof(struct sc_chan_info);

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
				// rep->chan_count = 1;
				// rep->features = SC_FEATURE_FLAG_FDF | SC_FEATURE_FLAG_EHD
				// 	| SC_FEATURE_FLAG_TXR | SC_FEATURE_FLAG_MON_MODE
				// 	| SC_FEATURE_FLAG_RES_MODE | SC_FEATURE_FLAG_EXT_LOOP_MODE;
				// rep->can_clk_hz = CAN_CLK_HZ;
				// rep->nmbt_brp_min = M_CAN_NMBT_BRP_MIN;
				// rep->nmbt_brp_max = M_CAN_NMBT_BRP_MAX;
				// rep->nmbt_tq_min = M_CAN_NMBT_TQ_MIN;
				// rep->nmbt_tq_max = M_CAN_NMBT_TQ_MAX;
				// rep->nmbt_sjw_min = M_CAN_NMBT_SJW_MIN;
				// rep->nmbt_sjw_max = M_CAN_NMBT_SJW_MAX;
				// rep->nmbt_tseg1_min = M_CAN_NMBT_TSEG1_MIN;
				// rep->nmbt_tseg1_max = M_CAN_NMBT_TSEG1_MAX;
				// rep->nmbt_tseg2_min = M_CAN_NMBT_TSEG2_MIN;
				// rep->nmbt_tseg2_max = M_CAN_NMBT_TSEG2_MAX;
				// rep->dtbt_brp_min = M_CAN_DTBT_BRP_MIN;
				// rep->dtbt_brp_max = M_CAN_DTBT_BRP_MAX;
				// rep->dtbt_tq_min = M_CAN_DTBT_TQ_MIN;
				// rep->dtbt_tq_max = M_CAN_DTBT_TQ_MAX;
				// rep->dtbt_sjw_min = M_CAN_DTBT_SJW_MIN;
				// rep->dtbt_sjw_max = M_CAN_DTBT_SJW_MAX;
				// rep->dtbt_tseg1_min = M_CAN_DTBT_TSEG1_MIN;
				// rep->dtbt_tseg1_max = M_CAN_DTBT_TSEG1_MAX;
				// rep->dtbt_tseg2_min = M_CAN_DTBT_TSEG2_MIN;
				// rep->dtbt_tseg2_max = M_CAN_DTBT_TSEG2_MAX;
				rep->fw_ver_major = MAJOR;
				rep->fw_ver_minor = MINOR;
				rep->fw_ver_patch = PATCH;
				rep->name_len = tu_min8(sizeof(BOARD_NAME " " SC_NAME)-1, sizeof(rep->name_bytes));
				memcpy(rep->name_bytes, BOARD_NAME " " SC_NAME, rep->name_len);
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
					// ouch
				}
			}
		} break;
		case SC_MSG_CAN_INFO: {
			LOG("ch%u SC_MSG_CAN_INFO\n", index);
			uint8_t bytes = sizeof(struct sc_msg_can_info) + sizeof(struct sc_chan_info);

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
				rep->chan_count = 1;
				rep->features = SC_FEATURE_FLAG_FDF | SC_FEATURE_FLAG_EHD
					| SC_FEATURE_FLAG_TXR | SC_FEATURE_FLAG_MON_MODE
					| SC_FEATURE_FLAG_RES_MODE | SC_FEATURE_FLAG_EXT_LOOP_MODE;
				rep->can_clk_hz = CAN_CLK_HZ;
				rep->nmbt_brp_min = M_CAN_NMBT_BRP_MIN;
				rep->nmbt_brp_max = M_CAN_NMBT_BRP_MAX;
				rep->nmbt_tq_min = M_CAN_NMBT_TQ_MIN;
				rep->nmbt_tq_max = M_CAN_NMBT_TQ_MAX;
				// rep->nmbt_sjw_min = M_CAN_NMBT_SJW_MIN;
				rep->nmbt_sjw_max = M_CAN_NMBT_SJW_MAX;
				rep->nmbt_tseg1_min = M_CAN_NMBT_TSEG1_MIN;
				rep->nmbt_tseg1_max = M_CAN_NMBT_TSEG1_MAX;
				rep->nmbt_tseg2_min = M_CAN_NMBT_TSEG2_MIN;
				rep->nmbt_tseg2_max = M_CAN_NMBT_TSEG2_MAX;
				rep->dtbt_brp_min = M_CAN_DTBT_BRP_MIN;
				rep->dtbt_brp_max = M_CAN_DTBT_BRP_MAX;
				rep->dtbt_tq_min = M_CAN_DTBT_TQ_MIN;
				rep->dtbt_tq_max = M_CAN_DTBT_TQ_MAX;
				// rep->dtbt_sjw_min = M_CAN_DTBT_SJW_MIN;
				rep->dtbt_sjw_max = M_CAN_DTBT_SJW_MAX;
				rep->dtbt_tseg1_min = M_CAN_DTBT_TSEG1_MIN;
				rep->dtbt_tseg1_max = M_CAN_DTBT_TSEG1_MAX;
				rep->dtbt_tseg2_min = M_CAN_DTBT_TSEG2_MIN;
				rep->dtbt_tseg2_max = M_CAN_DTBT_TSEG2_MAX;
				rep->tx_fifo_size = CAN_TX_FIFO_SIZE;
				rep->rx_fifo_size = CAN_RX_FIFO_SIZE;
				rep->chan_info[0].cmd_epp = usb_cmd->pipe;
				rep->chan_info[0].msg_epp = usb_can->pipe;
			} else {
				if (sc_cmd_bulk_in_ep_ready(index)) {
					sc_cmd_bulk_in_submit(index);
					goto send_can_info;
				} else {
					// ouch
				}
			}
		} break;
		case SC_MSG_BITTIMING: {
			LOG("ch%u SC_MSG_BITTIMING\n", index);
			struct sc_msg_bittiming const *tmsg = (struct sc_msg_bittiming const *)msg;
			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ch%u ERROR: msg too short\n", index);
				continue;
			}

			// if (unlikely(tmsg->channel != index)) {
			// 	LOG("ERROR: ch%u mismatch %u\n", index, tmsg->channel);
			// 	continue;
			// }

			uint16_t nmbt_brp, nmbt_tseg1;
			uint8_t nmbt_sjw, nmbt_tseg2, dtbt_brp, dtbt_sjw, dtbt_tseg1, dtbt_tseg2;

			nmbt_brp = tmsg->nmbt_brp;
			nmbt_tseg1 = tmsg->nmbt_tseg1;
			nmbt_sjw = tmsg->nmbt_sjw;
			nmbt_tseg2 = tmsg->nmbt_tseg2;
			dtbt_brp = tmsg->dtbt_brp;
			dtbt_sjw = tmsg->dtbt_sjw;
			dtbt_tseg1 = tmsg->dtbt_tseg1;
			dtbt_tseg2 = tmsg->dtbt_tseg2;

			// clamp
			can->nmbt_brp = tu_max16(M_CAN_NMBT_BRP_MIN, tu_min16(nmbt_brp, M_CAN_NMBT_BRP_MAX));
			can->nmbt_tseg1 = tu_max16(M_CAN_NMBT_TSEG1_MIN, tu_min16(nmbt_tseg1, M_CAN_NMBT_TSEG1_MAX));
			can->nmbt_sjw = tu_max8(M_CAN_NMBT_SJW_MIN, tu_min8(nmbt_sjw, M_CAN_NMBT_SJW_MAX));
			can->nmbt_tseg2 = tu_max8(M_CAN_NMBT_TSEG2_MIN, tu_min8(nmbt_tseg2, M_CAN_NMBT_TSEG2_MAX));
			can->dtbt_brp = tu_max8(M_CAN_DTBT_BRP_MIN, tu_min8(dtbt_brp, M_CAN_DTBT_BRP_MAX));
			can->dtbt_sjw = tu_max8(M_CAN_DTBT_SJW_MIN, tu_min8(dtbt_sjw, M_CAN_DTBT_SJW_MAX));
			can->dtbt_tseg1 = tu_max8(M_CAN_DTBT_TSEG1_MIN, tu_min8(dtbt_tseg1, M_CAN_DTBT_TSEG1_MAX));
			can->dtbt_tseg2 = tu_max8(M_CAN_DTBT_TSEG2_MIN, tu_min8(dtbt_tseg2, M_CAN_DTBT_TSEG2_MAX));

			// set nominal bitrate for timestamp calculation
			can->nm_bitrate_bps = CAN_CLK_HZ / ((uint32_t)can->nmbt_brp * (1 + can->nmbt_tseg1 + can->nmbt_tseg2));

			uint8_t bytes = sizeof(struct sc_msg_error);

			uint8_t *out_ptr;
			uint8_t *out_end;

send_bt_response:
			out_ptr = usb_cmd->tx_buffers[usb_cmd->tx_bank] + usb_cmd->tx_offsets[usb_cmd->tx_bank];
			out_end = usb_cmd->tx_buffers[usb_cmd->tx_bank] + CMD_BUFFER_SIZE;
			if (out_end - out_ptr >= bytes) {
				usb_cmd->tx_offsets[usb_cmd->tx_bank] += bytes;
				struct sc_msg_error *rep = (struct sc_msg_error *)out_ptr;
				rep->id = SC_MSG_ERROR;
				rep->len = sizeof(*rep);
				rep->channel = 0;
				rep->error = SC_ERROR_NONE;
			} else {
				if (sc_cmd_bulk_in_ep_ready(index)) {
					sc_cmd_bulk_in_submit(index);
					goto send_bt_response;
				} else {
					// ouch
				}
			}
		} break;
		// case SC_MSG_RESET: {
		// 	LOG("ch%u SC_MSG_RESET\n", index);
		// 	// NVIC_SystemReset();
		// } break;
		case SC_MSG_MODE: {
			LOG("ch%u SC_MSG_MODE\n", index);
			struct sc_msg_config const *tmsg = (struct sc_msg_config const *)msg;
			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ch%u ERROR: msg too short\n", index);
				continue;
			}

			// if (unlikely(tmsg->channel != index)) {
			// 	LOG("ERROR: ch%u mismatch %u\n", index, tmsg->channel);
			// 	continue;
			// }

			can->mode = tmsg->args[0];

			uint8_t bytes = sizeof(struct sc_msg_error);
			uint8_t *out_ptr;
			uint8_t *out_end;

send_mode_response:
			out_ptr = usb_cmd->tx_buffers[usb_cmd->tx_bank] + usb_cmd->tx_offsets[usb_cmd->tx_bank];
			out_end = usb_cmd->tx_buffers[usb_cmd->tx_bank] + CMD_BUFFER_SIZE;
			if (out_end - out_ptr >= bytes) {
				usb_cmd->tx_offsets[usb_cmd->tx_bank] += bytes;
				struct sc_msg_error *rep = (struct sc_msg_error *)out_ptr;
				rep->id = SC_MSG_ERROR;
				rep->len = sizeof(*rep);
				rep->channel = 0;
				rep->error = SC_ERROR_NONE;
			} else {
				if (sc_cmd_bulk_in_ep_ready(index)) {
					sc_cmd_bulk_in_submit(index);
					goto send_mode_response;
				} else {
					// ouch
				}
			}
		} break;
		case SC_MSG_FEATURES: {
			LOG("ch%u SC_MSG_FEATURES\n", index);
			struct sc_msg_config const *tmsg = (struct sc_msg_config const *)msg;
			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ch%u ERROR: msg too short\n", index);
				continue;
			}

			// if (unlikely(tmsg->channel != index)) {
			// 	LOG("ERROR: ch%u mismatch %u\n", index, tmsg->channel);
			// 	continue;
			// }

			can->features = tmsg->args[0];

			uint8_t bytes = sizeof(struct sc_msg_error);
			uint8_t *out_ptr;
			uint8_t *out_end;

send_features_response:
			out_ptr = usb_cmd->tx_buffers[usb_cmd->tx_bank] + usb_cmd->tx_offsets[usb_cmd->tx_bank];
			out_end = usb_cmd->tx_buffers[usb_cmd->tx_bank] + CMD_BUFFER_SIZE;
			if (out_end - out_ptr >= bytes) {
				usb_cmd->tx_offsets[usb_cmd->tx_bank] += bytes;
				struct sc_msg_error *rep = (struct sc_msg_error *)out_ptr;
				rep->id = SC_MSG_ERROR;
				rep->len = sizeof(*rep);
				rep->channel = 0;
				rep->error = SC_ERROR_NONE;
			} else {
				if (sc_cmd_bulk_in_ep_ready(index)) {
					sc_cmd_bulk_in_submit(index);
					goto send_features_response;
				} else {
					// ouch
				}
			}
		} break;
		case SC_MSG_BUS: {
			LOG("ch%u SC_MSG_BUS\n", index);
			struct sc_msg_config const *tmsg = (struct sc_msg_config const *)msg;

			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ERROR: msg too short\n");
				continue;
			}

			// if (unlikely(tmsg->channel != index)) {
			// 	LOG("ERROR: ch%u mismatch %u\n", index, tmsg->channel);
			// 	continue;
			// }

			bool was_enabled = can->enabled;
			can->enabled = tmsg->args[0] != 0;
			if (was_enabled != can->enabled) {
				LOG("ch%u enabled=%u\n", index, can->enabled);
				if (can->enabled) {
					can_configure(can);
					can->desync = false;
					can->status_flags = 0;
				}

				can_set_state1(can->m_can, can->interrupt_id, can->enabled);
				canled_set_status(can);
			}

			uint8_t bytes = sizeof(struct sc_msg_error);
			uint8_t *out_ptr;
			uint8_t *out_end;

send_bus_response:
			out_ptr = usb_cmd->tx_buffers[usb_cmd->tx_bank] + usb_cmd->tx_offsets[usb_cmd->tx_bank];
			out_end = usb_cmd->tx_buffers[usb_cmd->tx_bank] + CMD_BUFFER_SIZE;
			if (out_end - out_ptr >= bytes) {
				usb_cmd->tx_offsets[usb_cmd->tx_bank] += bytes;
				struct sc_msg_error *rep = (struct sc_msg_error *)out_ptr;
				rep->id = SC_MSG_ERROR;
				rep->len = sizeof(*rep);
				rep->channel = 0;
				rep->error = SC_ERROR_NONE;
			} else {
				if (sc_cmd_bulk_in_ep_ready(index)) {
					sc_cmd_bulk_in_submit(index);
					goto send_bus_response;
				} else {
					// ouch
				}
			}
		} break;
		default:
			TU_LOG2_MEM(msg, msg->len, 2);
			break;
		}
	}

	if (usb_cmd->tx_offsets[usb_cmd->tx_bank] > 0 && sc_cmd_bulk_in_ep_ready(index)) {
		// LOG("usb tx %u bytes\n", usb.tx_offsets[usb.tx_bank]);
		sc_cmd_bulk_in_submit(index);
	}
}

static void sc_can_bulk_out(uint8_t index, uint32_t xferred_bytes)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.can), );
	TU_ASSERT(index < TU_ARRAY_SIZE(cans.can), );

	struct can *can = &cans.can[index];
	struct usb_can *usb_can = &usb.can[index];
	led_burst(can->led_traffic, 8);


	const uint8_t rx_bank = usb_can->rx_bank;
	// LOG("CAN%u rx bank %u\n", index, rx_bank);
	(void)rx_bank;
	uint8_t const * const in_beg = usb_can->rx_buffers[usb_can->rx_bank];
	uint8_t const *in_ptr = in_beg;
	uint8_t const * const in_end = in_ptr + xferred_bytes;

	// start new transfer right away
	usb_can->rx_bank = !usb_can->rx_bank;
	(void)dcd_edpt_xfer(usb.port, usb_can->pipe, usb_can->rx_buffers[usb_can->rx_bank], MSG_BUFFER_SIZE);


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

			// if (unlikely(tmsg->channel != index)) {
			// 	LOG("ERROR: CAN%u channel %u mismatch\n", index, tmsg->channel);
			// 	continue;
			// }

			const uint8_t can_frame_len = dlc_to_len(tmsg->dlc);
			if (!(tmsg->flags & SC_CAN_FLAG_RTR)) {
				if (msg->len < sizeof(*tmsg) + can_frame_len) {
					LOG("ch%u ERROR: msg too short\n", index);
					continue;
				}
			}

			if (can->m_can->TXFQS.bit.TFQF) {
				++can->tx_dropped;
				if (can->features & SC_FEATURE_FLAG_TXR) {
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
						rep->channel = tmsg->channel;
						rep->track_id = tmsg->track_id;
						uint32_t ts = can->ts_high | can->m_can->TSCV.bit.TSC;
						rep->timestamp_us = can_time_to_us(can, ts);
						rep->flags = SC_CAN_FLAG_DRP;
					} else {
						if (sc_can_bulk_in_ep_ready(index)) {
							sc_can_bulk_in_submit(index);
							goto send_txr;
						} else {
							LOG("ch%u: desync\n", index);
							can->desync = true;
						}
					}
				}
			} else {
				uint32_t id = tmsg->can_id;
				uint16_t tid = tmsg->track_id;

				uint8_t put_index = can->m_can->TXFQS.bit.TFQPI;
				uint8_t marker_index = can->tx_fifo[put_index].T1.bit.MM;
				can->message_marker_map[marker_index] = tid; // save message marker

				CAN_TXBE_0_Type t0;
				t0.reg = (((tmsg->flags & SC_CAN_FLAG_ESI) == SC_CAN_FLAG_ESI) << CAN_TXBE_0_ESI_Pos)
					| (((tmsg->flags & SC_CAN_FLAG_RTR) == SC_CAN_FLAG_RTR) << CAN_TXBE_0_RTR_Pos)
					| (((tmsg->flags & SC_CAN_FLAG_EXT) == SC_CAN_FLAG_EXT) << CAN_TXBE_0_XTD_Pos)
					;


				if (tmsg->flags & SC_CAN_FLAG_EXT) {
					t0.reg |= CAN_TXBE_0_ID(id);
				} else {
					t0.reg |= CAN_TXBE_0_ID(id << 18);
				}

				can->tx_fifo[put_index].T0 = t0;
				can->tx_fifo[put_index].T1.bit.DLC = tmsg->dlc;
				can->tx_fifo[put_index].T1.bit.FDF = (tmsg->flags & SC_CAN_FLAG_FDF) == SC_CAN_FLAG_FDF;
				can->tx_fifo[put_index].T1.bit.BRS = (tmsg->flags & SC_CAN_FLAG_BRS) == SC_CAN_FLAG_BRS;

				if (!(tmsg->flags & SC_CAN_FLAG_RTR)) {
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
		// LOG("usb tx %u bytes\n", usb.tx_offsets[usb.tx_bank]);
		sc_can_bulk_in_submit(index);
	}
}

static void sc_cmd_bulk_in(uint8_t index)
{
	LOG("< cmd%u IN token\n", index);

	TU_ASSERT(index < TU_ARRAY_SIZE(usb.cmd), );

	struct usb_cmd *usb_cmd = &usb.cmd[index];

	usb_cmd->tx_offsets[!usb_cmd->tx_bank] = 0;

	if (usb_cmd->tx_offsets[usb_cmd->tx_bank]) {
		// LOG("usb msg tx %u bytes\n", usb.tx_offsets[usb.tx_bank]);
		sc_cmd_bulk_in_submit(index);
	}
}

static void sc_can_bulk_in(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(usb.can), );

	struct usb_can *usb_can = &usb.can[index];

	usb_can->tx_offsets[!usb_can->tx_bank] = 0;

	if (usb_can->tx_offsets[usb_can->tx_bank]) {
		// LOG("usb msg tx %u bytes\n", usb.tx_offsets[usb.tx_bank]);
		sc_can_bulk_in_submit(index);
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


	cans.can[0].led_status_green = LED_CAN0_STATUS_GREEN;
	cans.can[0].led_status_red = LED_CAN0_STATUS_RED;
	cans.can[0].led_traffic = CAN0_TRAFFIC_LED;
	cans.can[0].task = xTaskCreateStatic(&can_task, "can0", TU_ARRAY_SIZE(can_task_stack[0]), (void*)(uintptr_t)0, configMAX_PRIORITIES-1, can_task_stack[0], &can_task_mem[0]);
	cans.can[1].led_status_green = LED_CAN1_STATUS_GREEN;
	cans.can[1].led_status_red = LED_CAN1_STATUS_RED;
	cans.can[1].led_traffic = CAN1_TRAFFIC_LED;
	cans.can[1].task = xTaskCreateStatic(&can_task, "can1", TU_ARRAY_SIZE(can_task_stack[1]), (void*)(uintptr_t)1, configMAX_PRIORITIES-1, can_task_stack[1], &can_task_mem[1]);

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

	cans_led_status_set();
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	LOG("unmounted\n");
	led_blink(0, 1000);
	usb.mounted = false;

	cans_reset();
	cans_led_status_set();
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
	cans_led_status_set();
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	LOG("resume\n");
	usb.mounted = true;
	led_blink(0, 250);

	// can_engage();
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
	memset(&usb, 0, sizeof(usb));
	usb.port = rhport;
	usb.cmd[0].pipe = SC_M1_EP_CMD0_BULK_OUT;
	usb.cmd[1].pipe = SC_M1_EP_CMD1_BULK_OUT;
	usb.can[0].pipe = SC_M1_EP_MSG0_BULK_OUT;
	usb.can[1].pipe = SC_M1_EP_MSG1_BULK_OUT;
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
		TU_ASSERT(dcd_edpt_open(rhport, ep_desc));
	}

	TU_ASSERT(dcd_edpt_xfer(rhport, usb_cmd->pipe, usb_cmd->rx_buffers[usb_cmd->rx_bank], CMD_BUFFER_SIZE));
	TU_ASSERT(dcd_edpt_xfer(rhport, usb_can->pipe, usb_can->rx_buffers[usb_can->rx_bank], MSG_BUFFER_SIZE));

	*p_length = 9+eps*7;

	return true;
}

bool tud_custom_xfer_cb(
	uint8_t rhport,
	uint8_t ep_addr,
	xfer_result_t event,
	uint32_t xferred_bytes)
{
	USB_TRAFFIC_DO_LED;

	(void)event; // always success

	if (unlikely(rhport != usb.port)) {
		return false;
	}

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

	LOG("req type 0x%02x (reci %s type %s dir %s) req 0x%02x, value 0x%04x index 0x%04x reqlen %u\n",
		request->bmRequestType,
		recipient_str(request->bmRequestType_bit.recipient),
		type_str(request->bmRequestType_bit.type),
		dir_str(request->bmRequestType_bit.direction),
		request->bRequest, request->wValue, request->wIndex,
		request->wLength);

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

	if (DFU_STATE_APP_DETACH == dfu.status.bState) {
		LOG("Detected USB reset while detach timer is running");
		dfu_request_dfu(1);
		NVIC_SystemReset();
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
	TU_VERIFY(request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);
	TU_VERIFY(request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE);

	LOG("req type 0x%02x (reci %s type %s dir %s) req 0x%02x, value 0x%04x index 0x%04x reqlen %u\n",
		request->bmRequestType,
		recipient_str(request->bmRequestType_bit.recipient),
		type_str(request->bmRequestType_bit.type),
		dir_str(request->bmRequestType_bit.direction),
		request->bRequest, request->wValue, request->wIndex,
		request->wLength);


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
		}
		// return false; // stall pipe to trigger reset
		return tud_control_xfer(rhport, request, NULL, 0);
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

//--------------------------------------------------------------------+
// LED TASK
//--------------------------------------------------------------------+
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
						// LOG("led %s (%u) toggle\n", leds[i].name, i);
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

	LOG("CAN%u task start\n", index);

	struct can *can = &cans.can[index];
	struct usb_can *usb_can = &usb.can[index];

	while (42) {
		// LOG("CAN%u task wait\n", index);
		(void)ulTaskNotifyTake(pdFALSE, ~0);

		// LOG("CAN%u task loop\n", index);
		if (unlikely(!usb.mounted)) {
			continue;
		}

		if (unlikely(!can->enabled)) {
			continue;
		}

		led_burst(can->led_traffic, 8);
		canled_set_status(can);

		for (bool done = false; !done; ) {
			done = true;
			uint8_t *out_beg;
			uint8_t *out_end;
			uint8_t *out_ptr;
start:
			out_beg = usb_can->tx_buffers[usb_can->tx_bank];
			out_end = out_beg + MSG_BUFFER_SIZE;
			out_ptr = out_beg + usb_can->tx_offsets[usb_can->tx_bank];
			TU_ASSERT(out_ptr <= out_end, );

			if (out_ptr == out_beg) {
				// place status messages
				done = false;

				uint32_t ts = can->ts_high | can->m_can->TSCV.bit.TSC;
				uint32_t us = can_time_to_us(can, ts);
				CAN_ECR_Type ecr = can->m_can->ECR;

				struct sc_msg_can_status *msg = (struct sc_msg_can_status *)out_ptr;
				msg->len = sizeof(*msg);
				msg->id = SC_MSG_CAN_STATUS;
				msg->channel = index;
				msg->timestamp_us = us;
				msg->rx_lost = can->rx_lost;
				msg->tx_dropped = can->tx_dropped;
				msg->flags = can->status_flags;
				if (can->tx_dropped) {
					msg->flags |= SC_CAN_STATUS_FLAG_TX_FULL;
				}
				if (can->desync) {
					msg->flags |= SC_CAN_STATUS_FLAG_TXR_DESYNC;
				}
				msg->tx_errors = ecr.bit.TEC;
				msg->rx_errors = ecr.bit.REC;
				can->rx_lost = 0;
				can->tx_dropped = 0;
				out_ptr += msg->len;
			}

			if (m_can_rx0_msg_fifo_avail(can->m_can)) {
				done = false;
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
					struct sc_msg_can_rx *msg = (struct sc_msg_can_rx *)out_ptr;
					out_ptr += bytes;

					msg->id = SC_MSG_CAN_RX;
					msg->len = bytes;
					msg->channel = index;
					msg->flags = 0;
					uint32_t id = r0.bit.ID;
					if (r0.bit.XTD) {
						msg->flags |= SC_CAN_FLAG_EXT;
					} else {
						id >>= 18;
					}
					msg->can_id = id;

					// LOG("CAN%u ts hi=%08lx lo=%04x\n", index, can->ts_high, r1.bit.RXTS);
					uint32_t ts = can->ts_high | r1.bit.RXTS;
					msg->timestamp_us = can_time_to_us(can, ts);

					if (r1.bit.FDF) {
						msg->flags |= SC_CAN_FLAG_FDF;
					}
					if (r1.bit.BRS) {
						msg->flags |= SC_CAN_FLAG_BRS;
					}
					if (r0.bit.RTR) {
						msg->flags |= SC_CAN_FLAG_RTR;
					} else {
						memcpy(msg->data, can->rx_fifo[get_index].data, can_frame_len);
					}

					msg->dlc = r1.bit.DLC;
				} else {
					if (sc_can_bulk_in_ep_ready(index)) {
						TU_ASSERT(out_ptr <= out_end, );
						usb_can->tx_offsets[usb_can->tx_bank] = out_ptr - out_beg;
						sc_can_bulk_in_submit(index);
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
				if (can->features & SC_FEATURE_FLAG_TXR) {
					uint8_t bytes = sizeof(struct sc_msg_can_txr);
					CAN_TXEFE_0_Type t0 = can->tx_event_fifo[get_index].T0;
					CAN_TXEFE_1_Type t1 = can->tx_event_fifo[get_index].T1;

					if (out_end - out_ptr >= bytes) {
						struct sc_msg_can_txr *msg = (struct sc_msg_can_txr *)out_ptr;
						out_ptr += bytes;

						uint8_t marker_index = t1.bit.MM;

						msg->id = SC_MSG_CAN_TXR;
						msg->len = bytes;
						msg->channel = index;
						msg->track_id = can->message_marker_map[marker_index];
						uint32_t ts = can->ts_high | t1.bit.TXTS;
						msg->timestamp_us = can_time_to_us(can, ts);
						msg->flags = 0;
						if (t0.bit.ESI) {
							msg->flags |= SC_CAN_FLAG_ESI;
						}

						if (t1.bit.FDF) {
							msg->flags |= SC_CAN_FLAG_FDF;
						}

						if (t1.bit.BRS) {
							msg->flags |= SC_CAN_FLAG_BRS;
						}
					} else {
						if (sc_can_bulk_in_ep_ready(index)) {
							TU_ASSERT(out_ptr <= out_end, );
							usb_can->tx_offsets[usb_can->tx_bank] = out_ptr - out_beg;
							sc_can_bulk_in_submit(index);
							goto start;
						} else {
							can->desync = true;
						}
					}
				}

				can->m_can->TXEFA.reg = CAN_TXEFA_EFAI(get_index);
			}

			TU_ASSERT(out_ptr <= out_end, );
			usb_can->tx_offsets[usb_can->tx_bank] = out_ptr - out_beg;
		}


		if (usb_can->tx_offsets[usb_can->tx_bank] > 0 && sc_can_bulk_in_ep_ready(index)) {
			// LOG("usb tx %u bytes\n", usb.tx_offsets[usb.tx_bank]);
			sc_can_bulk_in_submit(index);
		}
	}
}
