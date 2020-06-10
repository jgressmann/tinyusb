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
#include <supercan_m1.h>

#include <m_can.h>




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
	Can *m_can;
	IRQn_Type interrupt_id;
	volatile uint32_t ts_high;
	// uint32_t arbitration_bitrate_bps;
    // uint32_t data_bitrate_bps;
	// uint8_t arbitration_sjw;
    // uint8_t arbitration_sample_point; // [0-1] scaled to [0-255]
	// uint8_t data_sjw;
    // uint8_t data_sample_point; // [0-1] scaled to [0-255]

    uint16_t nmbt_pre;
    uint16_t nmbt_seg1;
	uint8_t nmbt_sjw;
    uint8_t nmbt_seg2;
    uint8_t dtbt_pre;
    uint8_t dtbt_sjw;
    uint8_t dtbt_seg1;
    uint8_t dtbt_seg2;
	uint8_t mode_flags;
	uint16_t rx_lost;
	uint16_t tx_dropped;
	volatile uint8_t status_flags;

	bool enabled;
	bool desync;
};

static struct {
	struct can can[2];
	TaskHandle_t task;
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
	can->CCCR.bit.FDOE = (c->mode_flags & SC_MODE_FLAG_CAN_FD) == SC_MODE_FLAG_CAN_FD;
	can->CCCR.bit.BRSE = (c->mode_flags & SC_MODE_FLAG_BRS) == SC_MODE_FLAG_BRS;
	can->CCCR.bit.DAR = (c->mode_flags & SC_MODE_FLAG_AUTO_RE) != SC_MODE_FLAG_AUTO_RE;
	can->CCCR.bit.PXHD = (c->mode_flags & SC_MODE_FLAG_EH) != SC_MODE_FLAG_EH;

	TU_LOG2("nominal pre %u sjw %u seg1 %u seg2 %u\n",
		c->nmbt_pre, c->nmbt_sjw, c->nmbt_seg1, c->nmbt_seg2
	);

	TU_LOG2("data pre %u sjw %u seg1 %u seg2 %u\n",
		c->dtbt_pre, c->dtbt_sjw, c->dtbt_seg1, c->dtbt_seg2
	);

	can->TSCC.reg = CAN_TSCC_TCP(0) | CAN_TSCC_TSS(1);
	can->TOCC.reg = CAN_TOCC_TOP(0xffff) | CAN_TOCC_TOS(0); // Timeout Counter disabled, Reset-default
	can->NBTP.reg = CAN_NBTP_NSJW(c->nmbt_sjw-1)
			| CAN_NBTP_NBRP(c->nmbt_pre-1)
			| CAN_NBTP_NTSEG1(c->nmbt_seg1-1)
			| CAN_NBTP_NTSEG2(c->nmbt_seg2-1);
	can->DBTP.reg = CAN_DBTP_DBRP(c->dtbt_pre-1)
			| CAN_DBTP_DTSEG1(c->dtbt_seg1-1)
			| CAN_DBTP_DTSEG2(c->dtbt_seg2-1)
			| CAN_DBTP_DSJW(c->dtbt_sjw-1);

	// tx fifo
	can->TXBC.reg = CAN_TXBC_TBSA((uint32_t) c->tx_fifo) | CAN_TXBC_TFQS(CAN_TX_FIFO_SIZE);
	//	REG_CAN0_TXBC |= CAN_TXBC_TFQM; // reset default
	can->TXESC.reg = CAN_TXESC_TBDS_DATA64;

	//can->TXEFC.reg = CAN_TXEFC_EFSA((uint32_t) c->tx_event_fifo) | CAN_TXEFC_EFS(CAN_TX_FIFO_SIZE);


	// rx fifo
	can->RXF0C.reg = CAN_RXF0C_F0SA((uint32_t) c->rx_fifo) | CAN_RXF0C_F0S(CAN_RX_FIFO_SIZE) | CAN_RXF0C_F0OM; // FIFO 0 overwrite mode
	can->RXESC.reg = CAN_RXESC_RBDS_DATA64 + CAN_RXESC_F0DS_DATA64;

	// clear existing interrupt flags
	CAN0->IR.reg = CAN0->IR.reg;
	// enable interrupt line 0
	CAN0->ILE.reg = CAN_ILE_EINT0;

	// wanted interrupts
	CAN0->IE.reg =
		CAN_IE_TSWE 	// time stamp counter wrap
		| CAN_IE_BOE 	// bus off
		| CAN_IE_EWE 	// error warning
		| CAN_IE_EPE 	// error passive
		| CAN_IE_TEFNE 	// new message in tx event fifo
		| CAN_IE_RF0NE 	// new message in rx fifo0
		;


	m_can_conf_end(can);
	// m_can_init_end(can);
}

static void can_init_module()
{
	memset(&cans, 0, sizeof(cans));

	cans.can[0].m_can = CAN0;
	cans.can[1].m_can = CAN1;
	cans.can[0].interrupt_id = CAN0_IRQn;
	cans.can[1].interrupt_id = CAN1_IRQn;

	for (size_t j = 0; j < 2; ++j) {
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
	// NVIC_EnableIRQ(CAN0_IRQn);
	// NVIC_EnableIRQ(CAN1_IRQn);
}

static bool can_int(struct can *can)
{
	const uint8_t index = can==&cans.can[0] ? 0 : 1;
	(void)index;

	bool notify = false;
	CAN_IR_Type ir = can->m_can->IR;
	if (ir.bit.TSW) {
		uint32_t ts_high = __sync_add_and_fetch(&can->ts_high, 1u << M_CAN_TS_COUNTER_BITS);
		(void)ts_high;
		// TU_LOG2("CAN%u ts_high %08x\n", index, ts_high);
	}

	bool had_ep = (can->status_flags & SC_STATUS_FLAG_ERROR_PASSIVE) == SC_STATUS_FLAG_ERROR_PASSIVE;
	notify |= had_ep != ir.bit.EP;
	if (ir.bit.EP) {
		can->status_flags |= SC_STATUS_FLAG_ERROR_PASSIVE;
	} else {
		can->status_flags &= ~SC_STATUS_FLAG_ERROR_PASSIVE;
	}

	bool had_ew = (can->status_flags & SC_STATUS_FLAG_ERROR_WARNING) == SC_STATUS_FLAG_ERROR_WARNING;
	notify |= had_ew != ir.bit.EW;
	if (ir.bit.EW) {
		can->status_flags |= SC_STATUS_FLAG_ERROR_WARNING;
	} else {
		can->status_flags &= ~SC_STATUS_FLAG_ERROR_WARNING;
	}

	bool had_bo = (can->status_flags & SC_STATUS_FLAG_BUS_OFF) == SC_STATUS_FLAG_BUS_OFF;
	notify |= had_bo != ir.bit.BO;
	if (ir.bit.BO) {
		can->status_flags |= SC_STATUS_FLAG_BUS_OFF;
	} else {
		can->status_flags &= ~SC_STATUS_FLAG_BUS_OFF;
	}

	notify |= ir.bit.RF0N;

	// clear all interrupts
	can->m_can->IR = ir;

	return notify;


}

void CAN0_Handler(void)
{
	// TU_LOG2("CAN0 int\n");

	bool notify = can_int(&cans.can[0]);

	if (notify) {
		TU_LOG2("CAN%u notify\n", index);
		BaseType_t woken = pdFALSE;
		vTaskNotifyGiveFromISR(cans.task, &woken);
		// TU_ASSERT(pdFALSE == woken, );
		portYIELD_FROM_ISR(woken);
	}
}

void CAN1_Handler(void)
{
	// TU_LOG2("CAN1 int\n");

	can_int(&cans.can[1]);
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
		0, 1, 2, 3, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
	};
	return map[dlc & 0xf];
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

static StackType_t can_task_stack[configMINIMAL_STACK_SIZE];
static StaticTask_t can_task_mem;

StackType_t usb_in_task_stack[configMINIMAL_STACK_SIZE];
StaticTask_t usb_in_task_mem;

static void tusb_device_task(void* param);
//static void usb_in_task(void* param);
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

#define BUFFER_SIZE 512

static struct usb {
	CFG_TUSB_MEM_ALIGN uint8_t rx_buffer[BUFFER_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t tx_buffers[2][BUFFER_SIZE];
	uint16_t tx_offsets[2];
	uint8_t tx_bank;
	uint8_t port;
	// bool has_in_token;
	bool mounted;
} usb;

static inline bool usb_bulk_in_ep_ready(void)
{
	return 0 == usb.tx_offsets[!usb.tx_bank];
}

static inline void usb_bulk_in_submit(void)
{
	TU_ASSERT(usb_bulk_in_ep_ready(), );
	TU_ASSERT(usb.tx_offsets[usb.tx_bank] > 0, );
	usbd_edpt_xfer(usb.port, SC_M1_EP_BULK_IN, usb.tx_buffers[usb.tx_bank], usb.tx_offsets[usb.tx_bank]);
	usb.tx_bank = !usb.tx_bank;
}


int main(void)
{
	board_init();
	led_init();

	tusb_init();

	(void) xTaskCreateStatic(&tusb_device_task, "tusb", TU_ARRAY_SIZE(usb_device_stack), NULL, configMAX_PRIORITIES-1, usb_device_stack, &usb_device_stack_mem);
	//(void) xTaskCreateStatic(&usb_in_task, "usb_in", TU_ARRAY_SIZE(usb_in_task_stack), NULL, configMAX_PRIORITIES-1, usb_in_task_stack, &usb_in_task_mem);
	(void) xTaskCreateStatic(&led_task, "led", TU_ARRAY_SIZE(led_task_stack), NULL, configMAX_PRIORITIES-1, led_task_stack, &led_task_mem);
	cans.task = xTaskCreateStatic(&can_task, "can", TU_ARRAY_SIZE(can_task_stack), NULL, configMAX_PRIORITIES-1, can_task_stack, &can_task_mem);

	can_init_pins();
	can_init_clock();
	can_init_module();

	// cans.can[0].arbitration_sjw = 1;
	// cans.can[0].arbitration_bitrate_bps = 500000;
	// cans.can[0].arbitration_sample_point = (75 * 255) / 100;
	// cans.can[0].data_sjw = 1;
	// cans.can[0].data_bitrate_bps = 2000000;
	// cans.can[0].data_sample_point = (80 * 255) / 100;
	// cans.can[0].mode_flags = SC_MODE_FLAG_BRS | SC_MODE_FLAG_CAN_FD | SC_MODE_FLAG_AUTO_RE | SC_MODE_FLAG_EH;
	// can_configure(&cans.can[0]);
	// TU_LOG2("CAN0 NBTP %08lx DBTP %08lx\n",
	// 	cans.can[0].m_can->NBTP.reg,
	// 	cans.can[0].m_can->DBTP.reg);


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
		TU_LOG2("usb\n");
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

	can_engage();
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

	ptr += 9;

	for (uint8_t i = 0; i < 2; ++i) {
		TU_ASSERT(dcd_edpt_open(rhport, (tusb_desc_endpoint_t const *)(ptr + i * 7)));
	}

	TU_ASSERT(usbd_edpt_xfer(rhport, 0x01, usb.rx_buffer, TU_ARRAY_SIZE(usb.rx_buffer)));


	*p_length = 9+2*7;

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

	if (rhport != usb.port) {
		return false;
	}

	switch (ep_addr) {
	case SC_M1_EP_BULK_OUT: {
		uint8_t const *in_ptr = usb.rx_buffer;
		uint8_t const * const in_end = usb.rx_buffer + xferred_bytes;
		while (in_ptr + SC_HEADER_LEN <= in_end) {
			struct sc_msg const *msg = (struct sc_msg const *)in_ptr;
			if (in_ptr + msg->len > in_end) {
				TU_LOG1("malformed msg\n");
				break;
			}

			if (!msg->len) {
				break;
			}

			in_ptr += msg->len;

			switch (msg->id) {
			case SC_MSG_EOF: {
				TU_LOG2("SC_MSG_EOF\n");
				in_ptr = in_end;
			} break;

			case SC_MSG_HELLO_DEVICE: {
				TU_LOG2("SC_MSG_HELLO_DEVICE\n");
				// reset tx buffer
				uint8_t len = SC_HEADER_LEN + sizeof(struct sc_msg_hello);
				usb.tx_offsets[usb.tx_bank] = len;
				struct sc_msg *rep = (struct sc_msg *)&usb.tx_buffers[usb.tx_bank][0];
				rep->id = SC_MSG_HELLO_HOST;
				rep->len = len;
				rep->u.hello.proto_version = SC_VERSION;
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
				rep->u.hello.byte_order = SC_BYTE_ORDER_BE;
#else
				rep->u.hello.byte_order = SC_BYTE_ORDER_LE;
#endif
				rep->u.hello.buffer_size = cpu_to_be16(BUFFER_SIZE);

				// don't process any more messages
				in_ptr = in_end;

				// assume in token is available
				// usb.has_in_token = true;
			} break;
			case SC_MSG_DEVICE_INFO: {
				TU_LOG2("SC_MSG_DEVICE_INFO\n");
				uint8_t bytes = SC_HEADER_LEN + sizeof(struct sc_msg_info);
				uint8_t *ptr;
				uint8_t *end;
send_info:
				ptr = usb.tx_buffers[usb.tx_bank] + usb.tx_offsets[usb.tx_bank];
				end = ptr + BUFFER_SIZE;
				if (end - ptr >= bytes) {
					usb.tx_offsets[usb.tx_bank] += bytes;
					struct sc_msg *rep = (struct sc_msg *)ptr;
					uint16_t u16;
					uint32_t u32;
					rep->id = SC_MSG_DEVICE_INFO;
					rep->len = bytes;
					rep->u.info.channels = TU_ARRAY_SIZE(cans.can);
					rep->u.info.features = SC_FEATURE_FLAG_CAN_FD | SC_FEATURE_FLAG_AUTO_RE | SC_FEATURE_FLAG_EH;
					u32 = CAN_CLK_HZ;
					memcpy(&rep->u.info.can_clk_hz, &u32, sizeof(rep->u.info.can_clk_hz));
					rep->u.info.nmbt_pre_min = M_CAN_NMBT_PRE_MIN;
					memcpy(&rep->u.info.nmbt_pre_max, &u16, sizeof(rep->u.info.nmbt_pre_max));
					u16 = M_CAN_NMBT_PRE_MAX;
					memcpy(&rep->u.info.nmbt_pre_max, &u16, sizeof(rep->u.info.nmbt_pre_max));
					rep->u.info.nmbt_tq_min = M_CAN_NMBT_TQ_MIN;
					u16 = M_CAN_NMBT_TQ_MAX;
					memcpy(&rep->u.info.nmbt_tq_max, &u16, sizeof(rep->u.info.nmbt_tq_max));
					rep->u.info.nmbt_tq_max = M_CAN_NMBT_TQ_MAX;
    				rep->u.info.nmbt_sjw_min = M_CAN_NMBT_SJW_MIN;
    				rep->u.info.nmbt_sjw_max = M_CAN_NMBT_SJW_MAX;
    				rep->u.info.nmbt_seg1_min = M_CAN_NMBT_SEG1_MIN;
					u16 = M_CAN_NMBT_SEG1_MAX;
					memcpy(&rep->u.info.nmbt_seg1_max, &u16, sizeof(rep->u.info.nmbt_seg1_max));
    				rep->u.info.nmbt_seg2_min = M_CAN_NMBT_SEG2_MIN;
    				rep->u.info.nmbt_seg2_max = M_CAN_NMBT_SEG2_MAX;
					rep->u.info.dtbt_pre_min = M_CAN_DTBT_PRE_MIN;
					rep->u.info.dtbt_pre_max = M_CAN_DTBT_PRE_MAX;
					rep->u.info.dtbt_tq_min = M_CAN_DTBT_TQ_MIN;
					rep->u.info.dtbt_tq_max = M_CAN_DTBT_TQ_MAX;
    				rep->u.info.dtbt_sjw_min = M_CAN_DTBT_SJW_MIN;
    				rep->u.info.dtbt_sjw_max = M_CAN_DTBT_SJW_MAX;
    				rep->u.info.dtbt_seg1_min = M_CAN_DTBT_SEG1_MIN;
    				rep->u.info.dtbt_seg1_max = M_CAN_DTBT_SEG1_MAX;
    				rep->u.info.dtbt_seg2_min = M_CAN_DTBT_SEG2_MIN;
    				rep->u.info.dtbt_seg2_max = M_CAN_DTBT_SEG2_MAX;
				} else {
					if (usb_bulk_in_ep_ready()) {
						usb_bulk_in_submit();
						// usb.has_in_token = false;
						// usbd_edpt_xfer(rhport, SC_M1_EP_BULK_IN, usb.tx_buffers[usb.tx_bank], usb.tx_offsets[usb.tx_bank]);
						// usb.tx_bank = !usb.tx_bank;
						goto send_info;
					} else {
						// ouch
					}
				}
			} break;
			case SC_MSG_BITTIMING: {
				TU_LOG2("SC_MSG_BITTIMING\n");
				if (msg->len < SC_HEADER_LEN + sizeof(struct sc_msg_bittiming)) {
					TU_LOG1("ERROR: SC_MSG_BITTIMING msg too short\n");
					continue;
				}

				if (msg->u.bittiming.channel >= 2) {
					TU_LOG1("ERROR: SC_MSG_BITTIMING channel %u out of range\n", msg->u.bittiming.channel);
					continue;
				}

				struct can *can = &cans.can[msg->u.bittiming.channel];
				// memcpy(&can->arbitration_bitrate_bps, &msg->u.bittiming.arbitration_bitrate_bps, sizeof(can->arbitration_bitrate_bps));
				// memcpy(&can->data_bitrate_bps, &msg->u.bittiming.data_bitrate_bps, sizeof(can->data_bitrate_bps));
				// can->arbitration_sjw = msg->u.bittiming.arbitration_sjw;
				// can->arbitration_sample_point = msg->u.bittiming.arbitration_sample_point;
				// can->data_sample_point = msg->u.bittiming.data_sample_point;
				uint16_t nmbt_pre, nmbt_seg1;
				uint8_t nmbt_sjw, nmbt_seg2, dtbt_pre, dtbt_sjw, dtbt_seg1, dtbt_seg2;

				memcpy(&nmbt_pre, &msg->u.bittiming.nmbt_pre, sizeof(nmbt_pre));
				memcpy(&nmbt_seg1, &msg->u.bittiming.nmbt_seg1, sizeof(nmbt_seg1));
				nmbt_sjw = msg->u.bittiming.nmbt_sjw;
				nmbt_seg2 = msg->u.bittiming.nmbt_seg2;
				dtbt_pre = msg->u.bittiming.dtbt_pre;
				dtbt_sjw = msg->u.bittiming.dtbt_sjw;
				dtbt_seg1 = msg->u.bittiming.dtbt_seg1;
				dtbt_seg2 = msg->u.bittiming.dtbt_seg2;
				// clamp
				can->nmbt_pre = tu_max16(M_CAN_NMBT_PRE_MIN, tu_min16(nmbt_pre, M_CAN_NMBT_PRE_MAX));
				can->nmbt_seg1 = tu_max16(M_CAN_NMBT_SEG1_MIN, tu_min16(nmbt_seg1, M_CAN_NMBT_SEG1_MAX));
				can->nmbt_sjw = tu_max8(M_CAN_NMBT_SJW_MIN, tu_min8(nmbt_sjw, M_CAN_NMBT_SJW_MAX));
				can->nmbt_seg2 = tu_max8(M_CAN_NMBT_SEG2_MIN, tu_min8(nmbt_seg2, M_CAN_NMBT_SEG2_MAX));
				can->dtbt_pre = tu_max8(M_CAN_DTBT_PRE_MIN, tu_min8(dtbt_pre, M_CAN_DTBT_PRE_MAX));
				can->dtbt_sjw = tu_max8(M_CAN_DTBT_SJW_MIN, tu_min8(dtbt_sjw, M_CAN_DTBT_SJW_MAX));
				can->dtbt_seg1 = tu_max8(M_CAN_DTBT_SEG1_MIN, tu_min8(dtbt_seg1, M_CAN_DTBT_SEG1_MAX));
				can->dtbt_seg2 = tu_max8(M_CAN_DTBT_SEG2_MIN, tu_min8(dtbt_seg2, M_CAN_DTBT_SEG2_MAX));

				can_configure(can);

			} break;
			case SC_MSG_RESET: {
				TU_LOG2("SC_MSG_RESET\n");
				NVIC_SystemReset();
			} break;
			case SC_MSG_MODE: {
				TU_LOG2("SC_MSG_MODE\n");

				if (msg->len < SC_HEADER_LEN + sizeof(struct sc_msg_config)) {
					TU_LOG1("ERROR: SC_MSG_MODE msg too short\n");
					continue;
				}

				if (msg->u.config.channel >= 2) {
					TU_LOG1("ERROR: SC_MSG_MODE channel %u out of range\n", msg->u.config.channel);
					continue;
				}

				struct can *can = &cans.can[msg->u.config.channel];
				can->mode_flags = msg->u.config.args[0];

				can_configure(can);

			} break;
			case SC_MSG_BUS: {
				TU_LOG2("SC_MSG_BUS\n");

				if (msg->len < SC_HEADER_LEN + sizeof(struct sc_msg_config)) {
					TU_LOG1("ERROR: SC_MSG_BUS msg too short\n");
					continue;
				}

				if (msg->u.config.channel >= 2) {
					TU_LOG1("ERROR: SC_MSG_BUS channel %u out of range\n", msg->u.config.channel);
					continue;
				}

				struct can *can = &cans.can[msg->u.config.channel];
				bool was_enabled = can->enabled;
				can->enabled = msg->u.config.args[0] != 0;
				if (was_enabled != can->enabled) {
					TU_LOG2("channel %u enabled=%u\n", msg->u.config.channel, can->enabled);
					can_set_state1(can->m_can, can->interrupt_id, can->enabled);
				}
			} break;
			case SC_MSG_CAN_TX: {
				TU_LOG2("SC_MSG_CAN_TX\n");

				if (msg->len < SC_HEADER_LEN + sizeof(struct sc_msg_can_tx)) {
					TU_LOG1("ERROR: SC_MSG_CAN_TX msg too short\n");
					continue;
				}

				struct sc_msg_can_tx const *tx = &msg->u.tx;
				if (tx->channel >= 2) {
					TU_LOG1("ERROR: SC_MSG_CAN_TX channel %u out of range\n", tx->channel);
					continue;
				}

				const uint8_t can_frame_len = dlc_to_len(tx->dlc);
				if (!(tx->flags & SC_CAN_FLAG_RTR)) {
					if (msg->len < SC_HEADER_LEN + sizeof(struct sc_msg_can_tx) + can_frame_len) {
						TU_LOG1("ERROR: SC_MSG_CAN_TX msg too short\n");
						continue;
					}
				}

				struct can *can = &cans.can[tx->channel];

				if (can->m_can->TXFQS.bit.TFQF) {
					++can->tx_dropped;
					uint8_t *ptr;
					uint8_t *end;
send_txr:
					ptr = usb.tx_buffers[usb.tx_bank] + usb.tx_offsets[usb.tx_bank];
					end = ptr + BUFFER_SIZE;

					uint8_t bytes = SC_HEADER_LEN + sizeof(struct sc_msg_can_txr);
					if (end - ptr >= bytes) {
						usb.tx_offsets[usb.tx_bank] += bytes;

						struct sc_msg *rep = (struct sc_msg *)ptr;
						rep->id = SC_MSG_CAN_TXR;
						rep->len = bytes;
						rep->u.txr.channel = tx->channel;
						memcpy(&rep->u.txr.track_id, &tx->track_id, sizeof(rep->u.txr.track_id));
						uint32_t ts = can->ts_high | can->m_can->TSCV.bit.TSC;
						memcpy(&rep->u.txr.timestamp, &ts, sizeof(rep->u.txr.timestamp));
						rep->u.txr.flags = SC_CAN_FLAG_DRP;
					} else {
						if (usb_bulk_in_ep_ready()) {
							usb_bulk_in_submit();
							// usb.has_in_token = false;
							// usbd_edpt_xfer(rhport, SC_M1_EP_BULK_IN, usb.tx_buffers[usb.tx_bank], usb.tx_offsets[usb.tx_bank]);
							// usb.tx_bank = !usb.tx_bank;
							goto send_txr;
						} else {
							TU_LOG1("ch %u: desync\n", tx->channel);
							can->desync = true;
						}
					}
				} else {
					uint32_t id;
					memcpy(&id, &tx->can_id, sizeof(id));
					uint16_t tid;
					memcpy(&tid, &tx->track_id, sizeof(tid));

					uint8_t index = can->m_can->TXFQS.bit.TFQPI;
					uint8_t marker_index = can->tx_fifo[index].T1.bit.MM;
					can->message_marker_map[marker_index] = tid; // save message marker

					CAN_TXBE_0_Type t0;
					t0.reg = (((tx->flags & SC_CAN_FLAG_ESI) == SC_CAN_FLAG_ESI) << CAN_TXBE_0_ESI_Pos)
						| (((tx->flags & SC_CAN_FLAG_RTR) == SC_CAN_FLAG_RTR) << CAN_TXBE_0_RTR_Pos)
						| (((tx->flags & SC_CAN_FLAG_EXT) == SC_CAN_FLAG_EXT) << CAN_TXBE_0_XTD_Pos)
						;


					if (tx->flags & SC_CAN_FLAG_EXT) {
						t0.reg |= CAN_TXBE_0_ID(id);
					} else {
						t0.reg |= CAN_TXBE_0_ID(id << 18);
					}

					can->tx_fifo[index].T0 = t0;
					can->tx_fifo[index].T1.bit.DLC = tx->dlc;
					can->tx_fifo[index].T1.bit.FDF = (tx->flags & SC_CAN_FLAG_FDF) == SC_CAN_FLAG_FDF;
					can->tx_fifo[index].T1.bit.BRS = (tx->flags & SC_CAN_FLAG_BRS) == SC_CAN_FLAG_BRS;

					if (!(tx->flags & SC_CAN_FLAG_RTR)) {
						if (can_frame_len) {
							memcpy(can->tx_fifo[index].data, tx->data, can_frame_len);
						}
					}

					can->m_can->TXBAR.reg = 1UL << index;
				}
			} break;

			default:
				TU_LOG2_MEM(msg, msg->len, 2);
				break;
			}
		}

		if (usb.tx_offsets[usb.tx_bank] > 0 && usb_bulk_in_ep_ready()) {
			TU_LOG2("usb tx %u bytes\n", usb.tx_offsets[usb.tx_bank]);
			usb_bulk_in_submit();
			// usb.has_in_token = false;
			// usbd_edpt_xfer(rhport, SC_M1_EP_BULK_IN, usb.tx_buffers[usb.tx_bank], usb.tx_offsets[usb.tx_bank]);
			// usb.tx_bank = !usb.tx_bank;
		}

		// start new transaction
		usbd_edpt_xfer(rhport, ep_addr, usb.rx_buffer, TU_ARRAY_SIZE(usb.rx_buffer));
	} break;
	case SC_M1_EP_BULK_IN: {
		TU_ASSERT(!usbd_edpt_busy(rhport, ep_addr), false);
		TU_LOG2("< cmd IN token\n");

		// mark previous bank as free
		usb.tx_offsets[!usb.tx_bank] = 0;

		if (usb.tx_offsets[usb.tx_bank]) {
			// send off current bank data
			TU_LOG2("usb tx %u bytes\n", usb.tx_offsets[usb.tx_bank]);

			usb_bulk_in_submit();
			// usbd_edpt_xfer(usb.port, SC_M1_EP_BULK_IN, usb.tx_buffers[usb.tx_bank], usb.tx_offsets[usb.tx_bank]);
			// usb.tx_bank = !usb.tx_bank;
			// usb.has_in_token = false;
		} else {
			// usb.has_in_token = true;
			// uint8_t len = SC_HEADER_LEN + sizeof(struct sc_msg_can_rx) + 1;
			// struct sc_msg * rep = (struct sc_msg *)usb.tx_buffers[usb.tx_bank];
			// rep->id = SC_MSG_CAN_RX;
			// rep->len = len;
			// rep->u.rx.channel = 0;
			// rep->u.rx.dlc = 1;
			// rep->u.rx.can_id = 1;
			// rep->u.rx.data[0] = 42;

			// usb.tx_offsets[usb.tx_bank] = len;

			// usbd_edpt_xfer(rhport, ep_addr, usb.tx_buffers[usb.tx_bank], usb.tx_offsets[usb.tx_bank]);
			// usb.tx_bank = !usb.tx_bank;
		}

		// wake can task now that we can send to host
		//xTaskNotifyGive(cans.task);
	} break;
	default:
		TU_LOG2("port %u ep %02x event %d bytes %u\n", rhport, ep_addr, event, (unsigned)xferred_bytes);
		return false;
	}

	return true;
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
	(void) param;

	TU_LOG2("can task start\n");

	while (42) {
		// (void)ulTaskNotifyTake(pdFALSE, ~0);

		// TU_LOG2("can task loop\n");
		// led_burst(LED_GREEN1, 1000);
		vTaskDelay(1);

		if (unlikely(!usb.mounted)) {
			continue;
		}

		// continue;

		for (bool done = false; !done; ) {
			done = true;
			uint8_t *ptr;
			uint8_t *end;
start:
			ptr = usb.tx_buffers[usb.tx_bank] + usb.tx_offsets[usb.tx_bank];
			end = ptr + BUFFER_SIZE;

			if (!usb.tx_offsets[usb.tx_bank]) {

				// place status messages
				for (size_t i = 0; i < TU_ARRAY_SIZE(cans.can); ++i) {
					struct can *can = &cans.can[i];

					if (!can->enabled) {
						continue;
					}

					done = false;

					struct sc_msg *msg = (struct sc_msg *)ptr;
					msg->len = SC_HEADER_LEN + sizeof(struct sc_msg_status);
					msg->id = SC_MSG_CAN_STATUS;
					msg->u.status.channel = i;
					uint32_t ts = can->ts_high | can->m_can->TSCV.bit.TSC;
					memcpy(&msg->u.status.timestamp, &ts, sizeof(ts));
					memcpy(&msg->u.status.rx_lost, &can->rx_lost, sizeof(can->rx_lost));
					memcpy(&msg->u.status.tx_dropped, &can->tx_dropped, sizeof(can->tx_dropped));
					msg->u.status.flags = can->status_flags;
					if (can->desync) {
						msg->u.status.flags |= SC_STATUS_FLAG_TX_DESYNC;
					}
					can->rx_lost = 0;
					can->tx_dropped = 0;
					ptr += msg->len;
				}
			}

			for (size_t i = 0; i < TU_ARRAY_SIZE(cans.can); ++i) {
				struct can *can = &cans.can[i];

				if (!can->enabled) {
					continue;
				}

				if (m_can_rx0_msg_fifo_avail(can->m_can)) {
					done = false;
					uint8_t index = can->m_can->RXF0S.bit.F0GI;
					uint8_t bytes = SC_HEADER_LEN + sizeof(struct sc_msg_can_rx);
					CAN_RXF0E_0_Type r0 = can->rx_fifo[index].R0;
					CAN_RXF0E_1_Type r1 = can->rx_fifo[index].R1;
					uint8_t can_frame_len = dlc_to_len(r1.bit.DLC);
					if (!r0.bit.RTR) {
						bytes += can_frame_len;
					}

					if (end - ptr >= bytes) {
						struct sc_msg *msg = (struct sc_msg *)ptr;
						ptr += bytes;

						msg->id = SC_MSG_CAN_RX;
						msg->len = bytes;
						msg->u.rx.channel = i;
						msg->u.rx.flags = 0;
						uint32_t id = r0.bit.ID;
						if (r0.bit.XTD) {
							msg->u.rx.flags |= SC_CAN_FLAG_EXT;
						} else {
							id >>= 18;
						}
						memcpy(&msg->u.rx.can_id, &id, sizeof(id));

						uint32_t ts = can->ts_high | r1.bit.RXTS;
						memcpy(&msg->u.rx.timestamp, &ts, sizeof(ts));

						if (r1.bit.FDF) {
							msg->u.rx.flags |= SC_CAN_FLAG_FDF;
						}
						if (r1.bit.BRS) {
							msg->u.rx.flags |= SC_CAN_FLAG_BRS;
						}
						if (r0.bit.RTR) {
							msg->u.rx.flags |= SC_CAN_FLAG_RTR;
						} else {
							memcpy(msg->u.rx.data, can->rx_fifo[index].data, can_frame_len);
						}

						msg->u.rx.dlc = r1.bit.DLC;
					} else {
						if (usb_bulk_in_ep_ready()) {
							usb.tx_offsets[usb.tx_bank] = ptr - usb.tx_buffers[usb.tx_bank];
							usb_bulk_in_submit();
						// if (usb.has_in_token) {
						// 	usb.has_in_token = false;
						// 	usb.tx_offsets[usb.tx_bank] = ptr - usb.tx_buffers[usb.tx_bank];
						// 	usbd_edpt_xfer(usb.port, SC_M1_EP_BULK_IN, usb.tx_buffers[usb.tx_bank], usb.tx_offsets[usb.tx_bank]);
						// 	usb.tx_bank = !usb.tx_bank;
							goto start;
						} else {
							++can->rx_lost;
						}
					}

					can->m_can->RXF0A.reg = CAN_RXF0A_F0AI(index);
				}

				if (m_can_tx_event_fifo_avail(can->m_can)) {
					done = false;
					uint8_t index = can->m_can->TXEFS.bit.EFGI;
					uint8_t bytes = SC_HEADER_LEN + sizeof(struct sc_msg_can_txr);
					CAN_TXEFE_0_Type t0 = can->tx_event_fifo[index].T0;
					CAN_TXEFE_1_Type t1 = can->tx_event_fifo[index].T1;

					if (end - ptr >= bytes) {
						struct sc_msg *msg = (struct sc_msg *)ptr;
						ptr += bytes;

						uint8_t marker_index = t1.bit.MM;

						msg->id = SC_MSG_CAN_TXR;
						msg->len = bytes;
						msg->u.txr.channel = i;
						memcpy(&msg->u.txr.track_id, &can->message_marker_map[marker_index], sizeof(msg->u.txr.track_id));
						uint32_t ts = can->ts_high | t1.bit.TXTS;
						memcpy(&msg->u.txr.timestamp, &ts, sizeof(msg->u.txr.timestamp));
						msg->u.txr.flags = 0;
						if (t0.bit.ESI) {
							msg->u.txr.flags |= SC_CAN_FLAG_ESI;
						}

						if (t1.bit.FDF) {
							msg->u.txr.flags |= SC_CAN_FLAG_FDF;
						}

						if (t1.bit.BRS) {
							msg->u.txr.flags |= SC_CAN_FLAG_BRS;
						}
					} else {
						if (usb_bulk_in_ep_ready()) {
							usb.tx_offsets[usb.tx_bank] = ptr - usb.tx_buffers[usb.tx_bank];
							usb_bulk_in_submit();
						// if (usb.has_in_token) {
						// 	usb.has_in_token = false;
						// 	usb.tx_offsets[usb.tx_bank] = ptr - usb.tx_buffers[usb.tx_bank];
						// 	usbd_edpt_xfer(usb.port, SC_M1_EP_BULK_IN, usb.tx_buffers[usb.tx_bank], usb.tx_offsets[usb.tx_bank]);
						// 	usb.tx_bank = !usb.tx_bank;
							goto start;
						} else {
							can->desync = true;
						}
					}

					can->m_can->TXEFA.reg = CAN_TXEFA_EFAI(index);
				}
			}

			usb.tx_offsets[usb.tx_bank] = ptr - usb.tx_buffers[usb.tx_bank];
		}


		if (true) { // check if we should send

		}
	}
}