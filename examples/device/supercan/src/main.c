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
//#include <usbd_pvt.h>

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
	uint8_t option_flags;
	uint16_t rx_lost;
	uint16_t tx_dropped;
	volatile uint8_t status_flags;
	uint8_t led;
	bool enabled;
	bool desync;
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
		CAN_IE_TSWE 	// time stamp counter wrap
		| CAN_IE_BOE 	// bus off
		| CAN_IE_EWE 	// error warning
		| CAN_IE_EPE 	// error passive
		| CAN_IE_RF0NE 	// new message in rx fifo0
		| CAN_IE_RF0LE  // message lost b/c fifo0 was full
		;

	if (c->option_flags & SC_OPTION_TXR) {
		can->IE.reg |= CAN_IE_TEFNE; 	// new message in tx event fifo
	}

	m_can_conf_end(can);
	// m_can_init_end(can);
}

static void can_init_module();
static void can_init_module()
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

	bool notify = false;
	CAN_IR_Type ir = can->m_can->IR;
	if (ir.bit.TSW) {
		uint32_t ts_high = __sync_add_and_fetch(&can->ts_high, 1u << M_CAN_TS_COUNTER_BITS);
		(void)ts_high;
		// TU_LOG2("CAN%u ts_high %08x\n", index, ts_high);
	}

	uint8_t prev_status = can->status_flags;
	uint8_t curr_status = 0;

	// bool had_ep = (prev_status & SC_STATUS_FLAG_ERROR_PASSIVE) == SC_STATUS_FLAG_ERROR_PASSIVE;
	// notify |= had_ep != ir.bit.EP;
	if (ir.bit.EP) {
		TU_LOG2("CAN%u error passive\n", index);
		curr_status |= SC_STATUS_FLAG_ERROR_PASSIVE;
	}

	// bool had_ew = (prev_status & SC_STATUS_FLAG_ERROR_WARNING) == SC_STATUS_FLAG_ERROR_WARNING;
	// notify |= had_ew != ir.bit.EW;
	if (ir.bit.EW) {
		TU_LOG2("CAN%u error warning\n", index);
		curr_status |= SC_STATUS_FLAG_ERROR_WARNING;
	}

	// bool had_bo = (prev_status & SC_STATUS_FLAG_BUS_OFF) == SC_STATUS_FLAG_BUS_OFF;
	// notify |= had_bo != ir.bit.BO;
	if (ir.bit.BO) {
		TU_LOG2("CAN%u bus off\n", index);
		curr_status |= SC_STATUS_FLAG_BUS_OFF;
	}

	// bool had_rx_fifo0_msg_lost = (prev_status & SC_STATUS_FLAG_RX_FULL) == SC_STATUS_FLAG_RX_FULL;
	// notify |= had_rx_fifo0_msg_lost != ir.bit.RF0L;
	if (ir.bit.RF0L) {
		TU_LOG2("CAN%u msg lost\n", index);
		curr_status |= SC_STATUS_FLAG_RX_FULL;
	}

	notify = can->status_flags != curr_status;

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

#define BUFFER_SIZE 512

static struct usb {
	CFG_TUSB_MEM_ALIGN uint8_t rx_buffers[2][BUFFER_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t tx_buffers[2][BUFFER_SIZE];
	uint16_t rx_offsets[2];
	uint16_t tx_offsets[2];
	uint8_t rx_bank;
	uint8_t tx_bank;
	uint8_t port;
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
	(void)dcd_edpt_xfer(usb.port, SC_M1_EP_BULK_IN, usb.tx_buffers[usb.tx_bank], usb.tx_offsets[usb.tx_bank]);
	usb.tx_bank = !usb.tx_bank;
}


int main(void)
{
	board_init();
	led_init();

	tusb_init();

	can_init_pins();
	can_init_clock();
	can_init_module();

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

	if (unlikely(rhport != usb.port)) {
		return false;
	}

	uint8_t const *ptr = (void const *)desc_intf;

	ptr += 9;

	for (uint8_t i = 0; i < 2; ++i) {
		TU_ASSERT(dcd_edpt_open(rhport, (tusb_desc_endpoint_t const *)(ptr + i * 7)));
	}

	TU_ASSERT(dcd_edpt_xfer(rhport, SC_M1_EP_BULK_OUT, usb.rx_buffers[usb.rx_bank], BUFFER_SIZE));


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

	if (unlikely(rhport != usb.port)) {
		return false;
	}

	switch (ep_addr) {
	case SC_M1_EP_BULK_OUT: {
		uint8_t const *in_ptr = usb.rx_buffers[usb.rx_bank];
		uint8_t const * const in_end = in_ptr + xferred_bytes;

		// setup next transfer
		usb.rx_bank = !usb.rx_bank;
		(void)dcd_edpt_xfer(rhport, ep_addr, usb.rx_buffers[usb.rx_bank], BUFFER_SIZE);

		// process messages
		while (in_ptr + SC_HEADER_LEN <= in_end) {
			struct sc_msg_header const *msg = (struct sc_msg_header const *)in_ptr;
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
				uint8_t len = sizeof(struct sc_msg_hello);
				usb.tx_offsets[usb.tx_bank] = len;
				struct sc_msg_hello *rep = (struct sc_msg_hello *)&usb.tx_buffers[usb.tx_bank][0];
				rep->id = SC_MSG_HELLO_HOST;
				rep->len = len;
				rep->proto_version = SC_VERSION;
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
				rep->byte_order = SC_BYTE_ORDER_BE;
#else
				rep->byte_order = SC_BYTE_ORDER_LE;
#endif
				rep->buffer_size = cpu_to_be16(BUFFER_SIZE);

				// don't process any more messages
				in_ptr = in_end;

				// assume in token is available
			} break;
			case SC_MSG_DEVICE_INFO: {
				TU_LOG2("SC_MSG_DEVICE_INFO\n");
				uint8_t bytes = sizeof(struct sc_msg_info);
				uint8_t *ptr;
				uint8_t *end;
send_info:
				ptr = usb.tx_buffers[usb.tx_bank] + usb.tx_offsets[usb.tx_bank];
				end = ptr + BUFFER_SIZE;
				if (end - ptr >= bytes) {
					usb.tx_offsets[usb.tx_bank] += bytes;
					struct sc_msg_info *rep = (struct sc_msg_info *)ptr;
					rep->id = SC_MSG_DEVICE_INFO;
					rep->len = bytes;
					rep->channels = TU_ARRAY_SIZE(cans.can);
					rep->features = SC_FEATURE_FLAG_CAN_FD | SC_FEATURE_FLAG_AUTO_RE | SC_FEATURE_FLAG_EH;
					rep->can_clk_hz = CAN_CLK_HZ;
					rep->nmbt_brp_min = M_CAN_NMBT_BRP_MIN;
					rep->nmbt_brp_max = M_CAN_NMBT_BRP_MAX;
					rep->nmbt_tq_min = M_CAN_NMBT_TQ_MIN;
					rep->nmbt_tq_max = M_CAN_NMBT_TQ_MAX;
					rep->nmbt_tq_max = M_CAN_NMBT_TQ_MAX;
    				rep->nmbt_sjw_min = M_CAN_NMBT_SJW_MIN;
    				rep->nmbt_sjw_max = M_CAN_NMBT_SJW_MAX;
    				rep->nmbt_tseg1_min = M_CAN_NMBT_TSEG1_MIN;
					rep->nmbt_tseg1_max = M_CAN_NMBT_TSEG1_MAX;
    				rep->nmbt_tseg2_min = M_CAN_NMBT_TSEG2_MIN;
    				rep->nmbt_tseg2_max = M_CAN_NMBT_TSEG2_MAX;
					rep->dtbt_brp_min = M_CAN_DTBT_BRP_MIN;
					rep->dtbt_brp_max = M_CAN_DTBT_BRP_MAX;
					rep->dtbt_tq_min = M_CAN_DTBT_TQ_MIN;
					rep->dtbt_tq_max = M_CAN_DTBT_TQ_MAX;
    				rep->dtbt_sjw_min = M_CAN_DTBT_SJW_MIN;
    				rep->dtbt_sjw_max = M_CAN_DTBT_SJW_MAX;
    				rep->dtbt_tseg1_min = M_CAN_DTBT_TSEG1_MIN;
    				rep->dtbt_tseg1_max = M_CAN_DTBT_TSEG1_MAX;
    				rep->dtbt_tseg2_min = M_CAN_DTBT_TSEG2_MIN;
    				rep->dtbt_tseg2_max = M_CAN_DTBT_TSEG2_MAX;
				} else {
					if (usb_bulk_in_ep_ready()) {
						usb_bulk_in_submit();
						goto send_info;
					} else {
						// ouch
					}
				}
			} break;
			case SC_MSG_BITTIMING: {
				TU_LOG2("SC_MSG_BITTIMING\n");
				struct sc_msg_bittiming const *tmsg = (struct sc_msg_bittiming const *)msg;
				if (unlikely(msg->len < sizeof(*tmsg))) {
					TU_LOG1("ERROR: msg too short\n");
					continue;
				}

				if (unlikely(tmsg->channel >= TU_ARRAY_SIZE(cans.can))) {
					TU_LOG1("ERROR: channel %u out of range\n", tmsg->channel);
					continue;
				}

				struct can *can = &cans.can[tmsg->channel];

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

				// can_configure(can);

			} break;
			case SC_MSG_RESET: {
				TU_LOG2("SC_MSG_RESET\n");
				NVIC_SystemReset();
			} break;
			case SC_MSG_MODE: {
				TU_LOG2("SC_MSG_MODE\n");
				struct sc_msg_config const *tmsg = (struct sc_msg_config const *)msg;
				if (unlikely(msg->len < sizeof(*tmsg))) {
					TU_LOG1("ERROR: msg too short\n");
					continue;
				}

				if (unlikely(tmsg->channel >= TU_ARRAY_SIZE(cans.can))) {
					TU_LOG1("ERROR: channel %u out of range\n", tmsg->channel);
					continue;
				}

				struct can *can = &cans.can[tmsg->channel];
				can->mode_flags = tmsg->args[0];

				// can_configure(can);

			} break;
			case SC_MSG_OPTIONS: {
				TU_LOG2("SC_MSG_OPTIONS\n");
				struct sc_msg_config const *tmsg = (struct sc_msg_config const *)msg;

				if (unlikely(msg->len < sizeof(*tmsg))) {
					TU_LOG1("ERROR: msg too short\n");
					continue;
				}

				if (unlikely(tmsg->channel >= TU_ARRAY_SIZE(cans.can))) {
					TU_LOG1("ERROR: channel %u out of range\n", tmsg->channel);
					continue;
				}

				struct can *can = &cans.can[tmsg->channel];
				can->option_flags = tmsg->args[0];
			} break;
			case SC_MSG_BUS: {
				TU_LOG2("SC_MSG_BUS\n");
				struct sc_msg_config const *tmsg = (struct sc_msg_config const *)msg;

				if (unlikely(msg->len < sizeof(*tmsg))) {
					TU_LOG1("ERROR: msg too short\n");
					continue;
				}

				if (unlikely(tmsg->channel >= TU_ARRAY_SIZE(cans.can))) {
					TU_LOG1("ERROR: channel %u out of range\n", tmsg->channel);
					continue;
				}

				struct can *can = &cans.can[tmsg->channel];

				bool was_enabled = can->enabled;
				can->enabled = tmsg->args[0] != 0;
				if (was_enabled != can->enabled) {
					TU_LOG2("channel %u enabled=%u\n", tmsg->channel, can->enabled);
					if (can->enabled) {
						can_configure(can);
					}
					can_set_state1(can->m_can, can->interrupt_id, can->enabled);
				}
			} break;
			case SC_MSG_CAN_TX: {
				TU_LOG2("SC_MSG_CAN_TX\n");
				struct sc_msg_can_tx const *tmsg = (struct sc_msg_can_tx const *)msg;
				if (unlikely(msg->len < sizeof(*tmsg))) {
					TU_LOG1("ERROR: msg too short\n");
					continue;
				}

				if (unlikely(tmsg->channel >= TU_ARRAY_SIZE(cans.can))) {
					TU_LOG1("ERROR: channel %u out of range\n", tmsg->channel);
					continue;
				}

				const uint8_t can_frame_len = dlc_to_len(tmsg->dlc);
				if (!(tmsg->flags & SC_CAN_FLAG_RTR)) {
					if (msg->len < sizeof(*tmsg) + can_frame_len) {
						TU_LOG1("ERROR: msg too short\n");
						continue;
					}
				}

				struct can *can = &cans.can[tmsg->channel];

				if (can->m_can->TXFQS.bit.TFQF) {
					++can->tx_dropped;
					uint8_t *ptr;
					uint8_t *end;

					if (can->option_flags & SC_OPTION_TXR) {
send_txr:
						ptr = usb.tx_buffers[usb.tx_bank] + usb.tx_offsets[usb.tx_bank];
						end = ptr + BUFFER_SIZE;

						uint8_t bytes = sizeof(struct sc_msg_can_txr);
						if (end - ptr >= bytes) {
							usb.tx_offsets[usb.tx_bank] += bytes;

							struct sc_msg_can_txr *rep = (struct sc_msg_can_txr *)ptr;
							rep->id = SC_MSG_CAN_TXR;
							rep->len = bytes;
							rep->channel = tmsg->channel;
							rep->track_id = tmsg->track_id;
							uint32_t ts = can->ts_high | can->m_can->TSCV.bit.TSC;
							rep->timestamp_us = can_time_to_us(can, ts);
							rep->flags = SC_CAN_FLAG_DRP;
						} else {
							if (usb_bulk_in_ep_ready()) {
								usb_bulk_in_submit();
								goto send_txr;
							} else {
								TU_LOG1("ch %u: desync\n", tmsg->channel);
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

		if (usb.tx_offsets[usb.tx_bank] > 0 && usb_bulk_in_ep_ready()) {
			// TU_LOG2("usb tx %u bytes\n", usb.tx_offsets[usb.tx_bank]);
			usb_bulk_in_submit();
		}
	} break;
	case SC_M1_EP_BULK_IN: {
		// TU_ASSERT(!usbd_edpt_busy(rhport, ep_addr), false);
		TU_LOG2("< IN token\n");

		// mark previous bank as free
		usb.tx_offsets[!usb.tx_bank] = 0;

		if (usb.tx_offsets[usb.tx_bank]) {
			// send off current bank data
			// TU_LOG2("usb tx %u bytes\n", usb.tx_offsets[usb.tx_bank]);

			usb_bulk_in_submit();

		}
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
	const uint8_t index = (uint8_t)(uintptr_t)param;
	TU_ASSERT(index < TU_ARRAY_SIZE(cans.can), );
//	TU_ASSERT(index < TU_ARRAY_SIZE(usb.can), );

	TU_LOG2("CAN%u task start\n", index);

	struct can *can = &cans.can[index];

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
			ptr = usb.tx_buffers[usb.tx_bank] + usb.tx_offsets[usb.tx_bank];
			end = ptr + BUFFER_SIZE;

			if (!usb.tx_offsets[usb.tx_bank]) {

				// place status messages

				done = false;

				uint32_t ts = can->ts_high | can->m_can->TSCV.bit.TSC;
				uint32_t us = can_time_to_us(can, ts);

				struct sc_msg_status *msg = (struct sc_msg_status *)ptr;
				msg->len = sizeof(struct sc_msg_status);
				msg->id = SC_MSG_CAN_STATUS;
				msg->channel = index;
				msg->timestamp_us = us;
				msg->rx_lost = can->rx_lost;
				msg->tx_dropped = can->tx_dropped;
				msg->flags = can->status_flags;
				if (can->tx_dropped) {
					msg->flags |= SC_STATUS_FLAG_TX_FULL;
				}
				if (can->desync) {
					msg->flags |= SC_STATUS_FLAG_TX_DESYNC;
				}
				can->rx_lost = 0;
				can->tx_dropped = 0;
				ptr += msg->len;
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

				if (end - ptr >= bytes) {
					struct sc_msg_can_rx *msg = (struct sc_msg_can_rx *)ptr;
					ptr += bytes;

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
					if (usb_bulk_in_ep_ready()) {
						usb.tx_offsets[usb.tx_bank] = ptr - usb.tx_buffers[usb.tx_bank];
						usb_bulk_in_submit();
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
				if (can->option_flags & SC_OPTION_TXR) {
					uint8_t bytes = sizeof(struct sc_msg_can_txr);
					CAN_TXEFE_0_Type t0 = can->tx_event_fifo[get_index].T0;
					CAN_TXEFE_1_Type t1 = can->tx_event_fifo[get_index].T1;

					if (end - ptr >= bytes) {
						struct sc_msg_can_txr *msg = (struct sc_msg_can_txr *)ptr;
						ptr += bytes;

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
						if (usb_bulk_in_ep_ready()) {
							usb.tx_offsets[usb.tx_bank] = ptr - usb.tx_buffers[usb.tx_bank];
							usb_bulk_in_submit();
							goto start;
						} else {
							can->desync = true;
						}
					}
				}

				can->m_can->TXEFA.reg = CAN_TXEFA_EFAI(get_index);
			}

			usb.tx_offsets[usb.tx_bank] = ptr - usb.tx_buffers[usb.tx_bank];
		}


		if (usb.tx_offsets[usb.tx_bank] > 0 && usb_bulk_in_ep_ready()) {
			usb_bulk_in_submit();
		}
	}
}
