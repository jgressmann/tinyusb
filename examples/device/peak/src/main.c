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

#include <peak.h>




#define TOE_RB_SIZE_TYPE uint8_t
#define TOE_RB_ASSERT(x) \
	do { \
		if (!(x)) { \
			TU_LOG1("%s(%d): assertion failed '%s'\n", __FILE__, __LINE__, # x); \
			while (1); \
		} \
	} while (0)

#define TOE_RB_PREFIX pc
#define TOE_RB_VALUE_TYPE struct peak_cmd
#include "ring_buffer.h"
#undef TOE_RB_PREFIX
#undef TOE_RB_VALUE_TYPE

TU_VERIFY_STATIC(sizeof(struct peak_cmd) == 16, "");


#ifndef likely
#define likely(x) __builtin_expect((x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect((x),0)
#endif

#define CAN0_TX_BUFFER_NUM 32
#define CAN0_RX_FIFO_NUM 64
#define CAN0_ELEMENT_DATA_SIZE 8

struct can_tx_element
{
	volatile CAN_TXBE_0_Type T0;
	volatile CAN_TXBE_1_Type T1;
	uint8_t data[CAN0_ELEMENT_DATA_SIZE];
};

struct can_rx_fifo_element
{
	volatile CAN_RXF0E_0_Type R0;
	volatile CAN_RXF0E_1_Type R1;
	uint8_t data[CAN0_ELEMENT_DATA_SIZE];
};

CFG_TUSB_MEM_ALIGN
static struct can_tx_element can0_tx_buffer[CAN0_TX_BUFFER_NUM];

CFG_TUSB_MEM_ALIGN
static struct can_rx_fifo_element can0_rx_fifo[CAN0_RX_FIFO_NUM];


static inline void init_can0_pins(void) // controller and hardware specific setup of i/o pins for CAN
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


static inline void init_can0_clock(void) // controller and hardware specific setup of clock for the m_can module
{
	REG_MCLK_AHBMASK |= MCLK_AHBMASK_CAN0;
	REG_GCLK_PCHCTRL27 = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN0 to use GLCK0 -> 120MHz
}

static inline void configure_can0(uint32_t nbtp)
{
	REG_CAN0_CCCR |= CAN_CCCR_CCE; // set CCE bit to change config
	while((REG_CAN0_CCCR & CAN_CCCR_CCE) == 0);

	//REG_CAN0_CCCR |= CAN_CCCR_BRSE | CAN_CCCR_FDOE;// enable bit rate switching and FD operation
	/* TX Buffer Configuration */
	//REG_CAN0_TXBC = CAN_TXBC_TBSA((uint32_t) can0_tx_buffer) | CAN_TXBC_NDTB(CAN0_TX_BUFFER_NUM); // dedicated tx buffers
	REG_CAN0_TXBC = CAN_TXBC_TBSA((uint32_t) can0_tx_buffer) | CAN_TXBC_TFQS(CAN0_TX_BUFFER_NUM);
	//	REG_CAN0_TXBC |= CAN_TXBC_TFQM; // reset default
	REG_CAN0_TXESC = CAN_TXESC_TBDS_DATA8; // 8 byte data field

	/* RX FIFO Configuration */
	REG_CAN0_RXF0C = CAN_RXF0C_F0SA((uint32_t) can0_rx_fifo) | CAN_RXF0C_F0S(CAN0_RX_FIFO_NUM) | CAN_RXF0C_F0OM; // FIFO 0 overwrite mode
	REG_CAN0_RXESC = CAN_RXESC_RBDS_DATA8 + CAN_RXESC_F0DS_DATA8; // Buffer and FIFO Element size config

	/* ID filter buffer configuration */
	//REG_CAN0_SIDFC = CAN_SIDFC_FLSSA((uint32_t) can0_standard_id_filter) | CAN_SIDFC_LSS(CAN0_STANDARD_ID_FILTER_NUM);

	/* Timing setting. */
	//	REG_CAN0_NBTP =	CAN_NBTP_NBRP(0) | CAN_NBTP_NTSEG1(24) | CAN_NBTP_NTSEG2(5) |	CAN_NBTP_NSJW(5);
	//	REG_CAN0_DBTP = CAN_DBTP_DBRP(0) | CAN_DBTP_DTSEG1(4) | CAN_DBTP_DTSEG2(1) | CAN_DBTP_DSJW(1);
	//REG_CAN0_NBTP = CAN_NBTP_NBRP(2) | CAN_NBTP_NTSEG1(62) | CAN_NBTP_NTSEG2(15) | CAN_NBTP_NSJW (15); /* 500kBit @ 120 / 3 = 40MHz, 80% */
	// REG_CAN0_DBTP = CAN_DBTP_DBRP(2) | CAN_DBTP_DTSEG1(12) | CAN_DBTP_DTSEG2(5) | CAN_DBTP_DSJW (5); /* 2MBit @ 120 / 3 = 40MHz, 70% */
	REG_CAN0_NBTP = nbtp;

	// REG_CAN0_RWD = CAN_RWD_WDC(0); // disable Watchdog, Reset-default
	REG_CAN0_CCCR |= CAN_CCCR_TXP; // Enable Transmit Pause
	REG_CAN0_CCCR |= CAN_CCCR_EFBI; // Enable Edge Filtering
	// REG_CAN0_CCCR |= CAN_CCCR_PXHD; // Protocoll Exception Handling disabled - no error frames
	//	REG_CAN0_CCCR |= CAN_CCCR_DAR; // Disable Automatic Retransmission
	//	REG_CAN0_CCCR |= CAN_CCCR_CSR; // Clock Stop Request
	//	REG_CAN0_CCCR |= CAN_CCCR_CSA; // Clock Stop Acknowledge
	REG_CAN0_TSCC = CAN_TSCC_TCP(0) | CAN_TSCC_TSS(1);
	REG_CAN0_TOCC = CAN_TOCC_TOP(0xffff) | CAN_TOCC_TOS(0); // Timeout Counter disabled, Reset-default
	// REG_CAN0_TDCR = CAN_TDCR_TDCO(0) | CAN_TDCR_TDCF(0); // Reset-default
	//REG_CAN0_GFC =	CAN_GFC_ANFS(2) | CAN_GFC_ANFE(2) | CAN_GFC_RRFS | CAN_GFC_RRFE; // reject all incoming non-matching and remote frames
	// REG_CAN0_XIDAM = CAN_XIDAM_EIDM(0x1FFFFFFF); // Reset-default
	REG_CAN0_RXF1C |= CAN_RXF1C_F1OM; // FIFO 1 overwrite mode
	// REG_CAN0_RXF1C |= CAN_RXF1C_F1WM(0); // Watermark interrupt disabled - Reset-default
	//REG_CAN0_TXEFC = CAN_TXEFC_EFWM(0); // Reset-default


	REG_CAN0_CCCR &= ~CAN_CCCR_CCE; // clear CCE bit to start Tx and Rx Handler

}


static inline void init_can0(uint32_t nbtp)
{
	REG_CAN0_CCCR |= CAN_CCCR_INIT; // set CAN-Module to init, Reset-default
	while((REG_CAN0_CCCR & CAN_CCCR_INIT) == 0);

	configure_can0(nbtp);

	REG_CAN0_CCCR &= ~CAN_CCCR_INIT; // start CAN-Module
	while(REG_CAN0_CCCR & CAN_CCCR_INIT);
}


static inline bool can0_rx0_msg_fifo_avail(void)
{
	return (REG_CAN0_RXF0S & CAN_RXF0S_F0FL_Msk) != 0;
}

static inline bool can0_rx0_msg_fifo_full(void)
{
	return (REG_CAN0_RXF0S & CAN_RXF0S_F0F) != 0;
}

static inline void can0_rx0_clear1(void)
{
	uint8_t index = CAN0->RXF0S.bit.F0GI;
	REG_CAN0_RXF0A = CAN_RXF0A_F0AI(index);
}

static inline void can0_rx0_clear(void)
{
	while (can0_rx0_msg_fifo_avail()) {
		// TU_LOG2("msg at %u\n", (unsigned)CAN0->RXF0S.bit.F0GI);
		can0_rx0_clear1();
	}
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

static void usb_device_task(void* param);
static void led_task(void* param);


static inline uint32_t bitrate_to_nbtp(uint16_t bitrate)
{
	switch (bitrate) {
	case 20:
		return 0x4E1D9E27;
		break;
	case 33:
		return 0x2E1DFB60;
		break;
	case 80:
		return 0x6205C631;
		break;
	case 83:
		return 0x5E063C03;
		break;
	case 100:
		return 0x4E059E27;
		break;
	case 125:
		return 0x5E03BE2F;
		break;
	case 150:
		return 0x4E039E27;
		break;
	case 175:
		return 0x42DEF4D8;
		break;
	case 200:
		return 0x3A03761D;
		break;
	case 225:
		return 0x4590C506;
		break;
	case 250:
		return 0x3E027E1F;
		break;
	case 275:
		return 0x38EB2C4B;
		break;
	case 300:
		return 0x32AD13C4;
		break;
	case 500:
		return 0x2E015E17;
		break;
	case 625:
		return 0x4A009825;
		break;
	case 800:
		return 0x3A00761D;
		break;
	case 1000:
		return 0x2E005E17;
		break;
	default:
		return REG_CAN0_NBTP;
		break;
	}
}


static inline uint32_t bitrate_to_dbtp(uint16_t bitrate)
{
	switch (bitrate) {
	case 2000:
		return 0x2E005E17; // FIX ME
		break;
	default:
		return REG_CAN0_DBTP;
		break;
	}
}


struct peak_reply_ctx {
	uint8_t *ptr;
	uint8_t offset;
};

static void peak_process_cmd(struct peak_reply_ctx* ctx, struct peak_cmd const *cmd);
static bool peak_process_msg(uint8_t* *_ptr, uint8_t* end, struct can_tx_element *tx);
static bool peak_reply_pack_append_can_frame(
	struct peak_reply_ctx *ctx,
	uint32_t can_id,
	bool eff,
	bool rtr,
	void const *ptr,
	uint8_t dlc,
	uint16_t timestamp,
	bool first);
static void peak_reply_pack_append_can_frames(struct peak_reply_ctx* ctx);

static inline void peak_reply_cmd_init(struct peak_reply_ctx *ctx, uint8_t *ptr)
{
	ctx->ptr = ptr;
	ctx->offset = 0;
}

static inline void peak_reply_cmd_append_default(struct peak_reply_ctx *ctx)
{
	memset(ctx->ptr + ctx->offset, 0, sizeof(struct peak_cmd));
	ctx->offset += sizeof(struct peak_cmd);
}

static inline void peak_reply_init(struct peak_reply_ctx *ctx, uint8_t *ptr)
{
	ctx->ptr = ptr;
	ctx->ptr[0] = 0;
	ctx->ptr[PEAK_REPLY_PACK_RECORD_COUNTER_OFFSET] = 0;
	ctx->offset = PEAK_USB_REPLY_PACK_INIT_SIZE;
}

static inline void peak_reply_pack_init(struct peak_reply_ctx *ctx)
{
	ctx->ptr[0] = 0;
	ctx->ptr[PEAK_REPLY_PACK_RECORD_COUNTER_OFFSET] = 0;
	ctx->offset = PEAK_USB_REPLY_PACK_INIT_SIZE;
}

static inline bool peak_reply_pack_fits(struct peak_reply_ctx *ctx, uint8_t bytes)
{
	return ctx->offset + bytes <= PEAK_USB_EP_SIZE;
}

static inline void peak_reply_pack_append_unchecked_byte(
	struct peak_reply_ctx *ctx,
	uint8_t value)
{
	ctx->ptr[ctx->offset++] = value;
}

static inline void peak_reply_pack_append_unchecked(
	struct peak_reply_ctx *ctx,
	void const * ptr, uint8_t len)
{
	memcpy(ctx->ptr + ctx->offset, ptr, len);
	ctx->offset += len;
}


#if 0
typedef union {
  struct {
    uint32_t LEC:3;            /*!< bit:  0.. 2  Last Error Code                    */
    uint32_t ACT:2;            /*!< bit:  3.. 4  Activity                           */
    uint32_t EP:1;             /*!< bit:      5  Error Passive                      */
    uint32_t EW:1;             /*!< bit:      6  Warning Status                     */
    uint32_t BO:1;             /*!< bit:      7  Bus_Off Status                     */
    uint32_t DLEC:3;           /*!< bit:  8..10  Data Phase Last Error Code         */
    uint32_t RESI:1;           /*!< bit:     11  ESI flag of last received CAN FD Message */
    uint32_t RBRS:1;           /*!< bit:     12  BRS flag of last received CAN FD Message */
    uint32_t RFDF:1;           /*!< bit:     13  Received a CAN FD Message          */
    uint32_t PXE:1;            /*!< bit:     14  Protocol Exception Event           */
    uint32_t :1;               /*!< bit:     15  Reserved                           */
    uint32_t TDCV:7;           /*!< bit: 16..22  Transmitter Delay Compensation Value */
    uint32_t :9;               /*!< bit: 23..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} CAN_PSR_Type;
#endif
static inline void peak_reply_pack_append_error_msg(
	struct peak_reply_ctx *ctx,
	uint8_t errors)
{
	peak_reply_pack_append_unchecked_byte(ctx, PCAN_USB_STATUSLEN_INTERNAL);
	peak_reply_pack_append_unchecked_byte(ctx, PCAN_USB_REC_ERROR);
	peak_reply_pack_append_unchecked_byte(ctx, errors);
	++ctx->ptr[PEAK_REPLY_PACK_RECORD_COUNTER_OFFSET];
}

static inline void can_add_ts_msg_to_reply_pack(
	struct peak_reply_ctx *ctx,
	uint16_t ts)
{
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
		ts = __builtin_bswap16(ts);
#endif

	peak_reply_pack_append_unchecked_byte(ctx, 2 | PCAN_USB_STATUSLEN_INTERNAL | PCAN_USB_STATUSLEN_TIMESTAMP);
	peak_reply_pack_append_unchecked(ctx, &ts, sizeof(ts));
}

static inline uint8_t can0_consume_errors(void)
{
	CAN_PSR_Type psr = CAN0->PSR;
	CAN_RXF0S_Type rxf0s = CAN0->RXF0S;
	CAN_TXFQS_Type txfqs = CAN0->TXFQS;
	uint8_t errors = 0;

	if (psr.bit.BO) {
		errors |= PCAN_USB_ERROR_BUS_OFF;
	}

	if (psr.bit.EP) {
		errors |= PCAN_USB_ERROR_BUS_HEAVY;
	}

	if (psr.bit.EW) {
		errors |= PCAN_USB_ERROR_BUS_LIGHT;
	}

	if (txfqs.bit.TFQF) {
		errors |= PCAN_USB_ERROR_QOVR;
	}

	if (rxf0s.bit.RF0L) {
		errors |= PCAN_USB_ERROR_RXQOVR;
		can0_rx0_clear();
	}

	// if (rxf0s.bit.F0F) {
	// 	errors |= PCAN_USB_ERROR_RX;
	// }



	return errors;
}




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
	LED_DEBUG = 0,
	LED_RED1 = 1,
	LED_ORANGE1,
	LED_GREEN1,
	LED_RED2,
	LED_ORANGE2,
	LED_GREEN2
};


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

static struct can {
	bool swap;
	volatile bool enabled;
} can;

#define PEAK_CMDS_PER_USB_TRANSFER (PEAK_USB_EP_SIZE / sizeof(struct peak_cmd))

static struct usb {
	CFG_TUSB_MEM_ALIGN uint8_t cmd_rx_buffer[PEAK_USB_EP_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t cmd_tx_buffer[PEAK_USB_EP_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t msg_rx_buffer[PEAK_USB_EP_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t msg_tx_buffer[PEAK_USB_EP_SIZE];
	struct peak_cmd rx_cmd_queue_mem[PEAK_CMDS_PER_USB_TRANSFER];
	struct peak_cmd tx_cmd_queue_mem[PEAK_CMDS_PER_USB_TRANSFER];
	pc_buf rx_cmd_queue;
	pc_buf tx_cmd_queue;
	uint16_t rx_can_frames_dropped;
	uint16_t tx_can_errors;
	uint16_t rx_can_errors;
	uint8_t cmd_tx_offset;
	volatile uint8_t msg_tx_offset;
	uint8_t port;
	volatile bool mounted;
} usb;

#define PEAK_CMD_REPLIES_PER_USB_TRANSFER (PEAK_USB_EP_SIZE / sizeof(struct peak_cmd))
// #define PEAK_CMD_REPLIES_PER_USB_TRANSFER 1

int main(void)
{
	can.enabled = false;
	can.swap = 0x87654321 != REG_CAN0_ENDN;

	board_init();
	led_init();

	init_can0_pins();
	init_can0_clock();

	// init_can0(bitrate_to_nbtp(500));

	tusb_init();

	(void) xTaskCreateStatic(&usb_device_task, "usbd", TU_ARRAY_SIZE(usb_device_stack), NULL, configMAX_PRIORITIES-1, usb_device_stack, &usb_device_stack_mem);
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
static void usb_device_task(void* param)
{
	(void) param;

	while (1) {
		tud_task();
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
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	TU_LOG2("resume\n");
	usb.mounted = true;
	led_blink(0, 250);
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
	toe_rb_init(&usb.rx_cmd_queue, usb.rx_cmd_queue_mem, TU_ARRAY_SIZE(usb.rx_cmd_queue_mem));
	toe_rb_init(&usb.tx_cmd_queue, usb.tx_cmd_queue_mem, TU_ARRAY_SIZE(usb.tx_cmd_queue_mem));
	TU_LOG2("rx q cap %u\n", toe_rb_capacity(&usb.rx_cmd_queue));
	TU_LOG2("tx q cap %u\n", toe_rb_capacity(&usb.tx_cmd_queue));
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

	for (uint8_t i = 0; i < 4; ++i) {
		TU_ASSERT(dcd_edpt_open(rhport, (tusb_desc_endpoint_t const *)(ptr + i * 7)));
	}
	// TU_ASSERT(dcd_edpt_open(rhport, (tusb_desc_endpoint_t const *)(ptr + )));
	// TU_ASSERT(dcd_edpt_open(rhport, (tusb_desc_endpoint_t const *)(ptr + 2*7)));

	TU_ASSERT(usbd_edpt_xfer(rhport, PEAK_USB_EP_BULK_OUT_CMD, usb.cmd_rx_buffer, TU_ARRAY_SIZE(usb.cmd_rx_buffer)));
	TU_ASSERT(usbd_edpt_xfer(rhport, PEAK_USB_EP_BULK_OUT_MSG, usb.msg_rx_buffer, TU_ARRAY_SIZE(usb.msg_rx_buffer)));

	//*p_length = 2*9+6*7;
	*p_length = 9+4*7;

	return true;
}

bool tud_custom_control_request_cb(uint8_t rhport, tusb_control_request_t const * request)
{
	(void)request;
	TU_LOG2("port %u req\n", rhport);

	if (rhport != usb.port) {
		return false;
	}

	return true;
}
bool tud_custom_control_complete_cb(uint8_t rhport, tusb_control_request_t const * request)
{
	(void)request;
	TU_LOG2("port %u req com\n", rhport);

	if (rhport != usb.port) {
		return false;
	}

	return true;
}



bool tud_custom_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
	(void)event; // always success
	// TU_LOG2("port %u ep %02x event %d bytes %u\n", rhport, ep_addr, event, (unsigned)xferred_bytes);

	if (rhport != usb.port) {
		return false;
	}

	switch (ep_addr) {
	case PEAK_USB_EP_BULK_OUT_CMD: {
		uint8_t cmd_count = xferred_bytes / sizeof(struct peak_cmd);
		struct peak_cmd const *cmd_ptr = (struct peak_cmd const *)usb.cmd_rx_buffer;
		struct peak_reply_ctx rep;
		for (bool done = false; !done; ) {
			bool added = false;
			done = true;

			TU_LOG2("rx q size %u tx q size %u\n", toe_rb_size(&usb.rx_cmd_queue), toe_rb_size(&usb.tx_cmd_queue));

			if (cmd_count) {
				// try to avoid copy to rx queue
				if (!toe_rb_size(&usb.rx_cmd_queue) && toe_rb_left(&usb.tx_cmd_queue)) {
					TU_LOG2("process & tx q cmd\n");

					uint8_t *mem = (uint8_t *)pc_at_ptr(&usb.tx_cmd_queue, toe_rb_size(&usb.tx_cmd_queue));
					peak_reply_cmd_init(&rep, mem);
					peak_process_cmd(&rep, cmd_ptr);
					if (rep.offset) {
						added = true;
						++usb.tx_cmd_queue.size;
					}
					++cmd_ptr;
					--cmd_count;
					done = false;
				} else {
					if (toe_rb_left(&usb.rx_cmd_queue)) {
						TU_LOG2("rx q cmd\n");
						pc_push_back_ptr(&usb.rx_cmd_queue, cmd_ptr);
						--cmd_count;
						++cmd_ptr;
						done = false;
					} else {
						// error
					}
				}
			}

			// process one cmd
			if (toe_rb_left(&usb.tx_cmd_queue) && toe_rb_size(&usb.rx_cmd_queue)) {

				uint8_t *mem = (uint8_t *)pc_at_ptr(&usb.tx_cmd_queue, toe_rb_size(&usb.tx_cmd_queue));
				peak_reply_cmd_init(&rep, mem);
				peak_process_cmd(&rep, pc_at_cptr(&usb.rx_cmd_queue, 0));
				pc_drop_front(&usb.rx_cmd_queue);
				if (rep.offset) {
					added = true;
					++usb.tx_cmd_queue.size;
				}
				done = false;
			}

			// if (!added && toe_rb_size(&usb.tx_cmd_queue)
			// 	&& !usbd_edpt_busy(rhport, PEAK_USB_EP_BULK_IN_CMD)) {

			// 	// uint8_t before = toe_rb_size(&usb.tx_cmd_queue);
			// 	usb.cmd_tx_offset = 0;

			// 	struct peak_cmd *out_ptr = usb.cmd_tx_buffer;
			// 	// for (uint8_t i = 0;
			// 	// 	i < PEAK_CMD_REPLIES_PER_USB_TRANSFER && pc_try_pop_front(&usb.tx_cmd_queue, out_ptr);
			// 	// 	++i, ++out_ptr, usb.cmd_tx_offset += sizeof(*out_ptr));

			// 	for (uint8_t i = 0; i < PEAK_CMD_REPLIES_PER_USB_TRANSFER; ++i) {
			// 		// TU_LOG2("pre pop tx q zero %u size %u\n", usb.tx_cmd_queue.zero, usb.tx_cmd_queue.size);
			// 		if (pc_try_pop_front(&usb.tx_cmd_queue, out_ptr)) {
			// 			// TU_LOG2("post pop tx q zero %u size %u\n", usb.tx_cmd_queue.zero, usb.tx_cmd_queue.size);
			// 			++out_ptr;
			// 			usb.cmd_tx_offset += sizeof(*out_ptr);
			// 		} else {
			// 			break;
			// 		}
			// 	}

			// 	// uint8_t after = toe_rb_size(&usb.tx_cmd_queue);
			// 	// TU_LOG2("size before %u after %u\n", before, after);
			// 	// TU_ASSERT(before > after, );
			// 	TU_ASSERT(usb.cmd_tx_offset, );
			// 	TU_LOG2("usb tx %u cmd replies\n", (unsigned)(usb.cmd_tx_offset / sizeof(struct peak_cmd)));
			// 	TU_LOG2("tx q size %u\n", usb.tx_cmd_queue.size);
			// 	TU_ASSERT( usbd_edpt_xfer(rhport, PEAK_USB_EP_BULK_IN_CMD, usb.cmd_tx_buffer, usb.cmd_tx_offset) );
			// 	done = false;
			// }

			// if (!toe_rb_left(&usb.tx_cmd_queue)
			// 	&& !usbd_edpt_busy(rhport, PEAK_USB_EP_BULK_IN_CMD)) {

			// 	// uint8_t before = toe_rb_size(&usb.tx_cmd_queue);
			// 	usb.cmd_tx_offset = 0;

			// 	struct peak_cmd *out_ptr = usb.cmd_tx_buffer;
			// 	// for (uint8_t i = 0;
			// 	// 	i < PEAK_CMD_REPLIES_PER_USB_TRANSFER && pc_try_pop_front(&usb.tx_cmd_queue, out_ptr);
			// 	// 	++i, ++out_ptr, usb.cmd_tx_offset += sizeof(*out_ptr));

			// 	for (uint8_t i = 0; i < PEAK_CMD_REPLIES_PER_USB_TRANSFER; ++i) {
			// 		// TU_LOG2("pre pop tx q zero %u size %u\n", usb.tx_cmd_queue.zero, usb.tx_cmd_queue.size);
			// 		if (pc_try_pop_front(&usb.tx_cmd_queue, out_ptr)) {
			// 			// TU_LOG2("post pop tx q zero %u size %u\n", usb.tx_cmd_queue.zero, usb.tx_cmd_queue.size);
			// 			++out_ptr;
			// 			usb.cmd_tx_offset += sizeof(*out_ptr);
			// 		} else {
			// 			break;
			// 		}
			// 	}

			// 	// uint8_t after = toe_rb_size(&usb.tx_cmd_queue);
			// 	// TU_LOG2("size before %u after %u\n", before, after);
			// 	// TU_ASSERT(before > after, );
			// 	TU_ASSERT(usb.cmd_tx_offset, );
			// 	TU_LOG2("usb tx %u cmd replies\n", (unsigned)(usb.cmd_tx_offset / sizeof(struct peak_cmd)));
			// 	TU_LOG2("tx q size %u\n", usb.tx_cmd_queue.size);
			// 	TU_ASSERT( usbd_edpt_xfer(rhport, PEAK_USB_EP_BULK_IN_CMD, usb.cmd_tx_buffer, usb.cmd_tx_offset) );
			// 	done = false;
			// }

			(void)added;
		}

		TU_LOG2("!loop done\n");

		if (toe_rb_size(&usb.tx_cmd_queue)
			// && !usbd_edpt_busy(rhport, PEAK_USB_EP_BULK_IN_CMD)) {
				&& !usb.cmd_tx_offset) {
				// ) {

			// uint8_t before = toe_rb_size(&usb.tx_cmd_queue);
			usb.cmd_tx_offset = 0;

			struct peak_cmd *out_ptr = (struct peak_cmd *)usb.cmd_tx_buffer;
			// for (uint8_t i = 0;
			// 	i < PEAK_CMD_REPLIES_PER_USB_TRANSFER && pc_try_pop_front(&usb.tx_cmd_queue, out_ptr);
			// 	++i, ++out_ptr, usb.cmd_tx_offset += sizeof(*out_ptr));

			for (uint8_t i = 0; i < PEAK_CMD_REPLIES_PER_USB_TRANSFER; ++i) {
				// TU_LOG2("pre pop tx q zero %u size %u\n", usb.tx_cmd_queue.zero, usb.tx_cmd_queue.size);
				if (pc_try_pop_front(&usb.tx_cmd_queue, out_ptr)) {
					// TU_LOG2("post pop tx q zero %u size %u\n", usb.tx_cmd_queue.zero, usb.tx_cmd_queue.size);
					++out_ptr;
					usb.cmd_tx_offset += sizeof(*out_ptr);
				} else {
					break;
				}
			}

			// uint8_t after = toe_rb_size(&usb.tx_cmd_queue);
			// TU_LOG2("size before %u after %u\n", before, after);
			// TU_ASSERT(before > after, );
			TU_ASSERT(usb.cmd_tx_offset, false);
			TU_LOG2("> cmd tx %u cmd replies\n", (unsigned)(usb.cmd_tx_offset / sizeof(struct peak_cmd)));
			TU_LOG2("cmd tx q size %u\n", usb.tx_cmd_queue.size);
			usbd_edpt_xfer(rhport, PEAK_USB_EP_BULK_IN_CMD, usb.cmd_tx_buffer, usb.cmd_tx_offset);
		}


		// struct peak_reply_ctx c;
		// if (usb.cmd_tx_offset) {
		// 	TU_LOG2("usb: new cmd while busy\n");
		// 	static uint8_t dummn[PEAK_USB_EP_SIZE];
		// 	peak_reply_cmd_init(&c, dummn);
		// 	TU_ASSERT( usbd_edpt_busy(rhport, PEAK_USB_EP_BULK_IN_CMD) );
		// } else {
		// 	peak_reply_cmd_init(&c, usb.cmd_tx_buffer);

		// }

		// struct peak_cmd *cmd = (struct peak_cmd *)usb.cmd_rx_buffer;
		// while (xferred_bytes >= sizeof(*cmd_ptr)) {
		// 	peak_process_cmd(&c, cmd_ptr);
		// 	++cmd_ptr;
		// 	xferred_bytes -= sizeof(*cmd_ptr);
		// }


		// if (!usb.cmd_tx_offset && c.offset) {
		// 	usb.cmd_tx_offset = c.offset;
		// 	TU_ASSERT( usbd_edpt_xfer(rhport, PEAK_USB_EP_BULK_IN_CMD, usb.cmd_tx_buffer, usb.cmd_tx_offset) );
		// }

		// start new transaction
		usbd_edpt_xfer(rhport, ep_addr, usb.cmd_rx_buffer, TU_ARRAY_SIZE(usb.cmd_rx_buffer));


	} break;
	case PEAK_USB_EP_BULK_IN_CMD:
		TU_ASSERT(!usbd_edpt_busy(rhport, ep_addr), false);
		TU_LOG2("< cmd IN token\n");
		usb.cmd_tx_offset = 0;
		if (toe_rb_size(&usb.tx_cmd_queue)) {
			struct peak_cmd *out_ptr = (struct peak_cmd *)usb.cmd_tx_buffer;
			TU_LOG2("tx q size %u\n", usb.tx_cmd_queue.size);
			for (uint8_t i = 0;
				i < PEAK_CMD_REPLIES_PER_USB_TRANSFER && pc_try_pop_front(&usb.tx_cmd_queue, out_ptr);
				++i, ++out_ptr, usb.cmd_tx_offset += sizeof(*out_ptr));

			TU_ASSERT(usb.cmd_tx_offset, false);
			TU_LOG2("cmd tx %u cmd replies\n", (unsigned)(usb.cmd_tx_offset / sizeof(struct peak_cmd)));
			TU_LOG2("cmd tx q size %u\n", usb.tx_cmd_queue.size);
			usbd_edpt_xfer(rhport, PEAK_USB_EP_BULK_IN_CMD, usb.cmd_tx_buffer, usb.cmd_tx_offset);
		} else {
			TU_LOG2("! cmd tx q empty\n");
			// TU_ASSERT( usbd_edpt_xfer(rhport, PEAK_USB_EP_BULK_IN_CMD, usb.cmd_tx_buffer, 0) );
			//TU_ASSERT( usbd_edpt_xfer(rhport, PEAK_USB_EP_BULK_IN_CMD, usb.cmd_tx_buffer, 0) );
			// usbd_edpt_stall(rhport, PEAK_USB_EP_BULK_IN_CMD);
		}
		break;
	case PEAK_USB_EP_BULK_OUT_MSG: {

		// TU_LOG2("msg len %u: ", xferred_bytes);
		// for (uint8_t i = 0; i < xferred_bytes; ++i) {
		// 	TU_LOG2("%02x ", usb.msg_rx_buffer[i]);
		// }
		// TU_LOG2("\n");

		uint8_t *ptr = usb.msg_rx_buffer;
		uint8_t *end = usb.msg_rx_buffer + xferred_bytes;
		// looks like it is only a single message
		uint8_t msgs = 0;
		bool tx_full = false;
		while (ptr < end) {
			CAN_TXFQS_Type txfqs = CAN0->TXFQS;
			if (txfqs.bit.TFQF) {
				TU_LOG2("can tx q full\n");
				tx_full = true;
				break;
			} else {
				uint8_t index = txfqs.bit.TFQPI;
				if (peak_process_msg(&ptr, end, &can0_tx_buffer[index])) {
					TU_LOG2("can tx %u\n", index);
					REG_CAN0_TXBAR |= UINT32_C(1) << index;
				} else {
					if (0 == msgs) {
						TU_LOG1("!offset %u msg decode error\n", ptr - usb.msg_rx_buffer);
					}
					break;
				}
			}
			++msgs;
		}

		if (!usb.msg_tx_offset) { // send status, can frames
			struct peak_reply_ctx ctx;
			peak_reply_init(&ctx, usb.msg_tx_buffer);
			peak_reply_pack_init(&ctx);
			peak_reply_pack_append_error_msg(&ctx, can0_consume_errors() | (tx_full ? (PCAN_USB_ERROR_TXFULL | PCAN_USB_ERROR_TXFULL) : 0));
			peak_reply_pack_append_can_frames(&ctx);
			usb.msg_tx_offset = ctx.offset;
			usbd_edpt_xfer(rhport, PEAK_USB_EP_BULK_IN_MSG, usb.msg_tx_buffer, usb.msg_tx_offset);
		}
		usbd_edpt_xfer(rhport, ep_addr, usb.msg_rx_buffer, TU_ARRAY_SIZE(usb.msg_rx_buffer));
	} break;
	case PEAK_USB_EP_BULK_IN_MSG: {
		TU_ASSERT(!usbd_edpt_busy(rhport, ep_addr), false);
		// TU_LOG2("< msg IN token\n");
		usb.msg_tx_offset = 0;
		struct peak_reply_ctx ctx;
		peak_reply_init(&ctx, usb.msg_tx_buffer);
		peak_reply_pack_init(&ctx);
		peak_reply_pack_append_error_msg(&ctx, can0_consume_errors());
		peak_reply_pack_append_can_frames(&ctx);
		usb.msg_tx_offset = ctx.offset;
		usbd_edpt_xfer(usb.port, PEAK_USB_EP_BULK_IN_MSG, usb.msg_tx_buffer, usb.msg_tx_offset);
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
// PEAK
//--------------------------------------------------------------------+



static inline void peak_reply_cmd_append_f_n(
	struct peak_reply_ctx *ctx,
	uint8_t f, uint8_t n, void const *ptr, uint8_t len)
{
	ctx->ptr[ctx->offset+0] = f;
	ctx->ptr[ctx->offset+1] = n;
	if (len) {
		memcpy(ctx->ptr + ctx->offset + 2, ptr, len);
	}
	ctx->offset += sizeof(struct peak_cmd);

}

static bool peak_reply_pack_append_can_frame(
	struct peak_reply_ctx *ctx,
	uint32_t can_id,
	bool eff,
	bool rtr,
	void const *ptr,
	uint8_t dlc,
	uint16_t timestamp,
	bool first)
{
	uint8_t len = 4;
	uint8_t status = dlc;
	if (eff) {
		len += 2;
		status |= PCAN_USB_STATUSLEN_EXT_ID;
	}

	if (rtr) {
		status |= PCAN_USB_STATUSLEN_RTR;
	} else {
		len += dlc;
	}

	len += first;

	if (!peak_reply_pack_fits(ctx, len)) {
		return false;
	}

	// inc records
	++ctx->ptr[PEAK_REPLY_PACK_RECORD_COUNTER_OFFSET];

	ctx->ptr[ctx->offset++] = status;

	if (eff) {
		can_id <<= 3;
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
		can_id = __builtin_bswap32(can_id);
#endif
		memcpy(&ctx->ptr[ctx->offset], &can_id, sizeof(can_id));
		ctx->offset += sizeof(can_id);
	} else {
		uint16_t sff = can_id << 5;
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
		sff = __builtin_bswap16(sff);
#endif
		memcpy(&ctx->ptr[ctx->offset], &sff, sizeof(sff));
		ctx->offset += sizeof(sff);
	}

	if (first) {
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
		timestamp = __builtin_bswap16(timestamp);
#endif
		memcpy(&ctx->ptr[ctx->offset], &timestamp, sizeof(timestamp));
		ctx->offset += sizeof(timestamp);
	} else {
		ctx->ptr[ctx->offset++] = timestamp;
	}

	if (!rtr) {
		memcpy(&ctx->ptr[ctx->offset], ptr, dlc);
		ctx->offset += dlc;
	}

	return true;

}


static void peak_process_cmd(struct peak_reply_ctx *ctx, struct peak_cmd const *cmd)
{
	TU_LOG2("cmd %02d %02d ", cmd->func, cmd->num);

	bool send_default_reply = false;
	bool send_error_reply = false;

	switch (cmd->func) {
	case 1:
		switch (cmd->num) {
		case 2: {
			uint8_t brt0, brt1;
			brt0 = cmd->argv[1];
			brt1 = cmd->argv[0];

			// https://www.nxp.com/docs/en/data-sheet/SJA1000.pdf p. 50ff
			uint8_t sjw = (brt0 >> 6);
			uint8_t brp = brt0 & 0x3f;
			uint8_t sam = (brt1 & 0x80) != 0;
			uint8_t ts2 = (brt1 >> 4) & 0x7;
			uint8_t ts1 = brt1 & 0xf;
			uint8_t sample_point_percent = ((ts1 + 1) * 100) / (ts1 + 1 + ts2 + 1 + 1);

			uint32_t bps = PEAK_USB_F / (2 * (ts1+ts2+3) * (1+brp));

			TU_LOG2("set bittiming\n"
					"btr0=0x%02x btr1=0x%02x\n"
					"sjw=%u brp=%u sam=%u ts1=%u ts2=%u\n"
					"sample point=%u %%\n"
					"bitrate=%lu bps\n",
				brt0, brt1, sjw, brp, sam, ts1, ts2, sample_point_percent, bps);

			(void)sjw;
			(void)sam;
			(void)sample_point_percent;

			configure_can0(bitrate_to_nbtp(bps/1000));
		} break;
		}
		break;
	case 3:
		switch (cmd->num) {
		case 2: // pcan_usb_set_bus argv[0] == on/off
			TU_LOG2("set bus %d\n", cmd->argv[0]);
			if (cmd->argv[0]) {
				can.enabled = true;
				REG_CAN0_CCCR &= ~CAN_CCCR_INIT; // start CAN-Module
				while(REG_CAN0_CCCR & CAN_CCCR_INIT);

				// byte | value
				// -----+-------
				//    0 | ???
				//    1 | record counter (1 byte)
				//    2 | status/len (1 byte)

				// if byte 2 is flagged with 'internal status'
				//    3 | func
				//    4 | num


				struct peak_reply_ctx ctx;
				peak_reply_init(&ctx, usb.msg_tx_buffer);
				peak_reply_pack_init(&ctx);
				peak_reply_pack_append_error_msg(&ctx, can0_consume_errors());
				usb.msg_tx_offset = ctx.offset;
				TU_ASSERT(usbd_edpt_xfer(usb.port, PEAK_USB_EP_BULK_IN_MSG, usb.msg_tx_buffer, usb.msg_tx_offset), );
			} else {
				can.enabled = false;
				REG_CAN0_CCCR |= CAN_CCCR_INIT; // set CAN-Module to init, Reset-default
				while((REG_CAN0_CCCR & CAN_CCCR_INIT) == 0);
			}
			break;
		case 3: // pcan_usb_set_silent argv[0] == on/off
			TU_LOG2("set silent %u\n", cmd->argv[0]);
			break;
		}
		break;
	case 4:
		switch (cmd->num) {
		case 1: { // pcan_usb_get_device_id
			TU_LOG2("get device id\n");
			uint8_t id = 0;
			peak_reply_cmd_append_f_n(ctx, cmd->func, cmd->num, &id, 1);
			send_default_reply = false;
		} break;

		}
		break;
	case 6:
		switch (cmd->num) {
		case 1: { // pcan_usb_get_serial
			TU_LOG2("get serial\n");
			uint32_t serial = 0xdeadbeef;
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
			serial = __builtin_bswap32(serial);
#endif
			peak_reply_cmd_append_f_n(ctx, cmd->func, cmd->num, &serial, sizeof(serial));
			send_default_reply = false;
		} break;

		}
		break;
	case 9: {
		switch (cmd->num) {
		case 2: // num 2 pcan_usb_set_sja1000 argv[1] == mode
			TU_LOG2("set sja1000 mode %s\n", cmd->argv[1] == 0 ? "init" : "normal");
			break;
		}
	} break;
	case 10:
		switch (cmd->num) {
		case 2: // num 2 pcan_usb_set_ext_vcc argv[0] == on/off
			TU_LOG2("set ext vcc %u\n", cmd->argv[0]);
			break;
		}
		break;
	}

	TU_LOG2("\n");

	if (send_error_reply) {
		peak_reply_cmd_append_default(ctx);
	} else if (send_default_reply) {
		peak_reply_cmd_append_f_n(ctx, cmd->func, cmd->num, NULL, 0);
	}
}

static bool peak_process_msg(uint8_t* *_ptr, uint8_t* end, struct can_tx_element *tx)
{
	bool result = false;
	uint8_t *ptr = *_ptr;

	if (unlikely(end - ptr < PEAK_USB_REPLY_PACK_MIN_CAN_MSG_SIZE)) {
		TU_LOG1("msg too short: %u bytes\n", end - ptr);
		goto out;
	}

	// first byte == 2
	if (*ptr != 2) {
		TU_LOG1("msg[0] != 2: %02x\n", *ptr);
		goto out;
	}

	++ptr;

	// 2nd byte == 1
	if (unlikely(*ptr != 1)) {
		TU_LOG1("msg[1] != 1: %02x\n", *ptr);
		goto out;
	}

	++ptr;

	uint8_t status_len = *ptr++;
	uint8_t dlc = status_len & 0xf;
	if (unlikely(dlc > 8)) {
		TU_LOG1("dlc > 8: %u\n", dlc);
		goto out;
	}

	CAN_TXBE_0_Type t0;
	CAN_TXBE_1_Type t1;
	t0.reg = 0;
	t1.reg = 0;

	t1.bit.DLC = dlc;

	t0.bit.RTR = (status_len & PCAN_USB_STATUSLEN_RTR) == PCAN_USB_STATUSLEN_RTR;
	t0.bit.XTD = (status_len & PCAN_USB_STATUSLEN_EXT_ID) == PCAN_USB_STATUSLEN_EXT_ID;

	uint32_t can_id;
	if (t0.bit.XTD) {
		if (unlikely(ptr + 4 > end)) {
			TU_LOG1("msg no 4 byte eff can id\n");
			goto out;
		}

		memcpy(&can_id, ptr, 4);
		ptr += 4;
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
		can_id = __builtin_bswap32(can_id);
#endif
		can_id >>= 3;
	} else {
		if (unlikely(ptr + 2 > end)) {
			TU_LOG1("msg no 2 byte sff can id\n");
			goto out;
		}
		uint16_t x;
		memcpy(&x, ptr, 2);
		ptr += 2;
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
		x = __builtin_bswap16(x);
#endif
		can_id = x;
		can_id <<= 13;
	}

	t0.bit.ID = can_id;

	if (!t0.bit.RTR) {
		if (unlikely(ptr + dlc > end)) {
			TU_LOG1("msg no %u byte data\n", dlc);
			goto out;
		}

		memcpy(tx->data, ptr, dlc);
		ptr += dlc;
	}

	// TU_LOG2("canid %x eff %u rtr %u dlc %u\n", t0.bit.ID, t0.bit.XTD, t0.bit.RTR, t1.bit.DLC);

	tx->T0 = t0;
	tx->T1 = t1;

	result = true;

out:
	*_ptr = ptr;
	return result;
}



static void peak_reply_pack_append_can_frames(struct peak_reply_ctx* ctx)
{
	for (bool first = true; can0_rx0_msg_fifo_avail(); first = false) {
		uint8_t index = CAN0->RXF0S.bit.F0GI;
		struct can_rx_fifo_element const *e = &can0_rx_fifo[index];
		CAN_RXF0E_0_Type r0 = e->R0;
		CAN_RXF0E_1_Type r1 = e->R1;

// 		#define CAN_RXF0E_0_ID_Pos          0            /**< \brief (CAN_RXF0E_0) Identifier */
// #define CAN_RXF0E_0_ID_Msk          (_U_(0x1FFFFFFF) << CAN_RXF0E_0_ID_Pos)
// #define CAN_RXF0E_0_ID(value)       (CAN_RXF0E_0_ID_Msk & ((value) << CAN_RXF0E_0_ID_Pos))
// #define CAN_RXF0E_0_RTR_Pos         29           /**< \brief (CAN_RXF0E_0) Remote Transmission Request */
// #define CAN_RXF0E_0_RTR             (_U_(0x1) << CAN_RXF0E_0_RTR_Pos)
// #define CAN_RXF0E_0_XTD_Pos         30           /**< \brief (CAN_RXF0E_0) Extended Identifier */
// #define CAN_RXF0E_0_XTD             (_U_(0x1) << CAN_RXF0E_0_XTD_Pos)
// #define CAN_RXF0E_0_ESI_Pos         31           /**< \brief (CAN_RXF0E_0) Error State Indicator */
// #define CAN_RXF0E_0_ESI             (_U_(0x1) << CAN_RXF0E_0_ESI_Pos)
// #define CAN_RXF0E_0_MASK            _U_(0xFFFFFFFF) /**< \brief (CAN_RXF0E_0) MASK Register */

		// format the can message
		uint32_t id = (r0.reg & CAN_RXF0E_0_ID_Msk) >> CAN_RXF0E_0_ID_Pos;
		if (!r0.bit.XTD) {
			id >>= 18;
		}

		if (peak_reply_pack_append_can_frame(
				ctx,
				id,
				r0.bit.XTD,
				r0.bit.RTR,
				e->data,
				r1.bit.DLC,
				r1.bit.RXTS,
				first)) {
			REG_CAN0_RXF0A = CAN_RXF0A_F0AI(index);
		} else {
			break;
		}
	}
}

