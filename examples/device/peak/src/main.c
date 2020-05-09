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


#include <sam.h>

#include <hal/include/hal_gpio.h>

#include <peak.h>


#ifndef likely
#define likely(x) __builtin_expect((x),1)
#endif

#define CAN0_TX_BUFFER_NUM 32
#define CAN0_RX_FIFO_NUM 64
#define CAN0_ELEMENT_DATA_SIZE 64

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


// static task for usbd
// Increase stack size when debug log is enabled
#if CFG_TUSB_DEBUG
	#define USBD_STACK_SIZE (3*configMINIMAL_STACK_SIZE)
#else
	#define USBD_STACK_SIZE (3*configMINIMAL_STACK_SIZE/2)
#endif

static StackType_t usb_device_stack[USBD_STACK_SIZE];
static StaticTask_t usb_device_stack_mem;


static StackType_t can_task_stack[configMINIMAL_STACK_SIZE];
static StaticTask_t can_task_mem;

static StackType_t led_task_stack[configMINIMAL_STACK_SIZE];
static StaticTask_t led_task_mem;

static void usb_device_task(void* param);
static void can_task(void* param);
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



static StaticSemaphore_t usb_lock_mem;
static SemaphoreHandle_t usb_lock;
static uint8_t usb_out_buffer[CFG_TUD_VENDOR_TX_BUFSIZE];
static uint8_t usb_out_offset;
static uint16_t rx_can_frames_dropped;
static uint16_t tx_can_errors;
static uint16_t rx_can_errors;
static volatile bool mounted;


static void peak_process_msg(void);
static void peak_reply_pack_init(void);
static bool peak_reply_pack_append_can_frame(
	uint32_t can_id,
	bool eff,
	bool rtr,
	void const *ptr,
	uint8_t dlc,
	uint16_t timestamp);


#define LED_MODE_OFF	0
#define LED_MODE_ON		1
#define LED_MODE_BLINK	2
struct led {
#if CFG_TUSB_DEBUG > 0
	const char* name;
#endif
	volatile uint16_t blink_delay_ms;
	volatile uint8_t mode;
	uint8_t pin;
};

#if CFG_TUSB_DEBUG > 0
#define LED_STATIC_INITIALIZER(name, ms, pin, mode) \
	{ name, ms, mode, pin }
#else
#define LED_STATIC_INITIALIZER(name, ms, pin, mode) \
	{ ms, mode, pin }
#endif

static struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", 0, PIN_PA02, LED_MODE_OFF),
	LED_STATIC_INITIALIZER("red1", 0, PIN_PB14, LED_MODE_OFF),
	LED_STATIC_INITIALIZER("orange1", 0, PIN_PB15, LED_MODE_OFF),
	LED_STATIC_INITIALIZER("green1", 0, PIN_PA12, LED_MODE_OFF),
	LED_STATIC_INITIALIZER("red2", 0, PIN_PA13, LED_MODE_OFF),
	LED_STATIC_INITIALIZER("orange2", 0, PIN_PA14, LED_MODE_OFF),
	LED_STATIC_INITIALIZER("green2", 0, PIN_PA15, LED_MODE_OFF),
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
	leds[index].mode = on;
	gpio_set_pin_level(leds[index].pin, on);
}

static inline void led_toggle(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].mode = LED_MODE_OFF;
	gpio_toggle_pin_level(leds[index].pin);
}

static inline void led_blink(uint8_t index, uint16_t delay_ms)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].blink_delay_ms = delay_ms;
	leds[index].mode = LED_MODE_BLINK;
}

int main(void)
{
	usb_lock = xSemaphoreCreateRecursiveMutexStatic(&usb_lock_mem);

	board_init();
	led_init();

	init_can0_pins();
	init_can0_clock();

	init_can0(bitrate_to_nbtp(500));

	tusb_init();

	(void) xTaskCreateStatic(&usb_device_task, "usbd", TU_ARRAY_SIZE(usb_device_stack), NULL, configMAX_PRIORITIES-1, usb_device_stack, &usb_device_stack_mem);
	(void) xTaskCreateStatic(&can_task, "can", TU_ARRAY_SIZE(can_task_stack), NULL, configMAX_PRIORITIES-1, can_task_stack, &can_task_mem);
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
	mounted = true;

}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	TU_LOG2("unmounted\n");
	led_blink(0, 1000);
	mounted = false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us	to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	TU_LOG2("suspend\n");
	mounted = false;
	led_blink(0, 500);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	TU_LOG2("resume\n");
	mounted = true;
	led_blink(0, 250);
}


//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when vendor interface received data from host
void tud_vendor_rx_cb(uint8_t itf)
{
	(void) itf;

	if (xSemaphoreTakeRecursive(usb_lock, pdMS_TO_TICKS(PEAK_USB_CMD_TIMEOUT_MS / 2))) {
		peak_process_msg();
		xSemaphoreGiveRecursive(usb_lock);
	}
}

// // Invoked when vendor interface received data from host
// bool vendord_open(uint8_t rhport, tusb_desc_interface_t const * desc_intf, uint16_t* p_length)
// {

// }


static inline bool can_msg_fifo_avail(void)
{
	return (REG_CAN0_RXF0S & CAN_RXF0S_F0FL_Msk) != 0;
}

static inline bool can_msg_fifo_full(void)
{
	return (REG_CAN0_RXF0S & CAN_RXF0S_F0F) != 0;
}

static inline void can_process1(void)
{
	// struct mcba_usb_msg_can msg;
	//TU_ASSERT(sizeof(msg) == 19, );

	uint8_t index = CAN0->RXF0S.bit.F0GI;
	struct can_rx_fifo_element const *e = &can0_rx_fifo[index];
	CAN_RXF0E_0_Type r0 = e->R0;
	CAN_RXF0E_1_Type r1 = e->R1;

	(void)r0;
	(void)r1;

	// format the can message
	uint32_t id = r0.bit.ID;
	id >>= r0.bit.XTD ? 0 : 18;
	// // mcba_format_can_msg(id, e->data, r1.bit.DLC, r0.bit.XTD, r0.bit.RTR, xTaskGetTickCount(), &msg);

	REG_CAN0_RXF0A = CAN_RXF0A_F0AI(index);

	// tud_vendor_write(&msg, sizeof(msg));
}

static inline void can_clear1(void)
{
	uint8_t index = CAN0->RXF0S.bit.F0GI;
	REG_CAN0_RXF0A = CAN_RXF0A_F0AI(index);
}

static inline void can_clear(void)
{
	while (can_msg_fifo_avail()) {
		// TU_LOG2("msg at %u\n", (unsigned)CAN0->RXF0S.bit.F0GI);
		can_clear1();
	}
}

// static inline bool mcba_process_usb(void)
// {
// 	bool result = false;
// 	// while (tud_vendor_available() >= sizeof(struct mcba_usb_msg) &&
// 	// 		tud_vendor_write_available() == CFG_TUD_VENDOR_TX_BUFSIZE) {
// 	// 		mcba_process_usb_msg1();
// 	// 		result = true;
// 	// }

// 	return result;
// }

//--------------------------------------------------------------------+
// CAN TASK
//--------------------------------------------------------------------+
static void can_task(void *param)
{
	(void) param;

	// bool work;




	const TickType_t TASK_DELAY_US = 352;
	const TickType_t TASK_DELAY_TICKS = (TASK_DELAY_US * configTICK_RATE_HZ) / 1000000UL;
	const TickType_t LED_DELAY_TICKS = pdMS_TO_TICKS(20);

	TU_LOG2("can: F %lu [Hz]\n", (unsigned long)configCPU_CLOCK_HZ);
	TU_LOG2("can: loop delay %lu [us] %lu [ticks] \n", (unsigned long)TASK_DELAY_US, (unsigned long)TASK_DELAY_TICKS);
	TU_LOG2("can: led delay %lu [ticks] \n", (unsigned long)LED_DELAY_TICKS);
	TickType_t can_loop_ts = xTaskGetTickCount();
	TickType_t data_avail_ts = 0;
	TickType_t rx_error_ts = 0;
	TickType_t tx_error_ts = 0;
	TickType_t mounted_ts = 0;


	while (42) {

		// TU_LOG2("can\n");
		// work = false;

		TickType_t loop_start_ts = xTaskGetTickCount();

		// update error counters
		CAN_ECR_Type ecr = (CAN_ECR_Type)REG_CAN0_ECR;
		tx_can_errors += ecr.bit.TEC;
		rx_can_errors += ecr.bit.REC;

		if (ecr.bit.REC) {
			rx_error_ts = loop_start_ts;
		}

		if (ecr.bit.TEC) {
			tx_error_ts = loop_start_ts;
		}

		if (can_msg_fifo_avail()) {
			data_avail_ts = loop_start_ts;

			if (mounted) {
				mounted_ts = loop_start_ts;


				bool removed = false;
				if (pdTRUE == xSemaphoreTakeRecursive(usb_lock, 0)) {
					if (tud_vendor_write_available() == CFG_TUD_VENDOR_TX_BUFSIZE) {
						peak_reply_pack_init();

						for (bool done = false; !done; ) {
							uint8_t index = CAN0->RXF0S.bit.F0GI;
							struct can_rx_fifo_element const *e = &can0_rx_fifo[index];
							CAN_RXF0E_0_Type r0 = e->R0;
							CAN_RXF0E_1_Type r1 = e->R1;

							(void)r0;
							(void)r1;

							// format the can message
							uint32_t id = r0.bit.ID;
							id >>= r0.bit.XTD ? 0 : 18;

							if (peak_reply_pack_append_can_frame(id, r0.bit.XTD, r0.bit.RTR, e->data, r1.bit.DLC, xTaskGetTickCount())) {
								REG_CAN0_RXF0A = CAN_RXF0A_F0AI(index);
							} else {
								done = true;
							}
						}

						tud_vendor_write(usb_out_buffer, usb_out_offset);
					}
					xSemaphoreGiveRecursive(usb_lock);
				}

				if (!removed) {
					if (can_msg_fifo_full()) {
						++rx_can_frames_dropped;
						can_clear1();
					}
				}
			} else {
				can_clear();
			}
		}

		vTaskDelay(TASK_DELAY_TICKS);
		// led_toggle(LED_ORANGE1);
		// led_toggle(LED_RED1);
		// led_toggle(LED_GREEN1);
		// led_toggle(LED_GREEN2);
		// led_toggle(LED_RED2);

		TickType_t loop_end_ts = xTaskGetTickCount();

		led_set(LED_ORANGE2, loop_end_ts - data_avail_ts <= LED_DELAY_TICKS);
		led_set(LED_GREEN1, loop_end_ts - tx_error_ts <= LED_DELAY_TICKS);
		led_set(LED_RED2, loop_end_ts - rx_error_ts <= LED_DELAY_TICKS);
		led_set(LED_GREEN2, loop_end_ts - mounted_ts <= LED_DELAY_TICKS);

		if (loop_end_ts - can_loop_ts >= pdMS_TO_TICKS(250)) {
			can_loop_ts = loop_end_ts;
			led_toggle(LED_ORANGE1);
		}
	}
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
	const uint8_t TICK_MS = 16;

	while (42) {
		for (uint8_t i = 0; i < TU_ARRAY_SIZE(leds); ++i) {
			switch (leds[i].mode) {
			case LED_MODE_BLINK: {
				uint16_t delay = leds[i].blink_delay_ms;
				// TU_LOG2("led %s (%u) blink delay %u left %u\n", leds[i].name, i, delay, left[i]);
				if (delay) {
					if (0 == left[i]) {
						// TU_LOG2("led %s (%u) toggle\n", leds[i].name, i);
						gpio_toggle_pin_level(leds[i].pin);
						left[i] = delay / TICK_MS;
					} else {
						--left[i];
					}
				}
				// } else {
				// 	goto off;
				// }
			} break;
// 			case LED_MODE_ON:
// 				// TU_LOG2("led %s (%u) on\n", leds[i].name, i);
// 				gpio_set_pin_level(leds[i].pin, 1);
// 				left[i] = 0;
// 				break;

// 			default:
// 			case LED_MODE_OFF:
// off:
// 				// TU_LOG2("led %s (%u) off\n", leds[i].name, i);
// 				gpio_set_pin_level(leds[i].pin, 0);
// 				left[i] = 0;
// 				break;
			}
		}

		vTaskDelay(pdMS_TO_TICKS(TICK_MS));
	}
}

//--------------------------------------------------------------------+
// PEAK
//--------------------------------------------------------------------+


#define PCAN_USB_STATUSLEN_TIMESTAMP	(1 << 7)
#define PCAN_USB_STATUSLEN_INTERNAL	(1 << 6)
#define PCAN_USB_STATUSLEN_EXT_ID	(1 << 5)
#define PCAN_USB_STATUSLEN_RTR		(1 << 4)
#define PCAN_USB_STATUSLEN_DLC		(0xf)

#define PCAN_USB_REC_ERROR		1
#define PCAN_USB_REC_ANALOG		2
#define PCAN_USB_REC_BUSLOAD		3
#define PCAN_USB_REC_TS			4
#define PCAN_USB_REC_BUSEVT		5

#define PEAK_REPLY_PACK_RECORD_COUNTER_OFFSET 1

/* PCAN-USB error flags */
#define PCAN_USB_ERROR_TXFULL		0x01
#define PCAN_USB_ERROR_RXQOVR		0x02
#define PCAN_USB_ERROR_BUS_LIGHT	0x04
#define PCAN_USB_ERROR_BUS_HEAVY	0x08
#define PCAN_USB_ERROR_BUS_OFF		0x10
#define PCAN_USB_ERROR_RXQEMPTY		0x20
#define PCAN_USB_ERROR_QOVR		0x40
#define PCAN_USB_ERROR_TXQFULL		0x80



static uint8_t reply_first_timestamp;


static inline void peak_reply_pack_init(void)
{
	usb_out_buffer[0] = 0;
	usb_out_buffer[PEAK_REPLY_PACK_RECORD_COUNTER_OFFSET] = 0;
	usb_out_offset = 2;
	reply_first_timestamp = true;
}

static inline bool peak_reply_pack_fits(uint8_t bytes)
{
	return usb_out_offset + bytes < TU_ARRAY_SIZE(usb_out_buffer);
}

static inline void peak_reply_pack_append_unchecked(void const * ptr, uint8_t len)
{
	memcpy(usb_out_buffer + usb_out_offset, ptr, len);
	usb_out_offset += len;
}

// static inline bool peak_reply_pack_append(uint8_t f, uint8_t n, void const *ptr, uint8_t len)
// {
// 	if (tu_likely((size_t)(2 + len + reply_offset) <= TU_ARRAY_SIZE(reply_buffer))) {
// 		++reply_buffer[PEAK_REPLY_PACK_RECORD_COUNTER_OFFSET];
// 		reply_buffer[reply_offset++] = f;
// 		reply_buffer[reply_offset++] = n;
// 		if (len) {
// 			peak_reply_pack_append_unchecked(ptr, len);
// 		}
// 		return true;
// 	}

// 	return false;
// }
// static inline bool peak_reply_pack_append_status_internal(uint8_t f, uint8_t n, void const *ptr, uint8_t len)
// {
// 	if (tu_likely((size_t)(1 + len + reply_offset) <= TU_ARRAY_SIZE(reply_buffer))) {
// 		++reply_buffer[PEAK_REPLY_PACK_RECORD_COUNTER_OFFSET];
// 		reply_buffer[reply_offset++] = (len & PCAN_USB_STATUSLEN_DLC) | PCAN_USB_STATUSLEN_INTERNAL;
// 		if (len) {
// 			peak_reply_pack_append_unchecked(ptr, len);
// 		}
// 		return true;
// 	}

// 	return false;
// }

static inline bool peak_reply_pack_append_can_frame(
	uint32_t can_id,
	bool eff,
	bool rtr,
	void const *ptr,
	uint8_t dlc,
	uint16_t timestamp)
{
	uint8_t len = 3;
	uint8_t status = dlc + 2;
	if (eff) {
		status |= PCAN_USB_STATUSLEN_EXT_ID;
		status += 2;
		len += 2;
	}

	if (rtr) {
		status |= PCAN_USB_STATUSLEN_RTR;
	} else {
		len += dlc;
	}

	len += !reply_first_timestamp;

	if (!peak_reply_pack_fits(len)) {
		return false;
	}

	// inc records
	++usb_out_buffer[PEAK_REPLY_PACK_RECORD_COUNTER_OFFSET];

	usb_out_buffer[usb_out_offset++] = status;

	if (eff) {
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
		can_id = tu_bswap32(can_id);
#endif
		memcpy(&usb_out_buffer[usb_out_offset], &can_id, sizeof(can_id));
		usb_out_offset += sizeof(can_id);
	} else {
		uint16_t sff = can_id;
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
		sff = tu_bswap16(sff);
#endif
		memcpy(&usb_out_buffer[usb_out_offset], &sff, sizeof(sff));
		usb_out_offset += sizeof(sff);
	}

	if (reply_first_timestamp) {
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
		timestamp = tu_bswap16(timestamp);
#endif
		memcpy(&usb_out_buffer[usb_out_offset], &timestamp, sizeof(timestamp));
		usb_out_offset += sizeof(timestamp);
	} else {
		usb_out_buffer[usb_out_offset++] = timestamp;
	}

	if (!rtr) {
		memcpy(&usb_out_buffer[usb_out_offset], ptr, len);
		usb_out_offset += len;
	}

	return true;

}


static inline void peak_process_msg1(struct peak_cmd *cmd)
{
	TU_LOG2("cmd %02d %02d\n", cmd->func, cmd->num);

	switch (cmd->func) {
	case 1:
		switch (cmd->num) {
		case 2: { // pcan_usb_set_bittiming argv[0] == btr0, argv[1] == btr1
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

			configure_can0(bitrate_to_nbtp(bps/1000));
		} break;
		}
		break;
	case 2:
		switch (cmd->num) {
		case 1: // can frame

			break;
		}
		break;
	case 3:
		switch (cmd->num) {
		case 2: // pcan_usb_set_bus argv[0] == on/off
			TU_LOG2("set bus %d\n", cmd->argv[0]);
			if (cmd->argv[0]) {
				REG_CAN0_CCCR |= CAN_CCCR_INIT;
			} else {
				REG_CAN0_CCCR &= ~CAN_CCCR_INIT;
			}
			break;
		case 3: // pcan_usb_set_silent argv[0] == on/off
			break;
		}
		break;
	case 4:
		switch (cmd->num) {
		case 1: { // pcan_usb_get_device_id
			TU_LOG2("get device id\n");
			usb_out_buffer[0] = 0;
			usb_out_buffer[1] = 0;
			usb_out_buffer[2] = 0;
			usb_out_offset = 3;
		} break;

		}
		break;
	case 6:
		switch (cmd->num) {
		case 1: { // pcan_usb_get_serial
			TU_LOG2("get serial\n");
			uint32_t serial = 0xdeadbeef;
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
			serial = tu_bswap32(serial);
#endif
			usb_out_buffer[0] = 0;
			usb_out_buffer[1] = 0;
			memcpy(&usb_out_buffer[2], &serial, sizeof(serial));
			usb_out_offset = 6;
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
		// num 2 pcan_usb_set_ext_vcc argv[0] == on/off
		break;
	}
}

static void peak_process_msg(void)
{
	TU_LOG2("peak_process_msg\n");
	struct peak_cmd cmd;
	bool process;
	peak_reply_pack_init();

	do {

		uint32_t r = tud_vendor_read(&cmd, sizeof(cmd));
		process = r == sizeof(cmd);
		if (process) {
			peak_process_msg1(&cmd);
		}
	} while (process);

	tud_vendor_write(usb_out_buffer, usb_out_offset);
}

