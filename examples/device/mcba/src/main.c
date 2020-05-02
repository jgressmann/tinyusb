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
#include <timers.h>
#include <queue.h>
#include <semphr.h>

#include <bsp/board.h>
#include <tusb.h>

#include <sam.h>

#include <hal/include/hal_gpio.h>

#include "mcba_protocol.h"


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


static inline void init_can0(uint32_t nbtp)
{
	REG_CAN0_CCCR |= CAN_CCCR_INIT; // set CAN-Module to init, Reset-default
	while((REG_CAN0_CCCR & CAN_CCCR_INIT) == 0);

	REG_CAN0_CCCR |= CAN_CCCR_CCE; // set CCE bit to change config
	while((REG_CAN0_CCCR & CAN_CCCR_CCE) == 0);

// MCBA is Classic CAN only
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
	//REG_CAN0_DBTP = CAN_DBTP_DBRP(2) | CAN_DBTP_DTSEG1(12) | CAN_DBTP_DTSEG2(5) | CAN_DBTP_DSJW (5); /* 2MBit @ 120 / 3 = 40MHz, 70% */
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
static StaticTask_t usb_device_taskdef;

#define MASTER_TASK_STACK_SIZE (2*configMINIMAL_STACK_SIZE)
static StackType_t master_task_stack[MASTER_TASK_STACK_SIZE];
static StaticTask_t master_task_mem;

static StackType_t led_task_stack[configMINIMAL_STACK_SIZE];
static StaticTask_t led_task_mem;

static void usb_device_task(void* param);
static void master_task(void* param);
static void led_task(void* param);

static inline uint16_t byte_swap(uint16_t x)
{
	return (x >> 8) | (x << 8);
}

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



static struct {
	uint32_t rx_can_frames_dropped;
	uint32_t tx_can_frames_dropped;
	uint32_t tx_can_frames_queued;
	uint32_t tx_usb_frames_dropped;
	uint32_t tx_can_errors;
	uint32_t rx_can_errors;
	uint16_t bitrate;
	volatile bool mounted;
} mcba_context;


#define DEBUG_LED1_PIN      PIN_PB14
#define DEBUG_LED2_PIN      PIN_PB15
#define DEBUG_LED3_PIN      PIN_PA12
#define DEBUG_LED4_PIN      PIN_PA13
#define DEBUG_LED5_PIN      PIN_PA14
#define DEBUG_LED6_PIN      PIN_PA15

int main(void)
{
	PORT->Group[1].DIRSET.reg = PORT_PB14; /* Debug-LED */
	PORT->Group[1].DIRSET.reg = PORT_PB15; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA12; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA13; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA14; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA15; /* Debug-LED */

	board_init();

	init_can0_pins();
	init_can0_clock();

	mcba_context.bitrate = 500;
	init_can0(bitrate_to_nbtp(mcba_context.bitrate));

	tusb_init();

	(void) xTaskCreateStatic(&usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-1, usb_device_stack, &usb_device_taskdef);
	(void) xTaskCreateStatic(&master_task, "mcba", MASTER_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES-1, master_task_stack, &master_task_mem);
	(void) xTaskCreateStatic(&led_task, "led", TU_ARRAY_SIZE(led_task_stack), NULL, configMAX_PRIORITIES-1, led_task_stack, &led_task_mem);
	gpio_set_pin_level(DEBUG_LED1_PIN, 1);

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
	mcba_context.mounted = true;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	TU_LOG2("unmounted\n");
	mcba_context.mounted = false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us	to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	TU_LOG2("suspend\n");
	mcba_context.mounted = false;

}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	TU_LOG2("resume\n");
	mcba_context.mounted = false;
}

static inline void mcba_format_can_msg(
	uint32_t can_id,
	void const *ptr,
	uint8_t len,
	bool eff,
	bool rtr,
	uint32_t timestamp,
	struct mcba_usb_msg_can *msg)
{
	TU_ASSERT(ptr, );
	TU_ASSERT(msg, );

	msg->cmd_id = MBCA_CMD_RECEIVE_MESSAGE;
	msg->checksum = 0;

	uint16_t sid, eid;

	if (eff) {
		/* SIDH		| SIDL								 | EIDH	 | EIDL
		 * 28 - 21 | 20 19 18 x x x 17 16 | 15 - 8 | 7 - 0
		 */
		sid = MCBA_SIDL_EXID_MASK;
		/* store 28-18 bits */
		sid |= (can_id & 0x1ffc0000) >> 13;
		/* store 17-16 bits */
		sid |= (can_id & 0x30000) >> 16;

		/* store 15-0 bits */
		eid = can_id & 0xffff;
	} else {
		/* SIDH	 | SIDL
		 * 10 - 3 | 2 1 0 x x x x x
		 */
		sid = (can_id & 0x7fff) << 5;
		eid = 0;
	}

	uint16_t beSid = byte_swap(sid);
	uint16_t beEid = byte_swap(eid);
	memcpy(&msg->sid, &beSid, sizeof(beSid));
	memcpy(&msg->eid, &beEid, sizeof(beEid));

	msg->dlc = len;

	if (rtr) {
		msg->dlc |= MCBA_DLC_RTR_MASK;
	}

	memcpy(msg->data, ptr, len);
	memcpy(msg->timestamp, &timestamp, sizeof(timestamp));
}


static inline CAN_TXBE_0_Type to_t0(struct mcba_usb_msg_can const *msg)
{
	be16_t beSid;
	uint16_t sid;
	CAN_TXBE_0_Type result;
	memcpy(&beSid, &msg->sid, sizeof(beSid));
	sid = byte_swap(beSid);
	result.reg = 0;
	result.bit.RTR = (msg->dlc & MCBA_DLC_RTR_MASK);

	if (sid & MCBA_SIDL_EXID_MASK) {
		be16_t beEid;
		uint16_t eid;
		result.bit.XTD = 1;
		result.bit.ID = (sid & 0xffe0) << 13;
		result.bit.ID = (sid & 3) << 16;
		memcpy(&beEid, &msg->eid, sizeof(beEid));
		eid = byte_swap(beEid);
		result.bit.ID |= eid;
	} else {
		result.bit.ID = ((sid & 0xffe0) >> 5) << 18;
	}

	return result;
}

static inline void mcba_process_usb_msg1(void)
{
	struct mcba_usb_msg msg;
	TU_ASSERT(sizeof(msg) == 19, );
	unsigned available = tud_vendor_available();
	TU_ASSERT(available == 19, );

	tud_vendor_read(&msg, sizeof(msg));
	switch (msg.cmd_id) {
	case MBCA_CMD_TRANSMIT_MESSAGE_EV: {
		struct mcba_usb_msg_can* msg_can = (struct mcba_usb_msg_can*)&msg;

		CAN_TXFQS_Type reg = (CAN_TXFQS_Type)REG_CAN0_TXFQS;
		bool full = reg.bit.TFQF;
		if (full) {
			// TU_LOG2("tud_vendor_rx_cb: tx can fifo full\n");
		} else {
			// TU_LOG2("tud_vendor_rx_cb: place in tx can fifo at index %u\n", index);
			uint8_t index = reg.bit.TFQPI;
			struct can_tx_element* e = &can0_tx_buffer[index];

			e->T0 = to_t0(msg_can);
			e->T1.bit.DLC = msg_can->dlc & MCBA_DLC_MASK;
			memcpy(e->data, msg_can->data, sizeof(msg_can->data));
			REG_CAN0_TXBAR |= UINT32_C(1) << index;
		}


		if (full) {
			++mcba_context.tx_can_frames_dropped;
		} else {
			++mcba_context.tx_can_frames_queued;
		}

		msg_can->cmd_id = MBCA_CMD_TRANSMIT_MESSAGE_RSP;
		uint32_t ts = xTaskGetTickCount();
		memcpy(&msg_can->timestamp, &ts, sizeof(ts));
		tud_vendor_write(&msg, sizeof(msg));
	} break;
	case MBCA_CMD_CHANGE_BIT_RATE: {
		struct mcba_usb_msg_change_bitrate * msg_bitrate = (struct mcba_usb_msg_change_bitrate*)&msg;
		be16_t bitrateLe;
		memcpy(&bitrateLe, &msg_bitrate->bitrate, sizeof(bitrateLe));
		init_can0(bitrate_to_nbtp(byte_swap(bitrateLe)));
		goto send_bitrate_response;
	} break;
	case MBCA_CMD_SETUP_TERMINATION_RESISTANCE:
		TU_LOG2("not implemented\n");
		goto send_termination_response;
		break;
	case MBCA_CMD_READ_FW_VERSION: {
		struct mcba_usb_msg_fw_ver const *msg_fw_ver = (struct mcba_usb_msg_fw_ver const *)&msg;
		switch (msg_fw_ver->pic) {
send_termination_response:
		case MCBA_VER_REQ_USB: {
			struct mcba_usb_msg_ka_usb* msg_ka_usb = (struct mcba_usb_msg_ka_usb*)&msg;
			msg_ka_usb->cmd_id = MBCA_CMD_I_AM_ALIVE_FROM_USB;
			msg_ka_usb->soft_ver_major = TUSB_VERSION_MAJOR;
			msg_ka_usb->soft_ver_minor = TUSB_VERSION_MINOR;
			msg_ka_usb->termination_state = 0;
			tud_vendor_write(&msg, sizeof(msg));
		} break;
send_bitrate_response:
		case MCBA_VER_REQ_CAN: {
			struct mcba_usb_msg_ka_can* msg_ka_can = (struct mcba_usb_msg_ka_can*)&msg;
			CAN_CREL_Type crel = (CAN_CREL_Type)REG_CAN0_CREL;

			msg_ka_can->cmd_id = MBCA_CMD_I_AM_ALIVE_FROM_CAN;
			msg_ka_can->tx_err_cnt = 0;
			msg_ka_can->rx_err_cnt = 0;
			msg_ka_can->rx_buff_ovfl = 0;
			msg_ka_can->can_stat = 0;
			msg_ka_can->soft_ver_major = crel.bit.REL;
			msg_ka_can->soft_ver_minor = crel.bit.STEP;
			msg_ka_can->debug_mode = 0;
			msg_ka_can->test_complete = 0;
			msg_ka_can->test_result = 0;


			uint16_t bitrate = mcba_context.bitrate;
			msg_ka_can->tx_err_cnt = mcba_context.tx_can_errors;
			msg_ka_can->rx_err_cnt = mcba_context.rx_can_errors;
			msg_ka_can->rx_buff_ovfl = mcba_context.rx_can_frames_dropped;


			uint16_t beBitrate = byte_swap(bitrate);
			memcpy(&msg_ka_can->can_bitrate, &beBitrate, sizeof(beBitrate));
			uint16_t beRxLost = byte_swap(mcba_context.rx_can_frames_dropped);
			memcpy(&msg_ka_can->rx_lost, &beRxLost, sizeof(beRxLost));
			tud_vendor_write(&msg, sizeof(msg));
		} break;

		default:
			TU_LOG1("tud_vendor_rx_cb: unhandled request for fw pic %02x\n", msg_fw_ver->pic);
			msg.cmd_id = 0xff;
			tud_vendor_write(&msg, sizeof(msg));
			break;
		}
	} break;

	default:
		TU_LOG1("tud_vendor_rx_cb: unhandled cmd %02x\n", msg.cmd_id);
		msg.cmd_id = 0xff;
		tud_vendor_write(&msg, sizeof(msg));
		break;
	}
}

static inline bool mcba_can_msg_fifo_avail(void)
{
	return (REG_CAN0_RXF0S & CAN_RXF0S_F0FL_Msk) != 0;
}

static inline bool mcba_can_msg_fifo_full(void)
{
	return (REG_CAN0_RXF0S & CAN_RXF0S_F0F) != 0;
}

static inline void mcba_can_process1(void)
{
	struct mcba_usb_msg_can msg;
	TU_ASSERT(sizeof(msg) == 19, );

	uint8_t index = CAN0->RXF0S.bit.F0GI;
	struct can_rx_fifo_element const *e = &can0_rx_fifo[index];
	CAN_RXF0E_0_Type r0 = e->R0;
	CAN_RXF0E_1_Type r1 = e->R1;

	// format the can message
	uint32_t id = r0.bit.ID;
	id >>= r0.bit.XTD ? 0 : 18;
	mcba_format_can_msg(id, e->data, r1.bit.DLC, r0.bit.XTD, r0.bit.RTR, xTaskGetTickCount(), &msg);

	REG_CAN0_RXF0A = CAN_RXF0A_F0AI(index);

	tud_vendor_write(&msg, sizeof(msg));
}

static inline void mcba_can_clear1(void)
{
	uint8_t index = CAN0->RXF0S.bit.F0GI;
	REG_CAN0_RXF0A = CAN_RXF0A_F0AI(index);
}

static inline bool mcba_process_usb(void)
{
	bool result = false;
	while (tud_vendor_available() >= sizeof(struct mcba_usb_msg) &&
			tud_vendor_write_available() == CFG_TUD_VENDOR_TX_BUFSIZE) {
			mcba_process_usb_msg1();
			result = true;
	}

	return result;
}

//--------------------------------------------------------------------+
// MCBA TASK
//--------------------------------------------------------------------+
static void master_task(void *param)
{
	(void) param;

	bool work;

	TickType_t start = xTaskGetTickCount();

	while (42) {

		work = false;
		if (mcba_process_usb()) {
			work = true;
		}


		// update error counters
		CAN_ECR_Type ecr = (CAN_ECR_Type)REG_CAN0_ECR;
		mcba_context.tx_can_errors += ecr.bit.TEC;
		mcba_context.rx_can_errors += ecr.bit.REC;
		gpio_set_pin_level(DEBUG_LED4_PIN, ecr.bit.REC > 0);
		gpio_set_pin_level(DEBUG_LED3_PIN, ecr.bit.TEC > 0);

		if (mcba_can_msg_fifo_avail()) {
			gpio_set_pin_level(DEBUG_LED5_PIN, 1);
			work = true;
			if (mcba_context.mounted) {
				if (tud_vendor_write_available() == CFG_TUD_VENDOR_TX_BUFSIZE) {
					mcba_can_process1();
					gpio_set_pin_level(DEBUG_LED6_PIN, 1);
				} else {
					++mcba_context.rx_can_frames_dropped;
				}
			} else {
				mcba_can_clear1();
			}
		} else {
			gpio_set_pin_level(DEBUG_LED5_PIN, 0);
			gpio_set_pin_level(DEBUG_LED6_PIN, 0);
		}

		if (work) {
			vTaskDelay(1); // required to allow other tasks to run
		} else {
			vTaskDelay((33 * configTICK_RATE_HZ) / 1000000);
		}

		TickType_t now = xTaskGetTickCount();

		if (now - start > configTICK_RATE_HZ / 4) {
			gpio_set_pin_level(DEBUG_LED2_PIN, !gpio_get_pin_level(DEBUG_LED2_PIN));
			start = now;
		}
	}
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
static void led_task(void *param)
{
	(void) param;

	bool state = false;
	board_led_write(state);

	while (42) {
		if (mcba_context.mounted) {
			vTaskDelay(pdMS_TO_TICKS(250));
		} else {
			vTaskDelay(pdMS_TO_TICKS(1000));
		}

		state = !state;
		board_led_write(state);
	}
}
