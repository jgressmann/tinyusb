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
//#include <timers.h>
//#include <queue.h>
#include <semphr.h>

#include <bsp/board.h>
#include <tusb.h>


#include <sam.h>

#include <hal/include/hal_gpio.h>

#include <slipstream.h>
#include <supercan.h>


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

// static union LZ4_stream_u lz4_stream_mem;
// static LZ4_stream_t *lz4_stream;


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

	REG_CAN0_CCCR |= CAN_CCCR_BRSE | CAN_CCCR_FDOE;// enable bit rate switching and FD operation
	/* TX Buffer Configuration */
	//REG_CAN0_TXBC = CAN_TXBC_TBSA((uint32_t) can0_tx_buffer) | CAN_TXBC_NDTB(CAN0_TX_BUFFER_NUM); // dedicated tx buffers
	REG_CAN0_TXBC = CAN_TXBC_TBSA((uint32_t) can0_tx_buffer) | CAN_TXBC_TFQS(CAN0_TX_BUFFER_NUM);
	//	REG_CAN0_TXBC |= CAN_TXBC_TFQM; // reset default
	REG_CAN0_TXESC = CAN_TXESC_TBDS_DATA64; // 8 byte data field

	/* RX FIFO Configuration */
	REG_CAN0_RXF0C = CAN_RXF0C_F0SA((uint32_t) can0_rx_fifo) | CAN_RXF0C_F0S(CAN0_RX_FIFO_NUM) | CAN_RXF0C_F0OM; // FIFO 0 overwrite mode
	REG_CAN0_RXESC = CAN_RXESC_RBDS_DATA64 + CAN_RXESC_F0DS_DATA64; // Buffer and FIFO Element size config

	/* ID filter buffer configuration */
	//REG_CAN0_SIDFC = CAN_SIDFC_FLSSA((uint32_t) can0_standard_id_filter) | CAN_SIDFC_LSS(CAN0_STANDARD_ID_FILTER_NUM);

	/* Timing setting. */
	//	REG_CAN0_NBTP =	CAN_NBTP_NBRP(0) | CAN_NBTP_NTSEG1(24) | CAN_NBTP_NTSEG2(5) |	CAN_NBTP_NSJW(5);
	//	REG_CAN0_DBTP = CAN_DBTP_DBRP(0) | CAN_DBTP_DTSEG1(4) | CAN_DBTP_DTSEG2(1) | CAN_DBTP_DSJW(1);
	//REG_CAN0_NBTP = CAN_NBTP_NBRP(2) | CAN_NBTP_NTSEG1(62) | CAN_NBTP_NTSEG2(15) | CAN_NBTP_NSJW (15); /* 500kBit @ 120 / 3 = 40MHz, 80% */
	REG_CAN0_DBTP = CAN_DBTP_DBRP(2) | CAN_DBTP_DTSEG1(12) | CAN_DBTP_DTSEG2(5) | CAN_DBTP_DSJW (5); /* 2MBit @ 120 / 3 = 40MHz, 70% */
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


static StackType_t sc_task_stack[2*configMINIMAL_STACK_SIZE];
static StaticTask_t sc_task_mem;

static StackType_t led_task_stack[configMINIMAL_STACK_SIZE];
static StaticTask_t led_task_mem;

static void usb_device_task(void* param);
static void sc_task(void* param);
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



static struct {
	uint32_t rx_can_frames_dropped;
	uint32_t tx_can_frames_dropped;
	uint32_t tx_can_frames_queued;
	uint32_t tx_usb_frames_dropped;
	uint32_t tx_can_errors;
	uint32_t rx_can_errors;
	uint16_t bitrate;
	uint8_t message_buffer_size;
	volatile bool mounted;
	volatile bool handshake_complete;
} sc_context;

static struct slipstream_frame slipstream_snd_queue[8];

static struct {
	uint8_t current_message_size;
	uint8_t current_message_offset;
	uint8_t index_put;
	uint8_t index_get;
	uint8_t seq_ack;
} slipstream_context;


static _Alignas(4) uint8_t slipstream_rx_message_assembly_buffer[sizeof(struct slipstream_message) + SC_MAX_MESSAGE_SIZE];
// static _Alignas(4) uint8_t slipstream_tx_message_assembly_buffer[sizeof(slipstream_message) + SC_MAX_MESSAGE_SIZE];

static StaticSemaphore_t slipstream_context_lock_mem;
static SemaphoreHandle_t slipstream_context_lock;





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

	// lz4_stream = LZ4_initStream(&lz4_stream_mem, sizeof(lz4_stream_mem));
	// LZ4_compress_fast_extState(lz4_stream. )
	slipstream_context.index_get = 0;
	slipstream_context.index_put = 0;
	slipstream_context.seq_ack = 0xff;
	for (size_t i = 0; i < TU_ARRAY_SIZE(slipstream_snd_queue); ++i) {
		slipstream_snd_queue[i].header.version = SLIPSTREAM_VERSION;
		slipstream_snd_queue[i].header.seq = i;
	}

	slipstream_context_lock = xSemaphoreCreateRecursiveMutexStatic(&slipstream_context_lock_mem);

	board_init();

	init_can0_pins();
	init_can0_clock();

	sc_context.bitrate = 500;
	init_can0(bitrate_to_nbtp(sc_context.bitrate));

	tusb_init();

	(void) xTaskCreateStatic(&usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-1, usb_device_stack, &usb_device_taskdef);
	(void) xTaskCreateStatic(&sc_task, "sc", TU_ARRAY_SIZE(sc_task_stack), NULL, configMAX_PRIORITIES-1, sc_task_stack, &sc_task_mem);
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
	sc_context.mounted = true;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	TU_LOG2("unmounted\n");
	sc_context.mounted = false;
	sc_context.handshake_complete = false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us	to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	TU_LOG2("suspend\n");
	sc_context.mounted = false;

}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	TU_LOG2("resume\n");
	sc_context.mounted = false;
}


// static inline void sc_process_buffer(uint8_t *ptr, uint16_t size)
// {
// 	uint8_t *beg = ptr;
// 	uint8_t *end = ptr + size;

// 	uint8_t error = SC_ERROR_NONE;

// 	// xSemaphoreTakeRecursive(slipstream_context_lock, ~0);

// 	uint8_t index = slipstream_context.index_put;
// 	void* data_ptr = slipstream_snd_queue[index].data;
// 	slipstream_context.index_put = (slipstream_context.index_put + 1) % TU_ARRAY_SIZE(slipstream_snd_queue);



// 	if (!sc_context.handshake_complete) {
// 		TU_LOG2("handshake incomplete\n");
// 		if ((size_t)(end - beg) >= sizeof(struct sc_hello_req)) {
// 			struct sc_hello_req *msg = (struct sc_hello_req *)beg;

// 			if (SC_MSG_HELLO == msg->id) {
// 				TU_LOG2("SC_MSG_HELLO\n");

// 				if (SC_VERSION == msg->version) {
// 					struct sc_hello_res *rep = data_ptr;
// 					rep->id = SC_MSG_HELLO;
// 					rep->byte_order = SC_BYTE_ORDER_LE;

// 					// handshake done
// 					sc_context.handshake_complete = true;

// 					 beg += sizeof(*msg);
// 				} else {
// 					error = SC_ERROR_VERSION_MISMATCH;
// 					goto send_error;
// 					// struct sc_msg_error *rep = (struct sc_msg_error *)slipstream_snd_queue[index].data;
// 					// rep->id = SC_MSG_HELLO;
// 					// rep->version = SC_VERSION;
// 					// rep->byte_order = SC_BYTE_ORDER_LE;
// 					// rep->error = SC_VERSION == msg->version ? SC_ERROR_NONE : SC_ERROR_VERSION_MISMATCH;

// 				}

// 				TU_LOG2("handshake comlete? %d\n", sc_context.handshake_complete);
// 			} else {
// 				error = SC_ERROR_HANDSHAKE_INCOMPLETE;
// 				goto send_error;
// 			}


// 		}
// 	}

// 	if (sc_context.handshake_complete) {
// 		if (message_buffer_size) {

// 		}
// 		while (end > beg) {
// 			switch (*beg) {
// 			case SC_MSG_ECHO: {
// 				// if (end - beg < sizeof(struct sc_msg_echo)) {
// 				// 	error =
// 				// }
// 				struct sc_msg_echo *msg = (struct sc_msg_echo *)beg;
// 				if (msg->len) {
// 					if (0 == msg->data[msg->len-1]) {
// 						TU_LOG2("echo: %s\n", (const char*)msg->data);
// 					}
// 				}
// 				beg += sizeof(struct sc_msg_echo) + msg->len;
// 				memcpy(data_ptr)
// 			} break;
// 			default:
// 				goto send_error;
// 			}
// 		}
// 	}

// out:
// 	// xSemaphoreGiveRecursive(slipstream_context_lock);
// 	return;

// send_error:
// 	{
// 		struct sc_msg_error *rep = data_ptr;
// 		rep->id = SC_MSG_ERROR;
// 		rep->version = SC_VERSION;
// 		rep->code = error;
// 	}
// 	goto out;
// }

static inline void slipstream_send_message(uint8_t *ptr, uint8_t size)
{
	uint16_t total_bytes = size + sizeof(struct slipstream_message);
	uint8_t messages = total_bytes / SLIPSTREAM_MAX_PAYLOAD;
	messages += messages * SLIPSTREAM_MAX_PAYLOAD != total_bytes;

	uint8_t slots_left = slipstream_context.index_get == slipstream_context.index_put
					? TU_ARRAY_SIZE(slipstream_snd_queue)
					: (slipstream_context.index_get - slipstream_context.index_put) % TU_ARRAY_SIZE(slipstream_snd_queue);

	slots_left -= 1; // when full put + 1 == get


	if (slots_left >= messages) {
		// fill in message header
		struct slipstream_message *msg = (struct slipstream_message *)slipstream_snd_queue[slipstream_context.index_put].data;
		msg->len = size;
		slipstream_snd_queue[slipstream_context.index_put].header.len = sizeof(*msg);

		// copy data
		for (uint8_t offset = sizeof(*msg); size; offset = 0) {
			uint8_t *out_ptr = slipstream_snd_queue[slipstream_context.index_put].data;
			uint8_t left = SLIPSTREAM_MAX_PAYLOAD - offset;
			uint8_t copy = tu_min8(left, size);
			memcpy(out_ptr + offset, ptr, copy);
			ptr += copy;
			size -= copy;
			slipstream_snd_queue[slipstream_context.index_put].header.len += copy;

			slipstream_context.index_put = (slipstream_context.index_put + 1) % TU_ARRAY_SIZE(slipstream_snd_queue);
		}
	} else {
		TU_LOG1("SLIPSTREAM: queue full, msg dropped\n");
		++sc_context.tx_usb_frames_dropped;
	}
}

static inline void sc_process_buffer(uint8_t *ptr, uint8_t size)
{
	TU_ASSERT(size, );

	uint8_t error = SC_ERROR_NONE;
	uint8_t reply_size = 0;
	uint8_t *reply_ptr = NULL;

	if (sc_context.handshake_complete) {
		switch (*ptr) {
		case SC_MSG_HELLO:
			TU_LOG2("SUPERCAN: re-shake hands\n");
			goto handshake;
			break;
		case SC_MSG_ECHO: {
			TU_LOG2("SUPERCAN: SC_MSG_ECHO\n");
			struct sc_msg_echo *msg = (struct sc_msg_echo *)ptr;
			uint8_t len = size - sizeof(*msg);
			if (len) {
				if (0 == msg->data[len-1]) {
					TU_LOG2("SUPERCAN: echo: %s\n", (const char*)msg->data);
				}
			}
			reply_ptr = ptr;
			reply_size = size;
		} break;
		}
	} else {
		TU_LOG2("SUPERCAN: handshake incomplete\n");
handshake:
		if (size == sizeof(struct sc_hello_req)) {
			struct sc_hello_req *msg = (struct sc_hello_req *)ptr;

			if (SC_MSG_HELLO == msg->id) {
				TU_LOG2("SUPERCAN: SC_MSG_HELLO\n");

				if (SC_VERSION == msg->version) {
					reply_ptr = ptr;
					struct sc_hello_res *rep = (struct sc_hello_res *)reply_ptr;
					rep->id = SC_MSG_HELLO;
#if TU_LITTLE_ENDIAN == TU_BYTE_ORDER
					rep->byte_order = SC_BYTE_ORDER_LE;
#else
					rep->byte_order = SC_BYTE_ORDER_BE;
#endif
					reply_size = sizeof(*rep);
					// handshake done
					sc_context.handshake_complete = true;

					TU_LOG2("SUPERCAN: handshake done\n");
				} else {
					TU_LOG2("SUPERCAN: handshake failed (version mismatch)\n");
					error = SC_ERROR_VERSION_MISMATCH;
					goto send_error;
				}
			} else {
				TU_LOG2("SUPERCAN: refusing message id %02x, handshake incomplete\n", msg->id);
				error = SC_ERROR_HANDSHAKE_INCOMPLETE;
				goto send_error;
			}
		} else {
			TU_LOG2("SUPERCAN: refusing message, handshake incomplete\n");
			error = SC_ERROR_HANDSHAKE_INCOMPLETE;
			goto send_error;
		}
	}

out:
	slipstream_send_message(reply_ptr, reply_size);
	return;

send_error:
	{
		reply_ptr = ptr;
		struct sc_msg_error *rep = (struct sc_msg_error *)reply_ptr;
		rep->id = SC_MSG_ERROR;
		rep->version = SC_VERSION;
		rep->code = error;
		reply_size = sizeof(*rep);
	}
	goto out;
}

static inline void slipstream_process_messages(uint8_t *ptr, uint8_t size)
{
	while (size) {
		if (slipstream_context.current_message_size) {
			uint8_t left = slipstream_context.current_message_size - slipstream_context.current_message_offset;
			uint8_t copy = tu_min8(left, size);
			memcpy(&slipstream_rx_message_assembly_buffer[slipstream_context.current_message_offset], ptr, left);
			ptr += copy;
			size -= copy;
			slipstream_context.current_message_offset += copy;

			if (left == copy) {
				// message complete
				TU_LOG2("SLIPSTREAM: complete msg size %u\n", slipstream_context.current_message_size);
				sc_process_buffer(slipstream_rx_message_assembly_buffer, slipstream_context.current_message_size);

				slipstream_context.current_message_size = 0;
				slipstream_context.current_message_offset = 0;
			}
		// } else if (((uintptr_t)ptr) & 1) { // align to uint16_t
		// 	++ptr;
		// 	--size;
		} else {
			TU_ASSERT(size >= sizeof(struct slipstream_message), );
			struct slipstream_message *msg = (struct slipstream_message *)ptr;
			//slipstream_context.current_message_size = tu_ntohs(msg->len);
			slipstream_context.current_message_size = msg->len;
			ptr += sizeof(struct slipstream_message);
			size -= sizeof(struct slipstream_message);
			// we don't expect empty messages
			TU_ASSERT(slipstream_context.current_message_size, );

			TU_LOG2("SLIPSTREAM: start msg size %u\n", slipstream_context.current_message_size);
		}
	}
}

static inline void slipstream_process_frame(void)
{
	struct slipstream_frame msg;
	uint32_t r = tud_vendor_read(&msg, sizeof(msg));
	if (r < sizeof(struct slipstream_frame_header)) {
		TU_LOG1("SLIPSTREAM: expect frame of size %u bytes, got %u\n", (unsigned)sizeof(struct slipstream_frame_header), (unsigned)r);
		return;
	}

	if (msg.header.version != SLIPSTREAM_VERSION) {
		TU_LOG1("SLIPSTREAM: unsupported protocol version %u\n", msg.header.version);
		return;
	}

	if (r - sizeof(struct slipstream_frame_header) < msg.header.len) {
		TU_LOG1("SLIPSTREAM: malformed frame, length in header (%u) exceeds actual data length (%u)\n", msg.header.len, (unsigned)(r - sizeof(struct slipstream_frame_header)));
		return;
	}

	if (msg.header.len > sizeof(msg.data)) {
		TU_LOG1("SLIPSTREAM: malformed frame, len > sizeof(data) %u\n", msg.header.len);
		return;
	}

	if (msg.header.ack >= TU_ARRAY_SIZE(slipstream_snd_queue)) {
		TU_LOG1("SLIPSTREAM: malformed frame, ack (%u) >= sizeof(send queue) (%u)\n", msg.header.ack, (unsigned)TU_ARRAY_SIZE(slipstream_snd_queue));
		return;
	}

	xSemaphoreTakeRecursive(slipstream_context_lock, ~0);

	// update get index if host ack'd
	if (slipstream_context.index_get != 0xff) {
		if (msg.header.ack == slipstream_context.index_get) {
			slipstream_context.index_get = (slipstream_context.index_get + 1) % TU_ARRAY_SIZE(slipstream_snd_queue);
		}
	}

	if (msg.header.len) {
		slipstream_process_messages(msg.data, msg.header.len);
	}

	struct slipstream_frame *reply;

	if (slipstream_context.index_get == slipstream_context.index_put) {
		reply = &slipstream_snd_queue[slipstream_context.index_put];
		slipstream_context.index_put = (slipstream_context.index_put + 1) % TU_ARRAY_SIZE(slipstream_snd_queue);;
		// zero len reply
		reply->header.len = 0;
	} else {
		reply = &slipstream_snd_queue[slipstream_context.index_get];
	}

	reply->header.ack = msg.header.seq;

	uint32_t w = tud_vendor_write(reply, sizeof(*reply));
	TU_ASSERT(sizeof(*reply) == w, );

	xSemaphoreGiveRecursive(slipstream_context_lock);
}


// Invoked when vendor interface received data from host
void tud_vendor_rx_cb(uint8_t itf)
{
	(void) itf;
	slipstream_process_frame();
}


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

	// // format the can message
	// uint32_t id = r0.bit.ID;
	// id >>= r0.bit.XTD ? 0 : 18;
	// // mcba_format_can_msg(id, e->data, r1.bit.DLC, r0.bit.XTD, r0.bit.RTR, xTaskGetTickCount(), &msg);

	REG_CAN0_RXF0A = CAN_RXF0A_F0AI(index);

	// tud_vendor_write(&msg, sizeof(msg));
}

static inline void can_clear1(void)
{
	uint8_t index = CAN0->RXF0S.bit.F0GI;
	REG_CAN0_RXF0A = CAN_RXF0A_F0AI(index);
}

static inline bool mcba_process_usb(void)
{
	bool result = false;
	// while (tud_vendor_available() >= sizeof(struct mcba_usb_msg) &&
	// 		tud_vendor_write_available() == CFG_TUD_VENDOR_TX_BUFSIZE) {
	// 		mcba_process_usb_msg1();
	// 		result = true;
	// }

	return result;
}

//--------------------------------------------------------------------+
// SUPERCAN TASK
//--------------------------------------------------------------------+
static void sc_task(void *param)
{
	(void) param;

	bool work;

	TickType_t start = xTaskGetTickCount();

	while (42) {

		work = false;
		// if (mcba_process_usb()) {
		// 	work = true;
		// }


		// update error counters
		CAN_ECR_Type ecr = (CAN_ECR_Type)REG_CAN0_ECR;
		sc_context.tx_can_errors += ecr.bit.TEC;
		sc_context.rx_can_errors += ecr.bit.REC;
		gpio_set_pin_level(DEBUG_LED4_PIN, ecr.bit.REC > 0);
		gpio_set_pin_level(DEBUG_LED3_PIN, ecr.bit.TEC > 0);

		if (can_msg_fifo_avail()) {
			gpio_set_pin_level(DEBUG_LED5_PIN, 1);
			work = true;
			if (sc_context.mounted && sc_context.handshake_complete) {
				gpio_set_pin_level(DEBUG_LED6_PIN, 1);
				// if (tud_vendor_write_available() == CFG_TUD_VENDOR_TX_BUFSIZE) {
				can_process1();

				// } else {
				// 	++sc_context.rx_can_frames_dropped;
				// }
			} else {
				can_clear1();
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
// LED TASK
//--------------------------------------------------------------------+
static void led_task(void *param)
{
	(void) param;

	bool state = false;
	board_led_write(state);

	while (42) {
		if (sc_context.mounted) {
			vTaskDelay(pdMS_TO_TICKS(250));
		} else {
			vTaskDelay(pdMS_TO_TICKS(1000));
		}

		state = !state;
		board_led_write(state);
	}
}
