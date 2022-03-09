/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
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

#include <FreeRTOS.h>
#include <task.h>


#include <tusb.h>
#include <sllin_board.h>
#include <usb_descriptors.h>
#include <leds.h>


enum {
	SLLIN_ERROR_TERMINATOR = 7,
	SLLIN_OK_TERMINATOR = 13,
	SLLIN_QUEUE_ELEMENT_TYPE_TIME_STAMP = SLLIN_QUEUE_ELEMENT_TYPE_COUNT,
};


SLLIN_RAMFUNC static void tusb_device_task(void* param);
SLLIN_RAMFUNC static void lin_usb_task(void *param);



static struct usb {
	StackType_t usb_device_stack[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t usb_device_stack_mem;
	uint8_t port;
	bool mounted;
} usb;

static struct lin {
	StackType_t usb_task_stack[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t usb_task_mem;
	TaskHandle_t usb_task_handle;
	sllin_queue_element rx_fifo[8];
	uint8_t rx_sl_buffer[129];
	uint8_t tx_sl_buffer[129];

	sllin_conf conf;

	uint8_t rx_sl_offset;
	uint8_t tx_sl_offset;
	uint8_t rx_fifo_gi; // not an index, uses full range of type
	uint8_t rx_fifo_pi; // not an index, uses full range of type

	bool tx_full;

	uint8_t rx_full;
	bool enabled;
	bool report_time_per_frame;
	bool report_time_periodically;
#if SLLIN_DEBUG
	int32_t last_ts;
#endif
} lins[SLLIN_BOARD_LIN_COUNT];

int main(void)
{
	// no uart here :(
	sllin_board_init_begin();
	LOG("sllin_board_init_begin\n");

	LOG("led_init\n");
	led_init();


	LOG("tusb_init\n");
	tusb_init();

	(void) xTaskCreateStatic(
		&tusb_device_task,
		"tusb",
		TU_ARRAY_SIZE(usb.usb_device_stack),
		NULL,
		configMAX_PRIORITIES-1,
		usb.usb_device_stack,
		&usb.usb_device_stack_mem);
	(void) xTaskCreateStatic(
		&led_task,
		"led",
		TU_ARRAY_SIZE(led_task_stack),
		NULL,
		configMAX_PRIORITIES-1,
		led_task_stack,
		&led_task_mem);


	sllin_board_init_end();
	LOG("sllin_board_init_end\n");

	for (unsigned i = 0; i < SLLIN_BOARD_LIN_COUNT; ++i) {
		struct lin *lin = &lins[i];

		lin->usb_task_handle = xTaskCreateStatic(
								&lin_usb_task,
								NULL,
								TU_ARRAY_SIZE(lin->usb_task_stack),
								(void*)(uintptr_t)i,
								configMAX_PRIORITIES-1,
								lin->usb_task_stack,
								&lin->usb_task_mem);

		lin->conf.master = false;
		lin->conf.sleep_timeout_ms = 4000;
		lin->conf.bitrate = 19200;
	}

	LOG("vTaskStartScheduler\n");
	vTaskStartScheduler();

	LOG("sllin_board_reset\n");
	sllin_board_reset();


	return 0;
}

void tud_mount_cb(void)
{
	LOG("mounted\n");
	led_blink(0, 250);
	usb.mounted = true;


	PORT->Group[2].OUTSET.reg |= 1ul << 7;
}

void tud_umount_cb(void)
{
	LOG("unmounted\n");
	led_blink(0, 1000);
	usb.mounted = false;

	PORT->Group[2].OUTCLR.reg |= 1ul << 7;
}

void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	LOG("suspend\n");
	usb.mounted = false;
	led_blink(0, 500);


	PORT->Group[2].OUTCLR.reg |= 1ul << 7;
}

void tud_resume_cb(void)
{
	LOG("resume\n");
	usb.mounted = true;
	led_blink(0, 250);

	PORT->Group[2].OUTCLR.reg |= 1ul << 7;
}

static inline uint8_t char_to_nibble(char c)
{
	if (likely(c >= '0' && c <= '9')) {
		return c - '0';
	}

	if (c >= 'a' && c <= 'f') {
		return (c - 'a') + 0xa;
	}

	return ((c - 'A') & 0xf) + 0xa;

}

static inline char nibble_to_char(uint8_t nibble)
{
	return "0123456789abcdef"[nibble & 0xf];
}



static void clear_rx_fifo(uint8_t index)
{
	struct lin *lin = NULL;

	lin = &lins[index];

	uint8_t pi = __atomic_load_n(&lin->rx_fifo_pi, __ATOMIC_ACQUIRE);
	__atomic_store_n(&lin->rx_fifo_gi, pi, __ATOMIC_RELAXED);
}

static void reset_channel_state(uint8_t index)
{
	struct lin *lin = NULL;

	lin = &lins[index];

	lin->rx_sl_offset = 0;
	lin->tx_sl_offset = 0;
	lin->enabled = false;
	lin->tx_full = false;
	lin->rx_full = 0;
	lin->report_time_per_frame = false;
	lin->report_time_periodically = false;
#if SLLIN_DEBUG
	lin->last_ts = -1;
#endif

	clear_rx_fifo(index);
}

SLLIN_RAMFUNC static inline void sllin_make_time_stamp_string(char *buffer, size_t size, uint16_t time_stamp_ms)
{
	SLLIN_DEBUG_ASSERT(size >= 5);

	buffer[4] = 0;
	buffer[3] = nibble_to_char(time_stamp_ms);
	time_stamp_ms >>= 4;
	buffer[2] = nibble_to_char(time_stamp_ms);
	time_stamp_ms >>= 4;
	buffer[1] = nibble_to_char(time_stamp_ms);
	time_stamp_ms >>= 4;
	buffer[0] = nibble_to_char(time_stamp_ms);
}

SLLIN_RAMFUNC static inline void sllin_store_tx_queue_full_error_response(uint8_t index)
{
	static const char can_crc_error_frame[] = "\x07T2000000480002000000000000"; // CAN_ERR_CRTL_TX_OVERFLOW
	struct lin *lin = NULL;

	lin = &lins[index];
	lin->tx_sl_offset = sizeof(can_crc_error_frame) - 1;
	memcpy(lin->tx_sl_buffer, can_crc_error_frame, lin->tx_sl_offset);

	if (lin->report_time_per_frame) {
		uint16_t ts = sllin_time_stamp_ms();

		sllin_make_time_stamp_string((char*)&lin->tx_sl_buffer[lin->tx_sl_offset], sizeof(lin->tx_sl_buffer) - lin->tx_sl_offset, ts);
		lin->tx_sl_offset += 4;
	}

	lin->tx_sl_buffer[lin->tx_sl_offset++] = SLLIN_OK_TERMINATOR;
}

SLLIN_RAMFUNC static void sllin_process_command(uint8_t index)
{
	// http://www.can232.com/docs/canusb_manual.pdf
	struct lin *lin = NULL;

	lin = &lins[index];

	lin->tx_sl_buffer[0] = SLLIN_OK_TERMINATOR;
	lin->tx_sl_offset = 1;

	// LOG("ch%u rx: %s\n", index, lin->rx_sl_buffer);

	switch (lin->rx_sl_buffer[0]) {
	case '\n':
		lin->tx_sl_offset = 0;
		break;
	case 'S': // CAN bitrate, values 0-8
		if (lin->enabled) {
			LOG("ch%u refusing to configure LIN when open\n", index);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else {
			if (unlikely(lin->rx_sl_offset < 2)) {
				LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
				unsigned seconds = lin->rx_sl_buffer[1] - '0';

				seconds = tu_min8(seconds, 6);

				lin->conf.sleep_timeout_ms = (4 + seconds) * 1000;
			}
		}
		break;
	case 's': // BTR0/BTR1 used to set bitrate
		if (unlikely(lin->rx_sl_offset < 4)) {
			LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else {
			if (lin->enabled) {
				LOG("ch%u refusing to configure LIN when open\n", index);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
				uint16_t bitrate = (char_to_nibble(lin->rx_sl_buffer[1]) << 12)
								| (char_to_nibble(lin->rx_sl_buffer[2]) << 8)
								| (char_to_nibble(lin->rx_sl_buffer[3]) << 4)
								| (char_to_nibble(lin->rx_sl_buffer[4]) << 0);

				if (lin->conf.bitrate <= 0)  {
					LOG("ch%u zero bitrate is invalid\n", index);
					lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
				} else {
					lin->conf.bitrate = bitrate;
					LOG("ch%u bitrate=%u\n", index, lin->conf.bitrate);
				}
			}
		}
		break;
	case 'L': // slave
	case 'O': // master
		clear_rx_fifo(index);
		lin->enabled = true;
		lin->conf.master = lin->rx_sl_buffer[0] == 'O';
		sllin_board_lin_init(index, &lin->conf);
		LOG("ch%u open %s bitrate=%u sleep timeout=%u [ms]\n", index, lin->conf.master ? "master" : "slave", lin->conf.bitrate, lin->conf.sleep_timeout_ms);
		break;
	case 'C':
		LOG("ch%u close\n", index);
		lin->enabled = false;
		tud_cdc_n_write_clear(index);
		break;
	case 't': // master: tx, slave: store response
		if (likely(lin->enabled)) {
			if (unlikely(lin->rx_sl_offset < 5)) {
				LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
				// fix me -> cleanup
				uint8_t can_id_nibble1 = char_to_nibble(lin->rx_sl_buffer[2]);
				uint8_t can_id_nibble2 = char_to_nibble(lin->rx_sl_buffer[3]);
				uint8_t frame_len = char_to_nibble(lin->rx_sl_buffer[4]);

				SLLIN_ASSERT(frame_len <= 8);

				if (unlikely(frame_len * 2 + 5 > lin->rx_sl_offset)) {
					LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
					lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
				} else {
					uint8_t id = ((can_id_nibble1 << 4) | can_id_nibble2) & 0x3f;
					uint8_t pid = sllin_id_to_pid(id);
					bool enhanced_crc = (can_id_nibble1 & 0x4) == 0x4;
					uint8_t data[8];
					unsigned crc = sllin_crc_start();
					uint8_t flags = 0;

					if (enhanced_crc) {
						crc = sllin_crc_update1(crc, pid);
						flags |= SLLIN_FRAME_FLAG_ENHANCED_CHECKSUM;
					}

					for (unsigned i = 0, j = 5; i < frame_len; ++i, j += 2) {
						uint8_t byte = (char_to_nibble(lin->rx_sl_buffer[j]) << 4) | char_to_nibble(lin->rx_sl_buffer[j+1]);

						data[i] = byte;
						crc = sllin_crc_update1(crc, byte);
					}

					crc = sllin_crc_finalize(crc);

					LOG("ch%u %s -> id=%x len=%u crc=%x flags=%x\n", index, lin->rx_sl_buffer, id, frame_len, crc, flags);

					if (lin->conf.master) {
						if (likely(sllin_board_lin_master_tx(index, id, frame_len, data, crc, flags))) {
							lin->tx_sl_buffer[0] = 'z';
							lin->tx_sl_buffer[1] = SLLIN_OK_TERMINATOR;
							lin->tx_sl_offset = 2;
						} else {
							sllin_store_tx_queue_full_error_response(index);
						}
					} else {
						sllin_board_lin_slave_tx(index, id, frame_len, data, crc, flags);
						lin->tx_sl_buffer[0] = 'z';
						lin->tx_sl_buffer[1] = SLLIN_OK_TERMINATOR;
						lin->tx_sl_offset = 2;
					}
				}
			}
		} else {
			LOG("ch%u refusing to transmit / store reponses when closed\n", index);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		}
		break;
	case 'r': // master: tx header
		if (likely(lin->enabled)) {
			if (unlikely(lin->rx_sl_offset < 5)) {
				LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else if (unlikely(!lin->conf.master)) {
				LOG("ch%u refusing to transmit in slave mode\n", index);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
				// fix me -> cleanup
				uint8_t can_id_nibble1 = char_to_nibble(lin->rx_sl_buffer[2]);
				uint8_t can_id_nibble2 = char_to_nibble(lin->rx_sl_buffer[3]);
				uint8_t id = ((can_id_nibble1 << 4) | can_id_nibble2) & 0x3f;
				bool enhanced_crc = (can_id_nibble1 & 0x4) == 0x4;
				uint8_t frame_len = char_to_nibble(lin->rx_sl_buffer[4]);
				uint8_t flags = 0;

				SLLIN_ASSERT(frame_len <= 8);

				if (enhanced_crc) {
					flags |= SLLIN_FRAME_FLAG_ENHANCED_CHECKSUM;
				}

				// LOG("ch%u %s -> id=%x len=%u flags=%x\n", index, lin->rx_sl_buffer, id, frame_len, flags);

				if (likely(sllin_board_lin_master_tx(index, id, frame_len, NULL, 0, flags))) {
					lin->tx_sl_buffer[0] = 'z';
					lin->tx_sl_buffer[1] = SLLIN_OK_TERMINATOR;
					lin->tx_sl_offset = 2;
				} else {
					sllin_store_tx_queue_full_error_response(index);
				}
			}
		} else {
			LOG("ch%u refusing to transmit / store reponses when closed\n", index);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		}
		break;
	case 'F': {
		uint8_t flags = 0;

		if (__atomic_fetch_and(&lin->rx_full, 0, __ATOMIC_ACQ_REL)) {
			flags |= 0x1;
		}

		if (lin->tx_full) {
			lin->tx_full = false;
			flags |= 0x2;
		}

		lin->tx_sl_buffer[0] = 'F';
		lin->tx_sl_buffer[1] = nibble_to_char(flags >> 4);
		lin->tx_sl_buffer[2] = nibble_to_char(flags);
		lin->tx_sl_buffer[3] = SLLIN_OK_TERMINATOR;
		lin->tx_sl_offset = 4;
	} break;
	case 'V': { // HW version?
		lin->tx_sl_buffer[0] = 'V';
		lin->tx_sl_buffer[1] = '0';
		lin->tx_sl_buffer[2] = '1';
		lin->tx_sl_buffer[3] = '0';
		lin->tx_sl_buffer[4] = '0';
		lin->tx_sl_buffer[5] = SLLIN_OK_TERMINATOR;
		lin->tx_sl_offset = 6;
	} break;
	case 'v': {
		lin->tx_sl_buffer[0] = 'V';
		lin->tx_sl_buffer[1] = '0';
		lin->tx_sl_buffer[2] = '1';
		lin->tx_sl_buffer[3] = nibble_to_char(SLLIN_VERSION_MAJOR);
		lin->tx_sl_buffer[4] = nibble_to_char(SLLIN_VERSION_MINOR);
		lin->tx_sl_buffer[5] = SLLIN_OK_TERMINATOR;
		lin->tx_sl_offset = 6;
	} break;
	case 'n': {
		size_t capa_len = sizeof(lin->tx_sl_buffer) - 2;

		usb_get_desc_string(4 + index, (char*)&lin->tx_sl_buffer[1], &capa_len);

		LOG("ch%u name: %s\n", index, &lin->tx_sl_buffer[1]);

		lin->tx_sl_buffer[0] = 'n';
		lin->tx_sl_buffer[1 + capa_len] = SLLIN_OK_TERMINATOR;
		lin->tx_sl_offset = 2 + capa_len;
	} break;
	case 'N': {
		uint32_t id = sllin_board_identifier();
		id = ((id >> 16) & 0xffff) ^ (id & 0xffff);
		lin->tx_sl_offset = usnprintf((char *)lin->tx_sl_buffer, sizeof(lin->tx_sl_buffer), "N%x%c", id, SLLIN_OK_TERMINATOR);
	} break;
	case 'Z':
		if (lin->enabled) {
			LOG("ch%u refusing to change time stamp setting when enabled\n", index);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else if (lin->rx_sl_offset < 2) {
			LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else {
			lin->report_time_per_frame = !(lin->rx_sl_buffer[1] == '0');
			LOG("ch%u report time stamp per frame %u\n", index, lin->report_time_per_frame);
		}
		break;
	case 'Y':
		if (lin->rx_sl_offset < 2) {
			LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else {
			lin->report_time_periodically = !(lin->rx_sl_buffer[1] == '0');
			LOG("ch%u report time stamp periodically %u\n", index, lin->report_time_periodically);
		}
		break;
	default:
		LOG("ch%u unhandled command '%s'\n", index, lin->rx_sl_buffer);
		lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		break;
	}
}

SLLIN_RAMFUNC static inline void process_rx_frame(uint8_t index, sllin_queue_element *e)
{
	struct lin *lin = &lins[index];
	uint16_t id = e->lin_frame.id;


	if (e->lin_frame.flags & SLLIN_FRAME_FLAG_ENHANCED_CHECKSUM) {
		id |= SLLIN_ID_FLAG_ENHANCED_CHECKSUM;
	}

	if (e->lin_frame.flags & SLLIN_FRAME_FLAG_FOREIGN) {
		id |= SLLIN_ID_FLAG_FOREIGN;
	}

	if (e->lin_frame.flags & SLLIN_FRAME_FLAG_MASTER_TX) {
		id |= SLLIN_ID_FLAG_MASTER_TX;
	}


	SLLIN_DEBUG_ASSERT(e->lin_frame.len <= 8);

#if SLLIN_DEBUG
	// if (lin->conf.master) {
	// 	char time_stamp_str[8];

	// 	sllin_make_time_stamp_string(time_stamp_str, sizeof(time_stamp_str), e->lin_frame.time_stamp_ms);
	// 	LOG("ch%u ts=%s\n", index, time_stamp_str);
	// }

	if (-1 != lin->last_ts) {
		uint16_t delta = e->lin_frame.time_stamp_ms - lin->last_ts;

		if (delta >= UINT16_MAX / 2) {
			LOG("ch%u last=%x curr=%x\n", index, lin->last_ts, e->lin_frame.time_stamp_ms);
		}
	}


	lin->last_ts = e->lin_frame.time_stamp_ms;
#endif

	if (e->lin_frame.flags & SLLIN_FRAME_FLAG_NO_RESPONSE) {
		lin->tx_sl_buffer[lin->tx_sl_offset++] = 'r';
		lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 8);
		lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 4);
		lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id);
		lin->tx_sl_buffer[lin->tx_sl_offset++] = '0' + e->lin_frame.len;
	} else if (e->lin_frame.flags & SLLIN_FRAME_FLAG_CRC_ERROR) {
		static const char can_crc_error_frame[] = "T2000000880000000800000000"; // CAN_ERR_PROT_LOC_CRC_SEQ

		lin->tx_sl_offset = sizeof(can_crc_error_frame) - 1;
		memcpy(lin->tx_sl_buffer, can_crc_error_frame, lin->tx_sl_offset);
	} else if (e->lin_frame.flags & SLLIN_FRAME_FLAG_PID_ERROR) {
		static const char can_pid_error_frame[] = "T2000000880000000F00000000"; // CAN_ERR_PROT_LOC_ID12_05

		lin->tx_sl_offset = sizeof(can_pid_error_frame) - 1;
		memcpy(lin->tx_sl_buffer, can_pid_error_frame, lin->tx_sl_offset);
	} else {
		lin->tx_sl_buffer[lin->tx_sl_offset++] = 't';
		lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 8);
		lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 4);
		lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id);
		lin->tx_sl_buffer[lin->tx_sl_offset++] = '0' + e->lin_frame.len;

		for (unsigned i = 0; i < e->lin_frame.len; ++i) {
			lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(e->lin_frame.data[i] >> 4);
			lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(e->lin_frame.data[i]);
		}
	}

	if (lin->report_time_per_frame) {
		sllin_make_time_stamp_string((char*)&lin->tx_sl_buffer[lin->tx_sl_offset], sizeof(lin->tx_sl_buffer) - lin->tx_sl_offset, e->lin_frame.time_stamp_ms);
		lin->tx_sl_offset += 4;
	}

	lin->tx_sl_buffer[lin->tx_sl_offset++] = SLLIN_OK_TERMINATOR;
}

SLLIN_RAMFUNC static inline void process_queue(uint8_t index, sllin_queue_element *e)
{
	struct lin *lin = &lins[index];

	switch (e->type) {
	case SLLIN_QUEUE_ELEMENT_TYPE_RX_FRAME:
		process_rx_frame(index, e);
		break;
	case SLLIN_QUEUE_ELEMENT_TYPE_TIME_STAMP: {
		if (lin->report_time_periodically) {
			lin->tx_sl_buffer[lin->tx_sl_offset++] = 'Y';
			sllin_make_time_stamp_string((char*)&lin->tx_sl_buffer[lin->tx_sl_offset], sizeof(lin->tx_sl_buffer) - lin->tx_sl_offset, e->lin_frame.time_stamp_ms);
			lin->tx_sl_offset += 4;
			lin->tx_sl_buffer[lin->tx_sl_offset++] = SLLIN_OK_TERMINATOR;
		}
	} break;
	case SLLIN_QUEUE_ELEMENT_TYPE_SLEEP:
	case SLLIN_QUEUE_ELEMENT_TYPE_WAKE_UP: {
		uint16_t id = e->type == SLLIN_QUEUE_ELEMENT_TYPE_WAKE_UP ? SLLIN_ID_FLAG_BUS_WAKE_UP : SLLIN_ID_FLAG_BUS_SLEEP;

		lin->tx_sl_buffer[lin->tx_sl_offset++] = 't';
		lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 8);
		lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 4);
		lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id);
		lin->tx_sl_buffer[lin->tx_sl_offset++] = '0'; // len

		if (lin->report_time_per_frame) {
			sllin_make_time_stamp_string((char*)&lin->tx_sl_buffer[lin->tx_sl_offset], sizeof(lin->tx_sl_buffer) - lin->tx_sl_offset, e->lin_frame.time_stamp_ms);
			lin->tx_sl_offset += 4;
		}

		lin->tx_sl_buffer[lin->tx_sl_offset++] = SLLIN_OK_TERMINATOR;
	} break;
	default:
		SLLIN_ASSERT(false && "unhandled queue element type");
		break;
	}
}

SLLIN_RAMFUNC static void tusb_device_task(void* param)
{
	(void) param;

	while (1) {
		LOG("tud_task\n");
		tud_task();
	}
}

SLLIN_RAMFUNC static void lin_usb_task(void* param)
{
	const uint8_t index = (uint8_t)(uintptr_t)param;
	SLLIN_ASSERT(index < TU_ARRAY_SIZE(lins));

	LOG("ch%u task start\n", index);

	struct lin *lin = &lins[index];
	bool written = false;

	while (42) {
		bool yield = false;

		(void)ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

		for (bool done = false; !done; ) {
			done = true;

			if (tud_cdc_n_connected(index)) {
				// int count = tud_cdc_n_available(index);
				// LOG("ch%u count=%d\n", index, count);
				int c = tud_cdc_n_read_char(index);
				uint8_t gi = lin->rx_fifo_gi;
				uint8_t pi = __atomic_load_n(&lin->rx_fifo_pi, __ATOMIC_ACQUIRE);

				if (-1 == c) {
					yield = true;
				} else {
					done = false;

					if (SLLIN_OK_TERMINATOR == c && lin->rx_sl_offset) {
						lin->rx_sl_buffer[lin->rx_sl_offset] = 0;
						sllin_process_command(index);
						lin->rx_sl_offset = 0;

						if (lin->tx_sl_offset) {
							uint32_t w = tud_cdc_n_write(index, lin->tx_sl_buffer, lin->tx_sl_offset);

							if (unlikely(w != lin->tx_sl_offset)) {
								lin->tx_full = true;
								LOG("ch%u TXF\n", index);
							}

							lin->tx_sl_offset = 0;

							if (!written) {
								xTaskNotifyGive(lin->usb_task_handle);
								written = true;
							}
						}
					} else {
						lin->rx_sl_buffer[lin->rx_sl_offset] = c;
						lin->rx_sl_offset = (lin->rx_sl_offset + 1) % (TU_ARRAY_SIZE(lin->rx_sl_buffer) - 1);
					}
				}

				if (pi != gi) {
					uint8_t fifo_index = gi % TU_ARRAY_SIZE(lin->rx_fifo);
					sllin_queue_element *e = &lin->rx_fifo[fifo_index];

					process_queue(index, e);

					if (likely(lin->tx_sl_offset)) {
						uint32_t w = tud_cdc_n_write(index, lin->tx_sl_buffer, lin->tx_sl_offset);

						if (unlikely(w != lin->tx_sl_offset)) {
							lin->tx_full = true;
							LOG("ch%u TXF\n", index);
						}

						lin->tx_sl_offset = 0;

						if (!written) {
							xTaskNotifyGive(lin->usb_task_handle);
							written = true;
						}
					}

					__atomic_store_n(&lin->rx_fifo_gi, gi + 1, __ATOMIC_RELEASE);

					done = false;
				}

				if (yield && written) {
					written = false;
					tud_cdc_n_write_flush(index);
					// LOG("ch%u tx flush\n", index);
				}
			} else {
				// LOG("ch%u TT\n", index);

				reset_channel_state(index);

				// can't call this if disconnected
				// tud_cdc_n_read_flush(index);

				yield = true;
			}
		}

		if (yield) {
			// yield to prevent this task from eating up the CPU
			// when the USB buffers are full/busy.
			yield = false;
			// taskYIELD();
			vTaskDelay(pdMS_TO_TICKS(1)); // 1ms for USB FS
			// LOG("+");
		}
	}
}

void tud_cdc_rx_cb(uint8_t index)
{
	struct lin *lin = &lins[index];

	// LOG("ch%u tud_cdc_rx_cb\n", index);

	xTaskNotifyGive(lin->usb_task_handle);
}


void tud_cdc_line_state_cb(uint8_t index, bool dtr, bool rts)
{
	struct lin *lin = &lins[index];

	(void) dtr;
	(void) rts;

	// LOG("ch%u tud_cdc_line_state_cb\n", index);

	xTaskNotifyGive(lin->usb_task_handle);
}

SLLIN_RAMFUNC extern void sllin_lin_task_queue(uint8_t index, sllin_queue_element const *element)
{
	struct lin *lin = &lins[index];
	uint8_t pi = __atomic_load_n(&lin->rx_fifo_pi, __ATOMIC_RELAXED);
	uint8_t gi = __atomic_load_n(&lin->rx_fifo_gi, __ATOMIC_ACQUIRE);
	uint8_t used = pi - gi;

	if (likely(used < TU_ARRAY_SIZE(lin->rx_fifo))) {
		uint8_t fifo_index = pi % TU_ARRAY_SIZE(lin->rx_fifo);

		lin->rx_fifo[fifo_index] = *element;

		__atomic_store_n(&lin->rx_fifo_pi, pi + 1, __ATOMIC_RELEASE);
	} else {
		__atomic_store_n(&lin->rx_full, 1, __ATOMIC_RELEASE);
	}
}


SLLIN_RAMFUNC extern void sllin_lin_task_notify_def(uint8_t index, uint32_t count)
{
	struct lin *lin = &lins[index];

	for (uint32_t i = 0; i < count; ++i) {
		xTaskNotifyGive(lin->usb_task_handle);
	}
}

SLLIN_RAMFUNC extern void sllin_lin_task_notify_isr(uint8_t index, uint32_t count)
{
	struct lin *lin = &lins[index];
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (likely(count)) {
		for (uint32_t i = 0; i < count - 1; ++i) {
			vTaskNotifyGiveFromISR(lin->usb_task_handle, NULL);
		}

		// LOG("ch%u notify\n", index);
		vTaskNotifyGiveFromISR(lin->usb_task_handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

uint16_t _sllin_time_stamp_ms;

// SLLIN_RAMFUNC static inline void sc_board_can_ts_request(void)
// {
// 	uint8_t reg;
// 	(void)index;

// 	while (1) {

// 		reg = __atomic_load_n(&TC2->COUNT16.CTRLBSET.reg, __ATOMIC_ACQUIRE);

// 		uint8_t cmd = reg & TC_CTRLBSET_CMD_Msk;

// 		// SC_DEBUG_ASSERT(cmd == TC_CTRLBSET_CMD_READSYNC || cmd == TC_CTRLBSET_CMD_NONE);
// 		if (cmd == TC_CTRLBSET_CMD_READSYNC) {
// 			break;
// 		}

// 		if (likely(__atomic_compare_exchange_n(
// 			&TC2->COUNT16.CTRLBSET.reg,
// 			&reg,
// 			TC_CTRLBSET_CMD_READSYNC,
// 			false, /* weak? */
// 			__ATOMIC_RELEASE,
// 			__ATOMIC_ACQUIRE))) {
// 				break;
// 			}

// 	}
// }

// #define same5x_counter_1MHz_is_current_value_ready() ((__atomic_load_n(&TC2->COUNT16.CTRLBSET.reg, __ATOMIC_ACQUIRE) & TC_CTRLBSET_CMD_Msk) == TC_CTRLBSET_CMD_NONE)
// #define same5x_counter_1MHz_read_unsafe() (TC2->COUNT16.COUNT.reg)

// #define same5x_counter_1MHz_wait_for_current_value() \
// 	({ \
// 		while (!same5x_counter_1MHz_is_current_value_ready()); \
// 		uint32_t counter = same5x_counter_1MHz_read_unsafe(); \
// 		counter; \
// 	})


SLLIN_RAMFUNC extern void vApplicationTickHook(void)
{
	_Static_assert(1000 == configTICK_RATE_HZ, "fix this function");

	uint16_t now = __atomic_load_n(&_sllin_time_stamp_ms, __ATOMIC_ACQUIRE);

	if (unlikely((now % 1024) == 0)) {
		sllin_queue_element e;
		e.type = SLLIN_QUEUE_ELEMENT_TYPE_TIME_STAMP;
		e.time_stamp_ms = now;

		for (uint8_t i = 0; i < TU_ARRAY_SIZE(lins); ++i) {
			sllin_lin_task_queue(i, &e);
			sllin_lin_task_notify_isr(i, 1);
		}
	}

	// update, wrap around at 1 [s] (60000 [ms])
	if (unlikely(now == 0xEA5F)) {
		__atomic_store_n(&_sllin_time_stamp_ms, 0, __ATOMIC_RELEASE);
	} else {
		__atomic_store_n(&_sllin_time_stamp_ms, now + 1, __ATOMIC_RELEASE);
	}

	// sc_board_can_ts_request();
	// static uint16_t last = 0;
	// uint16_t ts = same5x_counter_1MHz_wait_for_current_value();

	// if (ts != last) {
	// 	LOG("ts=%x ch0 sleep=%x\n", _sllin_time_stamp_ms, ts);
	// 	last = ts;
	// }

	// static int last_stopped = -1;
	// bool stopped = TC2->COUNT16.STATUS.bit.STOP;
	// if (stopped != last_stopped) {
	// 	LOG("ts=%x ch0 stop=%d\n", _sllin_time_stamp_ms, stopped);
	// 	last_stopped = stopped;
	// }
}
