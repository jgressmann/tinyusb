/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Jean Gressmann <jean@0x42.de>
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
	uint8_t rx_sl_buffer[33];
	uint8_t tx_sl_buffer[33];

	uint16_t baudrate;
	uint8_t rx_sl_offset;
	uint8_t tx_sl_offset;
	uint8_t rx_fifo_gi; // not an index, uses full range of type
	uint8_t rx_fifo_pi; // not an index, uses full range of type

	bool master;
	bool tx_full;
	uint8_t rx_full;
	bool enabled;
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

		lin->baudrate = 19200;
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

static inline uint8_t id_to_pid(uint8_t id)
{
	static const uint8_t map[] = {
		0x80,
		0xC1,
		0x42,
		0x03,
		0xC4,
		0x85,
		0x06,
		0x47,
		0x08,
		0x49,
		0xCA,
		0x8B,
		0x4C,
		0x0D,
		0x8E,
		0xCF,
		0x50,
		0x11,
		0x92,
		0xD3,
		0x14,
		0x55,
		0xD6,
		0x97,
		0xD8,
		0x99,
		0x1A,
		0x5B,
		0x9C,
		0xDD,
		0x5E,
		0x1F,
		0x20,
		0x61,
		0xE2,
		0xA3,
		0x64,
		0x25,
		0xA6,
		0xE7,
		0xA8,
		0xE9,
		0x6A,
		0x2B,
		0xEC,
		0xAD,
		0x2E,
		0x6F,
		0xF0,
		0xB1,
		0x32,
		0x73,
		0xB4,
		0xF5,
		0x76,
		0x37,
		0x78,
		0x39,
		0xBA,
		0xFB,
		0x3C,
		0x7D,
		0xFE,
		0xBF,
	};

	return map[id & 0x3f];
}

static inline uint8_t pid_to_id(uint8_t pid)
{
	return pid & 0x3f;
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
	// case 'S': // CAN bitrate, values 0-8
	// 	if (lin->enabled) {
	// 		LOG("ch%u refusing to configure LIN when open\n", index);
	// 		lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
	// 	}
	// 	break;
	case 's': // BTR0/BTR1 used to set bitrate
		if (unlikely(lin->rx_sl_offset < 4)) {
			LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else {
			if (lin->enabled) {
				LOG("ch%u refusing to configure LIN when open\n", index);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
				lin->baudrate = (char_to_nibble(lin->rx_sl_buffer[1]) << 12)
								| (char_to_nibble(lin->rx_sl_buffer[2]) << 8)
								| (char_to_nibble(lin->rx_sl_buffer[3]) << 4)
								| (char_to_nibble(lin->rx_sl_buffer[4]) << 0);
				LOG("ch%u baudrate=%u\n", index, lin->baudrate);
			}
		}
		break;
	case 'L': // slave
	case 'O': // master
		lin->enabled = true;
		lin->master = lin->rx_sl_buffer[0] == 'O';
		sllin_board_lin_init(index, lin->baudrate, lin->master);
		LOG("ch%u open %s\n", index, lin->master ? "master" : "slave");
		break;
	case 'C':
		LOG("ch%u close\n", index);
		lin->enabled = false;
		break;
	case 't': // master: tx, slave: store response
		if (likely(lin->enabled)) {
			if (unlikely(lin->rx_sl_offset < 5)) {
				LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
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

					for (unsigned i = 0, j = 5; i < frame_len; ++i, j += 2) {
						data[i] = (char_to_nibble(lin->rx_sl_buffer[j]) << 4) | char_to_nibble(lin->rx_sl_buffer[j+1]);
					}

					if (enhanced_crc) {
						crc = sllin_crc_update(crc, &pid, 1);
						flags |= SLLIN_FRAME_FLAG_ENHANCED_CHECKSUM;
					}

					crc = sllin_crc_update(crc, data, frame_len);
					crc = sllin_crc_finalize(crc);

					LOG("ch%u %s -> id=%x len=%u crc=%x flags=%x\n", index, lin->rx_sl_buffer, id, frame_len, crc, flags);

					if (lin->master) {
						if (likely(sllin_board_lin_master_tx(index, id, frame_len, data, crc, flags))) {
							lin->tx_sl_buffer[0] = 'z';
							lin->tx_sl_buffer[1] = SLLIN_OK_TERMINATOR;
							lin->tx_sl_offset = 2;
						} else {
							lin->tx_full = true;
						}
					} else {
						sllin_board_lin_slave_tx(index, id, frame_len, data, crc);
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
			} else if (unlikely(!lin->master)) {
				LOG("ch%u refusing to transmit in slave mode\n", index);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
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
					lin->tx_full = true;
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
	case 'V': {
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
	case 'N': {
		uint32_t id = sllin_board_identifier();
		id = ((id >> 16) & 0xffff) ^ (id & 0xffff);
		lin->tx_sl_offset = usnprintf((char *)lin->tx_sl_buffer, sizeof(lin->tx_sl_buffer), "N%x%c", id, SLLIN_OK_TERMINATOR);
	} break;
	default:
		LOG("ch%u unhandled command '%s'\n", index, lin->rx_sl_buffer);
		lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
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

					switch (e->type) {
					case SLLIN_QUEUE_ELEMENT_TYPE_RX_FRAME: {
						uint8_t id = e->lin_frame.id;
						uint32_t w = 0;

						// LOG("ch%u rx frame\n", index);
						SLLIN_DEBUG_ASSERT(e->lin_frame.len <= 8);

						lin->tx_sl_buffer[lin->tx_sl_offset++] = 't';
						lin->tx_sl_buffer[lin->tx_sl_offset++] = '0';
						lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 4);
						lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id & 0xf);
						lin->tx_sl_buffer[lin->tx_sl_offset++] = '0' + e->lin_frame.len;
						for (unsigned i = 0; i < 8; ++i) {
							lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(e->lin_frame.data[i] >> 4);
							lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(e->lin_frame.data[i] & 0xf);
						}
						lin->tx_sl_buffer[lin->tx_sl_offset++] = SLLIN_OK_TERMINATOR;

						w = tud_cdc_n_write(index, lin->tx_sl_buffer, lin->tx_sl_offset);

						if (unlikely(w != lin->tx_sl_offset)) {
							lin->tx_full = true;
							LOG("ch%u TXF\n", index);
						}

						lin->tx_sl_offset = 0;

						if (!written) {
							xTaskNotifyGive(lin->usb_task_handle);
							written = true;
						}

					} break;
					default:
						SLLIN_ASSERT(false && "unhandled queue element type");
						break;
					}

					__atomic_store_n(&lin->rx_fifo_gi, gi + 1, __ATOMIC_RELEASE);

					done = false;
				}

				if (yield && written) {
					written = false;
					tud_cdc_n_write_flush(index);
					// LOG("flush\n");
				}
			} else {
				LOG("T");

				lin->rx_sl_offset = 0;
				lin->tx_sl_offset = 0;
				lin->enabled = false;
				lin->tx_full = false;
				lin->rx_full = false;
				lin->master = true;
				__atomic_store_n(&lin->rx_fifo_gi, 0, __ATOMIC_RELEASE);
				__atomic_store_n(&lin->rx_fifo_pi, 0, __ATOMIC_RELEASE);


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
			LOG("+");
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
	uint8_t pi = lin->rx_fifo_pi;
	uint8_t gi = __atomic_load_n(&lin->rx_fifo_pi, __ATOMIC_ACQUIRE);
	uint8_t used = pi - gi;

	if (likely(used < TU_ARRAY_SIZE(lin->rx_fifo))) {
		uint8_t fifo_index = pi % TU_ARRAY_SIZE(lin->rx_fifo);

		lin->rx_fifo[fifo_index] = *element;

		++lin->rx_fifo_pi;
		__atomic_thread_fence(__ATOMIC_RELEASE);
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
