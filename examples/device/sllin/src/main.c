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

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>


#include <bsp/board.h>
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
	uint8_t rx_buffer[33];
	uint8_t tx_buffer[33];
	uint8_t rx_offset;
	uint8_t tx_offset;

	bool master;
	bool tx_full;
	bool setup;
	bool enabled;
	bool readonly;
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

	for (unsigned i = 0; i < SLLIN_BOARD_LIN_COUNT; ++i) {
		struct lin *lin = &lins[i];
	// 	struct usb_lin *usb_lin = &usb.lin[i];

	// 	usb_lin->mutex_handle = xSemaphoreCreateMutexStatic(&usb_lin->mutex_mem);
		lin->usb_task_handle = xTaskCreateStatic(
								&lin_usb_task,
								NULL,
								TU_ARRAY_SIZE(lin->usb_task_stack),
								(void*)(uintptr_t)i,
								configMAX_PRIORITIES-1,
								lin->usb_task_stack,
								&lin->usb_task_mem);
	}


	LOG("vTaskStartScheduler\n");
	vTaskStartScheduler();

	LOG("sllin_board_reset\n");
	sllin_board_reset();


	return 0;
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
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	LOG("unmounted\n");
	led_blink(0, 1000);
	usb.mounted = false;

	// can_usb_disconnect();
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

	// can_usb_disconnect();
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	LOG("resume\n");
	usb.mounted = true;
	led_blink(0, 250);
}

static inline uint8_t char_to_nibble(char c)
{
	if (likely(c >= '0' && c <= '9')) {
		return c - '0';
	}

	if (c >= 'a' && c <= 'f') {
		return c - 'a';
	}

	return (c - 'A') & 0xf;

}

static inline char nibble_to_char(uint8_t nibble)
{
	return "0123456789abcdef"[nibble & 0xf];
}

SLLIN_RAMFUNC static void sllin_process_command(uint8_t index)
{
	// http://www.can232.com/docs/canusb_manual.pdf
	struct lin *lin = NULL;

	lin = &lins[index];

	lin->tx_buffer[0] = SLLIN_OK_TERMINATOR;
	lin->tx_offset = 1;

	switch (lin->rx_buffer[0]) {
	case '\n':
		lin->tx_offset = 0;
		break;
	case 'S': // CAN bitrate, values 0-8
		if (lin->enabled) {
			LOG("ch%u refusing to configure LIN when open\n", index);
			lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
		}
		break;
	case 's': // BTR0/BTR1
		if (unlikely(lin->rx_offset < 4)) {
			LOG("ch%u malformed command '%s'\n", index, lin->rx_buffer);
			lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else {
			if (lin->enabled) {
				LOG("ch%u refusing to configure LIN when open\n", index);
				lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
				lin->master = '0' == lin->rx_buffer[2];
				if (lin->master) {
					LOG("ch%u master mode configured\n", index);
				} else {
					LOG("ch%u slave mode configured\n", index);
				}
			}
		}
		break;
	case 'L': // listen only
		if (lin->enabled) {
			LOG("ch%u refusing to configure LIN when open\n", index);
			lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else {
			lin->readonly = true;
		}
		break;
	case 'O':
		// TODO setup device
		LOG("ch%u open\n", index);
		lin->enabled = true;
		break;
	case 'C':
		LOG("ch%u close\n", index);
		lin->enabled = false;
		lin->readonly = false;
		break;
	case 't': // transmit 11 bit CAN frame -> master tx
	case 'T': // transmit 29 bit CAN frame -> slave tx reponse
		if (likely(lin->enabled)) {
			if (unlikely(lin->rx_offset < 5)) {
				LOG("ch%u malformed command '%s'\n", index, lin->rx_buffer);
				lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else if (unlikely((lin->master && lin->rx_buffer[0] == 'T') || (!lin->master && lin->rx_buffer[0] == 't'))) {
				LOG("ch%u refusing to transmit / store frame, wrong mode\n", index);
				lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
				uint8_t pid = (char_to_nibble(lin->rx_buffer[2]) << 4) | char_to_nibble(lin->rx_buffer[3]);
				uint8_t frame_len = char_to_nibble(lin->rx_buffer[4]);

				if (unlikely(frame_len * 2 + 5 > lin->rx_offset)) {
					LOG("ch%u malformed command '%s'\n", index, lin->rx_buffer);
					lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
				} else {
					// tx / store response

					lin->tx_buffer[0] = lin->rx_buffer[0] == 'T' ? 'Z' : 'z';
					lin->tx_buffer[1] = SLLIN_OK_TERMINATOR;
					lin->tx_offset = 2;

					LOG("%s\n", lin->rx_buffer);
				}
			}
		} else {
			LOG("ch%u refusing to transmit / store reponses when closed\n", index);
			lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
		}
		break;
	case 'r': // transmit RTR frame, master only, transmit header
		if (likely(lin->enabled)) {
			if (unlikely(lin->rx_offset < 5)) {
				LOG("ch%u malformed command '%s'\n", index, lin->rx_buffer);
				lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else if (unlikely(!lin->master)) {
				LOG("ch%u refusing to transmit in slave mode\n", index);
				lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
				uint8_t pid = (char_to_nibble(lin->rx_buffer[2]) << 4) | char_to_nibble(lin->rx_buffer[3]);

				// tx / store response
				lin->tx_buffer[0] = 'z';
				lin->tx_buffer[1] = SLLIN_OK_TERMINATOR;
				lin->tx_offset = 2;

				LOG("%s\n", lin->rx_buffer);
			}
		} else {
			LOG("ch%u refusing to transmit / store reponses when closed\n", index);
			lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
		}
		break;
	case 'F': {
		uint8_t flags = 0;

		if (lin->tx_full) {
			lin->tx_full = false;
			flags |= 0x2;
		}

		lin->tx_buffer[0] = 'F';
		lin->tx_buffer[1] = nibble_to_char(flags >> 4);
		lin->tx_buffer[2] = nibble_to_char(flags);
		lin->tx_buffer[3] = SLLIN_OK_TERMINATOR;
		lin->tx_offset = 4;
	} break;
	case 'V': {
		lin->tx_buffer[0] = 'V';
		lin->tx_buffer[1] = '0';
		lin->tx_buffer[2] = '1';
		lin->tx_buffer[3] = nibble_to_char(SLLIN_VERSION_MAJOR);
		lin->tx_buffer[4] = nibble_to_char(SLLIN_VERSION_MINOR);
		lin->tx_buffer[5] = SLLIN_OK_TERMINATOR;
		lin->tx_offset = 6;
	} break;
	case 'N': {
		uint32_t id = sllin_board_identifier();
		id = ((id >> 16) & 0xffff) ^ (id & 0xffff);
		lin->tx_offset = usnprintf((char *)lin->tx_buffer, sizeof(lin->tx_buffer), "N%x%c", id, SLLIN_OK_TERMINATOR);
	} break;
	default:
		LOG("ch%u unhandled command '%s'\n", index, lin->rx_buffer);
		lin->tx_buffer[0] = SLLIN_ERROR_TERMINATOR;
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
	// SLLIN_ASSERT(index < TU_ARRAY_SIZE(usb.lin));

	LOG("ch%u task start\n", index);

	struct lin *lin = &lins[index];
	// struct usb_lin *usb_lin = &usb.lin[index];

	while (42) {
		bool yield = false;

		if (tud_cdc_n_connected(index)) {
			int c = tud_cdc_n_read_char(index);

			if (-1 == c) {
				yield = true;
			} else {
				if (SLLIN_OK_TERMINATOR == c && lin->rx_offset) {
					lin->rx_buffer[lin->rx_offset] = 0;
					sllin_process_command(index);
					lin->rx_offset = 0;

					if (lin->tx_offset) {
						uint32_t w = tud_cdc_n_write(index, lin->tx_buffer, lin->tx_offset);

						if (unlikely(w != lin->tx_offset)) {
							lin->tx_full = true;
						}

						tud_cdc_n_write_flush(index);
						lin->tx_offset = 0;
					}
				} else {
					lin->rx_buffer[lin->rx_offset] = c;
					lin->rx_offset = (lin->rx_offset + 1) % (TU_ARRAY_SIZE(lin->rx_buffer) - 1);
				}
			}
		} else {
			lin->rx_offset = 0;
			lin->tx_offset = 0;
			lin->enabled = false;
			lin->setup = false;
			lin->tx_full = false;
			lin->master = true;

			// tud_cdc_n_read_flush(index);

			yield = true;
		}

		if (yield) {
			// yield to prevent this task from eating up the CPU
			// when the USB buffers are full/busy.
			yield = false;
			// taskYIELD();
			vTaskDelay(pdMS_TO_TICKS(1)); // 1ms for USB FS
			// LOG(".");
		}
	}
}

// void tud_cdc_rx_cb(uint8_t itf)
// {
// 	(void)itf;
// }


// void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
// {
//   (void) itf;
//   (void) rts;

//   // TODO set some indicator
//   if ( dtr )
//   {
//     // Terminal connected
//   }
//   else
//   {
//     // Terminal disconnected
//   }
// }




