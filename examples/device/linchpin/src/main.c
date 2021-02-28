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
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>

#include <bsp/board.h>
#include <tusb.h>
#include <hal/include/hal_gpio.h>
#include <sam.h>

#include <linchpin_debug.h>
#include <linchpin_api.h>

#ifndef likely
#define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x),0)
#endif

#define RAMFUNC __attribute__((section(".ramfunc")))

static char const * const error_messages[] = {
	"OK",
	"unknown command",
	"invalid parameter",
};

static inline char const * lp_strerror(int code) {
	if (code < 0) {
		code = -code;
	}

	if (unlikely((size_t)code >= TU_ARRAY_SIZE(error_messages))) {
		return "<invalid error code>";
	}

	return error_messages[code];
}


static void tusb_device_task(void* param);
RAMFUNC static void cdc_task(void* param);
RAMFUNC static void lin_task(void* param);
static void process_input(void);

#define LIN_TX_PIN PIN_PC04
#define LIN_RX_PIN PIN_PC05

#define OP_ZRO 0x0
#define OP_NOP 0x1
#define OP_RES 0x2
#define OP_ONE 0x3

typedef union {
	uint8_t value;
	struct {
		uint8_t out:1;
		uint8_t count:7;
	} bit;
} cmd;

enum state {
	IDLE, // no connection
	READY, // connected, not running
	RUNNING,
};

#define USB_BUFFER_SIZE 64

static struct linchpin {
	StackType_t usb_device_stack[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t usb_device_task_mem;
	StackType_t cdc_task_stack_mem[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t cdc_task_mem;
	StackType_t lin_task_stack_mem[configMINIMAL_STACK_SIZE];
	StaticTask_t lin_task_mem;
	// char error_message[64];
	// int error_code;
	// TaskHandle_t usb_task_handle;
	volatile uint8_t state;
	uint32_t signal_frequency;
	uint8_t cmd_buffer[USB_BUFFER_SIZE];
	uint8_t usb_rx_buffer[USB_BUFFER_SIZE];
	uint8_t usb_tx_buffer[USB_BUFFER_SIZE];
	uint8_t signal_tx_buffer[USB_BUFFER_SIZE];
	uint8_t signal_rx_buffer[USB_BUFFER_SIZE];
	uint8_t cmd_count;
	uint8_t usb_rx_count;
	uint8_t usb_tx_count;
	uint8_t usb_rx_base64_state;
	uint8_t usb_rx_base64_bits;
	volatile uint8_t signal_tx_buffer_pi;
	volatile uint8_t signal_tx_buffer_gi;
	volatile uint8_t signal_rx_buffer_pi;
	volatile uint8_t signal_rx_buffer_gi;
	bool running;
	bool started;
	bool overflow;
	volatile bool stall;
} lp;

static inline void counter_stop(void)
{
	TC0->COUNT16.CTRLA.bit.ENABLE = 0;
	while(1 == TC0->COUNT16.SYNCBUSY.bit.ENABLE);
}

static inline void restart_counter(void)
{
	counter_stop();
	TC0->COUNT16.COUNT.reg = 0;
	TC0->COUNT16.CTRLA.bit.ENABLE = 1;
}


int main(void)
{
	board_init();
	tusb_init();

	gpio_set_pin_function(LIN_TX_PIN, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(LIN_TX_PIN, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(LIN_TX_PIN, true);
	gpio_set_pin_pull_mode(LIN_TX_PIN, GPIO_PULL_OFF);

	gpio_set_pin_function(LIN_RX_PIN, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(LIN_RX_PIN, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(LIN_RX_PIN, GPIO_PULL_UP);
	PORT->Group[2].CTRL.reg |= 0b100000;


	NVIC_EnableIRQ(TC0_IRQn);

	MCLK->APBAMASK.bit.TC0_ = 1;
	GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;


	TC0->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while(1 == TC0->COUNT16.SYNCBUSY.bit.SWRST);
	// TC0->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1024;
	// TC0->COUNT16.CTRLBSET.reg = TC_CTRLBSET_DIR;
	TC0->COUNT16.INTENSET.reg = TC_INTENSET_OVF;
	TC0->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
	TC0->COUNT16.CC[0].reg = 40;
	// TC0->COUNT16.COUNT.reg = 12000;
	// TC0->COUNT16.CCBUF[0].reg = 12000;
	// TC0->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE;
	// while(1 == TC0->COUNT16.SYNCBUSY.bit.ENABLE);


	// TC0->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE;
	// while(1 == TC0->COUNT16.SYNCBUSY.bit.ENABLE);
	// TC0->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP;

	// restart_counter();

	// CMCC->CTRL.bit.CEN = 1;


	lp.state = IDLE;
	lp.signal_frequency = 1000000;

	(void) xTaskCreateStatic(&tusb_device_task, "tusb", TU_ARRAY_SIZE(lp.usb_device_stack), NULL, configMAX_PRIORITIES-1, lp.usb_device_stack, &lp.usb_device_task_mem);
	(void) xTaskCreateStatic(&cdc_task, "cdc", TU_ARRAY_SIZE(lp.cdc_task_stack_mem), NULL, configMAX_PRIORITIES-1, lp.cdc_task_stack_mem, &lp.cdc_task_mem);
	// // (void) xTaskCreateStatic(&lin_task, "lin", TU_ARRAY_SIZE(lp.lin_task_stack_mem), NULL, configMAX_PRIORITIES-2, lp.lin_task_stack_mem, &lp.lin_task_mem);


	board_uart_write("start scheduler\n", -1);
	// board_led_off();
	vTaskStartScheduler();
	NVIC_SystemReset();

	return 0;
}


static void tusb_device_task(void* param)
{
	(void) param;

	while (1) {
		LOG("tud_task\n");
		tud_task();
	}
}

static const uint8_t base64_table[64] = {
	'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
	'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
	'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
	'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
	'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
	'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
	'w', 'x', 'y', 'z', '0', '1', '2', '3',
	'4', '5', '6', '7', '8', '9', '+', '/'
};

#define BASE64_OP_CODE_BYTE

// RAMFUNC static inline bool base64_try_decode(
// 	char c,
// 	uint8_t* bits,
// 	uint8_t* op_code)
// {
// 	switch (c) {
// 	case '\n':
// 	case '\r':
// 	case ' ':
// 	case '\t':
// 		*bits = 0;
// 		op_code
// 		break;
// 	case '+':
// 	case '/':
// 		break;
// 	}
// }

static inline base64_flush(void)
{
	if (unlikely(!lp.usb_rx_base64_bits)) {
		return;
	}

	uint8_t gi = __atomic_load_n(&lp.signal_tx_buffer_gi, __ATOMIC_ACQUIRE);
	uint8_t pi = lp.signal_tx_buffer_pi;
	uint8_t used = pi - gi;
	if (likely(used < TU_ARRAY_SIZE(lp.signal_tx_buffer))) {
		uint8_t index = pi & (TU_ARRAY_SIZE(lp.signal_tx_buffer)-1);
		lp.signal_tx_buffer[index] = lp.usb_rx_base64_state;
		__atomic_store_n(&lp.signal_tx_buffer_pi, pi + 1, __ATOMIC_RELEASE);
	} else {
		lp.overflow = true;
	}

	lp.usb_rx_base64_bits = 0;
}

static inline base64_shift(uint8_t bits)
{
	LP_DEBUG_ASSERT(!(bits & 0xc0));
	switch (lp.usb_rx_base64_bits) {
	case 0:
		lp.usb_rx_base64_bits = 6;
		lp.usb_rx_base64_state = bits;
		break;
	case 2:
		lp.usb_rx_base64_state <<= 2;
		lp.usb_rx_base64_state |= bits;
		base64_flush();
		break;
	case 4:
		lp.usb_rx_base64_state <<= 4;
		lp.usb_rx_base64_state |= (bits >> 2) & 0xf;
		base64_flush();
		lp.usb_rx_base64_bits = 2;
		lp.usb_rx_base64_state = bits & 0x3;
	case 6:
		lp.usb_rx_base64_state <<= 2;
		lp.usb_rx_base64_state |= (bits >> 4) & 0x3;
		base64_flush();
		lp.usb_rx_base64_bits = 4;
		lp.usb_rx_base64_state = bits & 0xf;
		break;
	default:
		LP_ASSERT(false);
		break;
	}
}



static inline void place_unknown_cmd_response(void)
{
	lp.usb_tx_count = usnprintf((char*)lp.usb_tx_buffer, sizeof(lp.usb_tx_buffer), "%d %s\n", LP_ERROR_UNKNOWN_CMD, lp_strerror(LP_ERROR_UNKNOWN_CMD));
}

static void cdc_task(void* param)
{
	(void) param;

	while (1) {


		if (tud_cdc_n_connected(0)) {
			bool more = false;

			if (tud_cdc_n_available(0) && lp.usb_rx_count < TU_ARRAY_SIZE(lp.usb_rx_buffer)) {
				uint8_t offset = lp.usb_rx_count;
				lp.usb_rx_count += (uint8_t)tud_cdc_n_read(0, &lp.usb_rx_buffer[lp.usb_rx_count], TU_ARRAY_SIZE(lp.usb_rx_buffer) - lp.usb_rx_count);
				// LOG("read: ");
				// for (uint8_t i = offset; i < lp.usb_rx_count; ++i) {
				// 	LOG("%c %#x", lp.usb_rx_buffer[i], lp.usb_rx_buffer[i]);
				// }
				// LOG("\n");
				more = true;
			}

			if (lp.usb_rx_count) {
				if (lp.running) {
					char* p = (char*)lp.usb_rx_buffer;
					for (uint8_t i = 0; i < lp.usb_rx_count; ++i) {
						char c = p[i];
						switch (c) {
						case '\n':
						case '\r':
						case ' ':
						case '\t':
							break;
						case '+':
							base64_shift(0b111110);
							break;
						case '/':
							base64_shift(0b111111);
							break;
						case '=':
							base64_flush();
							lp.running = false;
							lp.started = false;
							break;
						default:
							if (c >= 'A' && c <= 'Z') {
								base64_shift(c - 'A');
							} else if (c >= 'a' && c <= 'z') {
								base64_shift(c - 'a' + 0b011010);
							} else if (c >= '0' && c <= '0') {
								base64_shift(c - '0' + 0b110100);
							} else {
								LOG("invalid char '%c'\n", c);
								lp.running = false;
							}
							break;
						}
					}
					lp.usb_rx_count = 0;
					if (!lp.started) {
						LOG("start\n");
						lp.stall = false;
						lp.overflow = false;
						TC0->COUNT16.CC[0].reg = CONF_CPU_FREQUENCY / lp.signal_frequency;
						restart_counter();
					}
				} else {
					LOG("1\n");
					char* p = (char*)lp.usb_rx_buffer;
					for (uint8_t i = 0; i < lp.usb_rx_count; ++i) {
						char c = p[i];
						switch (c) {
						case '\n':
						case '\r':
							// while (i < lp.usb_rx_count && (p[i] == '\r' || p[i] == '\r'))) {
							// 	++i;
							// }
							lp.cmd_buffer[lp.cmd_count] = 0;
							LOG("1.1: %s\n", lp.cmd_buffer);
							process_input();
							lp.cmd_count = 0;
							break;
						default:
							LOG("1.2\n");
							if (lp.cmd_count + 1 == TU_ARRAY_SIZE(lp.cmd_buffer)) {
								lp.cmd_count = 0;
								place_unknown_cmd_response();
							} else {
								lp.cmd_buffer[lp.cmd_count++] = c;
							}
							break;
						}
					}

					lp.usb_rx_count = 0;

					more = true;
				}
			}

			if (lp.usb_tx_count) {
				// LOG("tx %s\n", lp.usb_tx_buffer);
				uint32_t w = tud_cdc_n_write(0, lp.usb_tx_buffer, lp.usb_tx_count);
				if (w) {
					if (w < lp.usb_tx_count) {
						memmove(&lp.usb_tx_buffer[0], &lp.usb_tx_buffer[w], lp.usb_tx_count - w);
						lp.usb_tx_count -= w;
					} else {
						lp.usb_tx_count = 0;
					}

					more = true;
				}

				if (!lp.running) {
					tud_cdc_n_write_flush(0);
				}
			}

			if (!more) {
				// board_uart_write("no more\n", -1);
				vTaskDelay(pdMS_TO_TICKS(1));
			}
		} else {
			lp.usb_rx_count = 0;
			lp.usb_tx_count = 0;
			lp.usb_rx_base64_state = 0;
			lp.usb_rx_base64_bits = 0;
			counter_stop();
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
}

RAMFUNC static void lin_task(void* param)
{
	(void) param;


}

RAMFUNC static void foo(void)
{
	static char buf[16];

	TC0->COUNT16.INTFLAG.reg = ~0;

	uint32_t r = PORT->Group[2].IN.reg;

	bool value = (r & 0b100000) == 0b100000;
	PORT->Group[2].OUTTGL.reg = 0b10000;

	// usnprintf(buf, sizeof(buf), "%#lx %#lx\n", (unsigned long)value, r);
	// board_uart_write(buf, -1);
}

extern void TC0_Handler(void)
{
	foo();
}

static void process_input()
{
	uint8_t *ptr = lp.cmd_buffer;
	uint8_t count = lp.cmd_count;

	if (count > 0) {
		bool is_write = *ptr == '!';
		bool is_read = *ptr == '?';
		lp.usb_tx_count = 0;

		if (is_write) {
			if (count > 1)  {
				switch (ptr[1]) {
				case 'F': {
					char* start = (char*)&ptr[2];
					char* end = NULL;
					uint32_t f = strtoul(start, &end, 10);
					int error = LP_ERROR_NONE;
					if (end == NULL || end == start) {
						LOG("'%s' not integer\n", ptr);
						error = LP_ERROR_INVALID_PARAM;
						// lp.usb_tx_count = usnprintf(lp.usb_tx_buffer, sizeof(lp.usb_tx_buffer), "%d %s\n", LP_ERROR_INVALID_PARAM, lp_strerror(LP_ERROR_INVALID_PARAM));
					} else if (f > CONF_CPU_FREQUENCY / 16) {
						LOG("%lu > %lu / 16\n", f, CONF_CPU_FREQUENCY);
						// lp.usb_tx_count = usnprintf(lp.usb_tx_buffer, sizeof(lp.usb_tx_buffer), "%d %s\n", LP_ERROR_INVALID_PARAM, lp_strerror(LP_ERROR_INVALID_PARAM));
						error = LP_ERROR_OUT_OF_RANGE;
					} else {
						uint32_t cv = CONF_CPU_FREQUENCY / f;
						if (cv > 0xffff) {
							error = LP_ERROR_OUT_OF_RANGE;
						} else {
							LOG("set signal frequency to %lu [Hz]\n", f);
							lp.signal_frequency = f;
						}
					}
					lp.usb_tx_count = usnprintf((char*)lp.usb_tx_buffer, sizeof(lp.usb_tx_buffer), "%d %s\n", error, lp_strerror(error));
				} break;
				default:
					place_unknown_cmd_response();
					break;
				}
			} else {
				place_unknown_cmd_response();
			}
		} else if (is_read) {
			if (count > 1)  {
				switch (ptr[1]) {
				case 'F':
					lp.usb_tx_count = usnprintf((char*)lp.usb_tx_buffer, sizeof(lp.usb_tx_buffer), "%d %lu\n", LP_ERROR_NONE, lp.signal_frequency);
					break;
				case 'S':
					lp.usb_tx_count = usnprintf(
						(char*)lp.usb_tx_buffer,
						sizeof(lp.usb_tx_buffer),
						"0 %s %s\n",
						(lp.overflow ? "OVR" : ""),
						(lp.stall ? "STALL" : ""));
					break;
				default:
					place_unknown_cmd_response();
					break;
				}
			} else {
				place_unknown_cmd_response();
			}
		} else {
			if (count == 3) {
				lp.running = true;
				lp.started = false;
			} else {
				place_unknown_cmd_response();
			}
		}
	}
}