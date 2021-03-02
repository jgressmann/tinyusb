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

union rle_bit {
	uint8_t mux;
	struct {
		uint8_t count:7;
		uint8_t value:1;
	} bit;
};

#define RLE_BIT_MAX_COUNT 127

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
	bool finished;
	bool tx_overflow;
	volatile uint32_t output_count_total;
	volatile uint32_t output_flags;
	// union rle_bit input;
	// union rle_bit output;
	uint8_t input_bit;
	uint8_t input_count;
	// uint8_t output_bit;
	uint8_t output_count;
} lp;

#define OUTPUT_FLAG_STALLED     0x1
#define OUTPUT_FLAG_RX_OVERFLOW 0x2

static inline void counter_stop(void)
{
	TC0->COUNT32.CTRLA.bit.ENABLE = 0;
	while(1 == TC0->COUNT32.SYNCBUSY.bit.ENABLE);
}

static inline void restart_counter(void)
{
	counter_stop();
	TC0->COUNT32.COUNT.reg = 0;
	TC0->COUNT32.CTRLA.bit.ENABLE = 1;
}



int main(void)
{
	board_init();

	// increase main clock to 200MHz
	OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x0) | OSCCTRL_DPLLRATIO_LDR(99);
	while(0 == OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY); /* wait for the PLL0 to be ready */
#undef CONF_CPU_FREQUENCY
#define CONF_CPU_FREQUENCY 200000000

// 	OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x0) | OSCCTRL_DPLLRATIO_LDR(74);
// 	while(0 == OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY); /* wait for the PLL0 to be ready */
// #undef CONF_CPU_FREQUENCY
// #define CONF_CPU_FREQUENCY 150000000


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
	MCLK->APBAMASK.bit.TC1_ = 1;
	GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
	GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;


	TC0->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
	while(1 == TC0->COUNT32.SYNCBUSY.bit.SWRST);
	TC0->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32;
	// TC0->COUNT32.CTRLBSET.reg = TC_CTRLBSET_DIR;
	TC0->COUNT32.INTENSET.reg = TC_INTENSET_OVF;
	TC0->COUNT32.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
	// TC0->COUNT32.CC[0].reg = 40;
	// TC0->COUNT32.COUNT.reg = 12000;
	// TC0->COUNT32.CCBUF[0].reg = 12000;
	// TC0->COUNT32.CTRLA.reg = TC_CTRLA_ENABLE;
	// while(1 == TC0->COUNT32.SYNCBUSY.bit.ENABLE);


	// TC0->COUNT32.CTRLA.reg = TC_CTRLA_ENABLE;
	// while(1 == TC0->COUNT32.SYNCBUSY.bit.ENABLE);
	// TC0->COUNT32.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP;

	// restart_counter();

	CMCC->CTRL.bit.CEN = 1;


	lp.state = IDLE;
	lp.signal_frequency = 1250000;
	// lp.signal_frequency = 100000000;

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
		// LOG("dec %#x\n", lp.usb_rx_base64_state);
		lp.signal_tx_buffer[index] = lp.usb_rx_base64_state;
		__atomic_store_n(&lp.signal_tx_buffer_pi, pi + 1, __ATOMIC_RELEASE);
	} else {
		lp.tx_overflow = true;
	}

	lp.usb_rx_base64_bits = 0;
}

static inline void base64_shift(uint8_t bits)
{
	LP_DEBUG_ASSERT(!(bits & 0xc0));
	switch (lp.usb_rx_base64_bits) {
	case 0:
		lp.usb_rx_base64_bits = 6;
		lp.usb_rx_base64_state = bits;
		break;
	case 2:
		lp.usb_rx_base64_state <<= 6;
		lp.usb_rx_base64_state |= bits;
		base64_flush();
		break;
	case 4:
		lp.usb_rx_base64_state <<= 4;
		lp.usb_rx_base64_state |= (bits >> 2) & 0xf;
		base64_flush();
		lp.usb_rx_base64_bits = 2;
		lp.usb_rx_base64_state = bits & 0x3;
		break;
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

			if (lp.running && lp.started) {
				uint8_t flags = __atomic_load_n(&lp.output_flags, __ATOMIC_ACQUIRE);
				if (lp.finished) {
					if (flags) {
						flags &= ~OUTPUT_FLAG_STALLED;
						if (flags) {
							// ouch!
							LOG("ERROR: early abort %#x\n", flags);
						} else if (lp.tx_overflow) {
							LOG("ERROR: tx overflow\n");
							lp.running = false;
						} else {
							// thread fence and send last datum
							LOG("W00t!\n");
						}
					}

					lp.running = false;
				} else {
					if (flags) {
						// ouch!
						LOG("ERROR: early abort %#x\n", flags);
						lp.running = false;
					} else if (lp.tx_overflow) {
						LOG("ERROR: tx overflow\n");
						lp.running = false;
					} else {
						// LOG("running\n");

					}
				}
			}

			if (tud_cdc_n_available(0) && lp.usb_rx_count < TU_ARRAY_SIZE(lp.usb_rx_buffer)) {
				uint8_t offset = lp.usb_rx_count;
				lp.usb_rx_count += (uint8_t)tud_cdc_n_read(0, &lp.usb_rx_buffer[offset], TU_ARRAY_SIZE(lp.usb_rx_buffer) - offset);
				// LOG("read: ");
				// for (uint8_t i = offset; i < lp.usb_rx_count; ++i) {
				// 	LOG("%c %#x", lp.usb_rx_buffer[i], lp.usb_rx_buffer[i]);
				// }
				// LOG("\n");
				more = true;
			}

			if (lp.usb_rx_count) {
				if (lp.running) {
					if (!lp.finished) {
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
								base64_shift(62);
								break;
							case '/':
								base64_shift(63);
								break;
							case '=':
								base64_flush();
								// lp.running = false;
								// lp.started = false;

								// wait for stall
								lp.finished = true;
								break;
							default:
								if (c >= 'A' && c <= 'Z') {
									base64_shift(c - 'A');
								} else if (c >= 'a' && c <= 'z') {
									base64_shift((c - 'a') + 26);
								} else if (c >= '0' && c <= '0') {
									base64_shift((c - '0') + 52);
								} else {
									base64_flush();
									// LOG("invalid char '%c'\n", c);
									// lp.running = false;
									lp.finished = true;
								}
								break;
							}
						}
					}

					lp.usb_rx_count = 0;

					if (!lp.started) {
						uint8_t gi = __atomic_load_n(&lp.signal_tx_buffer_gi, __ATOMIC_ACQUIRE);
						uint8_t pi = lp.signal_tx_buffer_pi;
						if (pi != gi) {
							LOG("start\n");

							lp.started = true;
							lp.finished = false;
							lp.tx_overflow = false;
							lp.output_flags = 0;
							// lp.output_bits = 0;

							lp.output_count_total = 0;
							// lp.output.mux = 0;
							// lp.input.mux = 0;
							lp.output_count = 0;
							lp.input_count = 0;


							__atomic_thread_fence(__ATOMIC_RELEASE);

							// set pin to high to start
							gpio_set_pin_level(LIN_TX_PIN, true);
							TC0->COUNT32.CC[0].reg = CONF_CPU_FREQUENCY / lp.signal_frequency;


							restart_counter();
						}
					}
				} else {
					// LOG("1\n");
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
							// LOG("1.1: %s\n", lp.cmd_buffer);
							process_input();
							lp.cmd_count = 0;
							break;
						default:
							// LOG("1.2\n");
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
					}

					lp.usb_tx_count -= w;

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
			// lp.usb_rx_count = 0;
			// lp.usb_tx_count = 0;
			// lp.usb_rx_base64_state = 0;
			// lp.usb_rx_base64_bits = 0;
			// counter_stop();
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
}

RAMFUNC static void lin_task(void* param)
{
	(void) param;


}

// @120 MHz 1.625 us, cache off
// @200 MHz 1.250 us, cache off
// @200 MHz 550-580 ns, cache on
// @200 MHz 100 ns, cache on (pin toggle)
RAMFUNC static void output_next_bit(void)
{
	// clear interrupt
	TC0->COUNT32.INTFLAG.reg = ~0;

	// PORT->Group[2].OUTTGL.reg = 0b10000;
	// goto out;

	if (likely(lp.output_count_total)) {
		//
		uint32_t r = PORT->Group[2].IN.reg;
		bool value = (r & 0b100000) == 0b100000;


		if (likely(lp.input_count)) {
			bool store = true;
			if (value == lp.input_bit) {
				if (likely(lp.input_count != RLE_BIT_MAX_COUNT)) {
					++lp.input_count;
					store = false;
				}
			}

			if (store) {
				uint8_t pi = lp.signal_rx_buffer_pi;
				uint8_t gi = __atomic_load_n(&lp.signal_rx_buffer_gi, __ATOMIC_ACQUIRE);
				uint8_t used = pi - gi;
				if (unlikely(used == TU_ARRAY_SIZE(lp.signal_rx_buffer))) {
					__atomic_or_fetch(&lp.output_flags, OUTPUT_FLAG_RX_OVERFLOW, __ATOMIC_RELEASE);
					// stop timer
					TC0->COUNT32.CTRLA.bit.ENABLE = 0;
					goto out;
				} else {
					// LOG("IN pin=%u count=%u\n", lp.input.bit.value, lp.input.bit.count);
					union rle_bit r;
					uint8_t index = pi & (TU_ARRAY_SIZE(lp.signal_rx_buffer)-1);
					r.bit.value = lp.input_bit;
					r.bit.count = lp.input_count;
					lp.signal_rx_buffer[index] = r.mux;
					__atomic_store_n(&lp.signal_rx_buffer_pi, pi + 1, __ATOMIC_RELEASE);
				}

				lp.input_count = 1;
				lp.input_bit = value;
			}
		} else {
			lp.input_count = 1;
			lp.input_bit = value;
		}
	}

	// if (!lp.output.bit.count) {
	if (unlikely(!lp.output_count)) {
		uint8_t pi, gi;
fetch:
		pi = __atomic_load_n(&lp.signal_tx_buffer_pi, __ATOMIC_ACQUIRE);
		gi = lp.signal_tx_buffer_gi;

		if (unlikely(pi == gi)) {
			__atomic_or_fetch(&lp.output_flags, OUTPUT_FLAG_STALLED, __ATOMIC_RELEASE);
			// stop timer
			TC0->COUNT32.CTRLA.bit.ENABLE = 0;
			goto out;
		} else {
			uint8_t index = gi & (TU_ARRAY_SIZE(lp.signal_tx_buffer)-1);
			// lp.output.mux = lp.signal_tx_buffer[index];
			union rle_bit r;
			r.mux = lp.signal_tx_buffer[index];
			__atomic_store_n(&lp.signal_tx_buffer_gi, gi + 1, __ATOMIC_RELEASE);

			// lp.output_bit = r.bit.value;
			lp.output_count = r.bit.count;

			if (unlikely(!lp.output_count)) {
				goto fetch;
			}

			// LOG("OUT pin=%u count=%u\n", r.value, r.bit.count);

			if (r.bit.value) {
				PORT->Group[2].OUTSET.reg = 0b10000;
			} else {
				PORT->Group[2].OUTCLR.reg = 0b10000;
			}
		}
	}


	LP_ISR_ASSERT(lp.output_count);

	--lp.output_count;
	++lp.output_count_total;

	// LP_ISR_ASSERT(lp.output.bit.count);

	// --lp.output.bit.count;
	// ++lp.output_count_total;
	// LOG("output count=%lu\n", lp.output_count_total);


	// PORT->Group[2].OUT.reg = 0b10000;


out:
	;

	// PORT->Group[2].OUTTGL.reg = 0b10000;

	// usnprintf(buf, sizeof(buf), "%#lx %#lx\n", (unsigned long)value, r);
	// board_uart_write(buf, -1);
}

extern void TC0_Handler(void)
{
	output_next_bit();
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
						"0 %s %s %s\n",
						(lp.tx_overflow ? "TXOVR" : ""),
						((lp.output_flags & OUTPUT_FLAG_RX_OVERFLOW) == OUTPUT_FLAG_RX_OVERFLOW ? "RXOVR" : ""),
						((lp.output_flags & OUTPUT_FLAG_STALLED) == OUTPUT_FLAG_STALLED ? "STALLED" : "")
						);
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