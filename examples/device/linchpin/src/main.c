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


#include <linchpin.h>


#define BASE64_C
#include <base64.h>



static void tusb_device_task(void* param);
LP_RAMFUNC static void cdc_task(void* param);


#define LIN_TX_PIN PIN_PC04
#define LIN_RX_PIN PIN_PC05



// static struct linchpin lp;
struct linchpin lp;

static struct tasks {
	StackType_t usb_device_stack[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t usb_device_task_mem;
	StackType_t cdc_task_stack_mem[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t cdc_task_mem;
} tasks;


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

	LP_LOG("CONF_CPU_FREQUENCY=%lu\n", CONF_CPU_FREQUENCY);


	// increase main clock to 200MHz
// 	OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x0) | OSCCTRL_DPLLRATIO_LDR(99);
// 	while(0 == OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY); /* wait for the PLL0 to be ready */
// #undef CONF_CPU_FREQUENCY
// #define CONF_CPU_FREQUENCY 200000000

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


	NVIC_SetPriority(TC0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
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


	// lp.state = IDLE;
	// lp.signal_frequency = 1250000;
	// lp.signal_frequency = 100000000;
	lp_init();



	(void) xTaskCreateStatic(&tusb_device_task, "tusb", ARRAY_SIZE(tasks.usb_device_stack), NULL, configMAX_PRIORITIES-1, tasks.usb_device_stack, &tasks.usb_device_task_mem);
	(void) xTaskCreateStatic(&cdc_task, "cdc", ARRAY_SIZE(tasks.cdc_task_stack_mem), NULL, configMAX_PRIORITIES-1, tasks.cdc_task_stack_mem, &tasks.cdc_task_mem);
	// (void) xTaskCreateStatic(&lin_task, "lin", ARRAY_SIZE(lp.lin_task_stack_mem), NULL, configMAX_PRIORITIES-2, lp.lin_task_stack_mem, &lp.lin_task_mem);

	gpio_set_pin_level(LIN_TX_PIN, true);
	TC0->COUNT32.CC[0].reg = CONF_CPU_FREQUENCY;


	// restart_counter();

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
		LP_LOG("tud_task\n");
		tud_task();
	}
}



static void cdc_task(void* param)
{
	(void) param;

	while (1) {
		// PORT->Group[2].OUTTGL.reg = 0b10000;

		lp_cdc_task();
	}
}


//// @120 MHz 1.625 us, cache off
//// @200 MHz 1.250 us, cache off
//// @200 MHz 550-580 ns, cache on
//// @200 MHz 100 ns, cache on (pin toggle)
//LP_RAMFUNC void lp_output_next_bit(void)
//{
//	// clear interrupt
//	TC0->COUNT32.INTFLAG.reg = ~0;

//	// PORT->Group[2].OUTTGL.reg = 0b10000;
//	// goto out;

//	if (likely(lp.output_count_total)) {
//		//
//		uint32_t r = PORT->Group[2].IN.reg;
//		bool value = (r & 0b100000) == 0b100000;


//		if (likely(lp.input_count)) {
//			bool store = true;
//			if (value == lp.input_bit) {
//				if (likely(lp.input_count != RLE_BIT_MAX_COUNT)) {
//					++lp.input_count;
//					store = false;
//				}
//			}

//			if (store) {
//				uint8_t pi = lp.signal_rx_buffer_pi;
//				uint8_t gi = __atomic_load_n(&lp.signal_rx_buffer_gi, __ATOMIC_ACQUIRE);
//				uint8_t used = pi - gi;
//				if (unlikely(used == ARRAY_SIZE(lp.signal_rx_buffer))) {
//					__atomic_or_fetch(&lp.output_flags, OUTPUT_FLAG_RX_OVERFLOW, __ATOMIC_RELEASE);
//					// stop timer
//					TC0->COUNT32.CTRLA.bit.ENABLE = 0;
//					goto out;
//				} else {
//					// LP_LOG("IN pin=%u count=%u\n", lp.input.bit.value, lp.input.bit.count);
//					union rle_bit rle;
//					uint8_t index = pi & (ARRAY_SIZE(lp.signal_rx_buffer)-1);
//					rle.bit.value = lp.input_bit;
//					rle.bit.count = lp.input_count;
//					lp.signal_rx_buffer[index] = rle.mux;
//					__atomic_store_n(&lp.signal_rx_buffer_pi, pi + 1, __ATOMIC_RELEASE);
//				}

//				lp.input_count = 1;
//				lp.input_bit = value;
//			}
//		} else {
//			lp.input_count = 1;
//			lp.input_bit = value;
//		}
//	}

//	// if (!lp.output.bit.count) {
//	if (unlikely(!lp.output_count)) {
//		uint8_t pi, gi;
//fetch:
//		pi = __atomic_load_n(&lp.signal_tx_buffer_pi, __ATOMIC_ACQUIRE);
//		gi = lp.signal_tx_buffer_gi;

//		if (unlikely(pi == gi)) {
//			__atomic_or_fetch(&lp.output_flags, OUTPUT_FLAG_TX_STALLED, __ATOMIC_RELEASE);
//			// stop timer
//			TC0->COUNT32.CTRLA.bit.ENABLE = 0;
//			goto out;
//		} else {
//			uint8_t index = gi & (ARRAY_SIZE(lp.signal_tx_buffer)-1);
//			// lp.output.mux = lp.signal_tx_buffer[index];
//			union rle_bit r;
//			r.mux = lp.signal_tx_buffer[index];
//			__atomic_store_n(&lp.signal_tx_buffer_gi, gi + 1, __ATOMIC_RELEASE);

//			// lp.output_bit = r.bit.value;
//			lp.output_count = r.bit.count;

//			if (unlikely(!lp.output_count)) {
//				goto fetch;
//			}

//			// LP_LOG("OUT pin=%u count=%u\n", r.value, r.bit.count);

//			if (r.bit.value) {
//				PORT->Group[2].OUTSET.reg = 0b10000;
//			} else {
//				PORT->Group[2].OUTCLR.reg = 0b10000;
//			}
//		}
//	}


//	LP_ISR_ASSERT(lp.output_count);

//	--lp.output_count;
//	++lp.output_count_total;

//	// LP_ISR_ASSERT(lp.output.bit.count);

//	// --lp.output.bit.count;
//	// ++lp.output_count_total;
//	// LP_LOG("output count=%lu\n", lp.output_count_total);


//	// PORT->Group[2].OUT.reg = 0b10000;


//out:
//	;

//	// PORT->Group[2].OUTTGL.reg = 0b10000;

//	// usnprintf(buf, sizeof(buf), "%#lx %#lx\n", (unsigned long)value, r);
//	// board_uart_write(buf, -1);
//}

LP_RAMFUNC static void timer_interrupt(void)
{
	// clear interrupt
	TC0->COUNT32.INTFLAG.reg = ~0;

	lp_output_next_bit();
}

extern void TC0_Handler(void)
{
	timer_interrupt();
}

// LP_RAMFUNC bool lp_cdc_is_connected(void)
// {
// 	return tud_cdc_n_connected(0) != 0;
// }

// LP_RAMFUNC void lp_set_tx_pin(bool value)
// {
// 	gpio_set_pin_level(LIN_TX_PIN, value);
// }

// LP_RAMFUNC void lp_start_counter(void)
// {
// 	TC0->COUNT32.CC[0].reg = CONF_CPU_FREQUENCY / lp.signal_frequency;
// 	restart_counter();
// }

// LP_RAMFUNC void lp_cdc_tx_flush(void)
// {
// 	tud_cdc_n_write_flush(0);
// }

// LP_RAMFUNC uint32_t lp_cdc_tx_available(void)
// {
// 	return tud_cdc_n_write_available(0);
// }

// LP_RAMFUNC uint32_t lp_cdc_tx(uint8_t const *ptr, uint32_t count)
// {
// 	return tud_cdc_n_write(0, ptr, count);
// }

// LP_RAMFUNC uint32_t lp_cdc_rx_available(void)
// {
// 	return tud_cdc_n_available(0);
// }

// LP_RAMFUNC uint32_t lp_cdc_rx(uint8_t  *ptr, uint32_t count)
// {
// 	return tud_cdc_n_read(0, ptr, count);
// }

LP_RAMFUNC void lp_delay_ms(uint32_t ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms));
}

bool lp_pin_set(uint32_t pin, bool value)
{
	PortGroup* group = &PORT->Group[(pin >> 5) & 0x3];
	if (value) {
		group->OUTSET.reg = UINT32_C(1) << (pin & 0x1f);
	} else {
		group->OUTCLR.reg = UINT32_C(1) << (pin & 0x1f);
	}

	return true;
}

