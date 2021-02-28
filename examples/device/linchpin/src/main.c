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

#ifndef likely
#define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x),0)
#endif

#define RAMFUNC __attribute__((section(".ramfunc")))


static void tusb_device_task(void* param);
static void cdc_task(void* param);
RAMFUNC static void lin_task(void* param);

#define LIN_TX_PIN PIN_PC04
#define LIN_RX_PIN PIN_PC05

enum state {
	IDLE, // no connection
	READY, // connected, not running
	RUNNING,
};

static struct linchpin {
	StackType_t usb_device_stack[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t usb_device_task_mem;
	StackType_t cdc_task_stack_mem[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t cdc_task_mem;
	StackType_t lin_task_stack_mem[configMINIMAL_STACK_SIZE];
	StaticTask_t lin_task_mem;
	char error_message[64];
	int error_code;
	// TaskHandle_t usb_task_handle;
	volatile uint8_t state;
} lp;




int main(void)
{
	board_init();
	tusb_init();

	gpio_set_pin_function(LIN_TX_PIN, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(LIN_TX_PIN, GPIO_DIRECTION_OUT);
	// gpio_set_pin_level(LIN_TX_PIN, true);
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
	TC0->COUNT16.CC[0].reg = 4;
	// TC0->COUNT16.COUNT.reg = 12000;
	// TC0->COUNT16.CCBUF[0].reg = 12000;
	TC0->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE;
	while(1 == TC0->COUNT16.SYNCBUSY.bit.ENABLE);

	CMCC->CTRL.bit.CEN = 1;


	lp.state = IDLE;

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

static void cdc_task(void* param)
{
	(void) param;

	while (1) {
		if (tud_cdc_n_connected(0)) {
			if (tud_cdc_n_available(0)) {
				uint8_t buf[64];
				uint32_t count = tud_cdc_n_read(0, buf, sizeof(buf));

				if (count) {
					board_uart_write(buf, count);
				}
			}
		} else {
			// back to idle
			vTaskDelay(pdMS_TO_TICKS(1));
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
