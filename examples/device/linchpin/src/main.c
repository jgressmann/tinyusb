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

#include <bsp/board.h>
#include <tusb.h>
#include <hal/include/hal_gpio.h>

#include <linchpin.h>


#define RLEW_C
#include <rlew.h>
#undef RLEW_C



static void tusb_device_task(void* param);
static void cdc_cmd_task(void* param);
LP_RAMFUNC static void cdc_lin_task(void* param);

#define LIN_TX_PIN PIN_PC04
#define LIN_RX_PIN PIN_PC05


#define LP_VERSION_MAJOR 0
#define LP_VERSION_MINOR 1
#define LP_VERSION_PATCH 0
static struct tasks {
	StackType_t usb_device_stack[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t usb_device_task_mem;
	StackType_t cdc_cmd_task_stack_mem[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t cdc_cmd_task_mem;
	StackType_t cdc_lin_task_stack_mem[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t cdc_lin_task_mem;
} tasks;



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
	gpio_set_pin_pull_mode(LIN_RX_PIN, GPIO_PULL_OFF);
	// PORT->Group[2].CTRL.reg |= 0b100000; // continusous sampling for pin 5


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


	CMCC->CTRL.bit.CEN = 1;
	CMCC->MEN.bit.MENABLE = 1;


	lp_init();



	(void) xTaskCreateStatic(&tusb_device_task, "tusb", ARRAY_SIZE(tasks.usb_device_stack), NULL, configMAX_PRIORITIES-1, tasks.usb_device_stack, &tasks.usb_device_task_mem);
	(void) xTaskCreateStatic(&cdc_cmd_task, "cdc_cmd", ARRAY_SIZE(tasks.cdc_cmd_task_stack_mem), NULL, configMAX_PRIORITIES-1, tasks.cdc_cmd_task_stack_mem, &tasks.cdc_cmd_task_mem);
	(void) xTaskCreateStatic(&cdc_lin_task, "cdc_lin", ARRAY_SIZE(tasks.cdc_lin_task_stack_mem), NULL, configMAX_PRIORITIES-1, tasks.cdc_lin_task_stack_mem, &tasks.cdc_lin_task_mem);


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

static void cdc_cmd_task(void* param)
{
	(void) param;

	for (;;) {
		lp_cdc_cmd_task();
	}
}

LP_RAMFUNC static void cdc_lin_task(void* param)
{
	(void) param;

	while (1) {
		// PORT->Group[2].OUTTGL.reg = 0b10000;
		// (void)ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		lp_cdc_lin_task();
	}
}


LP_RAMFUNC static void timer_interrupt(void)
{
	// clear interrupt
	TC0->COUNT32.INTFLAG.reg = ~0;

	// LP_LOG("timer int\n");

	lp_timer_callback();
}

void TC0_Handler(void)
{
	timer_interrupt();
}

void lp_delay_ms(uint32_t ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms));
}

bool lp_pin_set(uint32_t pin, bool value)
{
	uint32_t pin_no = pin & 0x1f;
	uint32_t pin_mask = UINT32_C(1) << pin_no;

	LP_LOG("pin=%u value=%u\n", (unsigned)pin, value);

	PortGroup* group = &PORT->Group[(pin >> 5) & 0x3];
	group->DIRSET.reg = pin_mask;
	if (value) {
		group->OUTSET.reg = pin_mask;
	} else {
		group->OUTCLR.reg = pin_mask;
	}

	return true;
}

void lp_version(char* ptr, uint32_t capacity)
{
	usnprintf(ptr, capacity, "linChPiN version %u.%u.%u\n", LP_VERSION_MAJOR, LP_VERSION_MINOR, LP_VERSION_PATCH);
}

