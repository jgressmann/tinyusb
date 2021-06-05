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


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#define LED_PIN PIN_PC18
#define BUTTON_PIN PIN_PB31
#define BOARD_SERCOM SERCOM2

/** Initializes the clocks from the external 12 MHz crystal
 *
 * The goal of this setup is to preserve the second PLL
 * for the application code while still having a reasonable
 * 48 MHz clock for USB / UART.
 *
 * GCLK0:   CONF_CPU_FREQUENCY (default 120 MHz) from PLL0
 * GCLK1:   unused
 * GCLK2:   12 MHz from XOSC1
 * DFLL48M: closed loop from GLCK2
 * GCLK3:   48 MHz
 */
static inline void init_clock_xtal(void)
{
	/* configure for a 12MHz crystal connected to XIN1/XOUT1 */
	OSCCTRL->XOSCCTRL[1].reg =
		OSCCTRL_XOSCCTRL_STARTUP(6) | // 1.953 ms
		OSCCTRL_XOSCCTRL_RUNSTDBY |
		OSCCTRL_XOSCCTRL_ENALC |
		OSCCTRL_XOSCCTRL_IMULT(4) | OSCCTRL_XOSCCTRL_IPTAT(3) | // 8MHz to 16MHz
		OSCCTRL_XOSCCTRL_XTALEN |
		OSCCTRL_XOSCCTRL_ENABLE;
	while(0 == OSCCTRL->STATUS.bit.XOSCRDY1);

	OSCCTRL->Dpll[0].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_DIV(2) | OSCCTRL_DPLLCTRLB_REFCLK_XOSC1; /* 12MHz / 6 = 2Mhz, input = XOSC1 */
	OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x0) | OSCCTRL_DPLLRATIO_LDR((CONF_CPU_FREQUENCY / 1000000 / 2) - 1); /* multiply to get CONF_CPU_FREQUENCY (default = 120MHz) */
	OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_RUNSTDBY | OSCCTRL_DPLLCTRLA_ENABLE;
	while(0 == OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY); /* wait for the PLL0 to be ready */

	/* configure clock-generator 0 to use DPLL0 as source -> GCLK0 is used for the core */
	GCLK->GENCTRL[0].reg =
		GCLK_GENCTRL_DIV(0) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DPLL0 |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL0); /* wait for the synchronization between clock domains to be complete */

	// configure GCLK2 for 12MHz from XOSC1
	GCLK->GENCTRL[2].reg =
		GCLK_GENCTRL_DIV(0) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_XOSC1 |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL2); /* wait for the synchronization between clock domains to be complete */

	 /* setup DFLL48M to use GLCK2 */
	GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN;

	OSCCTRL->DFLLCTRLA.reg = 0;
	while(1 == OSCCTRL->DFLLSYNC.bit.ENABLE);

	OSCCTRL->DFLLCTRLB.reg = OSCCTRL_DFLLCTRLB_MODE | OSCCTRL_DFLLCTRLB_WAITLOCK;
	OSCCTRL->DFLLMUL.bit.MUL = 4; // 4 * 12MHz -> 48MHz

	OSCCTRL->DFLLCTRLA.reg =
		OSCCTRL_DFLLCTRLA_ENABLE |
		OSCCTRL_DFLLCTRLA_RUNSTDBY;
	while(1 == OSCCTRL->DFLLSYNC.bit.ENABLE);

	// setup 48 MHz GCLK3 from DFLL48M
	GCLK->GENCTRL[3].reg =
		GCLK_GENCTRL_DIV(0) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DFLL |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL3);
}

/* Initialize SERCOM2 for 115200 bps 8N1 using a 48 MHz clock */
static inline void uart_init(void)
{
	gpio_set_pin_function(PIN_PB24, PINMUX_PB24D_SERCOM2_PAD1);
	gpio_set_pin_function(PIN_PB25, PINMUX_PB25D_SERCOM2_PAD0);

	MCLK->APBBMASK.bit.SERCOM2_ = 1;
	GCLK->PCHCTRL[SERCOM2_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN;

	BOARD_SERCOM->USART.CTRLA.bit.SWRST = 1; /* reset and disable SERCOM -> enable configuration */
	while (BOARD_SERCOM->USART.SYNCBUSY.bit.SWRST);

	BOARD_SERCOM->USART.CTRLA.reg  =
		SERCOM_USART_CTRLA_SAMPR(0) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_SAMPA(0) | /* 16x over sampling */
		SERCOM_USART_CTRLA_FORM(0) | /* 0x0 USART frame, 0x1 USART frame with parity, ... */
		SERCOM_USART_CTRLA_DORD | /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) | /* 0x0 USART with external clock, 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(1) | /* SERCOM PAD[1] is used for data reception */
		SERCOM_USART_CTRLA_TXPO(0); /* SERCOM PAD[0] is used for data transmission */

	BOARD_SERCOM->USART.CTRLB.reg = /* RXEM = 0 -> receiver disabled, LINCMD = 0 -> normal USART transmission, SFDE = 0 -> start-of-frame detection disabled, SBMODE = 0 -> one stop bit, CHSIZE = 0 -> 8 bits */
		SERCOM_USART_CTRLB_TXEN | /* transmitter enabled */
		SERCOM_USART_CTRLB_RXEN; /* receiver enabled */
	// BOARD_SERCOM->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(0) | SERCOM_USART_BAUD_FRAC_BAUD(26); /* 48000000/(16*115200) = 26.041666667 */
	BOARD_SERCOM->USART.BAUD.reg = SERCOM_USART_BAUD_BAUD(63019); /* 65536*(1−16*115200/48000000) */

	BOARD_SERCOM->USART.CTRLA.bit.ENABLE = 1; /* activate SERCOM */
	while (BOARD_SERCOM->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */
}

/* copy of board_init which uses XTAL + PLL for 200MHz operation */
static inline void same54xplainedpro_init(void)
{
	// Uncomment this line and change the GCLK for UART/USB to run off the XTAL.
	init_clock_xtal();

	SystemCoreClock = CONF_CPU_FREQUENCY;

#if CFG_TUSB_OS  == OPT_OS_NONE
	SysTick_Config(CONF_CPU_FREQUENCY / 1000);
#endif

	uart_init();

#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " UART initialized\n");
	tu_printf(BOARD_NAME " reset cause %#02x\n", RSTC->RCAUSE.reg);
#endif

	// LED0 init
	gpio_set_pin_function(LED_PIN, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(LED_PIN, GPIO_DIRECTION_OUT);
	board_led_write(0);

#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " LED pin configured\n");
#endif

	// BTN0 init
	gpio_set_pin_function(BUTTON_PIN, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(BUTTON_PIN, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(BUTTON_PIN, GPIO_PULL_UP);

#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " Button pin configured\n");
#endif

#if CFG_TUSB_OS == OPT_OS_FREERTOS
	// If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
	NVIC_SetPriority(USB_0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(USB_1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(USB_2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(USB_3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif


#if TUSB_OPT_DEVICE_ENABLED
#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " USB device enabled\n");
#endif

	/* USB clock init
	 * The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
	 * for low speed and full speed operation.
	 */
	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK3_Val | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_AHBMASK_USB_bit(MCLK);
	hri_mclk_set_APBBMASK_USB_bit(MCLK);

	// USB pin init
	gpio_set_pin_direction(PIN_PA24, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PA24, false);
	gpio_set_pin_pull_mode(PIN_PA24, GPIO_PULL_OFF);
	gpio_set_pin_direction(PIN_PA25, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PA25, false);
	gpio_set_pin_pull_mode(PIN_PA25, GPIO_PULL_OFF);

	gpio_set_pin_function(PIN_PA24, PINMUX_PA24H_USB_DM);
	gpio_set_pin_function(PIN_PA25, PINMUX_PA25H_USB_DP);


#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " USB device configured\n");
#endif
#endif
}


int main(void)
{
	same54xplainedpro_init();

	LP_LOG("CONF_CPU_FREQUENCY=%lu\n", CONF_CPU_FREQUENCY);

	tusb_init();

	gpio_set_pin_function(LIN_TX_PIN, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(LIN_TX_PIN, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(LIN_TX_PIN, true);
	gpio_set_pin_pull_mode(LIN_TX_PIN, GPIO_PULL_OFF);

	gpio_set_pin_function(LIN_RX_PIN, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(LIN_RX_PIN, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(LIN_RX_PIN, GPIO_PULL_OFF);
	PORT->Group[2].CTRL.reg |= 0b100000; // continusous sampling for pin 5


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

	for (;;) {
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

