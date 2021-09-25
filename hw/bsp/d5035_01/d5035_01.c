/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020-2021 Jean Gressmann <jean@0x42.de>
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

#include <sam.h>
#include "bsp/board.h"

#include <hal/include/hal_gpio.h>

#if CONF_CPU_FREQUENCY != 80000000
# error "CONF_CPU_FREQUENCY" must 80000000
#endif

#if CONF_GCLK_USB_FREQUENCY != 48000000
# error "CONF_GCLK_USB_FREQUENCY" must 48000000
#endif

#if !defined(HWREV)
# error Define "HWREV"
#endif

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_0_Handler (void)
{
	tud_int_handler(0);
}

void USB_1_Handler (void)
{
	tud_int_handler(0);
}

void USB_2_Handler (void)
{
	tud_int_handler(0);
}

void USB_3_Handler (void)
{
	tud_int_handler(0);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#define LED_PIN      PIN_PA02
#define BOARD_SERCOM SERCOM0


static inline void init_clock(void)
{
	/* AUTOWS is enabled by default in REG_NVMCTRL_CTRLA - no need to change the number of wait states when changing the core clock */

	// We may be starting from an older bootloader that
	// configured the clocks differently than what we want them.
	//
	// Before we can start changing the PLLs, we need to ensure
	// we are running off the 48 MHz FLL. The FLL might be off,
	// so we first need to turn it on.
	OSCCTRL->DFLLCTRLA.reg =
		OSCCTRL_DFLLCTRLA_ENABLE |
		OSCCTRL_DFLLCTRLA_RUNSTDBY;
	while(1 == OSCCTRL->DFLLSYNC.bit.ENABLE);

	// Now that the FLL is running, we can change the
	// the main clock. This ensures we continue to run.
	GCLK->GENCTRL[0].reg =
		GCLK_GENCTRL_DIV(0) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DFLL |
		GCLK_GENCTRL_IDC ;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL0); /* wait for the synchronization between clock domains to be complete */

	// Here we are running of the FLL and can safely modify the PLLs.

	/* configure XOSC0 for a 16MHz crystal connected to XIN0/XOUT0 */
	OSCCTRL->XOSCCTRL[0].reg =
		OSCCTRL_XOSCCTRL_STARTUP(6) |    // 1,953 ms
		OSCCTRL_XOSCCTRL_RUNSTDBY |
		OSCCTRL_XOSCCTRL_ENALC |
		OSCCTRL_XOSCCTRL_IMULT(4) |
		OSCCTRL_XOSCCTRL_IPTAT(3) |
		OSCCTRL_XOSCCTRL_XTALEN |
		OSCCTRL_XOSCCTRL_ENABLE;
	while(0 == OSCCTRL->STATUS.bit.XOSCRDY0);

	/* pre-scaler = 8, input = XOSC0, ouput 2 MHz, ouput = 160 MHz (>= 96 MHz DS60001507E, page 763) */
	OSCCTRL->Dpll[0].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_DIV(3) | OSCCTRL_DPLLCTRLB_REFCLK(OSCCTRL_DPLLCTRLB_REFCLK_XOSC0_Val);
	OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x0) | OSCCTRL_DPLLRATIO_LDR(79);
	OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_RUNSTDBY | OSCCTRL_DPLLCTRLA_ENABLE;
	while(0 == OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY); /* wait for the PLL0 to be ready */

	// configure GCLK2 for 16MHz from XOSC0
	GCLK->GENCTRL[2].reg =
		GCLK_GENCTRL_DIV(0) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_XOSC0 |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL2); /* wait for the synchronization between clock domains to be complete */

	/* 80 MHz core clock */
	GCLK->GENCTRL[0].reg =
		GCLK_GENCTRL_DIV(2) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DPLL0 |  /* DPLL0 */
		GCLK_GENCTRL_IDC ;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL0); /* wait for the synchronization between clock domains to be complete */


	/* USB 48 MHz clock */
	/* setup DFLL48M to use GLCK2 (16 MHz) */
	GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN;

	OSCCTRL->DFLLCTRLA.reg = 0;
	while(1 == OSCCTRL->DFLLSYNC.bit.ENABLE);

	OSCCTRL->DFLLCTRLB.reg = OSCCTRL_DFLLCTRLB_MODE | OSCCTRL_DFLLCTRLB_WAITLOCK;
	OSCCTRL->DFLLMUL.bit.MUL = 3; // 3 * 16 MHz -> 48 MHz

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

static inline void uart_init(void)
{
	/* configure SERCOM0 on PA08 */
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(2) |    /* function C */
		PORT_WRCONFIG_DRVSTR |
		PORT_WRCONFIG_PINMASK(0x0100) | /* PA08 */
		PORT_WRCONFIG_PMUXEN;

	MCLK->APBAMASK.bit.SERCOM0_ = 1;
	GCLK->PCHCTRL[SERCOM0_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; /* setup SERCOM to use GLCK2 -> 80MHz */

	SERCOM0->USART.CTRLA.reg = 0x00; /* disable SERCOM -> enable config */
	while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);

	SERCOM0->USART.CTRLA.reg  =  /* CMODE = 0 -> async, SAMPA = 0, FORM = 0 -> USART frame, SMPR = 0 -> arithmetic baud rate */
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
//    SERCOM_USART_CTRLA_FORM(0) | /* 0 = USART Frame, 2 = LIN Master */
		SERCOM_USART_CTRLA_DORD | /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) | /* 0 = Asynchronous, 1 = USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(1) | /* SERCOM PAD[1] is used for data reception */
		SERCOM_USART_CTRLA_TXPO(0); /* SERCOM PAD[0] is used for data transmission */

	SERCOM0->USART.CTRLB.reg = /* RXEM = 0 -> receiver disabled, LINCMD = 0 -> normal USART transmission, SFDE = 0 -> start-of-frame detection disabled, SBMODE = 0 -> one stop bit, CHSIZE = 0 -> 8 bits */
		SERCOM_USART_CTRLB_TXEN; /* transmitter enabled */
	SERCOM0->USART.CTRLC.reg = 0x00;
	// 21.701388889 @ baud rate of 230400 bit/s, table 33-2, p 918 of DS60001507E
	SERCOM0->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(7) | SERCOM_USART_BAUD_FRAC_BAUD(21);

	SERCOM0->USART.CTRLA.bit.ENABLE = 1; /* activate SERCOM */
	while(SERCOM0->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */
}

static inline void uart_send_buffer(uint8_t const *text, size_t len)
{
	for (size_t i = 0; i < len; ++i) {
		BOARD_SERCOM->USART.DATA.reg = text[i];
		while((BOARD_SERCOM->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC) == 0);
	}
}

static inline void uart_send_str(const char* text)
{
	while (*text) {
		BOARD_SERCOM->USART.DATA.reg = *text++;
		while((BOARD_SERCOM->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC) == 0);
	}
}


void board_init(void)
{
	init_clock();

	SystemCoreClock = CONF_CPU_FREQUENCY;

#if CFG_TUSB_OS  == OPT_OS_NONE
	SysTick_Config(CONF_CPU_FREQUENCY / 1000);
#endif

	uart_init();
#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " UART initialized\n");
	tu_printf(BOARD_NAME " reset cause %#02x\n", RSTC->RCAUSE.reg);
#endif

	// LED init
	gpio_set_pin_direction(LED_PIN, GPIO_DIRECTION_OUT);
	board_led_off();

#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " LED pin configured\n");
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
	 * for low speed and full speed operation. */
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

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
	gpio_set_pin_level(LED_PIN, state);
}

uint32_t board_button_read(void)
{
	// this board has no button
	return 0;
}

int board_uart_read(uint8_t* buf, int len)
{
	(void) buf; (void) len;
	return 0;
}

int board_uart_write(void const * buf, int len)
{
	if (len < 0) {
		uart_send_str(buf);
	} else {
		uart_send_buffer(buf, len);
	}
	return len;
}

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void)
{
	system_ticks++;
}

uint32_t board_millis(void)
{
	return system_ticks;
}
#endif

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}
