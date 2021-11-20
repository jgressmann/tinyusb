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
#define LED_PIN            PIN_PA02
#define BOARD_SERCOM       SERCOM0
#define CONF_CPU_FREQUENCY 48000000L
#define USART_BAURATE      115200L

static inline void init_clock(void)
{
	/* The chip boots up with 48 MHz clock from DFLL48M in open loop mode.
	 * We don't need more for UART, USB
	 */
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
	GCLK->PCHCTRL[SERCOM0_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

	SERCOM0->USART.CTRLA.reg = 0x00; /* disable SERCOM -> enable config */
	while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);

	SERCOM0->USART.CTRLA.reg  =
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_DORD |     /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) |  /* 0x0 USART with external clock, 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(1) |  /* SERCOM PAD[1] is used for data reception */
		SERCOM_USART_CTRLA_TXPO(0);   /* SERCOM PAD[0] is used for data transmission */

	SERCOM0->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN; /* transmitter enabled */
	uint16_t baud = CONF_CPU_FREQUENCY / (16 * USART_BAURATE);
	uint16_t frac = CONF_CPU_FREQUENCY / (2 * USART_BAURATE) - 8 * baud;
	SERCOM0->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(frac) | SERCOM_USART_BAUD_FRAC_BAUD(baud);

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
	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK0_Val | GCLK_PCHCTRL_CHEN);
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
