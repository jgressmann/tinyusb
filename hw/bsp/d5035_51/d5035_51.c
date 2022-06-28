/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
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
void USB_Handler (void)
{
	tud_int_handler(0);
}



//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#define LED_PIN            PIN_PA02
#define BOARD_SERCOM       SERCOM2
#define CONF_CPU_FREQUENCY 48000000L
#define USART_BAURATE      115200L
#define BOARD_NAME         "D5035-51"

static inline void init_clock_usb(void)
{
	// Setup DFLL
	// 1. turn off DFLL, also work around Errata reference: 9905
	SYSCTRL->DFLLCTRL.reg = 0;
	while (SYSCTRL->PCLKSR.bit.DFLLRDY);


	// 2. load calibration

	// Load coarse value from NVM, Atmel-42181G–SAM-D21_Datasheet–09/2015 p.32
	// 63:58 DFLL48M COARSE CALDFLL48M Coarse calibration value. Should be written to DFLLVAL register.
	// 73:64 DFLL48M FINE CALDFLL48M Fine calibration value. Should be written to DFLLVAL register.
	uint32_t const NVM_CAL_AREA = 0x806020;
	uint8_t const coarse = ((*(uint8_t*)(NVM_CAL_AREA + 7)) >> 2) & 0x3f;
	uint16_t const fine = (*(uint16_t*)(NVM_CAL_AREA + 8)) & 0x3ff;


	// LOG("DFLL coarse=%x fine=%x\n", coarse, fine);

	SYSCTRL->DFLLVAL.reg =
		SYSCTRL_DFLLVAL_COARSE(coarse) |
		SYSCTRL_DFLLVAL_FINE(fine);


	// 3. Setup target frequency from 1 [kHz] USB SOF
	// USB clock sync, Atmel-42181G–SAM-D21_Datasheet–09/2015 p.156
	SYSCTRL->DFLLMUL.reg =
		SYSCTRL_DFLLMUL_MUL(0xBB80) |
		SYSCTRL_DFLLMUL_FSTEP(1) |
		SYSCTRL_DFLLMUL_CSTEP(1) |
		0;


	// 4. start DFLL
	SYSCTRL->DFLLCTRL.reg =
		SYSCTRL_DFLLCTRL_ENABLE |
		SYSCTRL_DFLLCTRL_USBCRM |
		SYSCTRL_DFLLCTRL_MODE |
		SYSCTRL_DFLLCTRL_CCDIS |
		0;
	while (!SYSCTRL->PCLKSR.bit.DFLLRDY);


	// Must increase to 2 for 48 MHz
	NVMCTRL->CTRLB.bit.RWS = 2;

	// Setup CPU to use 48 MHz
	GCLK->GENCTRL.reg =
		GCLK_GENCTRL_SRC_DFLL48M |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_IDC |
		GCLK_GENCTRL_ID(0);
	while (GCLK->STATUS.bit.SYNCBUSY);

	SystemCoreClock = CONF_CPU_FREQUENCY;
}

void init_clock_xtal(void)
{
	SYSCTRL->XOSC.reg =
		SYSCTRL_XOSC_STARTUP(6) |    // 1,953 ms
		SYSCTRL_XOSC_AMPGC |
		SYSCTRL_XOSC_GAIN(3) |
		SYSCTRL_XOSC_XTALEN |
		SYSCTRL_XOSC_ENABLE;
	while(0 == SYSCTRL->PCLKSR.bit.XOSCRDY);

	/* pre-scaler = 8, input = XOSC, output 2 MHz, output = 48 MHz */
	SYSCTRL->DPLLCTRLB.reg = SYSCTRL_DPLLCTRLB_DIV(3) | SYSCTRL_DPLLCTRLB_REFCLK_REF1;
	SYSCTRL->DPLLRATIO.reg = SYSCTRL_DPLLRATIO_LDRFRAC(0x0) | SYSCTRL_DPLLRATIO_LDR(23);
	SYSCTRL->DPLLCTRLA.reg = SYSCTRL_DPLLCTRLA_ENABLE;
	while((SYSCTRL->DPLLSTATUS.reg & (SYSCTRL_DPLLSTATUS_ENABLE | SYSCTRL_DPLLSTATUS_CLKRDY | SYSCTRL_DPLLSTATUS_LOCK)) != (SYSCTRL_DPLLSTATUS_ENABLE | SYSCTRL_DPLLSTATUS_CLKRDY | SYSCTRL_DPLLSTATUS_LOCK));

	// Must increase to 2 for 48 MHz
	NVMCTRL->CTRLB.bit.RWS = 2;

	GCLK->GENDIV.reg =
		GCLK_GENDIV_DIV(1) |
		GCLK_GENDIV_ID(0);

	// Setup CPU to use 48 MHz
	GCLK->GENCTRL.reg =
		GCLK_GENCTRL_SRC_DPLL96M |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_IDC |
		GCLK_GENCTRL_ID(0);
	while (GCLK->STATUS.bit.SYNCBUSY);

	SystemCoreClock = CONF_CPU_FREQUENCY;
}

void uart_init(void)
{
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(2) |    /* function C */
		PORT_WRCONFIG_DRVSTR |
		PORT_WRCONFIG_PINMASK(0x1000) | /* PA12 */
		PORT_WRCONFIG_PMUXEN;

	PM->APBCMASK.bit.SERCOM2_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK0 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_SERCOM2_CORE;

	BOARD_SERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST; /* disable SERCOM -> enable config */
	while(BOARD_SERCOM->USART.SYNCBUSY.bit.SWRST); /* wait for SERCOM to be ready */

	BOARD_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN; /* transmitter enabled */
	uint16_t baud = CONF_CPU_FREQUENCY / (16 * USART_BAURATE);
	uint16_t frac = CONF_CPU_FREQUENCY / (2 * USART_BAURATE) - 8 * baud;
	BOARD_SERCOM->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(frac) | SERCOM_USART_BAUD_FRAC_BAUD(baud);

	BOARD_SERCOM->USART.CTRLA.reg =
		SERCOM_USART_CTRLA_SAMPR(1) | /* 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_DORD |     /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) |  /* 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(1) |  /* SERCOM PAD[1] is used for data reception */
		SERCOM_USART_CTRLA_TXPO(0) |  /* SERCOM PAD[0] is used for data transmission */
		SERCOM_USART_CTRLA_ENABLE;

	while(BOARD_SERCOM->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */
}

void uart_send_buffer(uint8_t const *text, size_t len)
{
	for (size_t i = 0; i < len; ++i) {
		BOARD_SERCOM->USART.DATA.reg = text[i];
		while((BOARD_SERCOM->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC) == 0);
	}
}

void uart_send_str(const char* text)
{
	while (*text) {
		BOARD_SERCOM->USART.DATA.reg = *text++;
		while((BOARD_SERCOM->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC) == 0);
	}
}

void board_fixups(void)
{
	// Errata reference: 13134, die rev. E
	NVMCTRL->CTRLB.bit.MANW = 1;
}

void board_init(void)
{
	board_fixups();

	// LED init
	gpio_set_pin_direction(LED_PIN, GPIO_DIRECTION_OUT);
	board_led_off();

	// init_clock_xtal();
	init_clock_usb();

#if CFG_TUSB_OS  == OPT_OS_NONE
	SysTick_Config(CONF_CPU_FREQUENCY / 1000);
#endif

	uart_init();
#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " UART initialized\n");
#endif



#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " LED pin configured\n");
#endif

#if CFG_TUSB_OS == OPT_OS_FREERTOS
	// If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
	NVIC_SetPriority(USB_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif


#if TUSB_OPT_DEVICE_ENABLED
#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " USB device enabled\n");
#endif

	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK0 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_USB;

  	gpio_set_pin_function(PIN_PA24, PINMUX_PA24G_USB_DM);
	gpio_set_pin_function(PIN_PA25, PINMUX_PA25G_USB_DP);

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
