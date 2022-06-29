/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include <sllin_board.h>

#if TRINKET_M0


#include <leds.h>

#include <hal/include/hal_gpio.h>
#include <bsp/board.h>
#include <tusb.h>

#define BOARD_SERCOM SERCOM0	// TinyUSB board setup
#define USART_BAURATE      115200
#define CONF_CPU_FREQUENCY 48000000

enum {
	DOTSTAR_NORMAL_LED_INTENSITY = 0x40,

	DMA_DESCS = 1
};


static uint32_t dma_wb[DMA_DESCS * 4] __attribute__ ((aligned(16)));	// 16 byte per channel
static DmacDescriptor dma_desc[DMA_DESCS] __attribute__ ((aligned(16)));

struct sam_lin sam_lins[SLLIN_BOARD_LIN_COUNT] = {
	{
		.sercom = SERCOM2,
		.timer = TC3,
		.timer_irq = TC3_IRQn,
		.rx_port_pin_mux = 9,
		.master_slave_port_pin_mux = 2,
		.led_status_green = SLLIN_BOARD_DOTSTAR_GREEN,
		.led_status_red = SLLIN_BOARD_DOTSTAR_RED,
	}
};

static struct {
	volatile uint32_t sof;
	volatile uint8_t a;
	volatile uint8_t b;
	volatile uint8_t g;
	volatile uint8_t r;
	volatile uint32_t eof;
} dotstar = {
	.sof = 0,
	.a = 0xff,
	.b = 0,
	.g = 0,
	.r = 0,
	.eof = 0xffffffff,
};

_Static_assert(sizeof(dotstar) == 12, "");

#define dotstar_update() do { DMAC->CHCTRLA.reg = DMAC_CHCTRLA_ENABLE; } while (0)

static void leds_init(void)
{
	Sercom * const s = SERCOM1;

	// the one true LED
	PORT->Group[0].DIRSET.reg = 10;

	// Drive APA-102-2022 with SPI on SERCOM1

	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_DRVSTR |
		PORT_WRCONFIG_PINMASK(0x0003) | /* PA00, PA1 */
		PORT_WRCONFIG_PMUXEN;

	PM->APBCMASK.bit.SERCOM1_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK2 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_SERCOM1_CORE;


	s->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_SWRST; /* disable SERCOM -> enable config */
	while(s->SPI.SYNCBUSY.bit.SWRST); /* wait for SERCOM to be ready */

	s->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD(7); // 1 MHz
	s->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_DRE | SERCOM_SPI_INTENSET_ERROR;
	s->SPI.CTRLA.reg =
		SERCOM_SPI_CTRLA_ENABLE |
		SERCOM_SPI_CTRLA_DIPO(2) | // input on PAD2 PAD0 = DO, PAD1 = SCK
		SERCOM_SPI_CTRLA_MODE(3); // master mode

	while(s->SPI.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */


	// DMA setup to transfer w/o CPU

	dma_desc[0].BTCTRL.reg =
		DMAC_BTCTRL_VALID |
		DMAC_BTCTRL_BEATSIZE_BYTE |
		DMAC_BTCTRL_SRCINC;

	dma_desc[0].BTCNT.reg = 12;
	dma_desc[0].SRCADDR.reg = (uint32_t)(((uint8_t*)&dotstar) + sizeof(dotstar));
	dma_desc[0].DSTADDR.reg = (uint32_t)&s->SPI.DATA;
	dma_desc[0].DESCADDR.reg = 0;


	DMAC->CTRL.bit.SWRST = 1;
	while (DMAC->CTRL.bit.SWRST);

	DMAC->BASEADDR.reg = (uint32_t)(void*)dma_desc;
	DMAC->WRBADDR.reg = (uint32_t)(void*)dma_wb;
	DMAC->CHID.reg = 0;
	DMAC->CHCTRLB.reg =
		DMAC_CHCTRLB_TRIGSRC(0x04) | // SERCOM1 TX
		DMAC_CHCTRLB_TRIGACT_BEAT |
		0;
	DMAC->CHCTRLA.reg = DMAC_CHCTRLA_ENABLE;

	DMAC->CTRL.reg =
		DMAC_CTRL_DMAENABLE |
		DMAC_CTRL_LVLEN0 |
		DMAC_CTRL_LVLEN1 |
		DMAC_CTRL_LVLEN2 |
		DMAC_CTRL_LVLEN3;
}


static uint32_t device_identifier;

extern uint32_t sllin_board_identifier(void)
{
	return device_identifier;
}

extern void sllin_board_led_set(uint8_t index, bool on)
{
	switch (index) {
	case SLLIN_BOARD_DEBUG_DEFAULT:
		if (on) {
			PORT->Group[0].OUTSET.reg = UINT32_C(1) << 10;
		} else {
			PORT->Group[0].OUTCLR.reg = UINT32_C(1) << 10;
		}
		break;
	case SLLIN_BOARD_DOTSTAR_RED:
		dotstar.r = on * DOTSTAR_NORMAL_LED_INTENSITY;
		dotstar_update();
		break;
	case SLLIN_BOARD_DOTSTAR_GREEN:
		dotstar.g = on * DOTSTAR_NORMAL_LED_INTENSITY;
		dotstar_update();
		break;
	case SLLIN_BOARD_DOTSTAR_BLUE:
		dotstar.b = on * DOTSTAR_NORMAL_LED_INTENSITY;
		dotstar_update();
		break;
	}
}

extern void sllin_board_leds_on_unsafe(void)
{
	PORT->Group[0].OUTSET.reg = UINT32_C(1) << 10;

	dotstar.r = DOTSTAR_NORMAL_LED_INTENSITY;
	dotstar.g = DOTSTAR_NORMAL_LED_INTENSITY;
	dotstar.b = DOTSTAR_NORMAL_LED_INTENSITY;
	dotstar_update();
}

static inline void uart_init(void)
{
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_DRVSTR |
		PORT_WRCONFIG_PINMASK(0x0040) | /* PA06 */
		PORT_WRCONFIG_PMUXEN;

	PM->APBCMASK.bit.SERCOM0_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK0 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_SERCOM0_CORE;

	BOARD_SERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST; /* disable SERCOM -> enable config */
	while(BOARD_SERCOM->USART.SYNCBUSY.bit.SWRST); /* wait for SERCOM to be ready */

	BOARD_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN; /* transmitter enabled */
	uint16_t baud = CONF_CPU_FREQUENCY / (16 * USART_BAURATE);
	uint16_t frac = CONF_CPU_FREQUENCY / (2 * USART_BAURATE) - 8 * baud;
	BOARD_SERCOM->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(frac) | SERCOM_USART_BAUD_FRAC_BAUD(baud);

	BOARD_SERCOM->USART.CTRLA.reg =
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_DORD |     /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) |  /* 0x0 USART with external clock, 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(3) |  /* SERCOM PAD[3] is used for data reception */
		SERCOM_USART_CTRLA_TXPO(1) |   /* SERCOM PAD[2] is used for data transmission */
		SERCOM_USART_CTRLA_ENABLE;

	while(BOARD_SERCOM->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */
}


static inline void clock_init(void)
{
	// Setup DFLL
	// 1. turn off DFLL
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


	// 3. Setup target frequency from 1 [ms] USB SOF
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

	// Setup 16 MHz on GCLK2
	GCLK->GENDIV.reg =
		GCLK_GENDIV_DIV(3) |
		GCLK_GENDIV_ID(2);

	GCLK->GENCTRL.reg =
		GCLK_GENCTRL_SRC_DFLL48M |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_IDC |
		GCLK_GENCTRL_ID(2);
	while (GCLK->STATUS.bit.SYNCBUSY);
}


static inline void lin_init_once(void)
{
	// lin0
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		// PORT_WRCONFIG_DRVSTR |
		PORT_WRCONFIG_PINMASK(0x0300) | /* PA08, PA9 */
		PORT_WRCONFIG_PMUXEN;

	PM->APBCMASK.bit.SERCOM2_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK0 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_SERCOM2_CORE;


	NVIC_SetPriority(SERCOM2_IRQn, SLLIN_ISR_PRIORITY);

	NVIC_EnableIRQ(SERCOM2_IRQn);

	// master / slave pin
	// PA02 lin0
	PORT->Group[0].DIRSET.reg = (UINT32_C(1) << 2);

	sam_lin_init_once();
}

static void timer_init(void)
{
	PM->APBCMASK.bit.TCC2_ = 1;
	PM->APBCMASK.bit.TC3_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK2 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_TCC2_TC3;

	NVIC_SetPriority(TC3_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_EnableIRQ(TC3_IRQn);

	for (size_t i = 0; i < TU_ARRAY_SIZE(sam_lins); ++i) {
		struct sam_lin *lin = &sam_lins[i];
		Tc* timer = lin->timer;

		timer->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
		while (timer->COUNT16.STATUS.bit.SYNCBUSY);

		// timer overflow interrupt
		timer->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_ERR;

		// enable COUNT register sampling
		timer->COUNT16.READREQ.reg = TC_READREQ_RCONT | 0x10;

		// set to max so we don't time out
		timer->COUNT16.CC[0].reg = 0xffff;

		// enable
		timer->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_WAVEGEN_MFRQ;

		// stop & oneshot
		timer->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT | TC_CTRLBSET_CMD_STOP;
		while (timer->COUNT16.STATUS.bit.SYNCBUSY);

		// reset to zero
		timer->COUNT16.COUNT.reg = 0;
	}
}

static inline void usb_init(void)
{
	NVIC_SetPriority(USB_IRQn, SLLIN_ISR_PRIORITY);

	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK0 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_USB;

	// gpio_set_pin_direction(PIN_PA24, GPIO_DIRECTION_OUT);
	// gpio_set_pin_level(PIN_PA24, false);
	// gpio_set_pin_pull_mode(PIN_PA24, GPIO_PULL_OFF);
	// gpio_set_pin_direction(PIN_PA25, GPIO_DIRECTION_OUT);
	// gpio_set_pin_level(PIN_PA25, false);
	// gpio_set_pin_pull_mode(PIN_PA25, GPIO_PULL_OFF);

	gpio_set_pin_function(PIN_PA24, PINMUX_PA24G_USB_DM);
	gpio_set_pin_function(PIN_PA25, PINMUX_PA25G_USB_DP);
}

extern void sllin_board_init_begin(void)
{
	uint32_t const serial_number[4] = {
		*(uint32_t const *)0x0080A00C,
		*(uint32_t const *)0x0080A040,
		*(uint32_t const *)0x0080A044,
		*(uint32_t const *)0x0080A048
	};

	clock_init();
	uart_init();

	leds_init();

	device_identifier = sam_init_device_identifier(serial_number);

	LOG("USB init\n");
	usb_init();

	LOG("timer init\n");
	timer_init();

	LOG("LIN init once\n");
	lin_init_once();
}

extern void sllin_board_init_end(void)
{
	led_blink(0, 2000);
}

SLLIN_RAMFUNC void SERCOM2_Handler(void)
{
	sam_lin_usart_int(0);
}

SLLIN_RAMFUNC void TC3_Handler(void)
{
	sam_lin_timer_int(0);
}

#endif // #if TRINKET_M0
