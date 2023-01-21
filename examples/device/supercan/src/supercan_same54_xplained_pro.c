/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2023 Jean Gressmann <jean@0x42.de>
 *
 */

#include <supercan_board.h>

#if SAME54XPLAINEDPRO

#include <supercan_debug.h>
#include <sam_crc32.h>
#include <bsp/board.h>

#include <hal/include/hal_gpio.h>

// controller and hardware specific setup of i/o pins for CAN
static inline void can_init_pins(void)
{
	gpio_set_pin_function(PIN_PC13, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(PIN_PC13, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PC13, false);
	gpio_set_pin_pull_mode(PIN_PC13, GPIO_PULL_OFF);

	PORT->Group[1].WRCONFIG.reg =
		PORT_WRCONFIG_PINMASK(0x3000) | // PB12/13 = 0x3000
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(7) |         // H, CAN1, DS60001507E page 32, 910
		PORT_WRCONFIG_PMUXEN;
}


extern void sc_board_led_set(uint8_t index, bool on)
{
	SC_DEBUG_ASSERT(0 == index);
	(void)index;

	board_led_write(on);
}


extern void sc_board_leds_on_unsafe(void)
{
	board_led_write(true);
}

static void can_init_module(void)
{
	same5x_can_init();

	same5x_cans[0].m_can = CAN1;
	same5x_cans[0].interrupt_id = CAN1_IRQn;

	for (size_t j = 0; j < TU_ARRAY_SIZE(same5x_cans); ++j) {
		struct same5x_can *can = &same5x_cans[j];

		can->features = CAN_FEAT_PERM;

		for (size_t i = 0; i < TU_ARRAY_SIZE(same5x_cans[0].rx_fifo); ++i) {
			SC_DEBUG_ASSERT(can->rx_frames[i].ts == 0);
		}

		for (size_t i = 0; i < TU_ARRAY_SIZE(same5x_cans[0].tx_fifo); ++i) {
			SC_DEBUG_ASSERT(can->tx_frames[i].ts == 0);
		}
	}

	m_can_init_begin(CAN1);

	CAN1->MRCFG.reg = CAN_MRCFG_QOS_HIGH;

	NVIC_SetPriority(CAN1_IRQn, SC_ISR_PRIORITY);
}



static inline void can_init_clock(void) // controller and hardware specific setup of clock for the m_can module
{
	// PLL output must be >= 96 MHz, see DS60001507E, p. 763, configure for 160 MHz
	OSCCTRL->Dpll[1].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_DIV(2) | OSCCTRL_DPLLCTRLB_REFCLK_XOSC1; /* 12 MHz / 6 = 2 Mhz, input = XOSC1 */
	OSCCTRL->Dpll[1].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x0) | OSCCTRL_DPLLRATIO_LDR(79);
	OSCCTRL->Dpll[1].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_RUNSTDBY | OSCCTRL_DPLLCTRLA_ENABLE;
	while(0 == OSCCTRL->Dpll[1].DPLLSTATUS.bit.CLKRDY); /* wait for the PLL to be ready */

	// Setup GCLK4 to provide 80 MHz
	GCLK->GENCTRL[4].reg =
		GCLK_GENCTRL_DIV(2) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DPLL1 |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL4); /* wait for the synchronization between clock domains to be complete */


	MCLK->AHBMASK.bit.CAN1_ = 1;
	GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;
}


SC_RAMFUNC extern void CAN1_Handler(void)
{
	// LOG("CAN1 int\n");

	same5x_can_int(0); // map to index 0
}





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
static inline void clock_init(void)
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

	SERCOM2->USART.CTRLA.bit.SWRST = 1; /* reset and disable SERCOM -> enable configuration */
	while (SERCOM2->USART.SYNCBUSY.bit.SWRST);

	SERCOM2->USART.CTRLA.reg  =
		SERCOM_USART_CTRLA_SAMPR(0) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_SAMPA(0) | /* 16x over sampling */
		SERCOM_USART_CTRLA_FORM(0) | /* 0x0 USART frame, 0x1 USART frame with parity, ... */
		SERCOM_USART_CTRLA_DORD | /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) | /* 0x0 USART with external clock, 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(1) | /* SERCOM PAD[1] is used for data reception */
		SERCOM_USART_CTRLA_TXPO(0); /* SERCOM PAD[0] is used for data transmission */

	SERCOM2->USART.CTRLB.reg = /* RXEM = 0 -> receiver disabled, LINCMD = 0 -> normal USART transmission, SFDE = 0 -> start-of-frame detection disabled, SBMODE = 0 -> one stop bit, CHSIZE = 0 -> 8 bits */
		SERCOM_USART_CTRLB_TXEN | /* transmitter enabled */
		SERCOM_USART_CTRLB_RXEN; /* receiver enabled */
	// SERCOM2->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(0) | SERCOM_USART_BAUD_FRAC_BAUD(26); /* 48000000/(16*115200) = 26.041666667 */
	SERCOM2->USART.BAUD.reg = SERCOM_USART_BAUD_BAUD(63019); /* 65536*(1−16*115200/48000000) */

	SERCOM2->USART.CTRLA.bit.ENABLE = 1; /* activate SERCOM */
	while (SERCOM2->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */
}

static inline void usb_init(void)
{
	NVIC_SetPriority(USB_0_IRQn, SC_ISR_PRIORITY);
	NVIC_SetPriority(USB_1_IRQn, SC_ISR_PRIORITY);
	NVIC_SetPriority(USB_2_IRQn, SC_ISR_PRIORITY);
	NVIC_SetPriority(USB_3_IRQn, SC_ISR_PRIORITY);


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
}

void board_init_app(void)
{
	sam_crc32_unlock();
	clock_init();

	SystemCoreClock = CONF_CPU_FREQUENCY;

	uart_init();
	LOG("CONF_CPU_FREQUENCY=%lu\n", (unsigned long)CONF_CPU_FREQUENCY);

	// LED0 init
	gpio_set_pin_function(PIN_PC18, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(PIN_PC18, GPIO_DIRECTION_OUT);
	board_led_write(0);

	usb_init();
}

static inline void counter_1MHz_init(void)
{
	GCLK->GENCTRL[5].reg =
		GCLK_GENCTRL_DIV(10) |	/* 160Mhz -> 16MHz */
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DPLL1 |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL5); /* wait for the synchronization between clock domains to be complete */

	// TC0 and TC1 pair to form a single 32 bit counter
	// TC1 is enslaved to TC0 and doesn't need to be configured.
	// DS60001507E-page 1716, 48.6.2.4 Counter Mode
	MCLK->APBAMASK.bit.TC0_ = 1;
	MCLK->APBAMASK.bit.TC1_ = 1;
	GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK5 | GCLK_PCHCTRL_CHEN;
	GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK5 | GCLK_PCHCTRL_CHEN;

	TC0->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
	while(1 == TC0->COUNT32.SYNCBUSY.bit.SWRST);

	/* 16MHz -> 1MHz */
	TC0->COUNT32.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV16;
	while(1 == TC0->COUNT32.SYNCBUSY.bit.ENABLE);
}

extern void sc_board_init_begin(void)
{
	board_init_app();

	LOG("Enabling cache\n");
	same5x_enable_cache();

	same5x_init_device_identifier();


	can_init_pins();
	can_init_clock();
	can_init_module();

	counter_1MHz_init();
}

extern void sc_board_init_end(void)
{

}

#endif // #ifdef SAME54XPLAINEDPRO
