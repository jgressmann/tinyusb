/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */


#include <sllin_board.h>

#if D5035_50

#include <leds.h>

#include <hal/include/hal_gpio.h>
#include <mcu.h>
#include <sam_crc32.h>


#include <tusb.h>

#define BOARD_SERCOM SERCOM2
#define USART_BAURATE      115200
#define CONF_CPU_FREQUENCY 48000000


/* The board runs off of the 48 MHz internal RC oscillator
 *
 * @115200 debug prints KILL LIN timing assumptions!
*/

struct sam_lin sam_lins[SLLIN_BOARD_LIN_COUNT] = {
	{
		.sercom = SERCOM1,
		.timer = TC0, // NOTE: TC0/TC1 don't seem to support different clock speeds, setting one will affect the other.
		.timer_irq = TC0_IRQn,
		.rx_port_pin_mux = 1,
		.master_slave_port_pin_mux = (1ul << 5) | 4, // PB04
		.led_status_green = 5,
		.led_status_red = 6,
	},
	{
		.sercom = SERCOM0,
		.timer = TC1,
		.timer_irq = TC1_IRQn,
		.rx_port_pin_mux = 5,
		.master_slave_port_pin_mux = (1ul << 5) | 9, // PB09
		.led_status_green = 7,
		.led_status_red = 8,
	},
};

struct led {
	uint8_t pin;
};

#define LED_STATIC_INITIALIZER(name, pin) \
	{ pin }


static const struct led leds[] = {
	LED_STATIC_INITIALIZER("debug_default", PIN_PA02), // board led
	LED_STATIC_INITIALIZER("debug_red", PIN_PA18),
	LED_STATIC_INITIALIZER("debug_orange", PIN_PA19),
	LED_STATIC_INITIALIZER("debug_green", PIN_PB16),
	LED_STATIC_INITIALIZER("debug_blue", PIN_PB17),
	LED_STATIC_INITIALIZER("lin0_green", PIN_PB00),
	LED_STATIC_INITIALIZER("lin0_red", PIN_PB01),
	LED_STATIC_INITIALIZER("lin1_green", PIN_PB02),
	LED_STATIC_INITIALIZER("lin1_red", PIN_PB03),
};

static inline void leds_init(void)
{
	PORT->Group[0].DIRSET.reg = PORT_PA02 | PORT_PA18 | PORT_PA19;
	PORT->Group[1].DIRSET.reg = PORT_PB16 | PORT_PB17 | PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03;
}

#define POWER_LED LED_DEBUG_0
#define USB_LED LED_DEBUG_3



extern uint32_t _svectors;
extern uint32_t _evectors;

static void move_vector_table_to_ram(void)
{
	uint8_t* svectors = (void*)&_svectors;
	uint8_t* evectors = (void*)&_evectors;

	memcpy(svectors, (void*)SCB->VTOR, evectors - svectors);

	SCB->VTOR = (uint32_t)svectors;
}


static inline void same5x_enable_cache(void)
{
	// DS60001507E-page 83
	if (!CMCC->SR.bit.CSTS) {
		CMCC->CTRL.bit.CEN = 1;
	}
}

static uint32_t device_identifier;

extern uint32_t sllin_board_identifier(void)
{
	return device_identifier;
}


static inline void same5x_init_device_identifier(void)
{
	uint32_t serial_number[4];

	sam_get_serial_number(serial_number);

	device_identifier = sam_init_device_identifier(serial_number);
}

extern void sllin_board_led_set(uint8_t index, bool on)
{
	SLLIN_DEBUG_ASSERT(index < TU_ARRAY_SIZE(leds));

	gpio_set_pin_level(leds[index].pin, on);
}

extern void sllin_board_leds_on_unsafe(void)
{
	for (size_t i = 0; i < TU_ARRAY_SIZE(leds); ++i) {
		gpio_set_pin_level(leds[i].pin, 1);
	}
}

static inline void clock_init(void)
{
	// /* AUTOWS is enabled by default in REG_NVMCTRL_CTRLA - no need to change the number of wait states when changing the core clock */

	// /* We assume we are running the chip in default settings.
	//  * This means we are running off of the 48 MHz FLL.
	//  */

	// /* configure XOSC0 for a 16MHz crystal  / oscillator connected to XIN0/XOUT0 */
	// OSCCTRL->XOSCCTRL[0].reg =
	// 	OSCCTRL_XOSCCTRL_STARTUP(6) |    // 1,953 ms
	// 	OSCCTRL_XOSCCTRL_RUNSTDBY |
	// 	OSCCTRL_XOSCCTRL_ENALC |
	// 	OSCCTRL_XOSCCTRL_IMULT(4) |
	// 	OSCCTRL_XOSCCTRL_IPTAT(3) |
	// 	OSCCTRL_XOSCCTRL_XTALEN |
	// 	OSCCTRL_XOSCCTRL_ENABLE;
	// while(0 == OSCCTRL->STATUS.bit.XOSCRDY0);

	// /* pre-scaler = 8, input = XOSC0, output 2 MHz, output = 96 MHz (>= 96 MHz DS60001507E, page 763) */
	// OSCCTRL->Dpll[0].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_DIV(3) | OSCCTRL_DPLLCTRLB_REFCLK(OSCCTRL_DPLLCTRLB_REFCLK_XOSC0_Val);
	// OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x0) | OSCCTRL_DPLLRATIO_LDR(47);
	// OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_RUNSTDBY | OSCCTRL_DPLLCTRLA_ENABLE;
	// while(0 == OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY); /* wait for the PLL0 to be ready */

	// /* 48 MHz core clock */
	// GCLK->GENCTRL[0].reg =
	// 	GCLK_GENCTRL_DIV(2) |
	// 	GCLK_GENCTRL_RUNSTDBY |
	// 	GCLK_GENCTRL_GENEN |
	// 	GCLK_GENCTRL_SRC_DPLL0 |  /* DPLL0 */
	// 	GCLK_GENCTRL_IDC;
	// while(1 == GCLK->SYNCBUSY.bit.GENCTRL0); /* wait for the synchronization between clock domains to be complete */

	// /* Here we are running from the 48 MHz oscillator clock */


	SystemCoreClock = CONF_CPU_FREQUENCY;

	// 16MHz on GCLK2
	GCLK->GENCTRL[2].reg =
		GCLK_GENCTRL_DIV(3) |	/* 48Mhz -> 16MHz */
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DFLL |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL2); /* wait for the synchronization between clock domains to be complete */

	// 1kHz on GCLK3
	GCLK->GENCTRL[3].reg =
		GCLK_GENCTRL_DIV(32) |	/* 32.768Hz -> ~1kHz */
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_OSCULP32K |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL3); /* wait for the synchronization between clock domains to be complete */
}

static inline void uart_init(void)
{
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_DRVSTR |
		PORT_WRCONFIG_PINMASK(0x0300) | /* PA08, PA9 */
		PORT_WRCONFIG_PMUXEN;

	MCLK->APBBMASK.bit.SERCOM2_ = 1;
	GCLK->PCHCTRL[SERCOM2_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

	BOARD_SERCOM->USART.CTRLA.bit.SWRST = 1; /* disable SERCOM -> enable config */
	while(BOARD_SERCOM->USART.SYNCBUSY.bit.SWRST);

	BOARD_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN; /* transmitter enabled */
	uint16_t baud = CONF_CPU_FREQUENCY / (16 * USART_BAURATE);
	uint16_t frac = CONF_CPU_FREQUENCY / (2 * USART_BAURATE) - 8 * baud;
	BOARD_SERCOM->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(frac) | SERCOM_USART_BAUD_FRAC_BAUD(baud);

	BOARD_SERCOM->USART.CTRLA.reg =
		SERCOM_USART_CTRLA_ENABLE |
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_DORD |     /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) |  /* 0x0 USART with external clock, 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(1) |  /* SERCOM PAD[1] is used for data reception */
		SERCOM_USART_CTRLA_TXPO(0);   /* SERCOM PAD[0] is used for data transmission */

	while(BOARD_SERCOM->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */
}


static inline void lin_init_once(void)
{
	// lin0
	// gpio_set_pin_function(PIN_PA00, PINMUX_PA00D_SERCOM1_PAD0);
	// gpio_set_pin_function(PIN_PA01, PINMUX_PA01D_SERCOM1_PAD1);
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_INEN |
		PORT_WRCONFIG_PINMASK(0x0003) | /* PA00, PA01 */
		PORT_WRCONFIG_PMUXEN;

	MCLK->APBAMASK.bit.SERCOM1_ = 1;
	GCLK->PCHCTRL[SERCOM1_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

	NVIC_SetPriority(SERCOM1_0_IRQn, SLLIN_ISR_PRIORITY);
	// NVIC_SetPriority(SERCOM1_1_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(SERCOM1_2_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(SERCOM1_3_IRQn, SLLIN_ISR_PRIORITY);

	NVIC_EnableIRQ(SERCOM1_0_IRQn); // DRE
	// NVIC_EnableIRQ(SERCOM1_1_IRQn); // TXC
	NVIC_EnableIRQ(SERCOM1_2_IRQn); // RXC
	NVIC_EnableIRQ(SERCOM1_3_IRQn); // RXS, CTSIC, RXBRK, ERROR

	// lin1
	// gpio_set_pin_function(PIN_PA04, PINMUX_PA04D_SERCOM0_PAD0);
	// gpio_set_pin_function(PIN_PA05, PINMUX_PA05D_SERCOM0_PAD1);
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_INEN |
		PORT_WRCONFIG_PINMASK(0x0030) | /* PA04, PA05 */
		PORT_WRCONFIG_PMUXEN;

	MCLK->APBAMASK.bit.SERCOM0_ = 1;
	GCLK->PCHCTRL[SERCOM0_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

	NVIC_SetPriority(SERCOM0_0_IRQn, SLLIN_ISR_PRIORITY);
	// NVIC_SetPriority(SERCOM0_1_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(SERCOM0_2_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(SERCOM0_3_IRQn, SLLIN_ISR_PRIORITY);

	NVIC_EnableIRQ(SERCOM0_0_IRQn);
	// NVIC_EnableIRQ(SERCOM0_1_IRQn);
	NVIC_EnableIRQ(SERCOM0_2_IRQn);
	NVIC_EnableIRQ(SERCOM0_3_IRQn);


	sam_lin_init_once();
}

static void timer_init(void)
{
	MCLK->APBAMASK.bit.TC0_ = 1;
	MCLK->APBAMASK.bit.TC1_ = 1;
	// TC0/1 are connected to the SAME peripheral clock
	GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN;
	GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN;

	NVIC_SetPriority(TC0_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(TC1_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_EnableIRQ(TC0_IRQn);
	NVIC_EnableIRQ(TC1_IRQn);

	for (size_t i = 0; i < TU_ARRAY_SIZE(sam_lins); ++i) {
		struct sam_lin *lin = &sam_lins[i];
		Tc* timer = lin->timer;

		timer->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
		while (timer->COUNT16.SYNCBUSY.bit.SWRST);

		timer->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

		// timer overflow interrupt
		timer->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_ERR;

		// set to max so we don't time out
		timer->COUNT16.CC[0].reg = 0xffff;

		// enable
		timer->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV16;

		// stop & oneshot
		timer->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT | TC_CTRLBSET_LUPD | TC_CTRLBSET_CMD_STOP;
		while (timer->COUNT16.SYNCBUSY.bit.CTRLB);

		// reset to zero
		timer->COUNT16.COUNT.reg = 0;
	}

#if SUPERDFU_APP
	// DFU timer
	Tc* timer = TC2;

	MCLK->APBBMASK.bit.TC2_ = 1;
	GCLK->PCHCTRL[TC2_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN;

	NVIC_SetPriority(TC2_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_EnableIRQ(TC2_IRQn);

	timer->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while (timer->COUNT16.SYNCBUSY.bit.SWRST);
#endif // SUPERDFU_APP
}

static inline void usb_init(void)
{
	NVIC_SetPriority(USB_0_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(USB_1_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(USB_2_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(USB_3_IRQn, SLLIN_ISR_PRIORITY);


	/* USB clock init
	 * The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
	 * for low speed and full speed operation.
	 */
	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK0_Val | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_AHBMASK_USB_bit(MCLK);
	hri_mclk_set_APBBMASK_USB_bit(MCLK);

	gpio_set_pin_function(PIN_PA24, PINMUX_PA24H_USB_DM);
	gpio_set_pin_function(PIN_PA25, PINMUX_PA25H_USB_DP);
}

extern void sllin_board_init_begin(void)
{
#if !SUPERDFU_APP
	sam_crc32_unlock();
#endif

	clock_init();

#if !SUPERDFU_APP
	uart_init();
#endif
	LOG("CONF_CPU_FREQUENCY=%lu\n", (unsigned long)CONF_CPU_FREQUENCY);

	LOG("Vectors ROM @ %p\n", (void*)SCB->VTOR);
	move_vector_table_to_ram();
	LOG("Vectors RAM @ %p\n", (void*)SCB->VTOR);

	LOG("Enabling cache\n");
	same5x_enable_cache();

	same5x_init_device_identifier();

#if SUPERDFU_APP
	dfu_init_begin();
#endif

	leds_init();

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
	led_set(POWER_LED, 1);

#if SUPERDFU_APP
	dfu_init_end();
#endif
}


// DRE
SLLIN_RAMFUNC void SERCOM1_0_Handler(void)
{
	sam_lin_usart_int(0);
}


#if SUPERDFU_APP
extern void dfu_timer_start(uint16_t ms)
{
	Tc* timer = TC2;

	timer->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

	// timer overflow interrupt
	timer->COUNT16.INTENSET.reg = TC_INTENSET_OVF;

	// period
	timer->COUNT16.CC[0].reg = ms;

	// oneshot
	timer->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;

	// enable
	timer->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_PRESCALER_DIV1;
}
#endif

// // TXC
// SLLIN_RAMFUNC void SERCOM1_1_Handler(void)
// {
// 	sam_lin_usart_int(0);
// }

// RXC
SLLIN_RAMFUNC void SERCOM1_2_Handler(void)
{
	sam_lin_usart_int(0);
}

// everyhting else
SLLIN_RAMFUNC void SERCOM1_3_Handler(void)
{
	sam_lin_usart_int(0);
}

// DRE
SLLIN_RAMFUNC void SERCOM0_0_Handler(void)
{
	sam_lin_usart_int(1);
}

// // TXC
// SLLIN_RAMFUNC void SERCOM0_1_Handler(void)
// {
// 	sam_lin_usart_int(1);
// }

// RXC
SLLIN_RAMFUNC void SERCOM0_2_Handler(void)
{
	sam_lin_usart_int(1);
}

// everyhting else
SLLIN_RAMFUNC void SERCOM0_3_Handler(void)
{
	sam_lin_usart_int(1);
}

SLLIN_RAMFUNC void TC0_Handler(void)
{
	sam_lin_timer_int(0);
}

SLLIN_RAMFUNC void TC1_Handler(void)
{
	sam_lin_timer_int(1);
}

#if SUPERDFU_APP
void TC2_Handler(void)
{
	dfu_timer_expired();
}
#endif

#endif // #if D5035_50
