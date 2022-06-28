/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */


#include <sllin_board.h>

#if D5035_51

#include <leds.h>

#include <hal/include/hal_gpio.h>
#include <mcu.h>
#include <sam_crc32.h>
#include <bsp/board.h>


#include <tusb.h>
// #include <class/dfu/dfu_rt_device.h>

#define BOARD_SERCOM SERCOM2
#define USART_BAURATE      115200
#define CONF_CPU_FREQUENCY 48000000

// in d5035_51.c
extern void board_fixups(void);
extern void init_clock_xtal(void);
extern void uart_init(void);

/* The board runs either off of the 48 MHz internal RC oscillator or the 16MHz crystal
 *
 * @115200 debug prints KILL LIN timing assumptions!
*/

struct sam_lin sam_lins[SLLIN_BOARD_LIN_COUNT] = {
	{
		.sercom = SERCOM1,
		.timer = TC4, // NOTE: TC4/TC5 are paired to same GCLK
		.timer_irq = TC4_IRQn,
		.rx_port_pin_mux = 1,	// PA01
		.master_slave_port_pin_mux = (1 << 5) | 8, // PB08
		.led_status_green = 5,
		.led_status_red = 6,
	},
#if SLLIN_BOARD_LIN_COUNT > 1
	{
		.sercom = SERCOM0,
		.timer = TC5,
		.timer_irq = TC5_IRQn,
		.rx_port_pin_mux = 9, // PA09
		.master_slave_port_pin_mux = 7, // PA07
		.led_status_green = 7,
		.led_status_red = 8,
	},
#endif
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
	LED_STATIC_INITIALIZER("debug_green", PIN_PA20),
	LED_STATIC_INITIALIZER("debug_blue", PIN_PA21),
	LED_STATIC_INITIALIZER("lin0_green", PIN_PA27),
	LED_STATIC_INITIALIZER("lin0_red", PIN_PA28),
	LED_STATIC_INITIALIZER("lin1_green", PIN_PB02),
	LED_STATIC_INITIALIZER("lin1_red", PIN_PB03),
};

static inline void leds_init(void)
{
	PORT->Group[0].DIRSET.reg = PORT_PA02 | PORT_PA18 | PORT_PA19 | PORT_PA20 | PORT_PA21 | PORT_PA27 | PORT_PA28;
	PORT->Group[1].DIRSET.reg = PORT_PB02 | PORT_PB03;
}

#define POWER_LED LED_DEBUG_0
#define USB_LED LED_DEBUG_3

// extern uint32_t _svectors;
// extern uint32_t _evectors;

// static void move_vector_table_to_ram(void)
// {
// 	uint8_t* svectors = (void*)&_svectors;
// 	uint8_t* evectors = (void*)&_evectors;

// 	memcpy(svectors, (void*)SCB->VTOR, evectors - svectors);

// 	SCB->VTOR = (uint32_t)svectors;
// }

static uint32_t device_identifier;

extern uint32_t sllin_board_identifier(void)
{
	return device_identifier;
}


static inline void samdxx_init_device_identifier(void)
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
	// Setup 48 MHz from crystal
	init_clock_xtal();

	// Setup 16 MHz on GCLK2
	GCLK->GENDIV.reg =
		GCLK_GENDIV_DIV(3) |
		GCLK_GENDIV_ID(2);

	GCLK->GENCTRL.reg =
		GCLK_GENCTRL_SRC_DPLL96M |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_IDC |
		GCLK_GENCTRL_ID(2);
	while (GCLK->STATUS.bit.SYNCBUSY);

#if SUPERDFU_APP
	// Setup 1 KHz on GCLK1
	GCLK->GENDIV.reg =
		GCLK_GENDIV_DIV(16000) |
		GCLK_GENDIV_ID(1);

	GCLK->GENCTRL.reg =
		GCLK_GENCTRL_SRC_XOSC |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_IDC |
		GCLK_GENCTRL_ID(1);
	while (GCLK->STATUS.bit.SYNCBUSY);
#endif
}

static inline void lin_init_once(void)
{
	// lin0
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_INEN |
		PORT_WRCONFIG_PINMASK(0x0003) | /* PA00, PA01 */
		PORT_WRCONFIG_PMUXEN;

	PM->APBCMASK.bit.SERCOM1_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK0 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_SERCOM1_CORE;

	NVIC_SetPriority(SERCOM1_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_EnableIRQ(SERCOM1_IRQn);

	// lin1
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_PINMASK(0x0300) | /* PA08, PA9 */
		PORT_WRCONFIG_PMUXEN;

	PM->APBCMASK.bit.SERCOM0_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK0 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_SERCOM0_CORE;

	NVIC_SetPriority(SERCOM0_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_EnableIRQ(SERCOM0_IRQn);

	sam_lin_init_once();
}

static void timer_init(void)
{
	// LIN timers
	PM->APBCMASK.bit.TC4_ = 1;
	PM->APBCMASK.bit.TC5_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK2 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_TC4_TC5;

	NVIC_SetPriority(TC4_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(TC5_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_EnableIRQ(TC4_IRQn);
	NVIC_EnableIRQ(TC5_IRQn);

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

#if SUPERDFU_APP
	// DFU timer
	Tc* timer = TC3;

	PM->APBCMASK.bit.TCC2_ = 1;
	PM->APBCMASK.bit.TC3_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK2 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_TCC2_TC3;

	NVIC_SetPriority(TC3_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_EnableIRQ(TC3_IRQn);

	timer->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while (timer->COUNT16.STATUS.bit.SYNCBUSY);
#endif // SUPERDFU_APP
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
#if !SUPERDFU_APP
	board_fixups();
	sam_crc32_unlock();
#endif

	clock_init();

#if !SUPERDFU_APP
	uart_init();
#endif

	// LOG("Vectors ROM @ %p\n", (void*)SCB->VTOR);
	// move_vector_table_to_ram();
	// LOG("Vectors RAM @ %p\n", (void*)SCB->VTOR);

	samdxx_init_device_identifier();

#if SUPERDFU_APP
	dfu_init_begin();
#endif

	leds_init();

#if !SUPERDFU_APP
	LOG("USB init\n");
	usb_init();
#endif
	LOG("timer init\n");
	timer_init();

	LOG("LIN init once\n");
	lin_init_once();
}

extern void sllin_board_init_end(void)
{
	led_blink(0, 2000);
	// led_set(POWER_LED, 1);

#if SUPERDFU_APP
	dfu_init_end();
#endif
}

#if SUPERDFU_APP
extern void dfu_timer_start(uint16_t ms)
{
	Tc* timer = TC3;

	// timer overflow interrupt
	timer->COUNT16.INTENSET.reg = TC_INTENSET_OVF;

	// period
	timer->COUNT16.CC[0].reg = ms;

	// oneshot
	timer->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;

	// enable
	timer->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_WAVEGEN_MFRQ;
}
#endif

SLLIN_RAMFUNC void SERCOM1_Handler(void)
{
	sam_lin_usart_int(0);
}

SLLIN_RAMFUNC void SERCOM0_Handler(void)
{
	sam_lin_usart_int(1);
}

#if SUPERDFU_APP
void TC3_Handler(void)
{
	dfu_timer_expired();
}
#endif

SLLIN_RAMFUNC void TC4_Handler(void)
{
	sam_lin_timer_int(0);
}

SLLIN_RAMFUNC void TC5_Handler(void)
{
	sam_lin_timer_int(1);
}




#endif // #if D5035_51
