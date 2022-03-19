/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */



#ifdef FEATHER_M4_CAN_EXPRESS

#include <FreeRTOS.h>
#include <timers.h>

#include <supercan_debug.h>
#include <supercan_board.h>

#include <hal/include/hal_gpio.h>
#include <leds.h>

#include <tusb.h>

#ifndef ARRAY_SIZE
#	define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif

#define CONF_CPU_FREQUENCY 48000000L
#define CONF_USB_FREQUENCY 48000000L
#define USART_BAUDRATE     115200L


enum {
	PIXEL_GROUP = 1,
	PIXEL_DATA_PIN = 2,
	PIXEL_POWER_PIN = 3,

	PIXEL_GREEN_ON = 0x8,
	PIXEL_BLUE_ON = 0x8,
	PIXEL_RED_ON = 0x8,
	PIXEL_GREEN_SHIFT = 24,
	PIXEL_RED_SHIFT = 16,
	PIXEL_BLUE_SHIFT = 8,
};



struct pixel {
	uint32_t target; // from the top, each 8 bit grb
} pixel;


// @48 MHz
#define delay_100_ns() \
	do { \
		__asm__ __volatile__( \
			"nop\t\n" \
			"nop\t\n" \
			"nop\t\n" \
			"nop\t\n"); \
	} while (0)



static inline void one(void)
{
	PORT->Group[PIXEL_GROUP].OUTSET.reg = UINT32_C(1) << PIXEL_DATA_PIN;
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	// delay_100_ns();
	// delay_100_ns();
	PORT->Group[PIXEL_GROUP].OUTCLR.reg = UINT32_C(1) << PIXEL_DATA_PIN;
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	// delay_100_ns();
	// delay_100_ns();
}

static inline void zero(void)
{
	PORT->Group[PIXEL_GROUP].OUTSET.reg = UINT32_C(1) << PIXEL_DATA_PIN;
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	// delay_100_ns();
	PORT->Group[PIXEL_GROUP].OUTCLR.reg = UINT32_C(1) << PIXEL_DATA_PIN;
	// delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	delay_100_ns();
	// delay_100_ns();
	// delay_100_ns();
}


SC_RAMFUNC void pixel_update(void)
{
	uint32_t target = __atomic_load_n(&pixel.target, __ATOMIC_ACQUIRE);

	for (int i = 0; i < 24; ++i) {
		if ((target & 0x80000000) == 0x80000000) {
			one();
		} else {
			zero();
		}

		target <<= 1;
	}
}


// controller and hardware specific setup of i/o pins for CAN
static inline void can_init_pins(void)
{
	// CAN1 port
	PORT->Group[1].WRCONFIG.reg =
		PORT_WRCONFIG_PINMASK(0xc000) | // PB14/15 = 0xc000, PB12/13 = 0x3000
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(7) |         // H, CAN1, DS60001507E page 32, 910
		PORT_WRCONFIG_PMUXEN;

	/* The transceiver TCAN1051HGV has a pin (S) for listen only mode.
	 * Listen-only mode is active when the pin is low (GND).
	 * This pin is connected to the MCU on PB12.
	 * We don't use this mode, but configure listen only in M_CAN.
	 */
	PORT->Group[1].DIRSET.reg = UINT32_C(1) << 12;
	PORT->Group[1].OUTSET.reg = UINT32_C(1) << 12;
}

static inline void can_init_clock(void) // controller and hardware specific setup of clock for the m_can module
{
	MCLK->AHBMASK.bit.CAN1_ = 1;
	GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN1 to use GLCK0
}


static void can_init_module(void)
{
	memset(&same5x_cans, 0, sizeof(same5x_cans));

	same5x_cans[0].m_can = CAN1;
	same5x_cans[0].interrupt_id = CAN1_IRQn;
	same5x_cans[0].led_traffic = SC_BOARD_DEBUG_DEFAULT;
	same5x_cans[0].led_status_green = SC_BOARD_PIXEL_GREEN;
	same5x_cans[0].led_status_red = SC_BOARD_PIXEL_RED;


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

	LOG("M_CAN release %u.%u.%u (%lx)\n", CAN0->CREL.bit.REL, CAN0->CREL.bit.STEP, CAN0->CREL.bit.SUBSTEP, CAN0->CREL.reg);
}

static inline void counter_1MHz_init(void)
{
	// TC0 and TC1 pair to form a single 32 bit counter
	// TC1 is enslaved to TC0 and doesn't need to be configured.
	// DS60001507E-page 1716, 48.6.2.4 Counter Mode

	MCLK->APBAMASK.bit.TC0_ = 1;
	MCLK->APBAMASK.bit.TC1_ = 1;
	GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN; /* setup TC0 to use GLCK2 */
	GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN; /* setup TC1 to use GLCK2 */

	TC0->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
	while(1 == TC0->COUNT32.SYNCBUSY.bit.SWRST);

	// 16MHz -> 1MHz
	TC0->COUNT32.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV16;
	while(1 == TC0->COUNT32.SYNCBUSY.bit.ENABLE);
}

static inline void leds_init(void)
{
	PORT->Group[0].DIRSET.reg = UINT32_C(1) << 23; // PA23, debug led
	// set pixel power, data pins as output
	PORT->Group[PIXEL_GROUP].DIRSET.reg = (UINT32_C(1) << PIXEL_DATA_PIN) | (UINT32_C(1) << PIXEL_POWER_PIN);
	// assert pixel power pin
	PORT->Group[PIXEL_GROUP].OUTSET.reg = (UINT32_C(1) << PIXEL_POWER_PIN);
	// clear pixel data pin
	PORT->Group[PIXEL_GROUP].OUTCLR.reg = (UINT32_C(1) << PIXEL_DATA_PIN);

	/* Hijack FREQM interrupt to set pixel led state from interrupt handler
	 *
	 * Looks like we don't need to turn the peripheral on to generated the interrupt.
	 */
	NVIC_SetPriority(FREQM_IRQn, SC_ISR_PRIORITY);
	NVIC_EnableIRQ(FREQM_IRQn);
}


extern void sc_board_led_set(uint8_t index, bool on)
{
	uint32_t target = pixel.target;

	switch (index) {
	case SC_BOARD_DEBUG_DEFAULT:
		gpio_set_pin_level(PIN_PA23, on);
		break;
	case SC_BOARD_PIXEL_RED: {
		target &= ~(0xff << PIXEL_RED_SHIFT);
		target |= (PIXEL_RED_ON * on) << PIXEL_RED_SHIFT;
	} break;
	case SC_BOARD_PIXEL_GREEN: {
		target &= ~(0xff << PIXEL_GREEN_SHIFT);
		target |= (PIXEL_GREEN_ON * on) << PIXEL_GREEN_SHIFT;
	} break;
	case SC_BOARD_PIXEL_BLUE: {
		target &= ~(0xff << PIXEL_BLUE_SHIFT);
		target |= (PIXEL_BLUE_ON * on) << PIXEL_BLUE_SHIFT;
	} break;
	}

	if (target != pixel.target) {
		LOG("CAN0 LED change\n");
		__atomic_store_n(&pixel.target, target, __ATOMIC_RELEASE);
		NVIC->STIR = FREQM_IRQn;
	}
}

extern void sc_board_leds_on_unsafe(void)
{
	gpio_set_pin_level(PIN_PA23, 1);
	pixel.target = 0xffffff00;
	pixel_update();
}


static inline void init_clock(void)
{
	/* AUTOWS is enabled by default in REG_NVMCTRL_CTRLA - no need to change the number of wait states when changing the core clock */

	/* We assume we are running the chip in default settings.
	 * This means we are running off of the 48 MHz FLL.
	 */

	// configure GCLK2 for 16MHz from DFLL
	GCLK->GENCTRL[2].reg =
		GCLK_GENCTRL_DIV(3) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DFLL |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL2); /* wait for the synchronization between clock domains to be complete */


	SystemCoreClock = CONF_CPU_FREQUENCY;
}


static inline void uart_init(void)
{
	/* Depending on whether we are running as app or standandlone,
	 * the USART may not be initialized.
	 */

	Sercom *s = SERCOM5;

	/* configure SERCOM5 PB16/17 (TX/RX) */
	PORT->Group[1].WRCONFIG.reg =
		PORT_WRCONFIG_HWSEL | // top 16 bit
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(2) |    /* function C */
		PORT_WRCONFIG_PINMASK(0x0003) |
		PORT_WRCONFIG_PMUXEN;

	MCLK->APBDMASK.bit.SERCOM5_ = 1;
	GCLK->PCHCTRL[SERCOM5_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

	s->USART.CTRLA.reg = 0x00; /* disable SERCOM -> enable config */
	while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);

	s->USART.CTRLA.reg  =
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_DORD |     /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) |  /* 0x0 USART with external clock, 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(1) |  /* SERCOM PAD[1] is used for data reception */
		SERCOM_USART_CTRLA_TXPO(0);   /* SERCOM PAD[0] is used for data transmission */

	s->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN; /* transmitter enabled */
	uint16_t baud = CONF_USB_FREQUENCY / (16 * USART_BAUDRATE);
	uint16_t frac = CONF_USB_FREQUENCY / (2 * USART_BAUDRATE) - 8 * baud;
	s->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(frac) | SERCOM_USART_BAUD_FRAC_BAUD(baud);

	s->USART.CTRLA.bit.ENABLE = 1; /* activate SERCOM */
	while(s->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */
}

static inline void init_usb(void)
{
	NVIC_SetPriority(USB_0_IRQn, SC_ISR_PRIORITY);
	NVIC_SetPriority(USB_1_IRQn, SC_ISR_PRIORITY);
	NVIC_SetPriority(USB_2_IRQn, SC_ISR_PRIORITY);
	NVIC_SetPriority(USB_3_IRQn, SC_ISR_PRIORITY);

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
}

extern void sc_board_init_begin(void)
{
	init_clock();
	uart_init();

	LOG("Enabling cache\n");
	same5x_enable_cache();

	same5x_init_device_identifier();

	leds_init();


	can_init_pins();
	can_init_clock();
	can_init_module();

	counter_1MHz_init();
	init_usb();

	// while (1) {
	// 	uint32_t c = counter_1MHz_read_sync();
	// 	counter_1MHz_request_current_value();
	// 	uint32_t x = 0;
	// 	while (!counter_1MHz_is_current_value_ready()) {
	// 		++x;
	// 	}

	// 	LOG("c=%lx, wait=%lx\n", c, x);
	// }
}

extern void sc_board_init_end(void)
{

}


SC_RAMFUNC extern void sc_board_led_can_status_set(uint8_t index, int status)
{
	struct same5x_can *can = &same5x_cans[index];

	switch (status) {
	case SC_CAN_LED_STATUS_DISABLED:
		led_set(can->led_status_green, 0);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_OFF_BUS:
		led_set(can->led_status_green, 1);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_PASSIVE:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_PASSIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ACTIVE:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_PASSIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_PASSIVE_MS);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_ACTIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_BUS_OFF:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, 1);
		break;
	default:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
		break;
	}
}


SC_RAMFUNC void CAN1_Handler(void)
{
	// LOG("CAN1 int\n");

	same5x_can_int(0);
}

SC_RAMFUNC void FREQM_Handler(void)
{
	// LOG("FREQM_Handler\n");

	pixel_update();
}

#endif // #ifdef FEATHER_M4_CAN_EXPRESS
