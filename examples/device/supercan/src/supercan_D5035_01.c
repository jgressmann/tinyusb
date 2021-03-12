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



#ifdef D5035_01

#include "supercan_D5035_01.h"
#include "supercan_debug.h"
#include "supercan_can.h"
#include "supercan_board.h"
#include "leds.h"

#include <hal/include/hal_gpio.h>

#ifndef ARRAY_SIZE
#	define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif


struct led {
	uint8_t pin;
};

struct can {
	Can *m_can;
	IRQn_Type interrupt_id;
	uint8_t led_status_green;
	uint8_t led_status_red;
	uint8_t led_traffic;
};

#define LED_STATIC_INITIALIZER(name, pin) \
	{ pin }


#if HWREV == 1
static struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PA02), // board led
	LED_STATIC_INITIALIZER("red1", PIN_PB14),
	LED_STATIC_INITIALIZER("orange1", PIN_PB15),
	LED_STATIC_INITIALIZER("green1", PIN_PA12),
	LED_STATIC_INITIALIZER("red2", PIN_PA13),
	LED_STATIC_INITIALIZER("orange2", PIN_PA14),
	LED_STATIC_INITIALIZER("green2", PIN_PA15),
};



extern void sc_board_led_init(void)
{
	PORT->Group[1].DIRSET.reg = PORT_PB14; /* Debug-LED */
	PORT->Group[1].DIRSET.reg = PORT_PB15; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA12; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA13; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA14; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA15; /* Debug-LED */
}

#define POWER_LED LED_RED1
#define CAN0_TRAFFIC_LED LED_GREEN1
#define CAN1_TRAFFIC_LED LED_GREEN2
#define USB_LED LED_ORANGE1

#else // HWREV > 1
static struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PA02), // board led
	LED_STATIC_INITIALIZER("red", PIN_PA18),
	LED_STATIC_INITIALIZER("orange", PIN_PA19),
	LED_STATIC_INITIALIZER("green", PIN_PB16),
	LED_STATIC_INITIALIZER("blue", PIN_PB17),
#if HWREV >= 3
	LED_STATIC_INITIALIZER("can1_red", PIN_PB00),
	LED_STATIC_INITIALIZER("can1_green", PIN_PB01),
	LED_STATIC_INITIALIZER("can0_red", PIN_PB02),
	LED_STATIC_INITIALIZER("can0_green", PIN_PB03),
#endif
};




extern void sc_board_led_init(void)
{
	PORT->Group[0].DIRSET.reg = PORT_PA18 | PORT_PA19;
	PORT->Group[1].DIRSET.reg =
		PORT_PB16 | PORT_PB17
#if HWREV >= 3
		| PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03
#endif
		;
}


#define POWER_LED LED_DEBUG_0
#define CAN0_TRAFFIC_LED LED_DEBUG_1
#define CAN1_TRAFFIC_LED LED_DEBUG_2
#define USB_LED LED_DEBUG_3

#endif // HWREV > 1


static struct can cans[SC_BOARD_CAN_COUNT];

extern void sc_board_led_set(uint8_t index, bool on)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(leds));

	gpio_set_pin_level(leds[index].pin, on);
}


extern void sc_board_leds_on_unsafe(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(leds); ++i) {
		gpio_set_pin_level(leds[i].pin, 1);
	}
}



extern void sc_board_can_init_module(void)
{
	cans[0].interrupt_id = CAN0_IRQn;
	cans[0].m_can = CAN0;
	NVIC_SetPriority(CAN0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	cans[0].led_traffic = CAN0_TRAFFIC_LED;
#if HWREV == 1

#else // HWREV != 1
	cans[1].interrupt_id = CAN1_IRQn;
	cans[1].m_can = CAN1;
	NVIC_SetPriority(CAN1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	cans[1].led_traffic = CAN1_TRAFFIC_LED;
#if HWREV < 3

#else
	cans[0].led_status_green = LED_CAN0_STATUS_GREEN;
	cans[0].led_status_red = LED_CAN0_STATUS_RED;
	cans[1].led_status_green = LED_CAN1_STATUS_GREEN;
	cans[1].led_status_red = LED_CAN1_STATUS_RED;
#endif
#endif
}

// controller and hardware specific setup of i/o pins for CAN
extern void sc_board_can_init_pins(void)
{
	// CAN0 port
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_HWSEL |           // upper half
		PORT_WRCONFIG_PINMASK(0x00c0) | // PA22/23
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(8) |         // I, CAN0, DS60001507E page 32, 910
		PORT_WRCONFIG_PMUXEN;
#if SC_BOARD_CAN_COUNT > 1
	// CAN1 port
	PORT->Group[1].WRCONFIG.reg =
		PORT_WRCONFIG_PINMASK(0xc000) | // PB14/15 = 0xc000, PB12/13 = 0x3000
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(7) |         // H, CAN1, DS60001507E page 32, 910
		PORT_WRCONFIG_PMUXEN;
#endif
}

extern void sc_board_can_init_clock(void) // controller and hardware specific setup of clock for the m_can module
{
	MCLK->AHBMASK.bit.CAN0_ = 1;
	GCLK->PCHCTRL[CAN0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN1 to use GLCK0
#if SC_BOARD_CAN_COUNT > 1
	MCLK->AHBMASK.bit.CAN1_ = 1;
	GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN1 to use GLCK0
#endif
}

extern void sc_board_can_interrupt_enable(uint8_t index, bool on)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(cans));

	if (on) {
		NVIC_EnableIRQ(cans[index].interrupt_id);
	} else {
		NVIC_DisableIRQ(cans[index].interrupt_id);
	}
}

extern void* sc_board_can_m_can(uint8_t index)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(cans));
	return cans[index].m_can;
}

extern void sc_board_can_burst_led(uint8_t index, uint16_t duration_ms)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(cans));

	led_burst(cans[index].led_traffic, duration_ms);
}


extern void sc_board_can_led_set_status(uint8_t index, int status)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(cans));
#if HWREV >= 3
	const uint16_t BLINK_DELAY_PASSIVE_MS = 512;
	const uint16_t BLINK_DELAY_ACTIVE_MS = 128;
	struct can* can = &cans[index];
	switch (status) {
	case CANLED_STATUS_DISABLED:
		led_set(can->led_status_green, 0);
		led_set(can->led_status_red, 0);
		break;
	case CANLED_STATUS_ENABLED_BUS_OFF:
		led_set(can->led_status_green, 1);
		led_set(can->led_status_red, 0);
		break;
	case CANLED_STATUS_ENABLED_BUS_ON_PASSIVE:
		led_blink(can->led_status_green, BLINK_DELAY_PASSIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case CANLED_STATUS_ENABLED_BUS_ON_ACTIVE:
		led_blink(can->led_status_green, BLINK_DELAY_ACTIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case CANLED_STATUS_ERROR_PASSIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, BLINK_DELAY_ACTIVE_MS);
		break;
	case CANLED_STATUS_ERROR_ACTIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, BLINK_DELAY_ACTIVE_MS);
		break;
	}
#else
	(void)index;
	(void)status;
#endif
}



void CAN0_Handler(void)
{
	// LOG("CAN0 int\n");

	sc_can_int(0);
}

#if SC_BOARD_CAN_COUNT > 1
void CAN1_Handler(void)
{
	// LOG("CAN1 int\n");

	sc_can_int(1);
}
#endif

extern void sc_board_power_led_on(void)
{
	led_set(POWER_LED, 1);
}

extern void sc_board_usb_led_burst(uint16_t duration_ms)
{
	led_burst(USB_LED, duration_ms);
}



extern void sc_board_counter_1MHz_init(void)
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

#endif // #ifdef D5035_01
