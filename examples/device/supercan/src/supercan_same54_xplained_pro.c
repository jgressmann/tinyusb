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

#ifdef SAME54XPLAINEDPRO

#include "supercan_same54_xplained_pro.h"
#include "supercan_debug.h"
#include "supercan_can.h"


#include <bsp/board.h>
#include <hal/include/hal_gpio.h>


extern void sc_board_led_init(void)
{

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

extern void sc_board_can_init_module(void)
{
	// cans[0].interrupt_id = CAN1_IRQn;
	// cans[0].m_can = CAN1;
	NVIC_SetPriority(CAN1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
}


// controller and hardware specific setup of i/o pins for CAN
extern void sc_board_can_init_pins(void)
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

extern void sc_board_can_init_clock(void) // controller and hardware specific setup of clock for the m_can module
{
	MCLK->AHBMASK.bit.CAN1_ = 1;
	GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN1 to use GLCK0
}

extern void sc_board_can_interrupt_enable(uint8_t index, bool on)
{
	SC_DEBUG_ASSERT(0 == index);
	(void)index;

	if (on) {
		NVIC_EnableIRQ(CAN1_IRQn);
	} else {
		NVIC_DisableIRQ(CAN1_IRQn);
	}
}

extern void* sc_board_can_m_can(uint8_t index)
{
	SC_DEBUG_ASSERT(0 == index);
	(void)index;

	return CAN1;
}

extern void sc_board_can_burst_led(uint8_t index, uint16_t duration_ms)
{
	SC_DEBUG_ASSERT(0 == index);
	(void)index;
	(void)duration_ms;
}

extern void sc_board_can_led_set_status(uint8_t index, int status)
{
	(void)index;
	(void)status;
}


extern void CAN1_Handler(void)
{
	// LOG("CAN1 int\n");

	sc_can_int(0); // map to index 0
}

extern void sc_board_power_led_on(void)
{

}

extern void sc_board_usb_led_burst(uint16_t duration_ms)
{
	(void)duration_ms;
}


#endif // #ifdef SAME54XPLAINEDPRO
