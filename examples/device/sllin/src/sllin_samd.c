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

#ifdef __SAMD__

#include <FreeRTOS.h>

#include <sllin_debug.h>
#include <sllin_board.h>


#include "sam.h"
#include "bsp/board.h"
#include "board.h"

#include "hal/include/hal_gpio.h"
#include "hal/include/hal_init.h"
#include "hri/hri_nvmctrl_d21.h"

#include "hpl/gclk/hpl_gclk_base.h"
#include "hpl_pm_config.h"
#include "hpl/pm/hpl_pm_base.h"

#include <leds.h>
#include <tusb.h>
#include <bsp/board.h>
#include <crc32.h>


#ifndef ARRAY_SIZE
#	define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif

#define LOGSTR(x) board_uart_write(x, sizeof(x)-1)


struct led {
	uint8_t pin;
};

#define LED_STATIC_INITIALIZER(name, pin) \
	{ pin }


static const struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PA10), // board led
	// LED_STATIC_INITIALIZER("red", PIN_PA18),
	// LED_STATIC_INITIALIZER("green", PIN_PA19),
	// LED_STATIC_INITIALIZER("blue", PIN_PB16),
};

static inline void leds_init(void)
{
	// PORT->Group[0].DIRSET.reg = PORT_PA18 | PORT_PA19;
	// PORT->Group[1].DIRSET.reg = PORT_PB16 | PORT_PB17 | PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03;
}

extern void sllin_board_led_set(uint8_t index, bool on)
{
	SLLIN_DEBUG_ASSERT(index < ARRAY_SIZE(leds));

	gpio_set_pin_level(leds[index].pin, on);
}

extern void sllin_board_leds_on_unsafe(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(leds); ++i) {
		gpio_set_pin_level(leds[i].pin, 1);
	}
}

__attribute__((noreturn)) extern void sllin_board_reset(void)
{
	NVIC_SystemReset();
	__unreachable();
}


static uint32_t device_identifier;

extern uint32_t sllin_board_identifier(void)
{
	return device_identifier;
}

void init_device_identifier(void)
{
	// Atmel-42181N-SAM-D21_Datasheet_Complete-10/2016, p.44
	uint32_t serial_number[4] = {
		*((uint32_t const *)0x0080A00C),
		*((uint32_t const *)0x0080A040),
		*((uint32_t const *)0x0080A044),
		*((uint32_t const *)0x0080A048),
	};

	int error = CRC32E_NONE;

#if SLLIN_DEBUG
	char serial_buffer[64];
	memset(serial_buffer, '0', 32);
	char hex_buffer[16];
	int chars = usnprintf(hex_buffer, sizeof(hex_buffer), "%x", serial_number[0]);
	memcpy(&serial_buffer[8-chars], hex_buffer, chars);
	chars = usnprintf(hex_buffer, sizeof(hex_buffer), "%x", serial_number[1]);
	memcpy(&serial_buffer[16-chars], hex_buffer, chars);
	chars = usnprintf(hex_buffer, sizeof(hex_buffer), "%x", serial_number[2]);
	memcpy(&serial_buffer[24-chars], hex_buffer, chars);
	chars = usnprintf(hex_buffer, sizeof(hex_buffer), "%x", serial_number[3]);
	memcpy(&serial_buffer[32-chars], hex_buffer, chars);
	serial_buffer[32] = 0;
	LOG("SAM serial number %s\n", serial_buffer);
#endif

#if TU_LITTLE_ENDIAN == TU_BYTE_ORDER
	// swap integers so they have printf layout
	serial_number[0] = __builtin_bswap32(serial_number[0]);
	serial_number[1] = __builtin_bswap32(serial_number[1]);
	serial_number[2] = __builtin_bswap32(serial_number[2]);
	serial_number[3] = __builtin_bswap32(serial_number[3]);
#endif


	error = crc32f((uint32_t)serial_number, 16, CRC32E_FLAG_UNLOCK, &device_identifier);
	if (unlikely(error)) {
		device_identifier = serial_number[0];
		LOG("ERROR: failed to compute CRC32: %d. Using fallback device identifier\n", error);
	}


#if SLLIN_DEBUG
	memset(serial_buffer, '0', 8);
	chars = usnprintf(hex_buffer, sizeof(hex_buffer), "%x", device_identifier);
	memcpy(&serial_buffer[8-chars], hex_buffer, chars);
	serial_buffer[8] = 0;
	LOG("device identifier %s\n", serial_buffer);
#endif
}

void uart_init(void);

/* Referenced GCLKs, should be initialized firstly */
#define _GCLK_INIT_1ST (1 << 0 | 1 << 1)

/* Not referenced GCLKs, initialized last */
#define _GCLK_INIT_LAST (~_GCLK_INIT_1ST)

static inline void board_init_samd21(void)
{
	// Clock init ( follow hpl_init.c )
	hri_nvmctrl_set_CTRLB_RWS_bf(NVMCTRL, 2);

	_pm_init();
	_sysctrl_init_sources();
#if _GCLK_INIT_1ST
	_gclk_init_generators_by_fref(_GCLK_INIT_1ST);
#endif
	_sysctrl_init_referenced_generators();
	_gclk_init_generators_by_fref(_GCLK_INIT_LAST);

	SystemCoreClock = CONF_CPU_FREQUENCY;

	// Led init
#ifdef LED_PIN
	gpio_set_pin_direction(LED_PIN, GPIO_DIRECTION_OUT);
	board_led_write(false);
#endif

	// Button init
#ifdef BUTTON_PIN
	gpio_set_pin_direction(BUTTON_PIN, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(BUTTON_PIN, BUTTON_STATE_ACTIVE ? GPIO_PULL_DOWN : GPIO_PULL_UP);
#endif

	uart_init();
	board_uart_write("USART initialized\n", sizeof("USART initialized\n")-1);

	// If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
	NVIC_SetPriority(USB_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);


	/* USB Clock init
	 * The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
	 * for low speed and full speed operation. */
	_pm_enable_bus_clock(PM_BUS_APBB, USB);
	_pm_enable_bus_clock(PM_BUS_AHB, USB);
	_gclk_enable_channel(USB_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK0_Val);

	// USB Pin Init
	gpio_set_pin_direction(PIN_PA24, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PA24, false);
	gpio_set_pin_pull_mode(PIN_PA24, GPIO_PULL_OFF);
	gpio_set_pin_direction(PIN_PA25, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PA25, false);
	gpio_set_pin_pull_mode(PIN_PA25, GPIO_PULL_OFF);

	gpio_set_pin_function(PIN_PA24, PINMUX_PA24G_USB_DM);
	gpio_set_pin_function(PIN_PA25, PINMUX_PA25G_USB_DP);
}

extern void sllin_board_init_begin(void)
{
	board_init_samd21();

	init_device_identifier();

	leds_init();
}

extern void sllin_board_init_end(void)
{
	LOG("SysTick enabled\n");
	SysTick_Config(SystemCoreClock / configTICK_RATE_HZ);
}

#endif // defined(__SAMD__)