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

#include "leds.h"
#include "supercan_debug.h"


#include <sam.h>

#include <hal/include/hal_gpio.h>

#ifndef ARRAY_SIZE
#	define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif

#ifndef likely
#	define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#	define unlikely(x) __builtin_expect(!!(x),0)
#endif

StackType_t led_task_stack[LED_STACK_SIZE];
StaticTask_t led_task_mem;

enum {
	LED_CMD_NO_CHANGE = 0,
	LED_CMD_OFF,
	LED_CMD_ON,
	LED_CMD_BLINK,
	LED_CMD_TOGGLE,
	LED_CMD_BURST,
};

typedef union {
	struct {
		uint16_t cmd : 3;
		uint16_t millis : 13;
	} bit;
	uint16_t reg;
} led_cmd_t;

struct led {
	uint16_t cmd;
	uint8_t pin;
};

#define LED_STATIC_INITIALIZER(name, pin) \
	{ 0, pin }


static struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PA02), // board led
	LED_STATIC_INITIALIZER("red", PIN_PA18),
	LED_STATIC_INITIALIZER("orange", PIN_PA19),
	LED_STATIC_INITIALIZER("green", PIN_PB16),
	LED_STATIC_INITIALIZER("blue", PIN_PB17),
	LED_STATIC_INITIALIZER("can1_red", PIN_PB00),
	LED_STATIC_INITIALIZER("can1_green", PIN_PB01),
	LED_STATIC_INITIALIZER("can0_red", PIN_PB02),
	LED_STATIC_INITIALIZER("can0_green", PIN_PB03),
};


SC_RAMFUNC extern void led_init(void)
{
	PORT->Group[0].DIRSET.reg = PORT_PA18 | PORT_PA19;
	PORT->Group[1].DIRSET.reg =
		PORT_PB16 | PORT_PB17 | PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03;
}


SC_RAMFUNC extern void led_set(uint8_t index, bool on)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(leds));

	led_cmd_t s;
	s.bit.cmd = on ? LED_CMD_ON : LED_CMD_OFF;
	__atomic_store_n(&leds[index].cmd, s.reg, __ATOMIC_RELEASE);
}


SC_RAMFUNC extern void led_toggle(uint8_t index)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(leds));

	led_cmd_t s;
	s.bit.cmd = LED_CMD_TOGGLE;
	__atomic_store_n(&leds[index].cmd, s.reg, __ATOMIC_RELEASE);
}

SC_RAMFUNC extern void led_blink(uint8_t index, uint16_t delay_ms)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(leds));

	led_cmd_t s;
	s.bit.cmd = LED_CMD_BLINK;
	s.bit.millis = delay_ms;
	__atomic_store_n(&leds[index].cmd, s.reg, __ATOMIC_RELEASE);

}

SC_RAMFUNC extern void led_burst(uint8_t index, uint16_t duration_ms)
{
	SC_DEBUG_ASSERT(index < ARRAY_SIZE(leds));

	led_cmd_t s;
	s.bit.cmd = LED_CMD_BURST;
	s.bit.millis = duration_ms;
	__atomic_store_n(&leds[index].cmd, s.reg, __ATOMIC_RELEASE);
}

SC_RAMFUNC extern void leds_on_unsafe(void)
{
	for (unsigned i = 0; i < LED_COUNT; ++i) {
		gpio_set_pin_level(leds[i].pin, 1);
	}
}

SC_RAMFUNC extern void led_task(void *param)
{
	(void) param;

	const uint8_t TICK_MS = 8;
	uint8_t cmd[ARRAY_SIZE(leds)];
	uint8_t state[ARRAY_SIZE(leds)];
	uint16_t interval[ARRAY_SIZE(leds)];
	uint16_t left[ARRAY_SIZE(leds)];

	memset(&cmd, 0, sizeof(cmd));
	memset(&state, 0, sizeof(state));
	memset(&interval, 0, sizeof(interval));
	memset(&left, 0, sizeof(left));


	while (42) {
		for (size_t i = 0; i < ARRAY_SIZE(leds); ++i) {
			led_cmd_t c = (led_cmd_t)__atomic_load_n(&leds[i].cmd, __ATOMIC_ACQUIRE);

			if (unlikely(LED_CMD_NO_CHANGE != c.bit.cmd)) {
				led_cmd_t x = c; // use a copy so that later can handle the command
				do {
					__atomic_compare_exchange_n(
						&leds[i].cmd,
						&x,
						0, /* new value */
						true, /* strong */
						__ATOMIC_ACQ_REL,
						__ATOMIC_ACQUIRE);
				} while (LED_CMD_NO_CHANGE != x.bit.cmd);
			}

			if (unlikely(LED_CMD_NO_CHANGE != c.bit.cmd)) {
				if (c.bit.cmd == cmd[i]) {
					// more of the same
					switch (c.bit.cmd) {
					case LED_CMD_BLINK:
						interval[i] = c.bit.millis;
						if (c.bit.millis < left[i]) {
							left[i] = c.bit.millis;
						}
						break;
					case LED_CMD_BURST:
						// LOG("%u burst %u millis\n", i, c.bit.millis);
						left[i] = c.bit.millis;
						break;
					}
				} else {
					switch (c.bit.cmd) {
					case LED_CMD_OFF:
						// LOG("%u off\n", i);
						state[i] = 0;
						break;
					case LED_CMD_ON:
						// LOG("%u on\n", i);
						state[i] = 1;
						break;
					case LED_CMD_BLINK:
						// LOG("%u blink %u millis\n", i, c.bit.millis);
						interval[i] = c.bit.millis;
						left[i] = c.bit.millis;
						break;
					case LED_CMD_TOGGLE:
						// LOG("%u toggle\n", i);
						state[i] = !state[i];
						// prevent further toggles
						cmd[i] = state[i] ? LED_CMD_ON : LED_CMD_OFF;
						break;
					case LED_CMD_BURST:
						// LOG("%u burst %u millis\n", i, c.bit.millis);
						left[i] = c.bit.millis;
						break;
					}

					cmd[i] = c.bit.cmd;
				}
			}

			switch (cmd[i]) {
			case LED_CMD_BLINK:
				if (left[i] >= TICK_MS) {
					left[i] -= TICK_MS;
				} else {
					left[i] = interval[i];
					state[i] = !state[i];
				}
				break;
			case LED_CMD_BURST:
				if (left[i] >= TICK_MS) {
					left[i] -= TICK_MS;
					state[i] = 1;
				} else {
					state[i] = 0;
					cmd[i] = LED_CMD_OFF;
				}
				break;
			}

			gpio_set_pin_level(leds[i].pin, state[i]);
		}

		vTaskDelay(pdMS_TO_TICKS(TICK_MS));
	}
}
