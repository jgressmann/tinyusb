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

#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <stdbool.h>
#include <sections.h>

#define LED_STACK_SIZE configMINIMAL_STACK_SIZE

extern StackType_t led_task_stack[LED_STACK_SIZE];
extern StaticTask_t led_task_mem;

SC_RAMFUNC extern void led_task(void *param);


enum {
	LED_DEBUG_DEFAULT,
	LED_DEBUG_0,
	LED_DEBUG_1,
	LED_DEBUG_2,
	LED_DEBUG_3,
	LED_CAN0_STATUS_GREEN,
	LED_CAN0_STATUS_RED,
	LED_CAN1_STATUS_GREEN,
	LED_CAN1_STATUS_RED,
	LED_COUNT
};


SC_RAMFUNC extern void led_init(void);
SC_RAMFUNC extern void led_set(uint8_t index, bool on);
SC_RAMFUNC extern void led_toggle(uint8_t index);
SC_RAMFUNC extern void led_blink(uint8_t index, uint16_t delay_ms);
SC_RAMFUNC extern void led_burst(uint8_t index, uint16_t duration_ms);
SC_RAMFUNC extern void leds_on_unsafe(void);

