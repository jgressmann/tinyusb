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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

#include "bsp/board.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : button is not pressed
 * - 1000 ms : button is pressed (and hold)
 */
enum  {
  BLINK_PRESSED = 250,
  BLINK_UNPRESSED = 1000
};

#define HELLO_STR   "Hello from TinyUSB\r\n"

// static timer
StaticTimer_t timer_mem;
TimerHandle_t timer_handle;

// static task for cdc
#define BUTTON_TASK_STACK_SZIE      configMINIMAL_STACK_SIZE
StackType_t  button_task_stack[BUTTON_TASK_STACK_SZIE];
StaticTask_t button_task_mem;

static void timer_cb(TimerHandle_t xTimer);
static void button_task(void* params);

int main(void)
{
  board_init();

  // soft timer for blinky
  timer_handle = xTimerCreateStatic(NULL, pdMS_TO_TICKS(BLINK_UNPRESSED), true, NULL, timer_cb, &timer_mem);
  xTimerStart(timer_handle, 0);

  // Create button task
  (void) xTaskCreateStatic(button_task, "button", BUTTON_TASK_STACK_SZIE, NULL, configMAX_PRIORITIES-2, button_task_stack, &button_task_mem);

  // skip starting scheduler (and return) for ESP32-S2 or ESP32-S3
#if CFG_TUSB_MCU != OPT_MCU_ESP32S2 && CFG_TUSB_MCU != OPT_MCU_ESP32S3
  vTaskStartScheduler();
#endif

  return 0;
}

#if CFG_TUSB_MCU == OPT_MCU_ESP32S2 || CFG_TUSB_MCU == OPT_MCU_ESP32S3
void app_main(void)
{
  main();
}
#endif

static void button_task(void* param)
{
  uint32_t interval_ms_last = BLINK_UNPRESSED;
  uint32_t interval_ms = BLINK_UNPRESSED;

  (void) param;

  while (1)
  {
    interval_ms = board_button_read() ? BLINK_PRESSED : BLINK_UNPRESSED;

    if (interval_ms != interval_ms_last) {
      interval_ms_last = interval_ms;
      xTimerChangePeriod(timer_handle, pdMS_TO_TICKS(interval_ms), 0);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void timer_cb(TimerHandle_t xTimer)
{
  static bool led_state = false;

  (void) xTimer;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle

  board_uart_write(HELLO_STR, strlen(HELLO_STR));
}