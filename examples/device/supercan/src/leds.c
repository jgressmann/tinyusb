#include "leds.h"
#include "supercan_debug.h"


#include <sam.h>

#include <hal/include/hal_gpio.h>

#ifndef ARRAY_SIZE
#	define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif

StackType_t led_task_stack[LED_STACK_SIZE];
StaticTask_t led_task_mem;

static StaticSemaphore_t mutex_mem;
static SemaphoreHandle_t mutex_handle;


struct led {
	volatile uint16_t time_ms;
	volatile uint8_t blink;
	uint8_t pin;
};

#define LED_STATIC_INITIALIZER(name, pin) \
	{ 0, 0, pin }


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

extern void led_init(void)
{
	mutex_handle = xSemaphoreCreateMutexStatic(&mutex_mem);

	PORT->Group[1].DIRSET.reg = PORT_PB14; /* Debug-LED */
	PORT->Group[1].DIRSET.reg = PORT_PB15; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA12; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA13; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA14; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA15; /* Debug-LED */
}
#else // HWREV > 1
static struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PA02), // board led
	LED_STATIC_INITIALIZER("red", PIN_PA18),
	LED_STATIC_INITIALIZER("orange", PIN_PA19),
	LED_STATIC_INITIALIZER("green", PIN_PB16),
	LED_STATIC_INITIALIZER("blue", PIN_PB17),
#if HWREV >= 3
	LED_STATIC_INITIALIZER("can0_green", PIN_PB03),
	LED_STATIC_INITIALIZER("can0_red", PIN_PB02),
	LED_STATIC_INITIALIZER("can1_green", PIN_PB01),
	LED_STATIC_INITIALIZER("can1_red", PIN_PB00),
#endif
};




extern void led_init(void)
{
	mutex_handle = xSemaphoreCreateMutexStatic(&mutex_mem);

	PORT->Group[0].DIRSET.reg = PORT_PA18 | PORT_PA19;
	PORT->Group[1].DIRSET.reg =
		PORT_PB16 | PORT_PB17
#if HWREV >= 3
		| PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03
#endif
		;
}

#endif // HWREV > 1


extern void led_set(uint8_t index, bool on)
{
	SC_ASSERT(index < ARRAY_SIZE(leds));
	while (pdTRUE != xSemaphoreTake(mutex_handle, ~0));
	leds[index].time_ms = 0;
	leds[index].blink = 0;
	gpio_set_pin_level(leds[index].pin, on);
	xSemaphoreGive(mutex_handle);
}

extern void led_toggle(uint8_t index)
{
	SC_ASSERT(index < ARRAY_SIZE(leds));
	while (pdTRUE != xSemaphoreTake(mutex_handle, ~0));
	leds[index].time_ms = 0;
	leds[index].blink = 0;
	gpio_toggle_pin_level(leds[index].pin);
	xSemaphoreGive(mutex_handle);
}

extern void led_blink(uint8_t index, uint16_t delay_ms)
{
	SC_ASSERT(index < ARRAY_SIZE(leds));
	while (pdTRUE != xSemaphoreTake(mutex_handle, ~0));
	leds[index].time_ms = delay_ms;
	leds[index].blink = 1;
	xSemaphoreGive(mutex_handle);
}

extern void led_burst(uint8_t index, uint16_t duration_ms)
{
	SC_ASSERT(index < ARRAY_SIZE(leds));
	while (pdTRUE != xSemaphoreTake(mutex_handle, ~0));
	leds[index].time_ms = duration_ms;
	leds[index].blink = 0;
	gpio_set_pin_level(leds[index].pin, 1);
	xSemaphoreGive(mutex_handle);
}


extern void led_on(void)
{
	for (unsigned i = 0; i < LED_COUNT; ++i) {
		led_set(i, 1);
	}
}

//--------------------------------------------------------------------+
// LED TASK
//--------------------------------------------------------------------+
extern void led_task(void *param)
{
	(void) param;

	uint16_t left[ARRAY_SIZE(leds)];
	memset(&left, 0, sizeof(left));
	const uint8_t TICK_MS = 8;

	while (42) {
		while (pdTRUE != xSemaphoreTake(mutex_handle, ~0));
		for (uint8_t i = 0; i < ARRAY_SIZE(leds); ++i) {
			uint16_t t = leds[i].time_ms;
			if (t) {
				if (leds[i].blink) {
					if (0 == left[i]) {
						// LOG("led %s (%u) toggle\n", leds[i].name, i);
						gpio_toggle_pin_level(leds[i].pin);
						left[i] = t / TICK_MS;
					} else {
						--left[i];
					}
				} else {
					// burst
					t -= tu_min16(t, TICK_MS);
					leds[i].time_ms = t;
					if (!t) {
						gpio_set_pin_level(leds[i].pin, 0);
					}
				}
			}
		}
		xSemaphoreGive(mutex_handle);

		vTaskDelay(pdMS_TO_TICKS(TICK_MS));
	}
}