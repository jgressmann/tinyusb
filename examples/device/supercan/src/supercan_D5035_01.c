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

#include <FreeRTOS.h>
#include <timers.h>

#include <supercan_debug.h>
#include <supercan_board.h>

#include <hal/include/hal_gpio.h>
#include <dfu_ram.h>
#include <dfu_app.h>
#include <leds.h>

#include <tusb.h>
#include <class/dfu/dfu_rt_device.h>

#include <usb_descriptors.h> // DFU_USB_RESET_TIMEOUT_MS

#ifndef ARRAY_SIZE
#	define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif




#if SUPERDFU_APP
struct dfu_hdr dfu_hdr __attribute__((section(DFU_RAM_HDR_SECTION_NAME)));
static struct dfu_app_hdr dfu_app_hdr __attribute__((used,section(DFU_APP_HDR_SECTION_NAME))) = {
	.hdr_magic = DFU_APP_HDR_MAGIC_STRING,
	.hdr_version = DFU_APP_HDR_VERSION,
	.app_version_major = SUPERCAN_VERSION_MAJOR,
	.app_version_minor = SUPERCAN_VERSION_MINOR,
	.app_version_patch = SUPERCAN_VERSION_PATCH,
	.app_watchdog_timeout_s = 1,
	.app_name = SC_NAME,
};

static struct dfu_app_ftr dfu_app_ftr __attribute__((used,section(DFU_APP_FTR_SECTION_NAME))) = {
	.magic = DFU_APP_FTR_MAGIC_STRING,
};

static struct dfu {
	StaticTimer_t timer_mem;
	TimerHandle_t timer_handle;
} dfu;

static void dfu_timer_expired(TimerHandle_t t);
static void dfu_timer_expired(TimerHandle_t t)
{
	(void)t;

	LOG("DFU detach timer expired\n");

	/*
	 * This is wrong. DFU 1.1. specification wants us to wait for USB reset.
	 * However, without these lines, the device can't be updated on Windows using
	 * dfu-util. Updating from Linux works either way. All hail compatibility (sigh).
	 */
	dfu_request_dfu(1);
	NVIC_SystemReset();
}

#if CFG_TUD_DFU_RUNTIME
void tud_dfu_runtime_reboot_to_dfu_cb(uint16_t ms)
{
	LOG("tud_dfu_runtime_reboot_to_dfu_cb\n");
	/* The timer seems to be necessary, else dfu-util
	 * will fail spurriously with EX_IOERR (74).
	 */
	xTimerStart(dfu.timer_handle, pdMS_TO_TICKS(ms));

	// dfu_request_dfu(1);
	// NVIC_SystemReset();
}
#endif // #if CFG_TUD_DFU_RT
#endif


// controller and hardware specific setup of i/o pins for CAN
static inline void can_init_pins(void)
{
	// CAN0 port
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_HWSEL |           // upper half
		PORT_WRCONFIG_PINMASK(0x00c0) | // PA22/23
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(8) |         // I, CAN0, DS60001507E page 32, 910
		PORT_WRCONFIG_PMUXEN;

	// CAN1 port
	PORT->Group[1].WRCONFIG.reg =
		PORT_WRCONFIG_PINMASK(0xc000) | // PB14/15 = 0xc000, PB12/13 = 0x3000
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(7) |         // H, CAN1, DS60001507E page 32, 910
		PORT_WRCONFIG_PMUXEN;

}

static inline void can_init_clock(void) // controller and hardware specific setup of clock for the m_can module
{
	MCLK->AHBMASK.bit.CAN0_ = 1;
	MCLK->AHBMASK.bit.CAN1_ = 1;
	GCLK->PCHCTRL[CAN0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN0 to use GLCK0
	GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN1 to use GLCK0
}


static void can_init_module(void)
{
	memset(&same5x_cans, 0, sizeof(same5x_cans));

	same5x_cans[0].m_can = CAN0;
	same5x_cans[1].m_can = CAN1;
	same5x_cans[0].interrupt_id = CAN0_IRQn;
	same5x_cans[1].interrupt_id = CAN1_IRQn;
	same5x_cans[0].led_traffic = CAN0_TRAFFIC_LED;
	same5x_cans[1].led_traffic = CAN1_TRAFFIC_LED;
	same5x_cans[0].led_status_green = LED_CAN0_STATUS_GREEN;
	same5x_cans[1].led_status_green = LED_CAN1_STATUS_GREEN;
	same5x_cans[0].led_status_red = LED_CAN0_STATUS_RED;
	same5x_cans[1].led_status_red = LED_CAN1_STATUS_RED;

	for (size_t j = 0; j < TU_ARRAY_SIZE(same5x_cans); ++j) {
		struct same5x_can *can = &same5x_cans[j];
		can->features = CAN_FEAT_PERM;

		for (size_t i = 0; i < TU_ARRAY_SIZE(same5x_cans[0].rx_fifo); ++i) {
			SC_DEBUG_ASSERT(can->rx_frames[i].ts == 0);
		}

		for (size_t i = 0; i < TU_ARRAY_SIZE(same5x_cans[0].tx_fifo); ++i) {
			can->tx_fifo[i].T1.bit.EFC = 1; // store tx events

			SC_DEBUG_ASSERT(can->tx_frames[i].ts == 0);
		}
	}

	m_can_init_begin(CAN0);
	m_can_init_begin(CAN1);

	NVIC_SetPriority(CAN0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(CAN1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
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

struct led {
	uint8_t pin;
};

#define LED_STATIC_INITIALIZER(name, pin) \
	{ pin }


static const struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PA02), // board led
	LED_STATIC_INITIALIZER("red", PIN_PA18),
	LED_STATIC_INITIALIZER("orange", PIN_PA19),
	LED_STATIC_INITIALIZER("green", PIN_PB16),
	LED_STATIC_INITIALIZER("blue", PIN_PB17),
	LED_STATIC_INITIALIZER("can0_green", PIN_PB00),
	LED_STATIC_INITIALIZER("can0_red", PIN_PB01),
	LED_STATIC_INITIALIZER("can1_green", PIN_PB02),
	LED_STATIC_INITIALIZER("can1_red", PIN_PB03),
};

static inline void leds_init(void)
{
	PORT->Group[0].DIRSET.reg = PORT_PA18 | PORT_PA19;
	PORT->Group[1].DIRSET.reg = PORT_PB16 | PORT_PB17 | PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03;
}

#define POWER_LED LED_DEBUG_0
#define USB_LED LED_DEBUG_3


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


extern uint32_t _svectors;
extern uint32_t _evectors;

static void move_vector_table_to_ram(void)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
	void* vectors_ram = (void*)(uint32_t)&_svectors;

	memcpy(vectors_ram, (void*)SCB->VTOR, MCU_VECTOR_TABLE_ALIGNMENT);
	SCB->VTOR = (uint32_t)vectors_ram;
#pragma GCC diagnostic pop
}

extern void sc_board_init_begin(void)
{
	board_init();

	LOG("Vectors ROM @ %p\n", (void*)SCB->VTOR);
	move_vector_table_to_ram();
	LOG("Vectors RAM @ %p\n", (void*)SCB->VTOR);

	LOG("Enabling cache\n");
	same5x_enable_cache();

	same5x_init_device_identifier();


#if SUPERDFU_APP
	LOG(
		"%s v%u.%u.%u starting...\n",
		dfu_app_hdr.app_name,
		dfu_app_hdr.app_version_major,
		dfu_app_hdr.app_version_minor,
		dfu_app_hdr.app_version_patch);

	dfu_request_dfu(0); // no bootloader request

	dfu.timer_handle = xTimerCreateStatic("dfu", pdMS_TO_TICKS(DFU_USB_RESET_TIMEOUT_MS), pdFALSE, NULL, &dfu_timer_expired, &dfu.timer_mem);
#endif
	leds_init();

	can_init_pins();
	can_init_clock();
	can_init_module();

	counter_1MHz_init();

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
	led_blink(0, 2000);
	led_set(POWER_LED, 1);

#if SUPERDFU_APP
	dfu_app_watchdog_disable();
#endif
}

SC_RAMFUNC extern void sc_board_led_can_status_set(uint8_t index, int status)
{
	struct same5x_can *can = &same5x_cans[index];

	switch (status) {
	case SC_CAN_LED_STATUS_DISABLED:
		led_set(can->led_status_green, 0);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_BUS_OFF:
		led_set(can->led_status_green, 1);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_BUS_ON_PASSIVE:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_PASSIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_BUS_ON_ACTIVE:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ERROR_ACTIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		break;
	case SC_CAN_LED_STATUS_ERROR_PASSIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		break;
	default:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
		break;
	}
}


SC_RAMFUNC void CAN0_Handler(void)
{
	// LOG("CAN0 int\n");

	same5x_can_int(0);
}


SC_RAMFUNC void CAN1_Handler(void)
{
	// LOG("CAN1 int\n");

	same5x_can_int(1);
}


#endif // #ifdef D5035_01
