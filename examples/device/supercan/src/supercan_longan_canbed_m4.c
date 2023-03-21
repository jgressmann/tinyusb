/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#ifdef LONGAN_CANBED_M4

#include <FreeRTOS.h>
#include <timers.h>

#include <supercan_debug.h>
#include <supercan_board.h>

#include <hal/include/hal_gpio.h>
#include <dfu_ram.h>
#include <dfu_app.h>
#include <sam_crc32.h>
#include <leds.h>
#include <bsp/board.h>

#include <tusb.h>
#include <class/dfu/dfu_rt_device.h>

#include <usb_descriptors.h> // DFU_USB_RESET_TIMEOUT_MS


#if SUPERDFU_APP
struct dfu_hdr dfu_hdr __attribute__((section(DFU_RAM_HDR_SECTION_NAME)));
static struct dfu_app_tag dfu_app_tag __attribute__((used,section(DFU_APP_TAG_SECTION_NAME))) = {
	.tag_magic = DFU_APP_TAG_MAGIC_STRING,
	.tag_version = DFU_APP_TAG_VERSION,
	.tag_bom = DFU_APP_TAG_BOM,
	.tag_dev_id = SUPERDFU_DEV_ID,
	.app_version_major = SUPERCAN_VERSION_MAJOR,
	.app_version_minor = SUPERCAN_VERSION_MINOR,
	.app_version_patch = SUPERCAN_VERSION_PATCH,
	.app_watchdog_timeout_s = 1,
	.app_name = SC_NAME,
};

static struct dfu_app_tag const * const dfu_app_tag_ptr __attribute__((used,section(DFU_APP_TAG_PTR_SECTION_NAME))) = &dfu_app_tag;

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
}

static inline void can_init_clock(void) // controller and hardware specific setup of clock for the m_can module
{
	MCLK->AHBMASK.bit.CAN0_ = 1;
	GCLK->PCHCTRL[CAN0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // setup CAN0 to use GLCK0
}


static void can_init_module(void)
{
	mcan_can_init();

	mcan_cans[0].m_can = CAN0;
	mcan_cans[0].interrupt_id = CAN0_IRQn;

	m_can_init_begin(CAN0);

	same5x_can_init();

	CAN0->MRCFG.reg = CAN_MRCFG_QOS_HIGH;

	NVIC_SetPriority(CAN0_IRQn, SC_ISR_PRIORITY);

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


extern uint32_t _svectors;
extern uint32_t _evectors;

static void move_vector_table_to_ram(void)
{
	uint8_t* svectors = (void*)&_svectors;
	uint8_t* evectors = (void*)&_evectors;

	memcpy(svectors, (void*)SCB->VTOR, evectors - svectors);

	SCB->VTOR = (uint32_t)svectors;
}

static inline void init_clock(void)
{
	// configure GCLK2 for 16MHz from DFLL48
	GCLK->GENCTRL[2].reg =
		GCLK_GENCTRL_DIV(3) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DFLL |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL2); /* wait for the synchronization between clock domains to be complete */
}


extern void sc_board_init_begin(void)
{
#if !SUPERDFU_APP
	board_init();
	sam_crc32_unlock();
#endif

	init_clock();

	LOG("Vectors ROM @ %p\n", (void*)SCB->VTOR);
	move_vector_table_to_ram();
	LOG("Vectors RAM @ %p\n", (void*)SCB->VTOR);

	LOG("Enabling cache\n");
	same5x_enable_cache();

	same5x_init_device_identifier();


#if SUPERDFU_APP
	LOG(
		"%s v%u.%u.%u starting...\n",
		dfu_app_tag.app_name,
		dfu_app_tag.app_version_major,
		dfu_app_tag.app_version_minor,
		dfu_app_tag.app_version_patch);

	dfu_request_dfu(0); // no bootloader request

	dfu.timer_handle = xTimerCreateStatic("dfu", pdMS_TO_TICKS(DFU_USB_RESET_TIMEOUT_MS), pdFALSE, NULL, &dfu_timer_expired, &dfu.timer_mem);
#endif

	can_init_pins();
	can_init_clock();
	can_init_module();

	counter_1MHz_init();
}

extern void sc_board_init_end(void)
{
	led_blink(0, 2000);

#if SUPERDFU_APP
	dfu_app_watchdog_disable();
#endif
}

SC_RAMFUNC void CAN0_Handler(void)
{
	// LOG("CAN0 int\n");

	mcan_can_int(0);
}

#endif // #ifdef LONGAN_CANBED_M4

