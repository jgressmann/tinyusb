/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */



#if D5035_50

#include <FreeRTOS.h>
#include <timers.h>

#include <sllin_board.h>
#include <leds.h>

#include <hal/include/hal_gpio.h>
#include <crc32.h>
#include <mcu.h>
#include <bsp/board.h>



#include <tusb.h>
#include <class/dfu/dfu_rt_device.h>

#define SLEEP_TIMER_HZ 3906 // (1 MHZ / 256)
#define LIN_SERCOM_PORT_GROUP 0
#define MASTER_SLAVE_PIN_GROUP 1
#define BOARD_SERCOM SERCOM2
#define USART_BAURATE      115200
#define CONF_CPU_FREQUENCY 48000000
#define CONF_LIN_UART_FREQUENCY 48000000



/* The board runs off of the 48 MHz internal RC oscillator
 *
 * @115200 debug prints KILL LIN timing assumptions!
*/


#if SUPERDFU_APP
struct dfu_hdr dfu_hdr __attribute__((section(DFU_RAM_HDR_SECTION_NAME)));
static struct dfu_app_hdr dfu_app_hdr __attribute__((used,section(DFU_APP_HDR_SECTION_NAME))) = {
	.hdr_magic = DFU_APP_HDR_MAGIC_STRING,
	.hdr_version = DFU_APP_HDR_VERSION,
	.app_version_major = SLLIN_VERSION_MAJOR,
	.app_version_minor = SLLIN_VERSION_MINOR,
	.app_version_patch = SLLIN_VERSION_PATCH,
	.app_watchdog_timeout_s = 1,
	.app_name = SLLIN_NAME,
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


enum {
	SLAVE_PROTO_STEP_RX_BREAK = 0,
	SLAVE_PROTO_STEP_RX_PID,
	SLAVE_PROTO_STEP_TX_DATA,
	SLAVE_PROTO_STEP_RX_DATA,
};


SLLIN_RAMFUNC static void lin_usart_int_slave(uint8_t index);
SLLIN_RAMFUNC static void lin_timer_int_data(uint8_t index);
SLLIN_RAMFUNC static void lin_timer_int_sof(uint8_t index);


struct slave {
	Tc* const sleep_timer;
	Tc* const data_timer;
	sllin_queue_element elem;
	uint8_t slave_frame_enabled[64];
	uint16_t timeout_pid_us;
	uint16_t timeout_data_us;
	uint8_t slave_proto_step;
	uint8_t slave_tx_offset;
	uint8_t slave_rx_offset;
};

struct master {
	Tc* const sof_timer;
	uint8_t busy;
};

struct lin {
	Sercom* const sercom;
	struct slave slave;
	struct master master;
	uint8_t const rx_pin_index;
	uint8_t const master_slave_pin;
	uint8_t const led_status_green;
	uint8_t const led_status_red;
	uint8_t bus_state;
	uint8_t bus_error;
};

static struct lin lins[SLLIN_BOARD_LIN_COUNT] = {
	{
		.sercom = SERCOM1,
		.slave.data_timer = TC0, // NOTE: TC0/TC1 don't seem to support different clock speeds, setting one will affect the other.
		.slave.sleep_timer = TC2,
		.master.sof_timer = TC4,
		.rx_pin_index = 1,
		.master_slave_pin = 4,
		.led_status_green = 5,
		.led_status_red = 6,
	},
	{
		.sercom = SERCOM0,
		.slave.data_timer = TC1,
		.slave.sleep_timer = TC3,
		.master.sof_timer = TC5,
		.rx_pin_index = 5,
		.master_slave_pin = 9,
		.led_status_green = 7,
		.led_status_red = 8,
	},
};

struct led {
	uint8_t pin;
};

#define LED_STATIC_INITIALIZER(name, pin) \
	{ pin }


static const struct led leds[] = {
	LED_STATIC_INITIALIZER("debug_default", PIN_PA02), // board led
	LED_STATIC_INITIALIZER("debug_red", PIN_PA18),
	LED_STATIC_INITIALIZER("debug_orange", PIN_PA19),
	LED_STATIC_INITIALIZER("debug_green", PIN_PB16),
	LED_STATIC_INITIALIZER("debug_blue", PIN_PB17),
	LED_STATIC_INITIALIZER("lin0_green", PIN_PB00),
	LED_STATIC_INITIALIZER("lin0_red", PIN_PB01),
	LED_STATIC_INITIALIZER("lin1_green", PIN_PB02),
	LED_STATIC_INITIALIZER("lin1_red", PIN_PB03),
};

static inline void leds_init(void)
{
	PORT->Group[0].DIRSET.reg = PORT_PA02 | PORT_PA18 | PORT_PA19;
	PORT->Group[1].DIRSET.reg = PORT_PB16 | PORT_PB17 | PORT_PB00 | PORT_PB01 | PORT_PB02 | PORT_PB03;
}

#define POWER_LED LED_DEBUG_0
#define USB_LED LED_DEBUG_3



extern uint32_t _svectors;
extern uint32_t _evectors;

static void move_vector_table_to_ram(void)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#pragma GCC diagnostic ignored "-Warray-bounds"
	void* vectors_ram = (void*)(uint32_t)&_svectors;

	memcpy(vectors_ram, (void*)SCB->VTOR, MCU_VECTOR_TABLE_ALIGNMENT);
	SCB->VTOR = (uint32_t)vectors_ram;
#pragma GCC diagnostic pop
}


static inline void same5x_enable_cache(void)
{
	// DS60001507E-page 83
	if (!CMCC->SR.bit.CSTS) {
		CMCC->CTRL.bit.CEN = 1;
	}
}

static uint32_t device_identifier;

extern uint32_t sllin_board_identifier(void)
{
	return device_identifier;
}


void same5x_init_device_identifier(void)
{
	uint32_t serial_number[4];
	int error = CRC32E_NONE;

	same51_get_serial_number(serial_number);

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

extern void sllin_board_led_set(uint8_t index, bool on)
{
	SLLIN_DEBUG_ASSERT(index < TU_ARRAY_SIZE(leds));

	gpio_set_pin_level(leds[index].pin, on);
}

extern void sllin_board_leds_on_unsafe(void)
{
	for (size_t i = 0; i < TU_ARRAY_SIZE(leds); ++i) {
		gpio_set_pin_level(leds[i].pin, 1);
	}
}

static inline void uart_init(void)
{
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_DRVSTR |
		PORT_WRCONFIG_PINMASK(0x0300) | /* PA08, PA9 */
		PORT_WRCONFIG_PMUXEN;

	MCLK->APBBMASK.bit.SERCOM2_ = 1;
	GCLK->PCHCTRL[SERCOM2_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

	BOARD_SERCOM->USART.CTRLA.reg = 0x00; /* disable SERCOM -> enable config */
	while(BOARD_SERCOM->USART.SYNCBUSY.bit.ENABLE);

	BOARD_SERCOM->USART.CTRLA.reg =
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_DORD |     /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) |  /* 0x0 USART with external clock, 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(1) |  /* SERCOM PAD[1] is used for data reception */
		SERCOM_USART_CTRLA_TXPO(0);   /* SERCOM PAD[0] is used for data transmission */

	BOARD_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN; /* transmitter enabled */
	uint16_t baud = CONF_CPU_FREQUENCY / (16 * USART_BAURATE);
	uint16_t frac = CONF_CPU_FREQUENCY / (2 * USART_BAURATE) - 8 * baud;
	BOARD_SERCOM->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(frac) | SERCOM_USART_BAUD_FRAC_BAUD(baud);

	BOARD_SERCOM->USART.CTRLA.bit.ENABLE = 1; /* activate SERCOM */
	while(BOARD_SERCOM->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */
}


static inline void lin_init_once(void)
{
	// lin0
	// gpio_set_pin_function(PIN_PA00, PINMUX_PA00D_SERCOM1_PAD0);
	// gpio_set_pin_function(PIN_PA01, PINMUX_PA01D_SERCOM1_PAD1);
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_INEN |
		PORT_WRCONFIG_PINMASK(0x0003) | /* PA00, PA01 */
		PORT_WRCONFIG_PMUXEN;

	MCLK->APBAMASK.bit.SERCOM1_ = 1;
	GCLK->PCHCTRL[SERCOM1_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

	NVIC_SetPriority(SERCOM1_0_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(SERCOM1_1_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(SERCOM1_2_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(SERCOM1_3_IRQn, SLLIN_ISR_PRIORITY);

	NVIC_EnableIRQ(SERCOM1_0_IRQn);
	NVIC_EnableIRQ(SERCOM1_1_IRQn);
	NVIC_EnableIRQ(SERCOM1_2_IRQn);
	NVIC_EnableIRQ(SERCOM1_3_IRQn);

	// lin1
	// gpio_set_pin_function(PIN_PA04, PINMUX_PA04D_SERCOM0_PAD0);
	// gpio_set_pin_function(PIN_PA05, PINMUX_PA05D_SERCOM0_PAD1);
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_INEN |
		PORT_WRCONFIG_PINMASK(0x0030) | /* PA04, PA05 */
		PORT_WRCONFIG_PMUXEN;

	MCLK->APBAMASK.bit.SERCOM0_ = 1;
	GCLK->PCHCTRL[SERCOM0_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

	NVIC_SetPriority(SERCOM0_0_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(SERCOM0_1_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(SERCOM0_2_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(SERCOM0_3_IRQn, SLLIN_ISR_PRIORITY);

	NVIC_EnableIRQ(SERCOM0_0_IRQn);
	NVIC_EnableIRQ(SERCOM0_1_IRQn);
	NVIC_EnableIRQ(SERCOM0_2_IRQn);
	NVIC_EnableIRQ(SERCOM0_3_IRQn);

	// master / slave pin
	// PB04 lin0
	// PB09 lin1
	PORT->Group[MASTER_SLAVE_PIN_GROUP].DIRSET.reg = (1ul << 4) | (1ul << 9);
	PORT->Group[MASTER_SLAVE_PIN_GROUP].OUTCLR.reg = (1ul << 4) | (1ul << 9);
}

static void timer_init(void)
{
	GCLK->GENCTRL[4].reg =
		GCLK_GENCTRL_DIV(3) |	/* 48Mhz -> 16MHz */
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DFLL |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL4); /* wait for the synchronization between clock domains to be complete */

	GCLK->GENCTRL[5].reg =
		GCLK_GENCTRL_DIV(48) |	/* 48Mhz -> 1MHz */
		// GCLK_GENCTRL_DIVSEL |   /* divide by 2^(DIV+1) */
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DFLL |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL5); /* wait for the synchronization between clock domains to be complete */


	MCLK->APBAMASK.bit.TC0_ = 1;
	MCLK->APBAMASK.bit.TC1_ = 1;
	MCLK->APBBMASK.bit.TC2_ = 1;
	MCLK->APBBMASK.bit.TC3_ = 1;
	MCLK->APBCMASK.bit.TC4_ = 1;
	MCLK->APBCMASK.bit.TC5_ = 1;

	// TC0/1 are connected to the SAME peripheral clock, TC2/3 likewise
	GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;
	GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;
	GCLK->PCHCTRL[TC2_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK5 | GCLK_PCHCTRL_CHEN;
	GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK5 | GCLK_PCHCTRL_CHEN;
	GCLK->PCHCTRL[TC4_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;
	GCLK->PCHCTRL[TC5_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;

	NVIC_SetPriority(TC0_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(TC1_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(TC2_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(TC3_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(TC4_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(TC5_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_EnableIRQ(TC0_IRQn);
	NVIC_EnableIRQ(TC1_IRQn);
	NVIC_EnableIRQ(TC2_IRQn);
	NVIC_EnableIRQ(TC3_IRQn);
	NVIC_EnableIRQ(TC4_IRQn);
	NVIC_EnableIRQ(TC5_IRQn);

	for (size_t i = 0; i < TU_ARRAY_SIZE(lins); ++i) {
		struct lin *lin = &lins[i];
		struct slave *sl = &lin->slave;
		struct master *ma = &lin->master;
		Tc* data_timer = sl->data_timer;
		Tc* stc = sl->sleep_timer;
		Tc* sof_timer = ma->sof_timer;

		// MASTER
		// sof timeout
		sof_timer->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
		while (sof_timer->COUNT16.SYNCBUSY.bit.SWRST);

		sof_timer->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

		// generate overflow interrupt
		sof_timer->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_ERR;

		// set to max so we don't time out
		sof_timer->COUNT16.CC[0].reg = 0xffff;

		// enable
		sof_timer->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV16;

		// stop & oneshot
		sof_timer->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT | TC_CTRLBSET_LUPD | TC_CTRLBSET_CMD_STOP;
		while (sof_timer->COUNT16.SYNCBUSY.bit.CTRLB);

		// reset to zero
		sof_timer->COUNT16.COUNT.reg = 0;

		// SLAVE

		// reponse timeout
		data_timer->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
		while (data_timer->COUNT16.SYNCBUSY.bit.SWRST);

		data_timer->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

		// generate overflow interrupt
		data_timer->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_ERR;

		// set to max so we don't time out
		data_timer->COUNT16.CC[0].reg = 0xffff;

		// enable
		data_timer->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV16;

		// stop & oneshot
		data_timer->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT | TC_CTRLBSET_LUPD | TC_CTRLBSET_CMD_STOP;
		// data_timer->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_STOP;
		while (data_timer->COUNT16.SYNCBUSY.bit.CTRLB);

		// reset to zero
		data_timer->COUNT16.COUNT.reg = 0;


		// sleep timeout
		stc->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
		while (stc->COUNT16.SYNCBUSY.bit.SWRST);

		stc->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

		// generate overflow interrupt
		stc->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_ERR;

		// set to max so we don't time out
		stc->COUNT16.CC[0].reg = 0xffff;

		// enable
		stc->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 |  TC_CTRLA_PRESCALER_DIV256;

		// stop & oneshot
		stc->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT | TC_CTRLBSET_LUPD | TC_CTRLBSET_CMD_STOP;
		while (stc->COUNT16.SYNCBUSY.bit.CTRLB);

		// reset to zero
		stc->COUNT16.COUNT.reg = 0;
	}
}

/* According to DS60001507E-page 1717 it should
 * suffice to write the re-trigger command. This
 * _does_ work if there is a pausse after the write
 * during which the timer isn't manipulated.
 * It does _not_ work for data byte timeouts or
 * wake up timeouts (basically any case in which the command
 * is repeatedly given).
 *
 * Thus here is a solution that appears to work.
 */
#define timer_start_or_restart16(tc) \
	do { \
		(tc)->COUNT16.COUNT.reg = 0; \
		if ((tc)->COUNT16.STATUS.bit.STOP) { \
			(tc)->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val; \
		} \
		(tc)->COUNT16.INTFLAG.reg = ~0; \
	} while (0)

#define timer_cleanup_begin(tc) do { (tc)->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val; } while (0)

SLLIN_RAMFUNC static inline void timer_cleanup_end(Tc* timer)
{
	// wait for sync
	while (timer->COUNT16.SYNCBUSY.bit.CTRLB);
	// reset value
	timer->COUNT16.COUNT.reg = 0;
	// clear interrupt flags
	timer->COUNT16.INTFLAG.reg = ~0;
}


#define timer_cleanup(tc) \
	do { \
		timer_cleanup_begin(tc); \
		timer_cleanup_end(tc); \
	} while (0)


#define sleep_timer_cleanup(lin) \
	do { \
		timer_cleanup((lin)->slave.sleep_timer); \
	} while (0)

#define sof_timer_cleanup(lin) timer_cleanup((lin)->master.sof_timer)


SLLIN_RAMFUNC static inline void lin_slave_cleanup(struct lin *lin)
{
	struct slave *sl = &lin->slave;

	// stop timer
	timer_cleanup_begin(sl->data_timer);


	// set PID timeout
	sl->data_timer->COUNT16.CC[0].reg = sl->timeout_pid_us;

	sl->slave_proto_step = SLAVE_PROTO_STEP_RX_BREAK;
	sl->slave_tx_offset = 0;
	sl->slave_rx_offset = 0;
	sl->elem.type = SLLIN_QUEUE_ELEMENT_TYPE_FRAME;
	sl->elem.frame.id = 0;
	sl->elem.frame.len = 0;

	lin->sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;

	timer_cleanup_end(sl->data_timer);
}

static inline void usb_init(void)
{
	NVIC_SetPriority(USB_0_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(USB_1_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(USB_2_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(USB_3_IRQn, SLLIN_ISR_PRIORITY);


	/* USB clock init
	 * The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
	 * for low speed and full speed operation.
	 */
	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK0_Val | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_AHBMASK_USB_bit(MCLK);
	hri_mclk_set_APBBMASK_USB_bit(MCLK);

	gpio_set_pin_function(PIN_PA24, PINMUX_PA24H_USB_DM);
	gpio_set_pin_function(PIN_PA25, PINMUX_PA25H_USB_DP);
}

extern void sllin_board_init_begin(void)
{
	// clock_init();

	SystemCoreClock = CONF_CPU_FREQUENCY;

	uart_init();
	LOG("CONF_CPU_FREQUENCY=%lu\n", (unsigned long)CONF_CPU_FREQUENCY);

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

	LOG("USB init\n");
	usb_init();

	LOG("timer init\n");
	timer_init();

	LOG("LIN init once\n");
	lin_init_once();
}

extern void sllin_board_init_end(void)
{
	led_blink(0, 2000);
	led_set(POWER_LED, 1);

#if SUPERDFU_APP
	dfu_app_watchdog_disable();
#endif
}

__attribute__((noreturn)) extern void sllin_board_reset(void)
{
	NVIC_SystemReset();
	__unreachable();
}

extern void sllin_board_lin_uninit(uint8_t index)
{
	struct lin *lin = &lins[index];
	Sercom *sercom = lin->sercom;

	// disable SERCOM
	sercom->USART.CTRLA.bit.SWRST = 1;
}

extern void sllin_board_lin_init(uint8_t index, sllin_conf *conf)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;
	struct master *ma = &lin->master;
	Sercom *sercom = lin->sercom;
	unsigned sof_len_bit_times = 0;

	for (size_t i = 0; i < TU_ARRAY_SIZE(sl->slave_frame_enabled); ++i) {
		sl->slave_frame_enabled[i] = 0;
	}

	__atomic_store_n(&lin->bus_state, SLLIN_ID_FLAG_BUS_STATE_ASLEEP, __ATOMIC_RELAXED);
	__atomic_store_n(&lin->bus_error, SLLIN_ID_FLAG_BUS_ERROR_NONE, __ATOMIC_RELAXED);

	// disable timers
	sl->data_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val;
	sl->sleep_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val;
	ma->sof_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val;

	// disable SERCOM
	sercom->USART.CTRLA.bit.SWRST = 1; /* reset and disable SERCOM -> enable configuration */

	// wait for SERCOM to be ready
	while (lin->sercom->USART.SYNCBUSY.bit.SWRST);

	sercom->USART.CTRLA.reg  =
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_SAMPA(0) |
		SERCOM_USART_CTRLA_FORM(conf->master ? 0x2 : 0x4) | /* 0x2 LIN master, 0x4 LIN slave, ... */
		SERCOM_USART_CTRLA_DORD | /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) | /* 0x0 USART with external clock, 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_IBON | /* assert RX buffer overflow immediately */
		SERCOM_USART_CTRLA_RXPO(1) |
		SERCOM_USART_CTRLA_TXPO(0);

	sercom->USART.CTRLB.reg = /* RXEM = 0 -> receiver disabled, LINCMD = 0 -> normal USART transmission, SFDE = 0 -> start-of-frame detection disabled, SBMODE = 0 -> one stop bit, CHSIZE = 0 -> 8 bits */
		SERCOM_USART_CTRLB_TXEN | /* transmitter enabled */
		SERCOM_USART_CTRLB_RXEN | /* receiver enabled */
		// SERCOM_USART_CTRLB_COLDEN; /* collision detection enabled */
		0;

	uint16_t baud = CONF_LIN_UART_FREQUENCY / (16 * conf->bitrate);
	uint16_t frac = CONF_LIN_UART_FREQUENCY / (2 * conf->bitrate) - 8 * baud;
	sercom->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(frac) | SERCOM_USART_BAUD_FRAC_BAUD(baud);

	// clear interrupts
	sercom->USART.INTENCLR.reg = ~0;

	// RXC _must_ be enabled, else data get's stuck in register and we get errors
	sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXBRK | SERCOM_USART_INTENSET_ERROR | SERCOM_USART_INTENSET_RXC;


	if (conf->master) {
		PORT->Group[MASTER_SLAVE_PIN_GROUP].OUTSET.reg = 1ul << lin->master_slave_pin;
	} else {
		PORT->Group[MASTER_SLAVE_PIN_GROUP].OUTCLR.reg = 1ul << lin->master_slave_pin;
	}

	lin_slave_cleanup(lin);
	sof_timer_cleanup(lin);
	__atomic_store_n(&lin->master.busy, 0, __ATOMIC_RELAXED);


	// LOG("ch%u data byte timeout CC=%x [us]\n", index, sl->data_timer->COUNT16.CC[0].reg);
	sl->timeout_data_us = (14 * UINT32_C(1000000)) / conf->bitrate;
	LOG("ch%u data byte timeout %x [us]\n", index, sl->timeout_data_us);
	sl->timeout_pid_us = ((14 + 10) * UINT32_C(1000000)) / conf->bitrate;
	LOG("ch%u PID byte timeout %x [us]\n", index, sl->timeout_pid_us);

	sl->data_timer->COUNT16.CC[0].reg = sl->timeout_data_us;

	sof_len_bit_times = 13 + sercom->USART.CTRLC.bit.BRKLEN * 4;
	sof_len_bit_times += 20; // SYNC, PID

	switch (sercom->USART.CTRLC.bit.HDRDLY) {
	case 0:
		sof_len_bit_times += 2;
		break;
	case 1:
		sof_len_bit_times += 8;
		break;
	case 2:
		sof_len_bit_times += 12;
		break;
	case 3:
		sof_len_bit_times += 18;
		break;
	}

	// // add a little bit of extra time for the RX to hit the UART
	// sof_len_bit_times += 4; // half a byte

	ma->sof_timer->COUNT16.CC[0].reg = (sof_len_bit_times * UINT32_C(1000000)) / conf->bitrate;
	LOG("ch%u SOF timeout %x [us]\n", index, ma->sof_timer->COUNT16.CC[0].reg);


	sercom->USART.CTRLA.bit.ENABLE = 1;
	while (sercom->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */

	// wait for sleep timer sync
	while (sl->sleep_timer->COUNT16.SYNCBUSY.bit.CTRLB);

	// reset value
	sl->sleep_timer->COUNT16.COUNT.reg = 0;

	// clear interrupts
	sl->sleep_timer->COUNT16.INTFLAG.reg = ~0;

	sl->sleep_timer->COUNT16.CC[0].reg = ((conf->sleep_timeout_ms * (uint32_t)SLEEP_TIMER_HZ) + 500) / 1000;
	LOG("ch%u sleep timer CC=%x COUNT=%x\n", index, sl->sleep_timer->COUNT16.CC[0].reg, sl->sleep_timer->COUNT16.COUNT.reg);

	// start sleep timer
	sl->sleep_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
}

SLLIN_RAMFUNC static bool sllin_board_lin_master_tx(uint8_t index, uint8_t data, uint8_t cmd)
{
	struct lin * const lin = &lins[index];
	struct master * const ma = &lin->master;
	struct slave * const sl = &lin->slave;
	Sercom * const s = lin->sercom;
	uint8_t busy = 0;

	(void)sl;

	SLLIN_DEBUG_ASSERT(index < TU_ARRAY_SIZE(lins));


	busy = __atomic_load_n(&lin->master.busy, __ATOMIC_ACQUIRE);

	if (unlikely(busy)) {
		LOG("ch%u master tx busy\n", index);
		LOG("ch%u data timer stop=%u sof timer stop=%u\n", index, sl->data_timer->COUNT16.STATUS.bit.STOP, ma->sof_timer->COUNT16.STATUS.bit.STOP);
		return false;
	}

	__atomic_store_n(&lin->master.busy, 1, __ATOMIC_RELEASE);

	SLLIN_DEBUG_ASSERT(0 == ma->sof_timer->COUNT16.COUNT.reg);

	// start timer
	// LOG("+");
	ma->sof_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;

	// race against other interrupt?

	s->USART.CTRLB.bit.LINCMD = cmd;
	s->USART.DATA.reg = data;

	return true;
}

SLLIN_RAMFUNC extern bool sllin_board_lin_master_break(uint8_t index)
{
	return sllin_board_lin_master_tx(index, 0, 1);
}

SLLIN_RAMFUNC extern bool sllin_board_lin_master_request(uint8_t index, uint8_t id)
{
	SLLIN_DEBUG_ASSERT(id < 64);

	return sllin_board_lin_master_tx(index, sllin_id_to_pid(id), 2);
}


SLLIN_RAMFUNC extern void sllin_board_lin_slave_respond(
	uint8_t index,
	uint8_t id,
	bool respond)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;

	SLLIN_DEBUG_ASSERT(index < TU_ARRAY_SIZE(lins));
	SLLIN_DEBUG_ASSERT(id < 64);

	__atomic_store_n(&sl->slave_frame_enabled[id], respond, __ATOMIC_RELEASE);
}

SLLIN_RAMFUNC extern void sllin_board_led_lin_status_set(uint8_t index, int status)
{
	struct lin *lin = &lins[index];

	// LOG("ch%u set led status %u\n", index, status);

	switch (status) {
	case SLLIN_LIN_LED_STATUS_DISABLED:
		led_set(lin->led_status_green, 0);
		led_set(lin->led_status_red, 0);
		break;
	case SLLIN_LIN_LED_STATUS_ENABLED_OFF_BUS:
		led_set(lin->led_status_green, 1);
		led_set(lin->led_status_red, 0);
		break;
	case SLLIN_LIN_LED_STATUS_ON_BUS_SLEEPING:
		led_blink(lin->led_status_green, SLLIN_LIN_LED_BLINK_DELAY_SLEEPING_MS);
		led_set(lin->led_status_red, 0);
		break;
	case SLLIN_LIN_LED_STATUS_ON_BUS_AWAKE_PASSIVE:
		led_blink(lin->led_status_green, SLLIN_LIN_LED_BLINK_DELAY_AWAKE_PASSIVE_MS);
		led_set(lin->led_status_red, 0);
		break;
	case SLLIN_LIN_LED_STATUS_ON_BUS_AWAKE_ACTIVE:
		led_blink(lin->led_status_green, SLLIN_LIN_LED_BLINK_DELAY_AWAKE_ACTIVE_MS);
		led_set(lin->led_status_red, 0);
		break;
	case SLLIN_LIN_LED_STATUS_ERROR:
		led_set(lin->led_status_green, 0);
		led_blink(lin->led_status_red, SLLIN_LIN_LED_BLINK_DELAY_AWAKE_ACTIVE_MS);
		break;
	default:
		led_blink(lin->led_status_green, SLLIN_LIN_LED_BLINK_DELAY_AWAKE_ACTIVE_MS / 2);
		led_blink(lin->led_status_red, SLLIN_LIN_LED_BLINK_DELAY_AWAKE_ACTIVE_MS / 2);
		break;
	}
}

SLLIN_RAMFUNC static inline void lin_int_update_bus_status(uint8_t index, uint8_t bus_state, uint8_t bus_error)
{
	struct lin *lin = &lins[index];
	uint8_t bus_state_current = __atomic_load_n(&lin->bus_state, __ATOMIC_ACQUIRE);
	uint8_t bus_error_current = __atomic_load_n(&lin->bus_error, __ATOMIC_ACQUIRE);

	if (unlikely(bus_state_current != bus_state || bus_error_current != bus_error)) {
		sllin_queue_element e;

		__atomic_store_n(&lin->bus_state, bus_state, __ATOMIC_RELEASE);
		__atomic_store_n(&lin->bus_error, bus_error, __ATOMIC_RELEASE);

		e.type = SLLIN_QUEUE_ELEMENT_TYPE_FRAME;
		e.time_stamp_ms = sllin_time_stamp_ms();
		e.frame.id = SLLIN_ID_FLAG_BUS_ERROR_FLAG | SLLIN_ID_FLAG_BUS_STATE_FLAG |
			(lin->bus_state << SLLIN_ID_FLAG_BUS_STATE_SHIFT) | (lin->bus_error << SLLIN_ID_FLAG_BUS_ERROR_SHIFT);
		e.frame.len = 0;

		sllin_lin_task_queue(index, &e);
		sllin_lin_task_notify_isr(index, 1);
	}
}

SLLIN_RAMFUNC static inline void lin_int_bus_sleep(uint8_t index)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;

	sl->sleep_timer->COUNT16.INTFLAG.reg = ~0;

	SLLIN_DEBUG_ISR_ASSERT(0 == sl->sleep_timer->COUNT16.COUNT.reg);

	lin_int_update_bus_status(index, SLLIN_ID_FLAG_BUS_STATE_ASLEEP, SLLIN_ID_FLAG_BUS_ERROR_NONE);
	// LOG("z");
}

SLLIN_RAMFUNC static inline void lin_int_wake_up(uint8_t index)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;

	lin_int_update_bus_status(index, SLLIN_ID_FLAG_BUS_STATE_AWAKE, SLLIN_ID_FLAG_BUS_ERROR_NONE);

	// (re-)start timer DS60001507E-page 1717
	// LOG("y");
	// time _might_ be running
	timer_start_or_restart16(sl->sleep_timer);
}

SLLIN_RAMFUNC static inline void lin_timer_int_data(uint8_t index)
{
	// frame byte timeout

	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;
	Sercom *s = lin->sercom;
	uint8_t const intflag = sl->data_timer->COUNT16.INTFLAG.reg;

	(void)intflag;

	sl->data_timer->COUNT16.INTFLAG.reg = ~0;

	SLLIN_DEBUG_ISR_ASSERT(0 == sl->data_timer->COUNT16.COUNT.reg);
	SLLIN_DEBUG_ISR_ASSERT((intflag & (TC_INTFLAG_OVF | TC_INTFLAG_ERR)) == TC_INTFLAG_OVF);

	// LOG("ch%u frame data timeout\n", index);

	// __atomic_thread_fence(__ATOMIC_ACQUIRE);

	// clear interupt and status
	s->USART.INTFLAG.reg = ~0;
	s->USART.STATUS.reg = ~0;

	// read data to prevent buffer overflow
	(void)s->USART.DATA.reg;

	switch (sl->slave_proto_step) {
	case SERCOM_USART_INTFLAG_RXBRK:
		LOG("ch%u SERCOM_USART_INTFLAG_RXBRK timeout unhandled\n", index);
		break;
	case SLAVE_PROTO_STEP_RX_PID: {
		LOG("ch%u rx timeout pid\n", index);
		bool rx_pin = (PORT->Group[LIN_SERCOM_PORT_GROUP].IN.reg & (1u << lin->rx_pin_index)) != 0;
		if (!rx_pin) {
			lin_int_update_bus_status(index, SLLIN_ID_FLAG_BUS_STATE_ERROR, SLLIN_ID_FLAG_BUS_ERROR_SHORT_TO_GND);
			sleep_timer_cleanup(lin);
		}
	} break;
	default: {
		bool rx_pin = true;

		if (sl->slave_rx_offset) {
			if (sl->slave_rx_offset < 9) {
				// attempt to reconstruct crc
				sl->elem.frame.crc = sl->elem.frame.data[--sl->elem.frame.len];
			} else if (sl->slave_rx_offset > 9) {
				sl->elem.frame.id |= SLLIN_ID_FLAG_LIN_ERROR_TRAILING;
			}

			sl->elem.frame.id |= sl->elem.frame.crc << SLLIN_ID_FLAG_CRC_SHIFT;
		} else {
			rx_pin = (PORT->Group[LIN_SERCOM_PORT_GROUP].IN.reg & (1u << lin->rx_pin_index)) != 0;
		}

		if (rx_pin) {

			sl->elem.time_stamp_ms = sllin_time_stamp_ms();

			SLLIN_DEBUG_ISR_ASSERT(sl->elem.frame.len <= 8);

			sllin_lin_task_queue(index, &sl->elem);
			sllin_lin_task_notify_isr(index, 1);
		} else {
			lin_int_update_bus_status(index, SLLIN_ID_FLAG_BUS_STATE_ERROR, SLLIN_ID_FLAG_BUS_ERROR_SHORT_TO_GND);
			sleep_timer_cleanup(lin);
		}
	} break;
	}


	lin_slave_cleanup(lin);

	// allow next header
	__atomic_store_n(&lin->master.busy, 0, __ATOMIC_RELEASE);

	// LOG("|");
}

SLLIN_RAMFUNC static inline void lin_timer_int_sof(uint8_t index)
{
	// sof timeout

	struct lin *lin = &lins[index];
	Sercom *s = lin->sercom;
	struct master *ma = &lin->master;
	struct slave *sl = &lin->slave;
	bool rx_pin = (PORT->Group[LIN_SERCOM_PORT_GROUP].IN.reg & (1u << lin->rx_pin_index)) != 0;

	(void)ma;
	(void)sl;

	SLLIN_DEBUG_ISR_ASSERT(0 == ma->sof_timer->COUNT16.COUNT.reg);

	sof_timer_cleanup(lin);
	sleep_timer_cleanup(lin);

	SLLIN_DEBUG_ISR_ASSERT(0 == sl->data_timer->COUNT16.COUNT.reg);
	SLLIN_DEBUG_ISR_ASSERT(0 == sl->data_timer->COUNT16.INTFLAG.reg);
	SLLIN_DEBUG_ISR_ASSERT(TC_CTRLBSET_CMD_NONE_Val == sl->data_timer->COUNT16.CTRLBSET.bit.CMD);
	SLLIN_DEBUG_ISR_ASSERT(sl->data_timer->COUNT16.STATUS.bit.STOP);

	// clear interupt and status
	s->USART.INTFLAG.reg = ~0;
	s->USART.STATUS.reg = ~0;

	// read data to prevent buffer overflow
	(void)s->USART.DATA.reg;

	LOG("ch%u sof timeout rx=%u\n", index, rx_pin);

	lin_int_update_bus_status(index, SLLIN_ID_FLAG_BUS_STATE_ERROR, rx_pin ? SLLIN_ID_FLAG_BUS_ERROR_SHORT_TO_VBAT : SLLIN_ID_FLAG_BUS_ERROR_SHORT_TO_GND);


	// allow next header
	__atomic_store_n(&lin->master.busy, 0, __ATOMIC_RELEASE);
	// LOG("*");
}


SLLIN_RAMFUNC static void lin_usart_int_slave(uint8_t index)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;
	Sercom *s = lin->sercom;
	struct sllin_frame_data const *fd = &sllin_frame_data[index];
	uint8_t intflag = s->USART.INTFLAG.reg;
	uint8_t status = s->USART.STATUS.reg;

	// LOG("ch%u INTFLAG=%x\n", index, intflag);

	s->USART.INTFLAG.reg = ~0;
	s->USART.STATUS.reg = ~0;

	// LOG("/");

	if (intflag & SERCOM_USART_INTFLAG_RXBRK) {
		lin_slave_cleanup(lin);

		// LOG("-");
		sof_timer_cleanup(lin);

		lin_int_wake_up(index);

		if (status & SERCOM_USART_STATUS_FERR) {
			status &= ~SERCOM_USART_STATUS_FERR;
			sl->elem.frame.id |= SLLIN_ID_FLAG_LIN_ERROR_FORM;
		}

		sl->slave_proto_step = SLAVE_PROTO_STEP_RX_PID;

		// start data timer for PID
		// LOG("[");
		SLLIN_DEBUG_ISR_ASSERT(0 == sl->data_timer->COUNT16.COUNT.reg);
		SLLIN_DEBUG_ISR_ASSERT(0 == sl->data_timer->COUNT16.INTFLAG.reg);
		SLLIN_DEBUG_ISR_ASSERT(TC_CTRLBSET_CMD_NONE_Val == sl->data_timer->COUNT16.CTRLBSET.bit.CMD);
		SLLIN_DEBUG_ISR_ASSERT(sl->data_timer->COUNT16.STATUS.bit.STOP);
		SLLIN_DEBUG_ISR_ASSERT(!(sl->data_timer->COUNT16.INTFLAG.reg & (TC_INTFLAG_OVF | TC_INTFLAG_ERR)));

		// simple start suffices here since we can be sure the timer is stopped
		sl->data_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
	}

	const uint8_t rx_byte = s->USART.DATA.reg;

	switch (sl->slave_proto_step) {
	case SLAVE_PROTO_STEP_RX_PID:
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			// LOG("ch%u PID=%x\n", index, rx_byte);

			uint8_t const id = sllin_pid_to_id(rx_byte);
			uint8_t const pid = sllin_id_to_pid(id);

			SLLIN_DEBUG_ISR_ASSERT(sl->data_timer->COUNT16.CC[0].reg == sl->timeout_pid_us);
			SLLIN_DEBUG_ISR_ASSERT(!(sl->data_timer->COUNT16.INTFLAG.reg & (TC_INTFLAG_OVF | TC_INTFLAG_ERR)));

			sl->elem.frame.id |= id;

			if (unlikely(pid != rx_byte)) {
				sl->elem.frame.id |= SLLIN_ID_FLAG_LIN_ERROR_PID;
			}

			// sum up length
			sl->elem.frame.len = 0;

			// clear out flag for PID
			intflag &= ~SERCOM_USART_INTFLAG_RXC;

			sl->data_timer->COUNT16.CC[0].reg = sl->timeout_data_us;
			timer_start_or_restart16(sl->data_timer);

			if (sl->slave_frame_enabled[id]) {
				SLLIN_DEBUG_ISR_ASSERT(fd->len[id] >= 0 && fd->len[id] <= 8);

				s->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
				sl->slave_proto_step = SLAVE_PROTO_STEP_TX_DATA;

				goto tx;
			} else {
				sl->slave_proto_step = SLAVE_PROTO_STEP_RX_DATA;
				sl->elem.frame.id |= SLLIN_ID_FLAG_FRAME_FOREIGN;
			}
		}
		break;
	case SLAVE_PROTO_STEP_TX_DATA:
		if (intflag & SERCOM_USART_INTFLAG_DRE) {
			uint8_t len = 0;
			uint8_t tx_byte = 0;
			uint8_t id = 0;
tx:
			id = sl->elem.frame.id & 0x3f;
			len = fd->len[id];

			if (likely(sl->slave_tx_offset < len)) {
				tx_byte = fd->data[id][sl->slave_tx_offset++];
			} else {
				sl->slave_proto_step = SLAVE_PROTO_STEP_RX_DATA;
				tx_byte = fd->crc[id];
				s->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
			}

			s->USART.DATA.reg = tx_byte;

			// LOG("ch%u TX=%x\n", index, tx_byte);
		}

		goto rx;
		break;
	case SLAVE_PROTO_STEP_RX_DATA:
rx:
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			// reset data timer
			SLLIN_DEBUG_ISR_ASSERT(sl->data_timer->COUNT16.CC[0].reg == sl->timeout_data_us);
			SLLIN_DEBUG_ISR_ASSERT(!(sl->data_timer->COUNT16.INTFLAG.reg & (TC_INTFLAG_OVF | TC_INTFLAG_ERR)));

			timer_start_or_restart16(sl->data_timer);

			// LOG("ch%u RX=%x\n", index, rx_byte);
			if (sl->slave_rx_offset < 8) {
				sl->elem.frame.data[sl->elem.frame.len++] = rx_byte;
			} else if (sl->slave_rx_offset == 8) {
				sl->elem.frame.crc = rx_byte;
			} else {
				// too much data
			}

			++sl->slave_rx_offset;
		}
		break;
	}

	if (unlikely(intflag & SERCOM_USART_INTFLAG_ERROR)) {
		if (status & SERCOM_USART_STATUS_ISF) {
			sl->elem.frame.id |= SLLIN_ID_FLAG_LIN_ERROR_SYNC;
		}

		if (status & SERCOM_USART_STATUS_FERR) {
			sl->elem.frame.id |= SLLIN_ID_FLAG_LIN_ERROR_FORM;
		}
	}
}

#define ISR_ATTRS SLLIN_RAMFUNC

ISR_ATTRS void SERCOM1_0_Handler(void)
{
	lin_usart_int_slave(0);
}

ISR_ATTRS void SERCOM1_1_Handler(void)
{
	lin_usart_int_slave(0);
}

ISR_ATTRS void SERCOM1_2_Handler(void)
{
	lin_usart_int_slave(0);
}

ISR_ATTRS void SERCOM1_3_Handler(void)
{
	lin_usart_int_slave(0);
}

ISR_ATTRS void SERCOM0_0_Handler(void)
{
	lin_usart_int_slave(1);
}

ISR_ATTRS void SERCOM0_1_Handler(void)
{
	lin_usart_int_slave(1);
}

ISR_ATTRS void SERCOM0_2_Handler(void)
{
	lin_usart_int_slave(1);
}

ISR_ATTRS void SERCOM0_3_Handler(void)
{
	lin_usart_int_slave(1);
}

ISR_ATTRS void TC0_Handler(void)
{
	lin_timer_int_data(0);
}

ISR_ATTRS void TC1_Handler(void)
{
	lin_timer_int_data(1);
}

ISR_ATTRS void TC2_Handler(void)
{
	lin_int_bus_sleep(0);
}

ISR_ATTRS void TC3_Handler(void)
{
	lin_int_bus_sleep(1);
}

ISR_ATTRS void TC4_Handler(void)
{
	lin_timer_int_sof(0);
}

ISR_ATTRS void TC5_Handler(void)
{
	lin_timer_int_sof(1);
}

#endif // #if D5035_50
