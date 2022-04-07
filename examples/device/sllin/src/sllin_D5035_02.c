/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */



#if D5035_02

#include <sllin_board.h>
#include <leds.h>

#include <hal/include/hal_gpio.h>
#include <crc32.h>
#include <mcu.h>
#include <bsp/board.h>



#include <tusb.h>
#include <class/dfu/dfu_rt_device.h>

#define SLEEP_TIMER_HZ 3906
#define MASTER_SLAVE_PIN_GROUP 1
#define BOARD_SERCOM SERCOM2
#define USART_BAURATE      115200
#define CONF_CPU_FREQUENCY 48000000
#define CONF_LIN_UART_FREQUENCY 48000000


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

/* The board runs off of the 48 MHz internal RC oscillator */

enum {
	SLAVE_PROTO_STEP_RX_BREAK = 0,
	SLAVE_PROTO_STEP_RX_PID,
	SLAVE_PROTO_STEP_TX_DATA,
	SLAVE_PROTO_STEP_TX_CRC,
	SLAVE_PROTO_STEP_RX_FOREIGN,

	MASTER_PROTO_STEP_RX_BREAK = 0,
	MASTER_PROTO_STEP_RX_PID,
	MASTER_PROTO_STEP_FINISHED,
};

typedef void (*sllin_lin_int_callback)(uint8_t index);
SLLIN_RAMFUNC static void lin_usart_int_master(uint8_t index);
SLLIN_RAMFUNC static void lin_usart_int_slave(uint8_t index);
SLLIN_RAMFUNC static void lin_node_timer_int(uint8_t index);

struct slave {
	Tc* const node_timer;
	Tc* const sleep_timer;
	sllin_queue_element elem;
	__attribute__ ((aligned(4))) uint8_t slave_frame_data[64][8];
	uint8_t slave_frame_len[64];
	uint8_t slave_frame_crc[64];
	uint8_t slave_frame_flags[64];
	uint8_t slave_proto_step;
	uint8_t slave_tx_offset;
	uint8_t slave_rx_offset;
	uint8_t sleep;
};

struct master {
	uint8_t step;
};

struct lin {
	Sercom* const sercom;
	sllin_lin_int_callback usart_irq_handler;
	struct slave slave;
	struct master master;
	uint8_t master_slave_pin;
};

static struct lin lins[SLLIN_BOARD_LIN_COUNT] = {
	{
		.sercom = SERCOM1,
		.slave.node_timer = TC0, // NOTE: TC0/TC1 don't seem to support different clock speeds, setting one will affect the other.
		.slave.sleep_timer = TC2,
		.master_slave_pin = 4,
	},
	{
		.sercom = SERCOM0,
		.slave.node_timer = TC1,
		.slave.sleep_timer = TC3,
		.master_slave_pin = 9,
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

	BOARD_SERCOM->USART.CTRLA.reg  =
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
	gpio_set_pin_function(PIN_PA00, PINMUX_PA00D_SERCOM1_PAD0);
	gpio_set_pin_function(PIN_PA01, PINMUX_PA01D_SERCOM1_PAD1);

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
	gpio_set_pin_function(PIN_PA04, PINMUX_PA04D_SERCOM0_PAD0);
	gpio_set_pin_function(PIN_PA05, PINMUX_PA05D_SERCOM0_PAD1);

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

	// TC0/1 are connected to the SAME peripheral clock, TC2/3 likewise
	GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;
	GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;
	GCLK->PCHCTRL[TC2_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK5 | GCLK_PCHCTRL_CHEN;
	GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK5 | GCLK_PCHCTRL_CHEN;

	NVIC_SetPriority(TC0_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(TC1_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(TC2_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_SetPriority(TC3_IRQn, SLLIN_ISR_PRIORITY);
	NVIC_EnableIRQ(TC0_IRQn);
	NVIC_EnableIRQ(TC1_IRQn);
	NVIC_EnableIRQ(TC2_IRQn);
	NVIC_EnableIRQ(TC3_IRQn);

	for (size_t i = 0; i < TU_ARRAY_SIZE(lins); ++i) {
		struct lin *lin = &lins[i];
		struct slave *sl = &lin->slave;
		Tc* rtc = sl->node_timer;
		Tc* stc = sl->sleep_timer;

		// reponse timeout
		rtc->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
		while (rtc->COUNT16.SYNCBUSY.bit.SWRST);

		rtc->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

		// generate overflow interrupt
		rtc->COUNT16.INTENSET.reg = TC_INTENSET_OVF;

		// set to max so we don't time out
		rtc->COUNT16.CC[0].reg = 0xffff;

		// enable
		rtc->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV16;

		// stop & oneshot
		rtc->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT | TC_CTRLBSET_CMD_STOP;
		while (rtc->COUNT16.SYNCBUSY.bit.CTRLB);

		// reset to zero
		rtc->COUNT16.COUNT.reg = 0;


		// sleep timeout
		stc->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
		while (stc->COUNT16.SYNCBUSY.bit.SWRST);

		stc->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

		// generate overflow interrupt
		stc->COUNT16.INTENSET.reg = TC_INTENSET_OVF;

		// set to max so we don't time out
		stc->COUNT16.CC[0].reg = 0xffff;

		// enable
		stc->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 |  TC_CTRLA_PRESCALER_DIV256;

		// stop & oneshot
		stc->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_STOP;
		while (stc->COUNT16.SYNCBUSY.bit.CTRLB);

		// reset to zero
		stc->COUNT16.COUNT.reg = 0;
	}
}



SLLIN_RAMFUNC static inline void lin_master_cleanup(struct lin *lin)
{
	SLLIN_DEBUG_ISR_ASSERT(lin);

	__atomic_store_n(&lin->master.step, MASTER_PROTO_STEP_FINISHED, __ATOMIC_RELEASE);
}

SLLIN_RAMFUNC static inline void lin_slave_cleanup(struct lin *lin)
{
	struct slave *sl = &lin->slave;

	// stop timer
	sl->node_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val;

	sl->slave_proto_step = SLAVE_PROTO_STEP_RX_BREAK;
	sl->slave_tx_offset = 0;
	sl->slave_rx_offset = 0;

	lin->sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;

	// wait for sync
	while (sl->node_timer->COUNT16.SYNCBUSY.bit.CTRLB);
	// reset value
	sl->node_timer->COUNT16.COUNT.reg = 0;
	// clear interrupt flags
	sl->node_timer->COUNT16.INTFLAG.reg = ~0;
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

	usb_init();

	// LOG("USB init\n");
	// usb_init();

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

extern void sllin_board_lin_init(uint8_t index, sllin_conf *conf)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;
	Sercom *sercom = lin->sercom;

	// disable timers
	sl->node_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val;
	sl->sleep_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val;

	// disable SERCOM
	sercom->USART.CTRLA.bit.SWRST = 1; /* reset and disable SERCOM -> enable configuration */

	lin->usart_irq_handler = conf->master ? &lin_usart_int_master : &lin_usart_int_slave;


	// wait for SERCOM to be ready
	while (lin->sercom->USART.SYNCBUSY.bit.SWRST);

	sercom->USART.CTRLA.reg  =
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_SAMPA(0) |
		SERCOM_USART_CTRLA_FORM(conf->master ? 0x2 : 0x4) | /* 0x2 LIN master, 0x4 LIN slave, ... */
		SERCOM_USART_CTRLA_DORD | /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) | /* 0x0 USART with external clock, 0x1 USART with internal clock */
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
		lin_master_cleanup(lin);
		PORT->Group[MASTER_SLAVE_PIN_GROUP].OUTSET.reg = 1ul << lin->master_slave_pin;
	} else {
		PORT->Group[MASTER_SLAVE_PIN_GROUP].OUTCLR.reg = 1ul << lin->master_slave_pin;
	}

	lin_slave_cleanup(lin);

	memset(sl->slave_frame_len, 0, sizeof(sl->slave_frame_len));

	sl->node_timer->COUNT16.CC[0].reg =  (14 * UINT32_C(1000000)) / conf->bitrate;
	LOG("ch%u data byte timeout CC=%x [us]\n", index, sl->node_timer->COUNT16.CC[0].reg);

	sercom->USART.CTRLA.bit.ENABLE = 1;
	while (sercom->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */

	// wait for sleep timer sync
	while (sl->sleep_timer->COUNT16.SYNCBUSY.bit.CTRLB);

	// reset value
	sl->sleep_timer->COUNT16.COUNT.reg = 0;

	// clear interrupts
	sl->sleep_timer->COUNT16.INTFLAG.reg = ~0;

	__atomic_store_n(&sl->sleep, 0, __ATOMIC_RELEASE);


	sl->sleep_timer->COUNT16.CC[0].reg = ((conf->sleep_timeout_ms * (uint32_t)SLEEP_TIMER_HZ) + 500) / 1000;
	LOG("ch%u sleep timer CC=%x COUNT=%x\n", index, sl->sleep_timer->COUNT16.CC[0].reg, sl->sleep_timer->COUNT16.COUNT.reg);

	// start timer
	sl->sleep_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
}

SLLIN_RAMFUNC static inline void sllin_board_lin_slave_tx_unsafe(
	struct slave *sl,
	uint8_t id,
	uint8_t len,
	uint8_t const *data,
	uint8_t crc,
	uint8_t flags)
{
	if (len) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
		uint32_t const *word_src = (uint32_t const *)data;
		uint32_t *word_dst = (uint32_t *)&sl->slave_frame_data[id][0];
#pragma GCC diagnostic pop

		word_dst[0] = word_src[0];
		word_dst[1] = word_src[1];

		sl->slave_frame_crc[id] = crc;
		sl->slave_frame_flags[id] = flags;
	}

	__atomic_store_n(&sl->slave_frame_len[id], len, __ATOMIC_RELEASE);
}

SLLIN_RAMFUNC extern bool sllin_board_lin_master_tx(
	uint8_t index,
	uint8_t id,
	uint8_t len,
	uint8_t const *data,
	uint8_t crc,
	uint8_t flags)
{
	struct lin * const lin = &lins[index];
	struct slave * const sl = &lin->slave;
	Sercom * const s = lin->sercom;
	uint8_t pid = sllin_id_to_pid(id);
	uint8_t step = 0;

	SLLIN_DEBUG_ASSERT(index < TU_ARRAY_SIZE(lins));
	SLLIN_DEBUG_ASSERT(len <= 8);
	SLLIN_DEBUG_ASSERT(id < 64);


	step = __atomic_load_n(&lin->master.step, __ATOMIC_ACQUIRE);

	if (unlikely(step != MASTER_PROTO_STEP_FINISHED)) {
		LOG("ch%u master tx busy, step=%u\n", index, step);
		return false;
	}

	if (unlikely(data)) { // store tx frame
		sllin_board_lin_slave_tx_unsafe(sl, id, len, data, crc, flags | SLLIN_FRAME_FLAG_MASTER_TX);
	} else {
		sllin_board_lin_slave_tx_unsafe(sl, id, 0, NULL, 0, 0);
	}

	__atomic_store_n(&lin->master.step, MASTER_PROTO_STEP_RX_BREAK, __ATOMIC_RELEASE);


	s->USART.CTRLB.bit.LINCMD = 0x2;
	s->USART.DATA.reg = pid;


	return true;
}


SLLIN_RAMFUNC extern void sllin_board_lin_slave_tx(
	uint8_t index,
	uint8_t id,
	uint8_t len,
	uint8_t const *data,
	uint8_t crc,
	uint8_t flags)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;

	SLLIN_DEBUG_ASSERT(index < TU_ARRAY_SIZE(lins));
	SLLIN_DEBUG_ASSERT(len <= 8);
	SLLIN_DEBUG_ASSERT(id < 64);

	sllin_board_lin_slave_tx_unsafe(sl, id, len, data, crc, flags);
}


SLLIN_RAMFUNC static inline void lin_int_bus_sleep(uint8_t index)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;
	uint8_t expected = 0;

	sl->sleep_timer->COUNT16.INTFLAG.reg = ~0;

	SLLIN_ISR_ASSERT(0 == sl->sleep_timer->COUNT16.COUNT.reg);

	if (__atomic_compare_exchange_n(&sl->sleep, &expected, 1, false, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)) {
		sllin_queue_element e;

		LOG("ch%u bus sleep\n", index);

		e.type = SLLIN_QUEUE_ELEMENT_TYPE_SLEEP;
		e.time_stamp_ms = sllin_time_stamp_ms();

		sllin_lin_task_queue(index, &e);
		sllin_lin_task_notify_isr(index, 1);
	}
}


SLLIN_RAMFUNC static inline void lin_int_wake_up(uint8_t index)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;
	uint8_t expected = 1;

	if (__atomic_compare_exchange_n(&sl->sleep, &expected, 0, false, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)) {
		sllin_queue_element e;

		LOG("ch%u wake up\n", index);

		e.type = SLLIN_QUEUE_ELEMENT_TYPE_WAKE_UP;
		e.time_stamp_ms = sllin_time_stamp_ms();

		sllin_lin_task_queue(index, &e);
		sllin_lin_task_notify_isr(index, 1);
	}

	// re-start timer
	sl->sleep_timer->COUNT16.COUNT.reg = 0;
}

SLLIN_RAMFUNC static void lin_node_timer_int(uint8_t index)
{
	// frame byte timeout

	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;

	sl->node_timer->COUNT16.INTFLAG.reg = ~0;

	SLLIN_ISR_ASSERT(0 == sl->node_timer->COUNT16.COUNT.reg);

	// LOG("ch%u frame data timeout\n", index);

	__atomic_thread_fence(__ATOMIC_ACQUIRE);

	SLLIN_ISR_ASSERT(sl->elem.lin_frame.flags & SLLIN_FRAME_FLAG_FOREIGN);

	if (sl->elem.lin_frame.len) {
		if (sl->elem.lin_frame.len < 8) {
			sl->elem.lin_frame.crc = sl->elem.lin_frame.data[--sl->elem.lin_frame.len];
		}
	} else {
		sl->elem.lin_frame.flags |= SLLIN_FRAME_FLAG_NO_RESPONSE;
	}

	sl->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();

	sllin_lin_task_queue(index, &sl->elem);
	sllin_lin_task_notify_isr(index, 1);

	lin_slave_cleanup(lin);
}

SLLIN_RAMFUNC static void lin_usart_int_slave_ex(uint8_t index, uint8_t intflag);
SLLIN_RAMFUNC static void lin_usart_int_master(uint8_t index)
{
	struct lin *lin = &lins[index];
	Sercom *s = lin->sercom;

	// LOG(".");

	uint8_t intflag = s->USART.INTFLAG.reg;

	// LOG("ch%u INTFLAG=%x\n", index, intflag);

	s->USART.INTFLAG.reg = ~0;

	uint8_t step = __atomic_load_n(&lin->master.step, __ATOMIC_ACQUIRE);

	switch (step) {
	case MASTER_PROTO_STEP_RX_BREAK:
		if (intflag & SERCOM_USART_INTFLAG_RXBRK) {
			__atomic_store_n(&lin->master.step, MASTER_PROTO_STEP_RX_PID, __ATOMIC_RELAXED);
		}
		break;
	case MASTER_PROTO_STEP_RX_PID:
		if (intflag & SERCOM_USART_INTFLAG_DRE) {
			intflag |= SERCOM_USART_INTFLAG_RXC; // fake PID
			lin_master_cleanup(lin);
		}
		break;
	default:
		break;
	}

	lin_usart_int_slave_ex(index, intflag);
}

SLLIN_RAMFUNC static void lin_usart_int_slave(uint8_t index)
{
	struct lin *lin = &lins[index];
	Sercom *s = lin->sercom;
	uint8_t intflag = s->USART.INTFLAG.reg;

	// LOG("ch%u INTFLAG=%x\n", index, intflag);

	s->USART.INTFLAG.reg = ~0;

	// LOG("/");

	__atomic_thread_fence(__ATOMIC_ACQUIRE);

	lin_usart_int_slave_ex(index, intflag);
}

SLLIN_RAMFUNC static void lin_usart_int_slave_ex(uint8_t index, uint8_t intflag)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;
	Sercom *s = lin->sercom;

	if (intflag & SERCOM_USART_INTFLAG_RXBRK) {
		lin_slave_cleanup(lin);
		sl->slave_proto_step = SLAVE_PROTO_STEP_RX_PID;

		// LOG("ch%u BREAK\n", index);

		lin_int_wake_up(index);
	}

	const uint8_t rx_byte = s->USART.DATA.reg;

	switch (sl->slave_proto_step) {
	case SLAVE_PROTO_STEP_RX_PID:
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			// LOG("ch%u PID=%x\n", index, rx_byte);

			uint8_t id = sllin_pid_to_id(rx_byte);

			if (likely(sllin_id_to_pid(id) == rx_byte)) {
				uint8_t len = sl->slave_frame_len[id];

				SLLIN_DEBUG_ISR_ASSERT(len <= 8);

				if (len) {
					sl->elem.type = SLLIN_QUEUE_ELEMENT_TYPE_RX_FRAME;
					sl->elem.lin_frame.id = id;
					sl->elem.lin_frame.len = len;
					sl->elem.lin_frame.flags = sl->slave_frame_flags[id];
					s->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
					sl->slave_proto_step = SLAVE_PROTO_STEP_TX_DATA;
					intflag &= ~SERCOM_USART_INTFLAG_RXC;

					goto tx;
				} else {
					sl->elem.type = SLLIN_QUEUE_ELEMENT_TYPE_RX_FRAME;
					sl->elem.lin_frame.id = id;
					sl->elem.lin_frame.len = 0;
					sl->elem.lin_frame.flags = SLLIN_FRAME_FLAG_FOREIGN;
					sl->slave_proto_step = SLAVE_PROTO_STEP_RX_FOREIGN;
					intflag &= ~SERCOM_USART_INTFLAG_RXC;

					// start frame data timer
					sl->node_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
				}
			} else {
				// bad PID
				sl->elem.type = SLLIN_QUEUE_ELEMENT_TYPE_RX_FRAME;
				sl->elem.lin_frame.id = id;
				sl->elem.lin_frame.flags = SLLIN_FRAME_FLAG_PID_ERROR;
				sl->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();

				sllin_lin_task_queue(index, &sl->elem);
				sllin_lin_task_notify_isr(index, 1);

				lin_slave_cleanup(lin);
			}
		}
		break;
	case SLAVE_PROTO_STEP_TX_DATA:
		if (intflag & SERCOM_USART_INTFLAG_DRE) {
			uint8_t len = 0;
			uint8_t tx_byte = 0;
tx:
			len = sl->slave_frame_len[sl->elem.lin_frame.id];

			if (likely(sl->slave_tx_offset < len)) {
				tx_byte = sl->slave_frame_data[sl->elem.lin_frame.id][sl->slave_tx_offset++];
			} else {
				sl->slave_proto_step = SLAVE_PROTO_STEP_TX_CRC;
				tx_byte = sl->slave_frame_crc[sl->elem.lin_frame.id];
				s->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
			}

			s->USART.DATA.reg = tx_byte;

			// LOG("ch%u TX=%x\n", index, tx_byte);
		}

		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			// LOG("ch%u RX=%x\n", index, rx_byte);
			if (rx_byte != sl->slave_frame_data[sl->elem.lin_frame.id][sl->slave_rx_offset]) {
				LOG("ch%u offset=%u TX!=RX %x %x\n", index, sl->slave_rx_offset, sl->slave_frame_data[sl->elem.lin_frame.id][sl->slave_rx_offset], rx_byte);
				sl->elem.lin_frame.flags |= SLLIN_FRAME_FLAG_CRC_ERROR;
			}

			sl->elem.lin_frame.data[sl->slave_rx_offset++] = rx_byte;
		}
		break;
	case SLAVE_PROTO_STEP_TX_CRC:
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			// LOG("ch%u RX=%x\n", index, rx_byte);
			if (sl->slave_rx_offset < sl->elem.lin_frame.len) {
				if (rx_byte != sl->slave_frame_data[sl->elem.lin_frame.id][sl->slave_rx_offset]) {
					// LOG("ch%u offset=%u TX!=RX %x %x\n", index, sl->slave_rx_offset, sl->slave_frame_data[sl->elem.lin_frame.id][sl->slave_rx_offset], rx_byte);
					sl->elem.lin_frame.flags |= SLLIN_FRAME_FLAG_CRC_ERROR;
				}

				sl->elem.lin_frame.data[sl->slave_rx_offset++] = rx_byte;
			} else {
				if (rx_byte != sl->slave_frame_crc[sl->elem.lin_frame.id]) {
					LOG("ch%u crc TX!=RX %x %x\n", index, sl->slave_frame_crc[sl->elem.lin_frame.id], rx_byte);
					sl->elem.lin_frame.flags |= SLLIN_FRAME_FLAG_CRC_ERROR;
				}

				sl->elem.lin_frame.crc = rx_byte;
				sl->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();

				// LOG("ch%u rx id=%x flags=%x\n", index, sl->elem.lin_frame.id, sl->elem.lin_frame.flags);

				sllin_lin_task_queue(index, &sl->elem);
				sllin_lin_task_notify_isr(index, 1);
				lin_slave_cleanup(lin);
			}
		}
		break;
	case SLAVE_PROTO_STEP_RX_FOREIGN:
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			// reset data timer
			sl->node_timer->COUNT16.COUNT.reg = 0;


			// LOG("ch%u RX=%x\n", index, rx_byte);
			if (sl->elem.lin_frame.len < 8) {
				sl->elem.lin_frame.data[sl->elem.lin_frame.len++] = rx_byte;
			} else {
				sl->elem.lin_frame.crc = rx_byte;
				// sl->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();

				// sllin_lin_task_queue(index, &sl->elem);
				// sllin_lin_task_notify_isr(index, 1);
				// lin_slave_cleanup(lin);
			}
		}
		break;
	}


	if (intflag & SERCOM_USART_INTFLAG_ERROR) {
		LOG("ch%u status=%x\n", index, s->USART.STATUS.reg);
		s->USART.STATUS.reg = ~0;
		lin_slave_cleanup(lin);
	}
}

SLLIN_RAMFUNC void SERCOM1_0_Handler(void)
{
	lins[0].usart_irq_handler(0);
}

SLLIN_RAMFUNC void SERCOM1_1_Handler(void)
{
	lins[0].usart_irq_handler(0);
}

SLLIN_RAMFUNC void SERCOM1_2_Handler(void)
{
	lins[0].usart_irq_handler(0);
}

SLLIN_RAMFUNC void SERCOM1_3_Handler(void)
{
	lins[0].usart_irq_handler(0);
}

SLLIN_RAMFUNC void SERCOM0_0_Handler(void)
{
	lins[1].usart_irq_handler(1);
}

SLLIN_RAMFUNC void SERCOM0_1_Handler(void)
{
	lins[1].usart_irq_handler(1);
}

SLLIN_RAMFUNC void SERCOM0_2_Handler(void)
{
	lins[1].usart_irq_handler(1);
}

SLLIN_RAMFUNC void SERCOM0_3_Handler(void)
{
	lins[1].usart_irq_handler(1);
}

SLLIN_RAMFUNC void TC0_Handler(void)
{
	lin_node_timer_int(0);
}

SLLIN_RAMFUNC void TC1_Handler(void)
{
	lin_node_timer_int(1);
}

SLLIN_RAMFUNC void TC2_Handler(void)
{
	lin_int_bus_sleep(0);
}

SLLIN_RAMFUNC void TC3_Handler(void)
{
	lin_int_bus_sleep(1);
}

#endif // #if D5035_02