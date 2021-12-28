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

#include <sllin_debug.h>
#include <sllin_board.h>


#include <hal/include/hal_gpio.h>
#include <crc32.h>
#include <mcu.h>
#include <bsp/board.h>

#define SLEEP_TIMER_HZ 3906

enum {
	SLAVE_PROTO_STEP_RX_BREAK = 0,
	SLAVE_PROTO_STEP_RX_PID,
	SLAVE_PROTO_STEP_TX_DATA,
	SLAVE_PROTO_STEP_TX_CRC,
	SLAVE_PROTO_STEP_RX_FOREIGN,

	MASTER_PROTO_STEP_RX_PID = 0,
	MASTER_PROTO_STEP_RX_DATA,
	MASTER_PROTO_STEP_TX_DATA,
	MASTER_PROTO_STEP_FINISHED,
};

typedef void (*sllin_lin_int_callback)(uint8_t index);
SLLIN_RAMFUNC static void lin_usart_int_master(uint8_t index);
SLLIN_RAMFUNC static void lin_usart_int_slave(uint8_t index);
SLLIN_RAMFUNC static void lin_node_timer_int_master(uint8_t index);
SLLIN_RAMFUNC static void lin_node_timer_int_slave(uint8_t index);

struct lin {
	Sercom* const sercom;
	Tc* const node_timer;
	Tc* const sleep_timer;
	sllin_lin_int_callback usart_irq_handler;
	sllin_lin_int_callback node_timer_irq_handler;
	sllin_queue_element elem;
	uint16_t bitrate;
	uint16_t master_crc;
	uint8_t master_tx_data[9];
	uint8_t master_proto_step;
	uint8_t tx_gi; // not an index, uses full range of type
	uint8_t tx_pi; // not an index, uses full range of type
	// uint8_t rx_gi; // not an index, uses full range of type
	uint8_t rx_pi; // not an index, uses full range of type

	uint8_t slave_frame_len[64];
	uint8_t slave_frame_data[64][8];
	uint8_t slave_frame_crc[64];
	uint8_t slave_frame_flags[64];
	uint8_t slave_proto_step;
	uint8_t slave_tx_offset;
	uint8_t slave_rx_offset;
	uint8_t master_busy;
	uint8_t sleep;
};

static struct lin lins[SLLIN_BOARD_LIN_COUNT] = {
	{
		.sercom = SERCOM6,
		.node_timer = TC0, // NOTE: TC0/TC1 don't seem to support different clock speeds, setting one will affect the other.
		.sleep_timer = TC2,
	},
	{
		.sercom = SERCOM7,
		.node_timer = TC1,
		.sleep_timer = TC3,
	},
};

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
	SLLIN_DEBUG_ASSERT(0 == index);
	(void)index;

	board_led_write(on);
}


extern void sllin_board_leds_on_unsafe(void)
{
	board_led_write(true);
}



/** Initializes the clocks from the external 12 MHz crystal
 *
 * The goal of this setup is to preserve the second PLL
 * for the application code while still having a reasonable
 * 48 MHz clock for USB / UART.
 *
 * GCLK0:   CONF_CPU_FREQUENCY (default 120 MHz) from PLL0
 * GCLK1:   unused
 * GCLK2:   12 MHz from XOSC1
 * DFLL48M: closed loop from GLCK2
 * GCLK3:   48 MHz
 */
static inline void clock_init(void)
{
	// reset to default
	GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;

	/* configure for a 12MHz crystal connected to XIN1/XOUT1 */
	OSCCTRL->XOSCCTRL[1].reg =
		OSCCTRL_XOSCCTRL_STARTUP(6) | // 1.953 ms
		OSCCTRL_XOSCCTRL_RUNSTDBY |
		OSCCTRL_XOSCCTRL_ENALC |
		OSCCTRL_XOSCCTRL_IMULT(4) | OSCCTRL_XOSCCTRL_IPTAT(3) | // 8MHz to 16MHz
		OSCCTRL_XOSCCTRL_XTALEN |
		OSCCTRL_XOSCCTRL_ENABLE;
	while(0 == OSCCTRL->STATUS.bit.XOSCRDY1);

	OSCCTRL->Dpll[0].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_DIV(2) | OSCCTRL_DPLLCTRLB_REFCLK_XOSC1; /* 12MHz / 6 = 2Mhz, input = XOSC1 */
	OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x0) | OSCCTRL_DPLLRATIO_LDR((CONF_CPU_FREQUENCY / 1000000 / 2) - 1); /* multiply to get CONF_CPU_FREQUENCY (default = 120MHz) */
	OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_RUNSTDBY | OSCCTRL_DPLLCTRLA_ENABLE;
	while(0 == OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY); /* wait for the PLL0 to be ready */

	/* configure clock-generator 0 to use DPLL0 as source -> GCLK0 is used for the core */
	GCLK->GENCTRL[0].reg =
		GCLK_GENCTRL_DIV(0) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DPLL0 |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL0); /* wait for the synchronization between clock domains to be complete */

	// configure GCLK2 for 12MHz from XOSC1
	GCLK->GENCTRL[2].reg =
		GCLK_GENCTRL_DIV(0) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_XOSC1 |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL2); /* wait for the synchronization between clock domains to be complete */

	 /* setup DFLL48M to use GLCK2 */
	GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN;

	OSCCTRL->DFLLCTRLA.reg = 0;
	while(1 == OSCCTRL->DFLLSYNC.bit.ENABLE);

	OSCCTRL->DFLLCTRLB.reg = OSCCTRL_DFLLCTRLB_MODE | OSCCTRL_DFLLCTRLB_WAITLOCK;
	OSCCTRL->DFLLMUL.bit.MUL = 4; // 4 * 12MHz -> 48MHz

	OSCCTRL->DFLLCTRLA.reg =
		OSCCTRL_DFLLCTRLA_ENABLE |
		OSCCTRL_DFLLCTRLA_RUNSTDBY;
	while(1 == OSCCTRL->DFLLSYNC.bit.ENABLE);

	// setup 48 MHz GCLK3 from DFLL48M
	GCLK->GENCTRL[3].reg =
		GCLK_GENCTRL_DIV(0) |
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_DFLL |
		GCLK_GENCTRL_IDC;
	while(1 == GCLK->SYNCBUSY.bit.GENCTRL3);
}


/* Initialize SERCOM2 for 115200 bps 8N1 using a 48 MHz clock */
static inline void uart_init(void)
{
	gpio_set_pin_function(PIN_PB24, PINMUX_PB24D_SERCOM2_PAD1);
	gpio_set_pin_function(PIN_PB25, PINMUX_PB25D_SERCOM2_PAD0);

	MCLK->APBBMASK.bit.SERCOM2_ = 1;
	GCLK->PCHCTRL[SERCOM2_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN;

	SERCOM2->USART.CTRLA.bit.SWRST = 1; /* reset and disable SERCOM -> enable configuration */
	while (SERCOM2->USART.SYNCBUSY.bit.SWRST);

	SERCOM2->USART.CTRLA.reg  =
		SERCOM_USART_CTRLA_SAMPR(0) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_SAMPA(0) | /* 16x over sampling */
		SERCOM_USART_CTRLA_FORM(0) | /* 0x0 USART frame, 0x1 USART frame with parity, ... */
		SERCOM_USART_CTRLA_DORD | /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) | /* 0x0 USART with external clock, 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(1) | /* SERCOM PAD[1] is used for data reception */
		SERCOM_USART_CTRLA_TXPO(0); /* SERCOM PAD[0] is used for data transmission */

	SERCOM2->USART.CTRLB.reg = /* RXEM = 0 -> receiver disabled, LINCMD = 0 -> normal USART transmission, SFDE = 0 -> start-of-frame detection disabled, SBMODE = 0 -> one stop bit, CHSIZE = 0 -> 8 bits */
		SERCOM_USART_CTRLB_TXEN | /* transmitter enabled */
		SERCOM_USART_CTRLB_RXEN; /* receiver enabled */
	// SERCOM2->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(0) | SERCOM_USART_BAUD_FRAC_BAUD(26); /* 48000000/(16*115200) = 26.041666667 */
	SERCOM2->USART.BAUD.reg = SERCOM_USART_BAUD_BAUD(63019); /* 65536*(1−16*115200/48000000) */

	SERCOM2->USART.CTRLA.bit.ENABLE = 1; /* activate SERCOM */
	while (SERCOM2->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */
}

static inline void lin_init_once(void)
{
	// lin0
	gpio_set_pin_function(PIN_PC04, PINMUX_PC04C_SERCOM6_PAD0);
	gpio_set_pin_function(PIN_PC05, PINMUX_PC05C_SERCOM6_PAD1);

	MCLK->APBDMASK.bit.SERCOM6_ = 1;
	GCLK->PCHCTRL[SERCOM6_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN;

	NVIC_SetPriority(SERCOM6_0_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(SERCOM6_1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(SERCOM6_2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(SERCOM6_3_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);

	NVIC_EnableIRQ(SERCOM6_0_IRQn);
	NVIC_EnableIRQ(SERCOM6_1_IRQn);
	NVIC_EnableIRQ(SERCOM6_2_IRQn);
	NVIC_EnableIRQ(SERCOM6_3_IRQn);

	// lin1
	// gpio_set_pin_function(PIN_PA04, PINMUX_PA04D_SERCOM0_PAD0);
	// gpio_set_pin_function(PIN_PA05, PINMUX_PA05D_SERCOM0_PAD1);

	// MCLK->APBAMASK.bit.SERCOM0_ = 1;
	// GCLK->PCHCTRL[SERCOM0_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN;

	// NVIC_SetPriority(SERCOM0_0_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	// NVIC_SetPriority(SERCOM0_1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	// NVIC_SetPriority(SERCOM0_2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	// NVIC_SetPriority(SERCOM0_3_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);

	// NVIC_EnableIRQ(SERCOM0_0_IRQn);
	// NVIC_EnableIRQ(SERCOM0_1_IRQn);
	// NVIC_EnableIRQ(SERCOM0_2_IRQn);
	// NVIC_EnableIRQ(SERCOM0_3_IRQn);

	gpio_set_pin_function(PIN_PD08, PINMUX_PD08C_SERCOM7_PAD0);
	gpio_set_pin_function(PIN_PD09, PINMUX_PD09C_SERCOM7_PAD1);

	MCLK->APBDMASK.bit.SERCOM7_ = 1;
	GCLK->PCHCTRL[SERCOM7_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN;

	NVIC_SetPriority(SERCOM7_0_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(SERCOM7_1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(SERCOM7_2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(SERCOM7_3_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);

	NVIC_EnableIRQ(SERCOM7_0_IRQn);
	NVIC_EnableIRQ(SERCOM7_1_IRQn);
	NVIC_EnableIRQ(SERCOM7_2_IRQn);
	NVIC_EnableIRQ(SERCOM7_3_IRQn);

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
		GCLK_GENCTRL_DIV(12) |	/* 12Mhz -> 1MHz */
		// GCLK_GENCTRL_DIVSEL |   /* divide by 2^(DIV+1) */
		GCLK_GENCTRL_RUNSTDBY |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC_XOSC1 |
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

	NVIC_SetPriority(TC0_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(TC1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(TC2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(TC3_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(TC0_IRQn);
	NVIC_EnableIRQ(TC1_IRQn);
	NVIC_EnableIRQ(TC2_IRQn);
	NVIC_EnableIRQ(TC3_IRQn);

	for (size_t i = 0; i < TU_ARRAY_SIZE(lins); ++i) {
		struct lin *lin = &lins[i];
		Tc* rtc = lin->node_timer;
		Tc* stc = lin->sleep_timer;

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
	// stop timer
	lin->node_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val;

	lin->master_proto_step = MASTER_PROTO_STEP_FINISHED;

	// no more interrupts
	lin->sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;


	// wait for sync
	while (lin->node_timer->COUNT16.SYNCBUSY.bit.CTRLB);
	// reset value
	lin->node_timer->COUNT16.COUNT.reg = 0;
	// clear interrupt flags
	lin->node_timer->COUNT16.INTFLAG.reg = ~0;


	__atomic_clear(&lin->master_busy, __ATOMIC_RELEASE);
}

SLLIN_RAMFUNC static inline void lin_slave_cleanup(struct lin *lin)
{
	// stop timer
	lin->node_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val;

	lin->slave_proto_step = SLAVE_PROTO_STEP_RX_BREAK;
	lin->slave_tx_offset = 0;
	lin->slave_rx_offset = 0;
	//lin->sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE | SERCOM_USART_INTENCLR_RXC;
	lin->sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;

	// wait for sync
	while (lin->node_timer->COUNT16.SYNCBUSY.bit.CTRLB);
	// reset value
	lin->node_timer->COUNT16.COUNT.reg = 0;
	// clear interrupt flags
	lin->node_timer->COUNT16.INTFLAG.reg = ~0;
}

static inline void usb_init(void)
{
	NVIC_SetPriority(USB_0_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(USB_1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(USB_2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_SetPriority(USB_3_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);


	/* USB clock init
	 * The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
	 * for low speed and full speed operation.
	 */
	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK3_Val | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_AHBMASK_USB_bit(MCLK);
	hri_mclk_set_APBBMASK_USB_bit(MCLK);

	// USB pin init
	gpio_set_pin_direction(PIN_PA24, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PA24, false);
	gpio_set_pin_pull_mode(PIN_PA24, GPIO_PULL_OFF);
	gpio_set_pin_direction(PIN_PA25, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PA25, false);
	gpio_set_pin_pull_mode(PIN_PA25, GPIO_PULL_OFF);

	gpio_set_pin_function(PIN_PA24, PINMUX_PA24H_USB_DM);
	gpio_set_pin_function(PIN_PA25, PINMUX_PA25H_USB_DP);
}

extern void sllin_board_init_begin(void)
{
	clock_init();

	SystemCoreClock = CONF_CPU_FREQUENCY;

	uart_init();
	LOG("CONF_CPU_FREQUENCY=%lu\n", (unsigned long)CONF_CPU_FREQUENCY);

	// LED0 init
	gpio_set_pin_function(PIN_PC18, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(PIN_PC18, GPIO_DIRECTION_OUT);
	board_led_write(0);

	LOG("USB init\n");
	usb_init();

	LOG("timer init\n");
	timer_init();

	LOG("LIN init once\n");
	lin_init_once();

	LOG("Enabling cache\n");
	same5x_enable_cache();

	same5x_init_device_identifier();

	// PC07 is hooked up to relay
	// configure as out and set to low
	PORT->Group[2].DIRSET.reg |= 1ul << 7;
	PORT->Group[2].OUTCLR.reg |= 1ul << 7;

	//PORT->Group[2].DIRSET.reg |= 1ul << 7;
}

extern void sllin_board_init_end(void)
{

}

__attribute__((noreturn)) extern void sllin_board_reset(void)
{
	NVIC_SystemReset();
	__unreachable();
}

extern void sllin_board_lin_init(uint8_t index, sllin_conf *conf)
{
	struct lin *lin = &lins[index];
	Sercom *sercom = lin->sercom;

	// disable timers
	lin->node_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val;
	lin->sleep_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val;

	// disable SERCOM
	sercom->USART.CTRLA.bit.SWRST = 1; /* reset and disable SERCOM -> enable configuration */

	lin->usart_irq_handler = conf->master ? &lin_usart_int_master : &lin_usart_int_slave;
	lin->node_timer_irq_handler = conf->master ? &lin_node_timer_int_master : &lin_node_timer_int_slave;
	lin->bitrate = conf->bitrate;


	// wait for SERCOM to be ready
	while (lin->sercom->USART.SYNCBUSY.bit.SWRST);

	sercom->USART.CTRLA.reg  =
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_SAMPA(0) |
		SERCOM_USART_CTRLA_FORM(conf->master ? 0x2 : 0x4) | /* 0x3 LIN master, 0x4 LIN slave, ... */
		SERCOM_USART_CTRLA_DORD | /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) | /* 0x0 USART with external clock, 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(1) |
		SERCOM_USART_CTRLA_TXPO(0);

	sercom->USART.CTRLB.reg = /* RXEM = 0 -> receiver disabled, LINCMD = 0 -> normal USART transmission, SFDE = 0 -> start-of-frame detection disabled, SBMODE = 0 -> one stop bit, CHSIZE = 0 -> 8 bits */
		SERCOM_USART_CTRLB_TXEN | /* transmitter enabled */
		SERCOM_USART_CTRLB_RXEN | /* receiver enabled */
		// SERCOM_USART_CTRLB_COLDEN; /* collision detection enabled */
		0;

	uint16_t baud = 48000000 / (16 * conf->bitrate);
	uint16_t frac = 48000000 / (2 * conf->bitrate) - 8 * baud;
	sercom->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(frac) | SERCOM_USART_BAUD_FRAC_BAUD(baud);

	sercom->USART.INTENCLR.reg = ~0;

	if (conf->master) {
		// RXC _must_ be enabled, else data get's stuck in register and we get errors
		sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXBRK | SERCOM_USART_INTENSET_ERROR | SERCOM_USART_INTENSET_RXC;

		lin_master_cleanup(lin);
	} else {
		// RXC _must_ be enabled, else data get's stuck in register and we get errors
		sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXBRK | SERCOM_USART_INTENSET_ERROR | SERCOM_USART_INTENSET_RXC;

		lin_slave_cleanup(lin);

		memset(lin->slave_frame_len, 0, sizeof(lin->slave_frame_len));

		lin->node_timer->COUNT16.CC[0].reg =  (14 * UINT32_C(1000000)) / lin->bitrate;
		LOG("ch%u data byte timeout CC=%x [us]\n", index, lin->node_timer->COUNT16.CC[0].reg);
	}

	sercom->USART.CTRLA.bit.ENABLE = 1;
	while (sercom->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */

	// wait for sleep timer sync
	while (lin->sleep_timer->COUNT16.SYNCBUSY.bit.CTRLB);

	// reset value
	lin->sleep_timer->COUNT16.COUNT.reg = 0;

	// clear interrupts
	lin->sleep_timer->COUNT16.INTFLAG.reg = ~0;

	__atomic_store_n(&lin->sleep, 0, __ATOMIC_RELEASE);


	lin->sleep_timer->COUNT16.CC[0].reg = ((conf->sleep_timeout_ms * (uint32_t)SLEEP_TIMER_HZ) + 500) / 1000;
	LOG("ch%u sleep timer CC=%x COUNT=%x\n", index, lin->sleep_timer->COUNT16.CC[0].reg, lin->sleep_timer->COUNT16.COUNT.reg);

	// start timer
	lin->sleep_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
}

SLLIN_RAMFUNC extern bool sllin_board_lin_master_tx(
	uint8_t index,
	uint8_t id,
	uint8_t len,
	uint8_t const *data,
	uint8_t crc,
	uint8_t flags)
{
	struct lin *lin = &lins[index];
	Sercom *s = NULL;
	uint8_t pid = sllin_id_to_pid(id);
	bool start_rx_timeout_timer = false;


	SLLIN_DEBUG_ASSERT(index < TU_ARRAY_SIZE(lins));
	SLLIN_DEBUG_ASSERT(len <= 8);

	if (unlikely(__atomic_load_n(&lin->master_busy, __ATOMIC_ACQUIRE))) {
		return false;
	}

	s = lin->sercom;

	lin->rx_pi = 0;
	lin->tx_gi = 0;
	lin->master_crc = sllin_crc_start();

	if (data) { // frame
		for (unsigned i = 0; i < len; ++i) {
			lin->master_tx_data[i] = data[i];
		}

		lin->master_tx_data[len] = crc;

		lin->tx_pi = len + 1;
		flags |= SLLIN_FRAME_FLAG_MASTER_TX;
	} else { // header only
		if (flags & SLLIN_FRAME_FLAG_ENHANCED_CHECKSUM) {
			lin->master_crc = sllin_crc_update1(lin->master_crc, pid);
		}

		lin->tx_pi = 0;

		// setup no response timeout
		uint32_t T_Header_Nominal = (34 * UINT32_C(1000000)) / lin->bitrate;
		uint32_t T_Response_Nominal = (10 * (len + 1) * UINT32_C(1000000)) / lin->bitrate;
		uint32_t T_Header_Maximum = (14 * T_Header_Nominal) / 10;
		uint32_t T_Response_Maximum = (14 * T_Response_Nominal) / 10;
		SLLIN_DEBUG_ASSERT(T_Header_Maximum + T_Response_Maximum <= 0xffff);

		lin->node_timer->COUNT16.CC[0].reg = T_Header_Maximum + T_Response_Maximum;
		// LOG("ch%u frame timeout CC=%x COUNT=%x\n", index, lin->node_timer->COUNT16.CC[0].reg, lin->node_timer->COUNT16.COUNT.reg);

		// start rx timeout timer
		start_rx_timeout_timer = true;
	}


	lin->elem.type = SLLIN_QUEUE_ELEMENT_TYPE_RX_FRAME;
	lin->elem.lin_frame.id = id;
	lin->elem.lin_frame.len = len;
	lin->elem.lin_frame.flags = flags;
	lin->master_proto_step = MASTER_PROTO_STEP_RX_PID;

	lin->master_busy = 1;

	__atomic_thread_fence(__ATOMIC_RELEASE);

	if (start_rx_timeout_timer) {
		lin->node_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
	}

	// lin->sleep_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
	// LOG("ch%u sleep CC=%x COUNT=%x CTRLB=%x\n", index, lin->sleep_timer->COUNT16.CC[0].reg, lin->sleep_timer->COUNT16.COUNT.reg, lin->sleep_timer->COUNT16.CTRLBSET.reg);


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

	SLLIN_DEBUG_ASSERT(index < TU_ARRAY_SIZE(lins));
	SLLIN_DEBUG_ASSERT(len <= 8);

	if (len) {
		lin->slave_frame_len[id] = len;
		memcpy(lin->slave_frame_data[id], data, len);
		lin->slave_frame_crc[id] = crc;
		lin->slave_frame_flags[id] = flags;
	} else {
		// clear tx frame
		lin->slave_frame_len[id] = 0;
	}

	__atomic_thread_fence(__ATOMIC_RELEASE);
}


SLLIN_RAMFUNC static inline void lin_int_bus_sleep(uint8_t index)
{
	struct lin *lin = &lins[index];
	uint8_t expected = 0;

	lin->sleep_timer->COUNT16.INTFLAG.reg = ~0;

	SLLIN_ISR_ASSERT(0 == lin->sleep_timer->COUNT16.COUNT.reg);

	if (__atomic_compare_exchange_n(&lin->sleep, &expected, 1, false, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)) {
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
	uint8_t expected = 1;

	if (__atomic_compare_exchange_n(&lin->sleep, &expected, 0, false, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)) {
		LOG("ch%u wake up\n", index);

		lin->elem.type = SLLIN_QUEUE_ELEMENT_TYPE_WAKE_UP;
		lin->elem.time_stamp_ms = sllin_time_stamp_ms();

		sllin_lin_task_queue(index, &lin->elem);
		sllin_lin_task_notify_isr(index, 1);
	}

	// re-start timer
	lin->sleep_timer->COUNT16.COUNT.reg = 0;
}

SLLIN_RAMFUNC static void lin_node_timer_int_master(uint8_t index)
{
	// response timeout

	struct lin *lin = &lins[index];

	lin->node_timer->COUNT16.INTFLAG.reg = ~0;

	SLLIN_ISR_ASSERT(0 == lin->node_timer->COUNT16.COUNT.reg);

	// LOG("ch%u no response\n", index);

	__atomic_thread_fence(__ATOMIC_ACQUIRE);

	lin->elem.lin_frame.flags |= SLLIN_FRAME_FLAG_NO_RESPONSE;
	lin->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();

	sllin_lin_task_queue(index, &lin->elem);
	sllin_lin_task_notify_isr(index, 1);

	lin_master_cleanup(lin);
}

SLLIN_RAMFUNC static void lin_node_timer_int_slave(uint8_t index)
{
	// frame byte timeout

	struct lin *lin = &lins[index];

	lin->node_timer->COUNT16.INTFLAG.reg = ~0;

	SLLIN_ISR_ASSERT(0 == lin->node_timer->COUNT16.COUNT.reg);

	// LOG("ch%u frame data timeout\n", index);

	__atomic_thread_fence(__ATOMIC_ACQUIRE);

	SLLIN_ISR_ASSERT(lin->elem.lin_frame.flags & SLLIN_FRAME_FLAG_FOREIGN);

	if (lin->elem.lin_frame.len) {
		if (lin->elem.lin_frame.len < 8) {
			lin->elem.lin_frame.crc = lin->elem.lin_frame.data[--lin->elem.lin_frame.len];
		}
	} else {
		lin->elem.lin_frame.flags |= SLLIN_FRAME_FLAG_NO_RESPONSE;
	}

	lin->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();

	sllin_lin_task_queue(index, &lin->elem);
	sllin_lin_task_notify_isr(index, 1);

	lin_slave_cleanup(lin);
}


SLLIN_RAMFUNC static void lin_usart_int_master(uint8_t index)
{
	struct lin *lin = &lins[index];
	Sercom *s = lin->sercom;

	uint32_t intflag = s->USART.INTFLAG.reg;

	// LOG("ch%u INTFLAG=%x\n", index, intflag);
	// LOG(".");
	s->USART.INTFLAG.reg = ~0;

	__atomic_thread_fence(__ATOMIC_ACQUIRE);

	uint8_t byte = s->USART.DATA.reg;

	if (intflag & SERCOM_USART_INTFLAG_RXBRK) {
		lin_int_wake_up(index);
	}

	switch (lin->master_proto_step) {
	case MASTER_PROTO_STEP_RX_PID: {
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			// LOG("ch%u RX PID\n", index);
			// s->USART.CTRLB.bit.LINCMD = 0x0;

			if (likely(byte == sllin_id_to_pid(lin->elem.lin_frame.id))) {
				if (lin->tx_pi) {
					s->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
					lin->master_proto_step = MASTER_PROTO_STEP_TX_DATA;
					goto tx;
				} else {
					lin->master_proto_step = MASTER_PROTO_STEP_RX_DATA;
				}
			} else {
				lin->elem.lin_frame.flags |= SLLIN_FRAME_FLAG_CRC_ERROR;
				lin->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();
				sllin_lin_task_queue(index, &lin->elem);
				sllin_lin_task_notify_isr(index, 1);
				lin_master_cleanup(lin);
			}
		}
	} break;

	case MASTER_PROTO_STEP_TX_DATA: {
tx:
		if (intflag & SERCOM_USART_INTFLAG_DRE) {
			uint8_t tx_gi = lin->tx_gi;
			uint8_t tx_pi = lin->tx_pi;

			if (tx_pi != tx_gi) {
				uint8_t tx_index = tx_gi;

				byte = lin->master_tx_data[tx_index];

				SLLIN_DEBUG_ISR_ASSERT(tx_gi < sizeof(lin->master_tx_data));
				SLLIN_DEBUG_ISR_ASSERT(tx_pi < sizeof(lin->master_tx_data));
				SLLIN_DEBUG_ISR_ASSERT(tx_gi < tx_pi);

				++lin->tx_gi;
				s->USART.DATA.reg = byte;
				LOG("ch%u TX=%x\n", index, byte);
			} else {
				// done
				lin->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();
				sllin_lin_task_queue(index, &lin->elem);
				sllin_lin_task_notify_isr(index, 1);
				lin_master_cleanup(lin);
			}
		}
	} break;
	case MASTER_PROTO_STEP_RX_DATA: {
		if (intflag & SERCOM_USART_INTFLAG_RXC) {

			if (lin->elem.lin_frame.len == lin->rx_pi) {
				lin->elem.lin_frame.crc = sllin_crc_finalize(lin->master_crc);

				if (likely(lin->elem.lin_frame.crc == byte)) {
					// LOG("ch%u LIN\n", index);
				} else {
					// LOG("ch%u crc mismatch want=%x have=%x data=%x %x %x %x %x %x %x %x\n",
					// 	index, lin->elem.lin_frame.crc, byte,
					// 	lin->elem.lin_frame.data[0], lin->elem.lin_frame.data[1],
					// 	lin->elem.lin_frame.data[2], lin->elem.lin_frame.data[3],
					// 	lin->elem.lin_frame.data[4], lin->elem.lin_frame.data[5],
					// 	lin->elem.lin_frame.data[6], lin->elem.lin_frame.data[7]);

					lin->elem.lin_frame.flags |= SLLIN_FRAME_FLAG_CRC_ERROR;
				}

				lin->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();

				sllin_lin_task_queue(index, &lin->elem);
				sllin_lin_task_notify_isr(index, 1);
				lin_master_cleanup(lin);
			} else {
				// lin->tc->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
				lin->elem.lin_frame.data[lin->rx_pi++] = byte;
				lin->master_crc = sllin_crc_update1(lin->master_crc, byte);
			}
		}
	} break;
	}

	if (unlikely(intflag & SERCOM_USART_INTFLAG_ERROR)) {
		LOG("ch%u status=%x\n", index, s->USART.STATUS.reg);
		s->USART.STATUS.reg = ~0;
		lin_master_cleanup(lin);
	}
}

SLLIN_RAMFUNC static void lin_usart_int_slave(uint8_t index)
{
	struct lin *lin = &lins[index];
	Sercom *s = lin->sercom;

	uint32_t intflag = s->USART.INTFLAG.reg;


	// LOG("ch%u INTFLAG=%x\n", index, intflag);

	s->USART.INTFLAG.reg = ~0;


	// LOG("/");


	__atomic_thread_fence(__ATOMIC_ACQUIRE);

	if (intflag & SERCOM_USART_INTFLAG_RXBRK) {
		lin_slave_cleanup(lin);
		lin->slave_proto_step = SLAVE_PROTO_STEP_RX_PID;

		// LOG("ch%u BREAK\n", index);

		lin_int_wake_up(index);
	}

	const uint8_t rx_byte = s->USART.DATA.reg;

	switch (lin->slave_proto_step) {
	case SLAVE_PROTO_STEP_RX_PID:
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			// LOG("ch%u PID=%x\n", index, byte);

			uint8_t id = sllin_pid_to_id(rx_byte);

			if (likely(sllin_id_to_pid(id) == rx_byte)) {
				uint8_t len = lin->slave_frame_len[id];

				SLLIN_DEBUG_ISR_ASSERT(len <= 8);

				if (len) {
					lin->elem.type = SLLIN_QUEUE_ELEMENT_TYPE_RX_FRAME;
					lin->elem.lin_frame.id = id;
					lin->elem.lin_frame.len = len;
					lin->elem.lin_frame.flags = lin->slave_frame_flags[id];
					s->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
					lin->slave_proto_step = SLAVE_PROTO_STEP_TX_DATA;
					intflag &= ~SERCOM_USART_INTFLAG_RXC;

					goto tx;
				} else {
					lin->elem.type = SLLIN_QUEUE_ELEMENT_TYPE_RX_FRAME;
					lin->elem.lin_frame.id = id;
					lin->elem.lin_frame.len = 0;
					lin->elem.lin_frame.flags = SLLIN_FRAME_FLAG_FOREIGN;
					lin->slave_proto_step = SLAVE_PROTO_STEP_RX_FOREIGN;
					intflag &= ~SERCOM_USART_INTFLAG_RXC;

					// start frame data timer
					lin->node_timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
				}
			} else {
				// bad PID
				lin->elem.type = SLLIN_QUEUE_ELEMENT_TYPE_RX_FRAME;
				lin->elem.lin_frame.id = id;
				lin->elem.lin_frame.flags = SLLIN_FRAME_FLAG_PID_ERROR;
				lin->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();

				sllin_lin_task_queue(index, &lin->elem);
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
			len = lin->slave_frame_len[lin->elem.lin_frame.id];

			if (likely(lin->slave_tx_offset < len)) {
				tx_byte = lin->slave_frame_data[lin->elem.lin_frame.id][lin->slave_tx_offset++];
			} else {
				lin->slave_proto_step = SLAVE_PROTO_STEP_TX_CRC;
				tx_byte = lin->slave_frame_crc[lin->elem.lin_frame.id];
				s->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
			}

			s->USART.DATA.reg = tx_byte;

			// LOG("ch%u TX=%x\n", index, tx_byte);
		}

		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			// LOG("ch%u RX=%x\n", index, rx_byte);
			if (rx_byte != lin->slave_frame_data[lin->elem.lin_frame.id][lin->slave_rx_offset]) {
				LOG("ch%u offset=%u TX!=RX %x %x\n", index, lin->slave_rx_offset, lin->slave_frame_data[lin->elem.lin_frame.id][lin->slave_rx_offset], rx_byte);
				lin->elem.lin_frame.flags |= SLLIN_FRAME_FLAG_CRC_ERROR;
			}

			lin->elem.lin_frame.data[lin->slave_rx_offset++] = rx_byte;
		}
		break;
	case SLAVE_PROTO_STEP_TX_CRC:
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			// LOG("ch%u RX=%x\n", index, rx_byte);
			if (lin->slave_rx_offset < lin->elem.lin_frame.len) {
				if (rx_byte != lin->slave_frame_data[lin->elem.lin_frame.id][lin->slave_rx_offset]) {
					// LOG("ch%u offset=%u TX!=RX %x %x\n", index, lin->slave_rx_offset, lin->slave_frame_data[lin->elem.lin_frame.id][lin->slave_rx_offset], rx_byte);
					lin->elem.lin_frame.flags |= SLLIN_FRAME_FLAG_CRC_ERROR;
				}

				lin->elem.lin_frame.data[lin->slave_rx_offset++] = rx_byte;
			} else {
				if (rx_byte != lin->slave_frame_crc[lin->elem.lin_frame.id]) {
					LOG("ch%u crc TX!=RX %x %x\n", index, lin->slave_frame_crc[lin->elem.lin_frame.id], rx_byte);
					lin->elem.lin_frame.flags |= SLLIN_FRAME_FLAG_CRC_ERROR;
				}

				lin->elem.lin_frame.crc = rx_byte;
				lin->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();

				sllin_lin_task_queue(index, &lin->elem);
				sllin_lin_task_notify_isr(index, 1);
				lin_slave_cleanup(lin);
			}
		}
		break;
	case SLAVE_PROTO_STEP_RX_FOREIGN:
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			// reset data timer
			lin->node_timer->COUNT16.COUNT.reg = 0;


			// LOG("ch%u RX=%x\n", index, rx_byte);
			if (lin->elem.lin_frame.len < 8) {
				lin->elem.lin_frame.data[lin->elem.lin_frame.len++] = rx_byte;
			} else {
				lin->elem.lin_frame.crc = rx_byte;
				// lin->elem.lin_frame.time_stamp_ms = sllin_time_stamp_ms();

				// sllin_lin_task_queue(index, &lin->elem);
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


SLLIN_RAMFUNC void SERCOM6_0_Handler(void)
{
	lins[0].usart_irq_handler(0);
}

SLLIN_RAMFUNC void SERCOM6_1_Handler(void)
{
	lins[0].usart_irq_handler(0);
}

SLLIN_RAMFUNC void SERCOM6_2_Handler(void)
{
	lins[0].usart_irq_handler(0);
}

SLLIN_RAMFUNC void SERCOM6_3_Handler(void)
{
	lins[0].usart_irq_handler(0);
}

SLLIN_RAMFUNC void SERCOM7_0_Handler(void)
{
	lins[1].usart_irq_handler(1);
}

SLLIN_RAMFUNC void SERCOM7_1_Handler(void)
{
	lins[1].usart_irq_handler(1);
}

SLLIN_RAMFUNC void SERCOM7_2_Handler(void)
{
	lins[1].usart_irq_handler(1);
}

SLLIN_RAMFUNC void SERCOM7_3_Handler(void)
{
	lins[1].usart_irq_handler(1);
}

SLLIN_RAMFUNC void TC0_Handler(void)
{
	lins[0].node_timer_irq_handler(0);
}

SLLIN_RAMFUNC void TC1_Handler(void)
{
	lins[1].node_timer_irq_handler(1);
}

SLLIN_RAMFUNC void TC2_Handler(void)
{
	lin_int_bus_sleep(0);
}

SLLIN_RAMFUNC void TC3_Handler(void)
{
	lin_int_bus_sleep(1);
}




#endif // #ifdef SAME54XPLAINEDPRO