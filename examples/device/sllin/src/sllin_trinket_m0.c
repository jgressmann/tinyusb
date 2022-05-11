/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include <sllin_board.h>

#if TRINKET_M0


#include <leds.h>

#include <hal/include/hal_gpio.h>
#include <crc32.h>
#include <bsp/board.h>



#include <tusb.h>

#define LIN_SERCOM_PORT_GROUP 0
#define MASTER_SLAVE_PIN_GROUP 1
#define BOARD_SERCOM SERCOM0	// TinyUSB board setup
#define USART_BAURATE      115200
#define CONF_CPU_FREQUENCY 48000000
#define CONF_LIN_UART_FREQUENCY 48000000



/* The board runs off of the 48 MHz internal RC oscillator
 *
 * @115200 debug prints KILL LIN timing assumptions!
*/



enum {
	SLAVE_PROTO_STEP_RX_BREAK = 0,
	SLAVE_PROTO_STEP_RX_PID,
	SLAVE_PROTO_STEP_TX_DATA,
	SLAVE_PROTO_STEP_RX_DATA,

	MASTER_PROTO_STEP_TX_BREAK = 0,
	MASTER_PROTO_STEP_TX_SYNC,
	MASTER_PROTO_STEP_FINISHED,

	MASTER_PROTO_TX_BREAK_ONLY_PID = 0xff, // any non-valid PID will do

	TIMER_TYPE_SLEEP = 0,
	TIMER_TYPE_BREAK,
	TIMER_TYPE_HIGH,
	TIMER_TYPE_SOF,
	TIMER_TYPE_DATA,
};


SLLIN_RAMFUNC static void lin_usart_int(uint8_t index);
SLLIN_RAMFUNC static void lin_timer_int(uint8_t index);



struct slave {
	sllin_queue_element elem;
	uint8_t slave_frame_enabled[64];
	uint32_t sleep_timeout_us;
	uint32_t sleep_elapsed_us;
	uint16_t data_timeout_us;
	uint8_t slave_proto_step;
	uint8_t slave_tx_offset;
	uint8_t slave_rx_offset;
};

struct master {
	uint16_t break_timeout_us;
	uint16_t high_timeout_us;
	uint8_t busy;
	uint8_t proto_step;
	uint8_t pid;
};

struct lin {
	Sercom* const sercom;
	Tc* const timer;
	struct slave slave;
	struct master master;
	uint16_t sof_timeout_us;
	uint16_t baud;
	uint8_t const rx_pin_index;
	uint8_t const master_slave_pin;  // set for master, clear for slave
	uint8_t const led_status_green;
	uint8_t const led_status_red;
	uint8_t bus_state;
	uint8_t bus_error;
	uint8_t timer_type;
};

static struct lin lins[SLLIN_BOARD_LIN_COUNT] = {
	{
		.sercom = SERCOM2,
		.timer = TC3,
		.rx_pin_index = 8,
		.master_slave_pin = 2,
		.led_status_green = SLLIN_BOARD_DOTSTAR_GREEN,
		.led_status_red = SLLIN_BOARD_DOTSTAR_RED,
	}
};

struct led {
	uint8_t pin;
};

#define LED_STATIC_INITIALIZER(name, pin) \
	{ pin }


static const struct led leds[] = {
	LED_STATIC_INITIALIZER("debug_default", PIN_PA10), // board led
};

static struct {
	// uint32_t sof;
	uint8_t a;
	uint8_t b;
	uint8_t g;
	uint8_t r;
	// uint32_t eof;
} dotstar = {
	// .sof = 0,
	.a = 0xe1,
	.b = 0,
	.g = 0,
	.r = 0,
	// .eof = 0xffffffff,
};

SLLIN_RAMFUNC static inline void dotstar_update(void)
{
	Sercom * const s = SERCOM1;

	s->SPI.DATA.reg = 0x00;
	while (!s->SPI.INTFLAG.bit.DRE);
	s->SPI.DATA.reg = 0x00;
	while (!s->SPI.INTFLAG.bit.DRE);
	s->SPI.DATA.reg = 0x00;
	while (!s->SPI.INTFLAG.bit.DRE);
	s->SPI.DATA.reg = 0x00;
	while (!s->SPI.INTFLAG.bit.DRE);
	s->SPI.DATA.reg = dotstar.a;
	while (!s->SPI.INTFLAG.bit.DRE);
	s->SPI.DATA.reg = dotstar.b;
	while (!s->SPI.INTFLAG.bit.DRE);
	s->SPI.DATA.reg = dotstar.g;
	while (!s->SPI.INTFLAG.bit.DRE);
	s->SPI.DATA.reg = dotstar.r;
	while (!s->SPI.INTFLAG.bit.DRE);
	s->SPI.DATA.reg = 0xff;
	while (!s->SPI.INTFLAG.bit.DRE);
	s->SPI.DATA.reg = 0xff;
	while (!s->SPI.INTFLAG.bit.DRE);
	s->SPI.DATA.reg = 0xff;
	while (!s->SPI.INTFLAG.bit.DRE);
	s->SPI.DATA.reg = 0xff;
	while (!s->SPI.INTFLAG.bit.DRE);
}

static inline void leds_init(void)
{
	Sercom * const s = SERCOM1;

	// the one true LED
	PORT->Group[0].DIRSET.reg = PIN_PA10;

	// Drive APA-102-2022 with SPI on SERCOM1

	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_DRVSTR |
		PORT_WRCONFIG_PINMASK(0x0003) | /* PA00, PA1 */
		PORT_WRCONFIG_PMUXEN;

	PM->APBCMASK.bit.SERCOM1_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK2 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_SERCOM1_CORE;


	s->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_SWRST; /* disable SERCOM -> enable config */
	while(s->SPI.SYNCBUSY.bit.SWRST); /* wait for SERCOM to be ready */

	s->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD(7); // 1 MHz

	s->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_DRE | SERCOM_SPI_INTENSET_ERROR;



	s->SPI.CTRLA.reg =
		SERCOM_SPI_CTRLA_ENABLE |
		SERCOM_SPI_CTRLA_DIPO(2) | // input on PAD2 PAD0 = DO, PAD1 = SCK
		SERCOM_SPI_CTRLA_MODE(3); // master mode
	while(s->SPI.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */

	LOG("dotstar init\n");

	dotstar_update();
}


static uint32_t device_identifier;

extern uint32_t sllin_board_identifier(void)
{
	return device_identifier;
}


static void sam_init_device_identifier(void)
{
	uint32_t serial_number[4] = {
		*(uint32_t const *)0x0080A00C,
		*(uint32_t const *)0x0080A040,
		*(uint32_t const *)0x0080A044,
		*(uint32_t const *)0x0080A048
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

extern void sllin_board_led_set(uint8_t index, bool on)
{
	switch (index) {
	case SLLIN_BOARD_DEBUG_DEFAULT:
		gpio_set_pin_level(leds[index].pin, on);
		break;
	case SLLIN_BOARD_DOTSTAR_RED:
		dotstar.r = on * 0xff;
		// LOG("r=%u\n", on > 0);
		dotstar_update();
		break;
	case SLLIN_BOARD_DOTSTAR_GREEN:
		dotstar.g = on * 0xff;
		// LOG("g=%u\n", on > 0);
		dotstar_update();
		break;
	case SLLIN_BOARD_DOTSTAR_BLUE:
		dotstar.b = on * 0xff;
		// LOG("b=%u\n", on > 0);
		dotstar_update();
		break;
	}
}

extern void sllin_board_leds_on_unsafe(void)
{
	for (size_t i = 0; i < TU_ARRAY_SIZE(leds); ++i) {
		gpio_set_pin_level(leds[i].pin, 1);
	}

	dotstar.r = 0xff;
	dotstar.g = 0xff;
	dotstar.b = 0xff;
	dotstar_update();
}

static inline void uart_init(void)
{
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_DRVSTR |
		PORT_WRCONFIG_PINMASK(0x00c0) | /* PA06, PA7 */
		PORT_WRCONFIG_PMUXEN;

	PM->APBCMASK.bit.SERCOM0_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK0 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_SERCOM0_CORE;

	BOARD_SERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST; /* disable SERCOM -> enable config */
	while(BOARD_SERCOM->USART.SYNCBUSY.bit.SWRST); /* wait for SERCOM to be ready */

	BOARD_SERCOM->USART.CTRLA.reg =
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_DORD |     /* LSB first */
		SERCOM_USART_CTRLA_MODE(1) |  /* 0x0 USART with external clock, 0x1 USART with internal clock */
		SERCOM_USART_CTRLA_RXPO(3) |  /* SERCOM PAD[3] is used for data reception */
		SERCOM_USART_CTRLA_TXPO(1);   /* SERCOM PAD[2] is used for data transmission */

	BOARD_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN; /* transmitter enabled */
	uint16_t baud = CONF_CPU_FREQUENCY / (16 * USART_BAURATE);
	uint16_t frac = CONF_CPU_FREQUENCY / (2 * USART_BAURATE) - 8 * baud;
	BOARD_SERCOM->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(frac) | SERCOM_USART_BAUD_FRAC_BAUD(baud);

	BOARD_SERCOM->USART.CTRLA.bit.ENABLE = 1; /* activate SERCOM */
	while(BOARD_SERCOM->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */
}


static inline void clock_init(void)
{
	// Setup DFLL
	// 1. turn off DFLL
	SYSCTRL->DFLLCTRL.reg = 0;
	while (SYSCTRL->PCLKSR.bit.DFLLRDY);


	// 2. load calibration

	// Load coarse value from NVM, Atmel-42181G–SAM-D21_Datasheet–09/2015 p.32
	// 63:58 DFLL48M COARSE CALDFLL48M Coarse calibration value. Should be written to DFLLVAL register.
	// 73:64 DFLL48M FINE CALDFLL48M Fine calibration value. Should be written to DFLLVAL register.
	uint32_t const NVM_CAL_AREA = 0x806020;
	uint8_t const coarse = ((*(uint8_t*)(NVM_CAL_AREA + 7)) >> 2) & 0x3f;
	uint16_t const fine = (*(uint16_t*)(NVM_CAL_AREA + 8)) & 0x3ff;


	// LOG("DFLL coarse=%x fine=%x\n", coarse, fine);

	SYSCTRL->DFLLVAL.reg =
		SYSCTRL_DFLLVAL_COARSE(coarse) |
		SYSCTRL_DFLLVAL_FINE(fine);


	// 3. Setup target frequency from 1 [ms] USB SOF
	// USB clock sync, Atmel-42181G–SAM-D21_Datasheet–09/2015 p.156
	SYSCTRL->DFLLMUL.reg =
		SYSCTRL_DFLLMUL_MUL(0xBB80) |
		SYSCTRL_DFLLMUL_FSTEP(1) |
		SYSCTRL_DFLLMUL_CSTEP(1) |
		0;


	// 4. start DFLL
	SYSCTRL->DFLLCTRL.reg =
		SYSCTRL_DFLLCTRL_ENABLE |
		SYSCTRL_DFLLCTRL_USBCRM |
		SYSCTRL_DFLLCTRL_MODE |
		SYSCTRL_DFLLCTRL_CCDIS |
		0;
	while (!SYSCTRL->PCLKSR.bit.DFLLRDY);


	// Must increase to 2 for 48 MHz
	NVMCTRL->CTRLB.bit.RWS = 2;

	// Setup CPU to use 48 MHz
	GCLK->GENCTRL.reg =
		GCLK_GENCTRL_SRC_DFLL48M |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_IDC |
		GCLK_GENCTRL_ID(0);
	while (GCLK->STATUS.bit.SYNCBUSY);

	SystemCoreClock = CONF_CPU_FREQUENCY;

	// Setup 16 MHz on GCLK2
	GCLK->GENDIV.reg =
		GCLK_GENDIV_DIV(3) |
		GCLK_GENDIV_ID(2);

	GCLK->GENCTRL.reg =
		GCLK_GENCTRL_SRC_DFLL48M |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_IDC |
		GCLK_GENCTRL_ID(2);
	while (GCLK->STATUS.bit.SYNCBUSY);


	// Setup 1 MHz on GCLK3
	GCLK->GENDIV.reg =
		GCLK_GENDIV_DIV(48) |
		GCLK_GENDIV_ID(3);

	GCLK->GENCTRL.reg =
		GCLK_GENCTRL_SRC_DFLL48M |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_IDC |
		GCLK_GENCTRL_ID(3);
	while (GCLK->STATUS.bit.SYNCBUSY);
}


static inline void lin_init_once(void)
{
	// // lin0
	PORT->Group[0].WRCONFIG.reg =
		PORT_WRCONFIG_WRPINCFG |
		PORT_WRCONFIG_WRPMUX |
		PORT_WRCONFIG_PMUX(3) |    /* function D */
		PORT_WRCONFIG_DRVSTR |
		PORT_WRCONFIG_PINMASK(0x0300) | /* PA08, PA9 */
		PORT_WRCONFIG_PMUXEN;

	PM->APBCMASK.bit.SERCOM2_ = 1;
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK0 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_SERCOM2_CORE;


	NVIC_SetPriority(SERCOM2_IRQn, SLLIN_ISR_PRIORITY);

	NVIC_EnableIRQ(SERCOM2_IRQn);

	// // master / slave pin
	// // PB04 lin0
	// // PB09 lin1
	// PORT->Group[MASTER_SLAVE_PIN_GROUP].DIRSET.reg = (1ul << 4) | (1ul << 9);

}

static void timer_init(void)
{
	// GCLK->GENCTRL[4].reg =
	// 	GCLK_GENCTRL_DIV(3) |	/* 48Mhz -> 16MHz */
	// 	GCLK_GENCTRL_RUNSTDBY |
	// 	GCLK_GENCTRL_GENEN |
	// 	GCLK_GENCTRL_SRC_DFLL |
	// 	GCLK_GENCTRL_IDC;
	// while(1 == GCLK->SYNCBUSY.bit.GENCTRL4); /* wait for the synchronization between clock domains to be complete */

	// GCLK->GENCTRL[5].reg =
	// 	GCLK_GENCTRL_DIV(48) |	/* 48Mhz -> 1MHz */
	// 	// GCLK_GENCTRL_DIVSEL |   /* divide by 2^(DIV+1) */
	// 	GCLK_GENCTRL_RUNSTDBY |
	// 	GCLK_GENCTRL_GENEN |
	// 	GCLK_GENCTRL_SRC_DFLL |
	// 	GCLK_GENCTRL_IDC;
	// while(1 == GCLK->SYNCBUSY.bit.GENCTRL5); /* wait for the synchronization between clock domains to be complete */


	// MCLK->APBAMASK.bit.TC0_ = 1;
	// MCLK->APBAMASK.bit.TC1_ = 1;
	// MCLK->APBBMASK.bit.TC2_ = 1;
	// MCLK->APBBMASK.bit.TC3_ = 1;
	// MCLK->APBCMASK.bit.TC4_ = 1;
	// MCLK->APBCMASK.bit.TC5_ = 1;

	// // TC0/1 are connected to the SAME peripheral clock, TC2/3 likewise
	// GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;
	// GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;
	// GCLK->PCHCTRL[TC2_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK5 | GCLK_PCHCTRL_CHEN;
	// GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK5 | GCLK_PCHCTRL_CHEN;
	// GCLK->PCHCTRL[TC4_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;
	// GCLK->PCHCTRL[TC5_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;

	// NVIC_SetPriority(TC0_IRQn, SLLIN_ISR_PRIORITY);
	// NVIC_SetPriority(TC1_IRQn, SLLIN_ISR_PRIORITY);
	// NVIC_SetPriority(TC2_IRQn, SLLIN_ISR_PRIORITY);
	// NVIC_SetPriority(TC3_IRQn, SLLIN_ISR_PRIORITY);
	// NVIC_SetPriority(TC4_IRQn, SLLIN_ISR_PRIORITY);
	// NVIC_SetPriority(TC5_IRQn, SLLIN_ISR_PRIORITY);
	// NVIC_EnableIRQ(TC0_IRQn);
	// NVIC_EnableIRQ(TC1_IRQn);
	// NVIC_EnableIRQ(TC2_IRQn);
	// NVIC_EnableIRQ(TC3_IRQn);
	// NVIC_EnableIRQ(TC4_IRQn);
	// NVIC_EnableIRQ(TC5_IRQn);

	// for (size_t i = 0; i < TU_ARRAY_SIZE(lins); ++i) {
	// 	struct lin *lin = &lins[i];
	// 	struct slave *sl = &lin->slave;
	// 	struct master *ma = &lin->master;
	// 	Tc* data_timer = sl->data_timer;
	// 	Tc* stc = sl->sleep_timer;
	// 	Tc* sof_timer = ma->sof_timer;

	// 	// MASTER
	// 	// sof timeout
	// 	sof_timer->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	// 	while (sof_timer->COUNT16.SYNCBUSY.bit.SWRST);

	// 	sof_timer->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

	// 	// generate overflow interrupt
	// 	sof_timer->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_ERR;

	// 	// set to max so we don't time out
	// 	sof_timer->COUNT16.CC[0].reg = 0xffff;

	// 	// enable
	// 	sof_timer->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV16;

	// 	// stop & oneshot
	// 	sof_timer->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT | TC_CTRLBSET_LUPD | TC_CTRLBSET_CMD_STOP;
	// 	while (sof_timer->COUNT16.SYNCBUSY.bit.CTRLB);

	// 	// reset to zero
	// 	sof_timer->COUNT16.COUNT.reg = 0;

	// 	// SLAVE

	// 	// reponse timeout
	// 	data_timer->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	// 	while (data_timer->COUNT16.SYNCBUSY.bit.SWRST);

	// 	data_timer->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

	// 	// generate overflow interrupt
	// 	data_timer->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_ERR;

	// 	// set to max so we don't time out
	// 	data_timer->COUNT16.CC[0].reg = 0xffff;

	// 	// enable
	// 	data_timer->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV16;

	// 	// stop & oneshot
	// 	data_timer->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT | TC_CTRLBSET_LUPD | TC_CTRLBSET_CMD_STOP;
	// 	// data_timer->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_STOP;
	// 	while (data_timer->COUNT16.SYNCBUSY.bit.CTRLB);

	// 	// reset to zero
	// 	data_timer->COUNT16.COUNT.reg = 0;


	// 	// sleep timeout
	// 	stc->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	// 	while (stc->COUNT16.SYNCBUSY.bit.SWRST);

	// 	stc->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

	// 	// generate overflow interrupt
	// 	stc->COUNT16.INTENSET.reg = TC_INTENSET_OVF | TC_INTENSET_ERR;

	// 	// set to max so we don't time out
	// 	stc->COUNT16.CC[0].reg = 0xffff;

	// 	// enable
	// 	stc->COUNT16.CTRLA.reg = TC_CTRLA_ENABLE | TC_CTRLA_MODE_COUNT16 |  TC_CTRLA_PRESCALER_DIV256;

	// 	// stop & oneshot
	// 	stc->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT | TC_CTRLBSET_LUPD | TC_CTRLBSET_CMD_STOP;
	// 	while (stc->COUNT16.SYNCBUSY.bit.CTRLB);

	// 	// reset to zero
	// 	stc->COUNT16.COUNT.reg = 0;
	// }
}




#define timer_cleanup_begin(tc) do { (tc)->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_STOP_Val; } while (0)

SLLIN_RAMFUNC static inline void timer_cleanup_end(Tc* timer)
{
	// wait for sync
	while (timer->COUNT16.STATUS.bit.SYNCBUSY);
	// reset value
	timer->COUNT16.COUNT.reg = 0;
	// clear interrupt flags
	timer->COUNT16.INTFLAG.reg = ~0;
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
#define timer_start_or_restart_begin(tc) timer_cleanup_begin(tc)
#define timer_start_or_restart_end(tc) \
	do { \
		(tc)->COUNT16.COUNT.reg = 0; \
		(tc)->COUNT16.INTFLAG.reg = ~0; \
		while ((tc)->COUNT16.STATUS.bit.SYNCBUSY); \
		(tc)->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val; \
	} while (0)

#define timer_start_or_restart(tc) \
	do { \
		timer_start_or_restart_begin(tc); \
		timer_start_or_restart_end(tc); \
	} while (0)


#define sof_start_or_restart_begin(lin) \
	do { \
		timer_start_or_restart_begin(lin->timer); \
		lin->timer->COUNT16.CC[0].reg = lin->sof_timeout_us; \
		lin->timer_type = TIMER_TYPE_SOF; \
	} while (0)

#define sof_start_or_restart_end(lin) timer_start_or_restart_end(lin->timer)


#define break_start_or_restart_begin(lin) \
	do { \
		timer_start_or_restart_begin(lin->timer); \
		lin->timer->COUNT16.CC[0].reg = lin->master.break_timeout_us; \
		lin->timer_type = TIMER_TYPE_BREAK; \
	} while (0)

#define break_start_or_restart_end(lin) timer_start_or_restart_end(lin->timer)


#define sof_start_or_restart(lin) \
	do { \
		sof_start_or_restart_begin(lin); \
		sof_start_or_restart_end(lin); \
	} while (0)


//

#define sleep_start(lin) \
	do { \
		(lin)->slave.sleep_elapsed_us = 0; \
		(lin)->timer_type = TIMER_TYPE_SLEEP; \
		(lin)->timer->COUNT16.CC[0].reg = 0xffff; \
		(lin)->timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val; \
	} while (0)

SLLIN_RAMFUNC static inline void lin_cleanup_master_tx(struct lin *lin)
{
	struct slave *sl = &lin->slave;


	sl->slave_proto_step = SLAVE_PROTO_STEP_RX_BREAK;
	sl->slave_tx_offset = 0;
	sl->slave_rx_offset = 0;
	sl->elem.frame.id = 0;
	sl->elem.frame.len = 0;
}

SLLIN_RAMFUNC static inline void lin_cleanup_full(struct lin *lin)
{
	Sercom *const s = lin->sercom;

	lin_cleanup_master_tx(lin);

	s->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
}

static inline void usb_init(void)
{
	NVIC_SetPriority(USB_IRQn, SLLIN_ISR_PRIORITY);

	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_GEN_GCLK0 |
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_ID_USB;

	// gpio_set_pin_direction(PIN_PA24, GPIO_DIRECTION_OUT);
	// gpio_set_pin_level(PIN_PA24, false);
	// gpio_set_pin_pull_mode(PIN_PA24, GPIO_PULL_OFF);
	// gpio_set_pin_direction(PIN_PA25, GPIO_DIRECTION_OUT);
	// gpio_set_pin_level(PIN_PA25, false);
	// gpio_set_pin_pull_mode(PIN_PA25, GPIO_PULL_OFF);

	gpio_set_pin_function(PIN_PA24, PINMUX_PA24G_USB_DM);
	gpio_set_pin_function(PIN_PA25, PINMUX_PA25G_USB_DP);
}

extern void sllin_board_init_begin(void)
{
	// Atmel-42181G–SAM-D21_Datasheet–09/2015, p. 367
	// setup deterministic wait states
	NVMCTRL->CTRLB.bit.READMODE = NVMCTRL_CTRLB_READMODE_DETERMINISTIC_Val;
	// NVMCTRL->CTRLB.bit.RWS = 2;

	// Don't divide peripheral clock
	PM->CPUSEL.reg = PM_CPUSEL_CPUDIV(0);
	PM->APBASEL.reg = PM_APBASEL_APBADIV(0);
	PM->APBBSEL.reg = PM_APBBSEL_APBBDIV(0);
	PM->APBCSEL.reg = PM_APBCSEL_APBCDIV(0);


	clock_init();
	uart_init();

	leds_init();

	sam_init_device_identifier();

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
	Sercom *sercom = lin->sercom;



	for (size_t i = 0; i < TU_ARRAY_SIZE(sl->slave_frame_enabled); ++i) {
		sl->slave_frame_enabled[i] = 0;
	}


	lin->master.pid = MASTER_PROTO_TX_BREAK_ONLY_PID;

	__atomic_store_n(&lin->bus_state, SLLIN_ID_FLAG_BUS_STATE_AWAKE, __ATOMIC_RELAXED);
	__atomic_store_n(&lin->bus_error, SLLIN_ID_FLAG_BUS_ERROR_NONE, __ATOMIC_RELAXED);

	// disable timer
	timer_cleanup_begin(lin->timer);

	// disable SERCOM
	sercom->USART.CTRLA.bit.SWRST = 1; /* reset and disable SERCOM -> enable configuration */

	// wait for SERCOM to be ready
	while (lin->sercom->USART.SYNCBUSY.bit.SWRST);

	sercom->USART.CTRLA.reg  =
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_SAMPA(0) |
		SERCOM_USART_CTRLA_FORM(0x4) | /* break & auto baud */
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
	LOG("ch%u data baud=%x frac=%x\n", index, baud, frac);
	lin->baud = sercom->USART.BAUD.reg;

	// clear interrupts
	sercom->USART.INTENCLR.reg = ~0;

	// RXC _must_ be enabled, else data get's stuck in register and we get errors
	sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXBRK | SERCOM_USART_INTENSET_ERROR | SERCOM_USART_INTENSET_RXC;


	if (conf->master) {
		PORT->Group[MASTER_SLAVE_PIN_GROUP].OUTSET.reg = 1ul << lin->master_slave_pin;
	} else {
		PORT->Group[MASTER_SLAVE_PIN_GROUP].OUTCLR.reg = 1ul << lin->master_slave_pin;
	}

	lin_cleanup_full(lin);
	lin->master.busy = 0;

	sl->data_timeout_us = (14 * 10 * UINT32_C(1000000)) / (conf->bitrate * UINT32_C(10));
	LOG("ch%u data byte timeout %x [us]\n", index, sl->data_timeout_us);

	lin->sof_timeout_us = (14 * 34 * UINT32_C(1000000)) / (conf->bitrate * UINT32_C(10));
	LOG("ch%u SOF timeout %x [us]\n", index, lin->sof_timeout_us);

	lin->master.break_timeout_us = (13 * 10 * UINT32_C(1000000)) / (conf->bitrate * UINT32_C(10));
	LOG("ch%u break timeout %x [us]\n", index, lin->master.break_timeout_us);

	lin->master.high_timeout_us = (1 * 10 * UINT32_C(1000000)) / (conf->bitrate * UINT32_C(10));
	LOG("ch%u high timeout %x [us]\n", index, lin->master.high_timeout_us);



	sercom->USART.CTRLA.bit.ENABLE = 1;
	while (sercom->USART.SYNCBUSY.reg); /* wait for SERCOM to be ready */

	timer_cleanup_end(lin->timer);

	sl->sleep_timeout_us = conf->sleep_timeout_ms * UINT32_C(1000);

	sleep_start(lin);
}

SLLIN_RAMFUNC static bool sllin_board_lin_master_tx(uint8_t index, uint8_t pid)
{
	struct lin * const lin = &lins[index];
	Sercom * const s = lin->sercom;
	uint8_t busy = 0;

	SLLIN_DEBUG_ASSERT(index < TU_ARRAY_SIZE(lins));


	busy = __atomic_load_n(&lin->master.busy, __ATOMIC_ACQUIRE);

	if (unlikely(busy)) {
		LOG("ch%u master busy\n", index);
		return false;
	}

	// Disable sercom, see below.
	s->USART.CTRLA.bit.ENABLE = 0;

	break_start_or_restart_begin(lin);

	__atomic_store_n(&lin->master.busy, 1, __ATOMIC_RELAXED);
	lin->master.pid = pid;
	lin->master.proto_step = MASTER_PROTO_STEP_TX_BREAK;

	/* Because the unit is in auto baud mode
	 * the baud rate register gets constantly updated.
	 * When sending the header it is thus imperative
	 * that we reset the baud rate to the proper value
	 * prior to sending sync.
	 */
	while (s->USART.SYNCBUSY.reg);
	s->USART.BAUD.reg = lin->baud;
	s->USART.CTRLA.bit.ENABLE = 1;
	while (s->USART.SYNCBUSY.reg);

	PORT->Group[LIN_SERCOM_PORT_GROUP].PINCFG[lin->rx_pin_index-1].reg = 0;

	// (re)start timer, could be running bc/ some fiddled with the LIN wire and BREAK was detected
	// LOG("+");

	break_start_or_restart_end(lin);

	return true;
}

SLLIN_RAMFUNC extern bool sllin_board_lin_master_break(uint8_t index)
{
	return sllin_board_lin_master_tx(index, MASTER_PROTO_TX_BREAK_ONLY_PID);
}

SLLIN_RAMFUNC extern bool sllin_board_lin_master_request(uint8_t index, uint8_t id)
{
	SLLIN_DEBUG_ASSERT(id < 64);

	return sllin_board_lin_master_tx(index, sllin_id_to_pid(id));
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

SLLIN_RAMFUNC static inline bool lin_int_update_bus_status(uint8_t index, uint8_t bus_state, uint8_t bus_error)
{
	struct lin *lin = &lins[index];
	uint8_t bus_state_current = __atomic_load_n(&lin->bus_state, __ATOMIC_RELAXED);
	uint8_t bus_error_current = __atomic_load_n(&lin->bus_error, __ATOMIC_RELAXED);

	if (unlikely(bus_state_current != bus_state || bus_error_current != bus_error)) {
		sllin_queue_element e;

		__atomic_store_n(&lin->bus_state, bus_state, __ATOMIC_RELAXED);
		__atomic_store_n(&lin->bus_error, bus_error, __ATOMIC_RELAXED);

		e.type = SLLIN_QUEUE_ELEMENT_TYPE_FRAME;
		e.time_stamp_ms = sllin_time_stamp_ms();
		e.frame.id = SLLIN_ID_FLAG_BUS_ERROR_FLAG | SLLIN_ID_FLAG_BUS_STATE_FLAG |
			(lin->bus_state << SLLIN_ID_FLAG_BUS_STATE_SHIFT) | (lin->bus_error << SLLIN_ID_FLAG_BUS_ERROR_SHIFT);
		e.frame.len = 0;

		sllin_lin_task_queue(index, &e);
		sllin_lin_task_notify_isr(index, 1);

		return true;
	}

	return false;
}

SLLIN_RAMFUNC static inline void usart_clear(Sercom *s)
{
	// clear interupt and status
	s->USART.INTFLAG.reg = ~0;
	s->USART.STATUS.reg = ~0;

	// read data to prevent buffer overflow
	(void)s->USART.DATA.reg;
}

SLLIN_RAMFUNC static void on_data_timeout(uint8_t index)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;
	Sercom *s = lin->sercom;


	// LOG("ch%u frame data timeout\n", index);
	usart_clear(s);
	// s->USART.CTRLA.bit.ENABLE = 0;
	// while (s->USART.SYNCBUSY.reg);
	// s->USART.CTRLA.bit.ENABLE = 1;


	switch (sl->slave_proto_step) {
	case SERCOM_USART_INTFLAG_RXBRK:
		LOG("ch%u SERCOM_USART_INTFLAG_RXBRK timeout unhandled\n", index);
		break;
	case SLAVE_PROTO_STEP_RX_PID: {
		LOG("ch%u rx timeout pid\n", index);
		bool rx_pin = (PORT->Group[LIN_SERCOM_PORT_GROUP].IN.reg & (1u << lin->rx_pin_index)) != 0;
		if (rx_pin) {
			sleep_start(lin);
		} else {
			lin_int_update_bus_status(index, SLLIN_ID_FLAG_BUS_STATE_ERROR, SLLIN_ID_FLAG_BUS_ERROR_SHORT_TO_GND);
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

			sleep_start(lin);
		} else {
			lin_int_update_bus_status(index, SLLIN_ID_FLAG_BUS_STATE_ERROR, SLLIN_ID_FLAG_BUS_ERROR_SHORT_TO_GND);
		}
	} break;
	}

	lin_cleanup_full(lin);

	// allow next header
	__atomic_store_n(&lin->master.busy, 0, __ATOMIC_RELEASE);

	// LOG("|");
}

SLLIN_RAMFUNC static void lin_timer_int(uint8_t index)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;
	struct master *ma = &lin->master;
	Sercom *s = lin->sercom;
	uint8_t const intflag = lin->timer->COUNT16.INTFLAG.reg;


	(void)intflag;

	lin->timer->COUNT16.INTFLAG.reg = ~0;

	SLLIN_DEBUG_ISR_ASSERT(0 == lin->timer->COUNT16.COUNT.reg);
	SLLIN_DEBUG_ISR_ASSERT(lin->timer->COUNT16.STATUS.bit.STOP);
	SLLIN_DEBUG_ISR_ASSERT((intflag & (TC_INTFLAG_OVF | TC_INTFLAG_ERR)) == TC_INTFLAG_OVF);

	switch (lin->timer_type) {
	case TIMER_TYPE_DATA:
		on_data_timeout(index);
		break;
	case TIMER_TYPE_SLEEP:
		sl->sleep_elapsed_us += 0x10000;

		if (likely(sl->sleep_elapsed_us < sl->sleep_timeout_us)) {
			SLLIN_DEBUG_ISR_ASSERT(lin->timer->COUNT16.STATUS.bit.STOP);
			SLLIN_DEBUG_ISR_ASSERT(!(lin->timer->COUNT16.INTFLAG.reg & (TC_INTFLAG_OVF | TC_INTFLAG_ERR)));
			lin->timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
		} else {
			if (lin_int_update_bus_status(index, SLLIN_ID_FLAG_BUS_STATE_ASLEEP, SLLIN_ID_FLAG_BUS_ERROR_NONE)) {
				LOG("ch%u asleep\n", index);
			}
			// LOG("z");
		}
		break;
	case TIMER_TYPE_SOF: {
		bool rx_pin = (PORT->Group[LIN_SERCOM_PORT_GROUP].IN.reg & (1u << lin->rx_pin_index)) != 0;

		// clean up master part
		ma->proto_step = MASTER_PROTO_STEP_FINISHED;
		s->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;

		LOG("ch%u sof timeout rx=%u\n", index, rx_pin);

		lin_int_update_bus_status(index, SLLIN_ID_FLAG_BUS_STATE_ERROR, rx_pin ? SLLIN_ID_FLAG_BUS_ERROR_SHORT_TO_VBAT : SLLIN_ID_FLAG_BUS_ERROR_SHORT_TO_GND);

		// allow next header
		__atomic_store_n(&lin->master.busy, 0, __ATOMIC_RELEASE);
		// LOG("*");
	} break;
	case TIMER_TYPE_BREAK: {
		// drive pin through UART
		PORT->Group[LIN_SERCOM_PORT_GROUP].PINCFG[lin->rx_pin_index-1].reg = PORT_PINCFG_PMUXEN;
		// LOG("break\n");


		if (unlikely(ma->pid == MASTER_PROTO_TX_BREAK_ONLY_PID)) {
			ma->proto_step = MASTER_PROTO_STEP_FINISHED;
			// start sof timer
			lin->timer->COUNT16.CC[0].reg = lin->sof_timeout_us;
			lin->timer_type = TIMER_TYPE_SOF;
			lin->timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
		} else {
			// start high timer
			lin->timer->COUNT16.CC[0].reg = ma->high_timeout_us;
			lin->timer_type = TIMER_TYPE_HIGH;
			lin->timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
		}
	} break;
	case TIMER_TYPE_HIGH: {
		ma->proto_step = MASTER_PROTO_STEP_TX_SYNC;
		s->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
		s->USART.DATA.reg = 0x55;

		lin->timer->COUNT16.CC[0].reg = lin->sof_timeout_us - ma->high_timeout_us;
		lin->timer_type = TIMER_TYPE_SOF;
		lin->timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;
	} break;
	default:
		LOG("ch%u unhandled timer type %u\n", index, lin->timer_type);
		SLLIN_DEBUG_ISR_ASSERT(false);
		break;
	}
}

SLLIN_RAMFUNC static void lin_usart_int(uint8_t index)
{
	struct lin *lin = &lins[index];
	struct slave *sl = &lin->slave;
	struct master *ma = &lin->master;
	Sercom *s = lin->sercom;
	struct sllin_frame_data const *fd = &sllin_frame_data[index];
	uint8_t intflag = s->USART.INTFLAG.reg;
	uint8_t status = s->USART.STATUS.reg;

	// LOG("ch%u INTFLAG=%x\n", index, intflag);

	s->USART.INTFLAG.reg = ~0;
	s->USART.STATUS.reg = ~0;

	// LOG("/");

	switch (ma->proto_step) {
	case MASTER_PROTO_STEP_TX_SYNC:
		if (intflag & SERCOM_USART_INTFLAG_DRE) {
			intflag &= ~SERCOM_USART_INTFLAG_DRE;

			s->USART.DATA.reg = ma->pid;
			s->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
			ma->proto_step = MASTER_PROTO_STEP_FINISHED;

			// LOG("tx pid=%x\n", ma->pid);
		}
		break;
	}

	if (intflag & SERCOM_USART_INTFLAG_RXBRK) {
		if (ma->proto_step != MASTER_PROTO_STEP_FINISHED) {
			// wait for pull down timer to expire before pulling up
			lin_cleanup_master_tx(lin);
		} else {
			// Restart the timer in case the BREAK wasn't sent by this device.
			// Technically the SOF timeout is too long but if we are lenient here,
			// we don't have to reconfigure the timer.
			sof_start_or_restart_begin(lin);
			lin_cleanup_full(lin);
		}

		// LOG("-");

		sl->slave_proto_step = SLAVE_PROTO_STEP_RX_PID;

		if (ma->proto_step != MASTER_PROTO_STEP_FINISHED) {

		} else {
			sof_start_or_restart_end(lin);
		}
	}

	const uint8_t rx_byte = s->USART.DATA.reg;

	switch (sl->slave_proto_step) {
	case SLAVE_PROTO_STEP_RX_PID:
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			uint8_t const id = sllin_pid_to_id(rx_byte);
			uint8_t const pid = sllin_id_to_pid(id);

			timer_cleanup_begin(lin->timer);

			if (lin_int_update_bus_status(index, SLLIN_ID_FLAG_BUS_STATE_AWAKE, SLLIN_ID_FLAG_BUS_ERROR_NONE)) {
				LOG("ch%u awake\n", index);
			}

			// LOG("ch%u PID=%x\n", index, rx_byte);

			sl->elem.frame.id |= id;

			if (unlikely(pid != rx_byte)) {
				sl->elem.frame.id |= SLLIN_ID_FLAG_LIN_ERROR_PID;
			}

			// sum up length
			sl->elem.frame.len = 0;

			// clear out flag for PID
			intflag &= ~SERCOM_USART_INTFLAG_RXC;

			// setup data timer
			lin->timer->COUNT16.CC[0].reg = sl->data_timeout_us;
			lin->timer_type = TIMER_TYPE_DATA;
			timer_cleanup_end(lin->timer);

			// here we know the timer is stopped
			lin->timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;

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
			SLLIN_DEBUG_ISR_ASSERT(!(lin->timer->COUNT16.INTFLAG.reg & (TC_INTFLAG_OVF | TC_INTFLAG_ERR)));

			timer_start_or_restart_begin(lin->timer);

			// LOG("ch%u RX=%x\n", index, rx_byte);
			if (sl->slave_rx_offset < 8) {
				sl->elem.frame.data[sl->elem.frame.len++] = rx_byte;
			} else if (sl->slave_rx_offset == 8) {
				sl->elem.frame.crc = rx_byte;
			} else {
				// too much data
			}

			++sl->slave_rx_offset;

			timer_start_or_restart_end(lin->timer);
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

#define ISR_ATTRS SLLIN_RAMFUNC __attribute__((naked))

ISR_ATTRS void SERCOM2_Handler(void)
{
	lin_usart_int(0);
}

ISR_ATTRS void TC3_Handler(void)
{
	lin_timer_int(0);
}

#endif // #if TRINKET_M0
