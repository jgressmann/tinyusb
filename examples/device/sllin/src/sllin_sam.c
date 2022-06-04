/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include <sllin_board.h>

#if TRINKET_M0 || D5035_50


#include <leds.h>
#include <crc32.h>
#include <bsp/board.h>



#include <tusb.h>

#define sam_uart_rx_toggle(lin) PORT->Group[SAM_UART_TX_PORT_PIN_MUX >> 5].OUTTGL.reg = UINT32_C(1) << (SAM_UART_TX_PORT_PIN_MUX & 0x1f)


void sam_lin_init_once(void)
{
	for (uint8_t i = 0; i < TU_ARRAY_SIZE(sam_lins); ++i) {
		struct sam_lin *lin = &sam_lins[i];
		struct slave *sl = &lin->slave;

		// set queue element type
		sl->elem.type = SLLIN_QUEUE_ELEMENT_TYPE_FRAME;

		// free RX PIN
		PORT->Group[SAM_UART_TX_PORT_PIN_MUX >> 5].PINCFG[SAM_UART_TX_PORT_PIN_MUX & 0x1f].reg = 0;
		// configure as out, set to zero
		PORT->Group[SAM_UART_TX_PORT_PIN_MUX >> 5].DIRSET.reg = UINT32_C(1) << (SAM_UART_TX_PORT_PIN_MUX & 0x1f);
		PORT->Group[SAM_UART_TX_PORT_PIN_MUX >> 5].OUTCLR.reg = UINT32_C(1) << (SAM_UART_TX_PORT_PIN_MUX & 0x1f);
	}
}

uint32_t sam_init_device_identifier(uint32_t const serial_number[4])
{
	uint32_t device_identifier = 0;
	uint32_t big_endian_serial[4];
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
	big_endian_serial[0] = __builtin_bswap32(serial_number[0]);
	big_endian_serial[1] = __builtin_bswap32(serial_number[1]);
	big_endian_serial[2] = __builtin_bswap32(serial_number[2]);
	big_endian_serial[3] = __builtin_bswap32(serial_number[3]);
#else
	big_endian_serial[0] = serial_number[0];
	big_endian_serial[1] = serial_number[1];
	big_endian_serial[2] = serial_number[2];
	big_endian_serial[3] = serial_number[3];
#endif

	error = crc32f((uint32_t)big_endian_serial, 16, CRC32E_FLAG_UNLOCK, &device_identifier);
	if (unlikely(error)) {
		device_identifier = big_endian_serial[0];
		LOG("ERROR: failed to compute CRC32: %d. Using fallback device identifier\n", error);
	}

#if SLLIN_DEBUG
	memset(serial_buffer, '0', 8);
	chars = usnprintf(hex_buffer, sizeof(hex_buffer), "%x", device_identifier);
	memcpy(&serial_buffer[8-chars], hex_buffer, chars);
	serial_buffer[8] = 0;
	LOG("device identifier %s\n", serial_buffer);
#endif

	return device_identifier;
}



__attribute__((noreturn)) extern void sllin_board_reset(void)
{
	NVIC_SystemReset();
	__unreachable();
}

extern void sllin_board_lin_uninit(uint8_t index)
{
	struct sam_lin *lin = &sam_lins[index];
	Sercom *sercom = lin->sercom;

	// disable SERCOM
	sercom->USART.CTRLA.bit.SWRST = 1;
}

extern void sllin_board_lin_init(uint8_t index, sllin_conf *conf)
{
	struct sam_lin *lin = &sam_lins[index];
	struct slave *sl = &lin->slave;
	Sercom *sercom = lin->sercom;



	for (size_t i = 0; i < TU_ARRAY_SIZE(sl->slave_frame_enabled); ++i) {
		sl->slave_frame_enabled[i] = 0;
	}


	lin->master.pid = MASTER_PROTO_TX_BREAK_ONLY_PID;

	__atomic_store_n(&lin->bus_state, SLLIN_ID_FLAG_BUS_STATE_AWAKE, __ATOMIC_RELAXED);
	__atomic_store_n(&lin->bus_error, SLLIN_ID_FLAG_BUS_ERROR_NONE, __ATOMIC_RELAXED);

	// disable timer
	sam_timer_cleanup_begin(lin);

	// disable SERCOM
	sercom->USART.CTRLA.bit.SWRST = 1; /* reset and disable SERCOM -> enable configuration */

	// wait for SERCOM to be ready
	while (lin->sercom->USART.SYNCBUSY.bit.SWRST);

	sercom->USART.CTRLA.reg  =
		SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
		SERCOM_USART_CTRLA_SAMPA(0) |
#if SAM_CONF_AUTOBAUD
		SERCOM_USART_CTRLA_FORM(0x4) | /* break & auto baud */
#else
		SERCOM_USART_CTRLA_FORM(0x0) | /* normal uart w/o parity */
#endif
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

	uint16_t baud = SAM_CONF_LIN_UART_FREQUENCY / (16 * conf->bitrate);
	uint16_t frac = SAM_CONF_LIN_UART_FREQUENCY / (2 * conf->bitrate) - 8 * baud;
	sercom->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(frac) | SERCOM_USART_BAUD_FRAC_BAUD(baud);
	LOG("ch%u data baud=%x frac=%x\n", index, baud, frac);
	lin->baud = sercom->USART.BAUD.reg;

	// clear interrupts
	sercom->USART.INTENCLR.reg = ~0;

	// RXC _must_ be enabled, else data get's stuck in register and we get errors
	sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXBRK | SERCOM_USART_INTENSET_ERROR | SERCOM_USART_INTENSET_RXC;


	if (conf->master) {
		PORT->Group[lin->master_slave_port_pin_mux >> 5].OUTSET.reg = UINT32_C(1) << (lin->master_slave_port_pin_mux & 0x1f);
	} else {
		PORT->Group[lin->master_slave_port_pin_mux >> 5].OUTCLR.reg = UINT32_C(1) << (lin->master_slave_port_pin_mux & 0x1f);
	}

	lin_cleanup_full(lin, SLAVE_PROTO_STEP_RX_BREAK);
	lin->master.busy = 0;

	sl->data_timeout_us = (14 * 10 * UINT32_C(1000000)) / (conf->bitrate * UINT32_C(10));
	LOG("ch%u data byte timeout %xh [us]\n", index, sl->data_timeout_us);

	lin->sof_timeout_us = (14 * 34 * UINT32_C(1000000)) / (conf->bitrate * UINT32_C(10));
	LOG("ch%u SOF timeout %xh [us]\n", index, lin->sof_timeout_us);

	lin->master.break_timeout_us = (10 * 13 * UINT32_C(1000000)) / (conf->bitrate * UINT32_C(10));
	LOG("ch%u break timeout %xh [us]\n", index, lin->master.break_timeout_us);
#if !SAM_CONF_AUTOBAUD
	lin->break_rem_timeout_us = (10 * 3 * UINT32_C(1000000)) / (conf->bitrate * UINT32_C(10));
	LOG("ch%u break rem timeout %xh [us]\n", index, lin->break_rem_timeout_us);
#endif
	lin->master.high_timeout_us = (10 * UINT32_C(1000000)) / (conf->bitrate * UINT32_C(10));
	LOG("ch%u high timeout %xh [us]\n", index, lin->master.high_timeout_us);



	sercom->USART.CTRLA.bit.ENABLE = 1;
	while (sercom->USART.SYNCBUSY.reg); /* wait for SERCOM to be ready */

	sam_timer_cleanup_end(lin);

	sl->sleep_timeout_us = conf->sleep_timeout_ms * UINT32_C(1000);

	sleep_start(lin);
}

SLLIN_RAMFUNC static bool sllin_board_lin_master_tx(uint8_t index, uint8_t pid)
{
	struct sam_lin * const lin = &sam_lins[index];
#if SAM_CONF_AUTOBAUD
	Sercom * const s = lin->sercom;
#endif
	uint8_t busy = 0;
	uint8_t const sercom_rx_pin_group = lin->rx_port_pin_mux >> 5;
	uint8_t const sercom_rx_pin = lin->rx_port_pin_mux & 0x1f;

	SLLIN_DEBUG_ASSERT(index < TU_ARRAY_SIZE(sam_lins));


	busy = __atomic_load_n(&lin->master.busy, __ATOMIC_ACQUIRE);

	if (unlikely(busy)) {
		LOG("ch%u master busy\n", index);
		return false;
	}

#if SAM_CONF_AUTOBAUD
	// Disable sercom, see below.
	s->USART.CTRLA.bit.ENABLE = 0;
#endif
	break_start_or_restart_begin(lin);

	__atomic_store_n(&lin->master.busy, 1, __ATOMIC_RELAXED);
	lin->master.pid = pid;
	lin->master.proto_step = MASTER_PROTO_STEP_TX_BREAK;

#if SAM_CONF_AUTOBAUD
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
#endif

	PORT->Group[sercom_rx_pin_group].PINCFG[sercom_rx_pin-1].reg = 0;

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
	struct sam_lin *lin = &sam_lins[index];
	struct slave *sl = &lin->slave;

	SLLIN_DEBUG_ASSERT(index < TU_ARRAY_SIZE(sam_lins));
	SLLIN_DEBUG_ASSERT(id < 64);

	__atomic_store_n(&sl->slave_frame_enabled[id], respond, __ATOMIC_RELEASE);
}

SLLIN_RAMFUNC extern void sllin_board_led_lin_status_set(uint8_t index, int status)
{
	struct sam_lin *lin = &sam_lins[index];

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
	struct sam_lin *lin = &sam_lins[index];
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

	sam_usart_clear_pending();
}

SLLIN_RAMFUNC static void on_data_timeout(uint8_t index)
{
	struct sam_lin *lin = &sam_lins[index];
	struct slave *sl = &lin->slave;
	Sercom *s = lin->sercom;
	uint8_t const sercom_rx_pin_group = lin->rx_port_pin_mux >> 5;
	uint8_t const sercom_rx_pin = lin->rx_port_pin_mux & 0x1f;

	LOG("T");


	// LOG("ch%u frame data timeout\n", index);
	usart_clear(s);


	switch (sl->slave_proto_step) {
	case SLAVE_PROTO_STEP_RX_BREAK:
		LOG("ch%u SLAVE_PROTO_STEP_RX_BREAK timeout unhandled\n", index);
		break;
	case SLAVE_PROTO_STEP_RX_SYNC:
		LOG("ch%u SLAVE_PROTO_STEP_RX_SYNC timeout unhandled\n", index);
		break;
	case SLAVE_PROTO_STEP_RX_PID: {
		LOG("ch%u rx timeout pid last rx=%x\n", index, sl->rx_byte);
		bool rx_pin = (PORT->Group[sercom_rx_pin_group].IN.reg & (UINT32_C(1) << sercom_rx_pin)) != 0;
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
			rx_pin = (PORT->Group[sercom_rx_pin_group].IN.reg & (UINT32_C(1) << sercom_rx_pin)) != 0;
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

	lin_cleanup_full(lin, SLAVE_PROTO_STEP_RX_BREAK);

	// allow next header
	__atomic_store_n(&lin->master.busy, 0, __ATOMIC_RELEASE);

	// LOG("|");
}



SLLIN_RAMFUNC static inline void rxbrk(struct sam_lin *lin, uint8_t next_step)
{
	struct master *ma = &lin->master;

	if (ma->proto_step == MASTER_PROTO_STEP_FINISHED) {
		// Restart the timer in case the BREAK wasn't sent by this device.
		// Technically the SOF timeout is too long but if we are lenient here,
		// we don't have to reconfigure the timer.
		sof_start_or_restart_begin(lin);
		lin_cleanup_full(lin, next_step);
	} else {
		// wait for pull down timer to expire before pulling up
		lin_cleanup_master_tx(lin, next_step);
	}

	sam_uart_rx_toggle(lin);
	LOG("_");

	if (ma->proto_step == MASTER_PROTO_STEP_FINISHED) {
		sof_start_or_restart_end(lin);
	} else {

	}
}

SLLIN_RAMFUNC void sam_lin_timer_int(uint8_t index)
{
	struct sam_lin *lin = &sam_lins[index];
	struct slave *sl = &lin->slave;
	struct master *ma = &lin->master;
	Sercom *s = lin->sercom;
	uint8_t const intflag = lin->timer->COUNT16.INTFLAG.reg;
	uint8_t const sercom_rx_pin_group = lin->rx_port_pin_mux >> 5;
	uint8_t const sercom_rx_pin = lin->rx_port_pin_mux & 0x1f;


	(void)intflag;

	lin->timer->COUNT16.INTFLAG.reg = ~0;

	SLLIN_DEBUG_ISR_ASSERT(lin->timer->COUNT16.CTRLBSET.bit.ONESHOT);
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
		bool rx_pin = (PORT->Group[sercom_rx_pin_group].IN.reg & (UINT32_C(1) << sercom_rx_pin)) != 0;

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
		PORT->Group[sercom_rx_pin_group].PINCFG[sercom_rx_pin-1].reg = PORT_PINCFG_PMUXEN;
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
	case TIMER_TYPE_BREAK_REM: {
		bool rx_pin = (PORT->Group[sercom_rx_pin_group].IN.reg & (UINT32_C(1) << sercom_rx_pin)) != 0;

		if (rx_pin) {
			// ?
		} else {
			rxbrk(lin, SLAVE_PROTO_STEP_RX_SYNC);
		}
	} break;
	default:
		LOG("ch%u unhandled timer type %u\n", index, lin->timer_type);
		SLLIN_DEBUG_ISR_ASSERT(false);
		break;
	}
}

SLLIN_RAMFUNC void sam_lin_usart_int(uint8_t index)
{
	struct sam_lin *lin = &sam_lins[index];
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

			s->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
			s->USART.DATA.reg = ma->pid;
			ma->proto_step = MASTER_PROTO_STEP_FINISHED;

			// LOG("tx pid=%x\n", ma->pid);
		}
		break;
	}


	const uint8_t rx_byte = s->USART.DATA.reg;

#if SLLIN_DEBUG
	sl->rx_byte = rx_byte;
#endif

#if SAM_CONF_AUTOBAUD
	if (intflag & SERCOM_USART_INTFLAG_RXBRK) {
		rxbrk(lin, SLAVE_PROTO_STEP_RX_PID);
	}
#else
	const uint8_t BREAK_INT_FLAGS = SERCOM_USART_INTFLAG_RXC | SERCOM_USART_INTFLAG_ERROR;

	if ((intflag & BREAK_INT_FLAGS) == BREAK_INT_FLAGS && rx_byte == 0 && (status & SERCOM_USART_STATUS_FERR)) {
		uint8_t const sercom_rx_pin_group = lin->rx_port_pin_mux >> 5;
		uint8_t const sercom_rx_pin = lin->rx_port_pin_mux & 0x1f;
		bool rx_pin = (PORT->Group[sercom_rx_pin_group].IN.reg & (UINT32_C(1) << sercom_rx_pin)) != 0;

		if (rx_pin) {
			// ?
		} else {
			// wait for remaining 3 bit times
			sam_timer_cleanup_begin(lin);

			intflag &= ~SERCOM_USART_INTFLAG_RXC;
			status &= ~SERCOM_USART_STATUS_FERR;

			// setup remaining break timer
			sam_timer_cleanup_end(lin);
			lin->timer_type = TIMER_TYPE_BREAK_REM;
			lin->timer->COUNT16.CC[0].reg = lin->break_rem_timeout_us;
			lin->timer->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_RETRIGGER_Val;

			sam_uart_rx_toggle(lin);
			LOG("0");
		}
	}
#endif

	// if (intflag & SERCOM_USART_INTFLAG_RXC) {
	// 	LOG("ch%u RX=%x\n", index, rx_byte);
	// }

	switch (sl->slave_proto_step) {
#if !SAM_CONF_AUTOBAUD
	case SLAVE_PROTO_STEP_RX_SYNC: {
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			sam_uart_rx_toggle(lin);

			if (unlikely(0x55 != rx_byte)) {
				sl->elem.frame.id |= SLLIN_ID_FLAG_LIN_ERROR_SYNC;
				LOG("X");
			} else {
				LOG("S");
			}

			sl->slave_proto_step = SLAVE_PROTO_STEP_RX_PID;
		}
	} break;
#endif
	case SLAVE_PROTO_STEP_RX_PID:
		if (intflag & SERCOM_USART_INTFLAG_RXC) {
			uint8_t const id = sllin_pid_to_id(rx_byte);
			uint8_t const pid = sllin_id_to_pid(id);

			sam_timer_cleanup_begin(lin);

			sam_uart_rx_toggle(lin);
			LOG("I");


			if (lin_int_update_bus_status(index, SLLIN_ID_FLAG_BUS_STATE_AWAKE, SLLIN_ID_FLAG_BUS_ERROR_NONE)) {
				LOG("ch%u awake\n", index);
			}

			// LOG("ch%u PID=%x baud=%x frac=%x\n", index, rx_byte, s->USART.BAUD.reg & 0x1fff, s->USART.BAUD.reg >> 13);

			sl->elem.frame.id |= id;

			if (unlikely(pid != rx_byte)) {
				sl->elem.frame.id |= SLLIN_ID_FLAG_LIN_ERROR_PID;
			}

			// clear out flag for PID
			intflag &= ~SERCOM_USART_INTFLAG_RXC;

			// setup data timer
			sam_timer_cleanup_end(lin);
			lin->timer_type = TIMER_TYPE_DATA;
			lin->timer->COUNT16.CC[0].reg = sl->data_timeout_us;

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
			// reset data timer TC_INTFLAG_OVF can happen if we debug log
			SLLIN_DEBUG_ISR_ASSERT(TIMER_TYPE_DATA == lin->timer_type);
			SLLIN_DEBUG_ISR_ASSERT(lin->timer->COUNT16.CC[0].reg == lin->slave.data_timeout_us);
			SLLIN_DEBUG_ISR_ASSERT(!(lin->timer->COUNT16.INTFLAG.reg & (TC_INTFLAG_ERR)));

			sam_timer_start_or_restart_begin(lin);

			sam_uart_rx_toggle(lin);
			LOG("R");


			// LOG("ch%u RX=%x\n", index, rx_byte);
			if (sl->slave_rx_offset < 8) {
				sl->elem.frame.data[sl->elem.frame.len++] = rx_byte;
			} else if (sl->slave_rx_offset == 8) {
				sl->elem.frame.crc = rx_byte;
			} else {
				// too much data
			}

			++sl->slave_rx_offset;

			sam_timer_start_or_restart_end(lin);
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


#endif
