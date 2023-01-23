/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#include <supercan_board.h>

#if STM32F3DISCOVERY

#include <supercan_debug.h>
#include <tusb.h>
#include <leds.h>
#include <bsp/board.h>
#include <stm32f3xx_hal.h>

static struct can {
	CAN_FIFOMailBox_TypeDef rx_fifo[SC_BOARD_CAN_RX_FIFO_SIZE];
	uint8_t txr_buffer[SC_BOARD_CAN_TX_FIFO_SIZE];
	uint8_t txr_get_index; // NOT an index, uses full range of type
	uint8_t txr_put_index; // NOT an index, uses full range of typ
	uint8_t rx_get_index; // NOT an index, uses full range of type
	uint8_t rx_put_index; // NOT an index, uses full range of type
	uint8_t tx_get_index; // NOT an index, uses full range of type
	uint8_t tx_put_index; // NOT an index, uses full range of type
	uint8_t int_prev_bus_status;
	uint8_t int_prev_rec; // RX error counter
	uint8_t int_prev_tec; // TX error counter
} cans[SC_BOARD_CAN_COUNT];

struct led {
	uint8_t port_pin_mux;
};

#define LED_STATIC_INITIALIZER(name, mux) \
	{ mux }


static const struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", (4 << 4) | 9),        // PE09, red
	LED_STATIC_INITIALIZER("USB traffic", (4 << 4) | 8),  // PE08, blue
	LED_STATIC_INITIALIZER("CAN traffic", (4 << 4) | 10), // PE10, orange
	LED_STATIC_INITIALIZER("CAN green", (4 << 4) | 11),   // PE11, green
	LED_STATIC_INITIALIZER("CAN red", (4 << 4) | 13),     // PE13, red
};

static inline void leds_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;

	// switch mode to output function
	GPIOE->MODER =
	  	(GPIOE->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | GPIO_MODER_MODER10 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14))
		| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER8_Pos)
		| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER9_Pos)
		| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER10_Pos)
		| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER11_Pos)
		| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER13_Pos);

	GPIOE->OSPEEDR =
	  	(GPIOE->OSPEEDR & ~(GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR13))
		| (GPIO_SPEED_FREQ_LOW << GPIO_OSPEEDER_OSPEEDR8_Pos)
		| (GPIO_SPEED_FREQ_LOW << GPIO_OSPEEDER_OSPEEDR9_Pos)
		| (GPIO_SPEED_FREQ_LOW << GPIO_OSPEEDER_OSPEEDR10_Pos)
		| (GPIO_SPEED_FREQ_LOW << GPIO_OSPEEDER_OSPEEDR11_Pos)
		| (GPIO_SPEED_FREQ_LOW << GPIO_OSPEEDER_OSPEEDR13_Pos);

	// disable output
	GPIOE->BSRR = GPIO_BSRR_BR_8 | GPIO_BSRR_BR_9 | GPIO_BSRR_BR_10 | GPIO_BSRR_BR_11 | GPIO_BSRR_BR_13;
}


static inline void can_init(void)
{
	/* Setup CAN on PB8 (RX) / PB9 (TX) */


	/* pins */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// pull up on RX pin
	GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPDR8)) | (UINT32_C(0x1) << GPIO_PUPDR_PUPDR8_Pos);
	// high speed output on TX pin
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(GPIO_OSPEEDER_OSPEEDR9)) | (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDER_OSPEEDR9_Pos);
	// alternate function to CAN
	GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(GPIO_AFRH_AFRH0 | GPIO_AFRH_AFRH1)) | (GPIO_AF9_CAN << GPIO_AFRH_AFRH0_Pos) | (GPIO_AF9_CAN << GPIO_AFRH_AFRH1_Pos);
	// switch mode to alternate function
	GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9)) | (GPIO_MODE_AF_PP << GPIO_MODER_MODER8_Pos) | (GPIO_MODE_AF_PP << GPIO_MODER_MODER9_Pos);


	/* CAN */
	RCC->APB1ENR |= RCC_APB1ENR_CANEN;

	// main config
	CAN->MCR =
		CAN_MCR_TXFP /* fifo mode for TX */
		| CAN_MCR_INRQ /* keep in init state */;

	// interrupts
	CAN->IER =
		CAN_IER_ERRIE /* error */
		| CAN_IER_LECIE /* last error */
		| CAN_IER_BOFIE /* bus-off */
		| CAN_IER_EPVIE /* error passive */
		| CAN_IER_EWGIE /* error warning */
		| CAN_IER_FOVIE0 /* RX fifo overrun */
		| CAN_IER_FMPIE0 /* RX fifo not empty */
		| CAN_IER_TMEIE /* TX box empty */
		;

	// filter
	// deactivate
	CAN->FMR = CAN_FMR_FINIT;

	CAN->FM1R = 0; // two 32-bit registers of filter bank x are in Identifier Mask mode.
	CAN->FS1R = (UINT32_C(1) << 14) - 1; // Single 32-bit scale configuration
	CAN->FFA1R = 0; // all filters to FIFO0
	// set to don't care
	CAN->sFilterRegister[0].FR1 = 0; // identifier ?
	CAN->sFilterRegister[0].FR2 = 0; // mask ?

	CAN->FA1R = 1; // activate filter index 0

	// activate
	CAN->FMR &= ~CAN_FMR_FINIT;

	NVIC_SetPriority(CAN_TX_IRQn, SC_ISR_PRIORITY);
	NVIC_SetPriority(CAN_RX0_IRQn, SC_ISR_PRIORITY);
	NVIC_SetPriority(CAN_SCE_IRQn, SC_ISR_PRIORITY);
	NVIC_SetPriority(CAN_RX1_IRQn, SC_ISR_PRIORITY);
}

extern void sc_board_led_set(uint8_t index, bool on)
{
	uint32_t pin = leds[index].port_pin_mux & 0xf;

	GPIOE->BSRR = UINT32_C(1) << (pin + (!on) * 16);
}

extern void sc_board_leds_on_unsafe(void)
{
	GPIOE->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_9 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_13;
}

extern void sc_board_init_begin(void)
{
	memset(cans, 0, sizeof(cans));

	board_init();

	leds_init();
	can_init();

}

extern void sc_board_init_end(void)
{
	led_blink(0, 2000);
}

__attribute__((noreturn)) extern void sc_board_reset(void)
{
	NVIC_SystemReset();
	__unreachable();
}

extern uint16_t sc_board_can_feat_perm(uint8_t index)
{
	(void)index;
	return CAN_FEAT_PERM;
}

extern uint16_t sc_board_can_feat_conf(uint8_t index)
{
	(void)index;
	return CAN_FEAT_CONF;
}

SC_RAMFUNC extern bool sc_board_can_tx_queue(uint8_t index, struct sc_msg_can_tx const * msg)
{
	struct can *can = &cans[index];
	uint8_t pi = can->txr_put_index;
	uint8_t gi = __atomic_load_n(&can->txr_get_index, __ATOMIC_ACQUIRE);
	uint8_t used = pi - gi;
	bool available = used < TU_ARRAY_SIZE(can->txr_buffer);

	if (available) {
		uint8_t txr_put_index = pi % TU_ARRAY_SIZE(can->txr_buffer);

		// store
		can->txr_buffer[txr_put_index] = msg->track_id;

		// mark available
		__atomic_store_n(&can->txr_put_index, pi + 1, __ATOMIC_RELEASE);

		LOG("ch%u queued TXR %u\n", index, msg->track_id);

		sc_can_notify_task_def(index, 1);
	}

	return available;
}


SC_RAMFUNC extern int sc_board_can_retrieve(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	struct can *can = &cans[index];
	int result = 0;
	bool have_data_to_place = false;

	for (bool done = false; !done; ) {
		done = true;
		uint8_t txr_pi = __atomic_load_n(&can->txr_put_index, __ATOMIC_ACQUIRE);

		if (can->txr_get_index != txr_pi) {
			struct sc_msg_can_txr *txr = NULL;
			uint8_t const bytes = sizeof(*txr);

			have_data_to_place = true;


			if ((size_t)(tx_end - tx_ptr) >= bytes) {
				uint8_t const txr_get_index = can->txr_get_index % TU_ARRAY_SIZE(can->txr_buffer);
				done = false;

				txr = (struct sc_msg_can_txr *)tx_ptr;

				tx_ptr += bytes;
				result += bytes;

				txr->flags = 0;
				txr->id = SC_MSG_CAN_TXR;
				txr->len = bytes;
				txr->track_id = cans->txr_buffer[txr_get_index];
				txr->timestamp_us = sc_board_can_ts_wait(index);

				__atomic_store_n(&can->txr_get_index, can->txr_get_index+1, __ATOMIC_RELEASE);

				LOG("ch%u retrievd TXR %u\n", index, txr->track_id);
			}
		}
	}

	if (result > 0) {
		return result;
	}

	return have_data_to_place - 1;
}


extern sc_can_bit_timing_range const* sc_board_can_nm_bit_timing_range(uint8_t index)
{
	(void)index;

	static const sc_can_bit_timing_range nm_range = {
		.min = {
			.brp = 1,
			.tseg1 = 1,
			.tseg2 = 1,
			.sjw = 1,
		},
		.max = {
			.brp = 1024,
			.tseg1 = 16,
			.tseg2 = 8,
			.sjw = 4,
		},
	};

	return &nm_range;
}

extern sc_can_bit_timing_range const* sc_board_can_dt_bit_timing_range(uint8_t index)
{
	(void)index;

	return NULL;
}

extern void sc_board_can_feat_set(uint8_t index, uint16_t features)
{
	(void)index;

	if (features & SC_FEATURE_FLAG_DAR) {
		CAN->MCR |= CAN_MCR_NART;
	} else {
		CAN->MCR &= ~CAN_MCR_NART;
	}

	if (features & SC_FEATURE_FLAG_MON_MODE) {
		CAN->BTR |= CAN_BTR_SILM;
	} else {
		CAN->BTR &= ~CAN_BTR_SILM;
	}
}

extern void sc_board_can_go_bus(uint8_t index, bool on)
{
	if (on) {
		NVIC_EnableIRQ(CAN_TX_IRQn);
		NVIC_EnableIRQ(CAN_RX0_IRQn);
		NVIC_EnableIRQ(CAN_SCE_IRQn);
		NVIC_EnableIRQ(CAN_RX1_IRQn);
		CAN->MCR &= ~CAN_MCR_INRQ;
	} else {
		NVIC_DisableIRQ(CAN_TX_IRQn);
		NVIC_DisableIRQ(CAN_RX0_IRQn);
		NVIC_DisableIRQ(CAN_SCE_IRQn);
		NVIC_DisableIRQ(CAN_RX1_IRQn);
		CAN->MCR |= CAN_MCR_INRQ;

		sc_board_can_reset(index);
	}
}

extern void sc_board_can_nm_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	(void)index;

	CAN->BTR =
		(CAN->BTR & ~(CAN_BTR_SJW | CAN_BTR_TS1 | CAN_BTR_TS1 | CAN_BTR_BRP))
		| (bt->sjw << CAN_BTR_SJW_Pos)
		| (bt->tseg1 << CAN_BTR_TS1_Pos)
		| (bt->tseg2 << CAN_BTR_TS2_Pos)
		| (bt->brp << CAN_BTR_BRP_Pos)
		;
}

extern void sc_board_can_dt_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	(void)index;
	(void)bt;
}

extern uint32_t sc_board_identifier(void)
{
	uint32_t *id_ptr = (uint32_t *)0x1FFFF7AC;
	uint32_t id = 0;

	// 96 bit unique device ID
	id ^= *id_ptr++;
	id ^= *id_ptr++;
	id ^= *id_ptr++;

	return id;
}

extern void sc_board_can_reset(uint8_t index)
{
	struct can* can = &cans[index];

	can->int_prev_bus_status = 0;
	can->int_prev_rec = 0;
	can->int_prev_tec = 0;
	can->rx_get_index = 0;
	can->rx_put_index = 0;
	can->tx_get_index = 0;
	can->tx_put_index = 0;

	__atomic_thread_fence(__ATOMIC_RELEASE); // int_*
}

SC_RAMFUNC static inline uint8_t can_map_bxcan_ec(uint8_t value)
{
	static const uint8_t can_map_bxcan_ec_table[8] = {
		SC_CAN_ERROR_NONE,
		SC_CAN_ERROR_STUFF,
		SC_CAN_ERROR_FORM,
		SC_CAN_ERROR_ACK,
		SC_CAN_ERROR_BIT1,
		SC_CAN_ERROR_BIT0,
		SC_CAN_ERROR_CRC,
		0xff
	};

	return can_map_bxcan_ec_table[value & 7];
}

SC_RAMFUNC extern void sc_board_led_can_status_set(uint8_t index, int status)
{
	(void)index;

	switch (status) {
	case SC_CAN_LED_STATUS_DISABLED:
		led_set(LED_CAN_STATUS_GREEN, 0);
		led_set(LED_CAN_STATUS_RED, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_OFF_BUS:
		led_set(LED_CAN_STATUS_GREEN, 1);
		led_set(LED_CAN_STATUS_RED, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_PASSIVE:
		led_blink(LED_CAN_STATUS_GREEN, SC_CAN_LED_BLINK_DELAY_PASSIVE_MS);
		led_set(LED_CAN_STATUS_RED, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ACTIVE:
		led_blink(LED_CAN_STATUS_GREEN, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		led_set(LED_CAN_STATUS_RED, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_PASSIVE:
		led_set(LED_CAN_STATUS_GREEN, 0);
		led_blink(LED_CAN_STATUS_RED, SC_CAN_LED_BLINK_DELAY_PASSIVE_MS);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_ACTIVE:
		led_set(LED_CAN_STATUS_GREEN, 0);
		led_blink(LED_CAN_STATUS_RED, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_BUS_OFF:
		led_set(LED_CAN_STATUS_GREEN, 0);
		led_set(LED_CAN_STATUS_RED, 1);
		break;
	default:
		led_blink(LED_CAN_STATUS_GREEN, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
		led_blink(LED_CAN_STATUS_RED, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
		break;
	}
}

SC_RAMFUNC void CAN_TX_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &cans[index];
	uint32_t tsr = CAN->TSR;

	LOG("TSR=%08x\n", tsr);

	// TODO

	CAN->TSR =
		CAN_TSR_TERR2
		| CAN_TSR_ALST2
		| CAN_TSR_TXOK2
		| CAN_TSR_RQCP2
		| CAN_TSR_TERR1
		| CAN_TSR_ALST1
		| CAN_TSR_TXOK1
		| CAN_TSR_RQCP1
		| CAN_TSR_TERR0
		| CAN_TSR_ALST0
		| CAN_TSR_TXOK0
		| CAN_TSR_RQCP0
		;
}

SC_RAMFUNC void CAN_RX0_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &cans[index];
	uint32_t rf0r = CAN->RF0R;
	uint8_t lost = 0;
	uint32_t tsc = 0;

	LOG("RF0R=%08x\n", rf0r);

	for ( ; (rf0r & CAN_RF0R_FMP0_Msk) >> CAN_RF0R_FMP0_Pos; rf0r = CAN->RF0R) {
		uint8_t pi = can->rx_put_index;
		uint8_t gi = __atomic_load_n(&can->rx_get_index, __ATOMIC_ACQUIRE);
		uint8_t used = pi - gi;

		if (unlikely(used == TU_ARRAY_SIZE(can->rx_fifo))) {
			++lost;
		} else {
			uint8_t const put_index = pi % TU_ARRAY_SIZE(can->rx_fifo);
			CAN_FIFOMailBox_TypeDef* box = &can->rx_fifo[put_index];

			box->RIR = CAN->sFIFOMailBox[0].RIR;
			box->RDTR = CAN->sFIFOMailBox[0].RDTR;
			box->RDLR = CAN->sFIFOMailBox[0].RDLR;
			box->RDHR = CAN->sFIFOMailBox[0].RDHR;

			__atomic_store_n(&can->rx_put_index, pi + 1, __ATOMIC_RELEASE);
		}

		// release hw mailbox
		CAN->RF0R |= CAN_RF0R_RFOM0;

		// wait for hw
		while (CAN->RF0R & CAN_RF0R_RFOM0);
	}

	if (unlikely(rf0r & CAN_RF0R_FOVR0)) {
		++lost;
	}

	CAN->RF0R = CAN_RF0R_FOVR0 | CAN_RF0R_FULL0;

	if (unlikely(lost)) {
		sc_can_status status;

		status.type = SC_CAN_STATUS_FIFO_TYPE_RX_LOST;
		status.timestamp_us = tsc;
		status.rx_lost = lost;

		sc_can_status_queue(index, &status);
		sc_can_notify_task_isr(index, 1);
	}
}

SC_RAMFUNC void CAN_RX1_IRQHandler(void)
{
	SC_ASSERT(false && "no messages expected in FIFO1");
}

SC_RAMFUNC void CAN_SCE_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &cans[index];
	sc_can_status status;
	uint32_t const esr = CAN->ESR;
	uint32_t const msr = CAN->MSR;
	uint8_t current_bus_state = 0;
	uint32_t events = 0;
	uint32_t tsc = 0;
	uint8_t rec, tec, lec;

	LOG("ESR=%08x\n", esr);

	if (esr & CAN_ESR_BOFF) {
		current_bus_state = SC_CAN_STATUS_BUS_OFF;
	} else if (esr & CAN_ESR_EPVF) {
		current_bus_state = SC_CAN_STATUS_ERROR_PASSIVE;
	} else if (esr & CAN_ESR_EWGF) {
		current_bus_state = SC_CAN_STATUS_ERROR_WARNING;
	} else {
		current_bus_state = SC_CAN_STATUS_ERROR_ACTIVE;
	}

	if (unlikely(can->int_prev_bus_status != current_bus_state)) {
		can->int_prev_bus_status = current_bus_state;

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_STATUS;
		status.timestamp_us = tsc;
		status.bus_state = current_bus_state;

		sc_can_status_queue(index, &status);
		++events;

		switch (current_bus_state) {
		case SC_CAN_STATUS_BUS_OFF:
			LOG("CAN%u bus off\n", index);
			break;
		case SC_CAN_STATUS_ERROR_PASSIVE:
			LOG("CAN%u error passive\n", index);
			break;
		case SC_CAN_STATUS_ERROR_WARNING:
			LOG("CAN%u error warning\n", index);
			break;
		case SC_CAN_STATUS_ERROR_ACTIVE:
			LOG("CAN%u error active\n", index);
			break;
		default:
			LOG("CAN%u unhandled bus state\n", index);
			break;
		}
	}

	rec = (esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos;
	tec = (esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos;

	if (unlikely(rec != can->int_prev_rec || tec != can->int_prev_tec)) {
		can->int_prev_tec = tec;
		can->int_prev_rec = rec;

		// LOG("CAN%u REC=%u TEC=%u\n", index, can->int_prev_rx_errors, can->int_prev_tx_errors);

		status.type = SC_CAN_STATUS_FIFO_TYPE_RXTX_ERRORS;
		status.timestamp_us = tsc;
		status.counts.rx = rec;
		status.counts.tx = tec;

		sc_can_status_queue(index, &status);
		++events;
	}

	lec = (esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos;

	if (unlikely(lec > 0 && lec <= 7)) {
		const bool is_tx_error = (msr & CAN_MSR_TXM) == CAN_MSR_TXM;

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		status.timestamp_us = tsc;
		status.bus_error.tx = is_tx_error;
		status.bus_error.code = can_map_bxcan_ec(lec),
		status.bus_error.data_part = 0;

		sc_can_status_queue(index, &status);
		++events;
	}


	if (likely(events)) {
		// LOG(">");
		sc_can_notify_task_isr(index, events);
	}
}


#endif // #if STM32F3DISCOVERY

