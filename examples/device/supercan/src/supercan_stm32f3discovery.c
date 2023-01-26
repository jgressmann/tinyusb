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


#define HW_TX_MB_COUNT 3
#define HW_TX_MB_FIFO_SIZE 4

struct rx_frame {
	uint32_t RIR;
	uint32_t RDTR;
	uint32_t RDLR;
	uint32_t RDHR;
	uint32_t ts;
};

struct tx_frame {
	uint32_t TIR;
	uint32_t TDTR;
	uint32_t TDLR;
	uint32_t TDHR;
	uint8_t track_id;
};

struct txr {
	uint32_t ts;
	uint8_t track_id;
};

TU_VERIFY_STATIC(SC_BOARD_CAN_RX_FIFO_SIZE == 32); // MUST BE a power of two
TU_VERIFY_STATIC(SC_BOARD_CAN_TX_FIFO_SIZE == 32); // MUST BE a power of two
TU_VERIFY_STATIC(HW_TX_MB_COUNT <= HW_TX_MB_FIFO_SIZE); // MUST BE a power of two

static struct can {
	struct rx_frame rx_fifo[SC_BOARD_CAN_RX_FIFO_SIZE];
	struct tx_frame tx_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];
	struct txr txr_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];
	uint32_t nm_us_per_bit;
#if SUPERCAN_DEBUG
	volatile uint32_t txr; // volatile b/c of use in interrupt handler
#endif
	uint8_t tx_track_id_boxes[HW_TX_MB_FIFO_SIZE];
	uint8_t tx_mailbox_free_fifo[HW_TX_MB_FIFO_SIZE];
	uint8_t tx_mailbox_used_fifo[HW_TX_MB_FIFO_SIZE];
	uint8_t txr_get_index; // NOT an index, uses full range of type
	uint8_t txr_put_index; // NOT an index, uses full range of typ
	uint8_t rx_get_index; // NOT an index, uses full range of type
	uint8_t rx_put_index; // NOT an index, uses full range of type
	uint8_t tx_get_index; // NOT an index, uses full range of type
	uint8_t tx_put_index; // NOT an index, uses full range of type
	uint8_t int_prev_bus_status;
	uint8_t int_prev_rec; // RX error counter
	uint8_t int_prev_tec; // TX error counter
	uint8_t tx_mailbox_free_get_index; // NOT an index, uses full range of type
	uint8_t tx_mailbox_free_put_index; // NOT an index, uses full range of type
	uint8_t tx_mailbox_used_get_index; // NOT an index, uses full range of type
	uint8_t tx_mailbox_used_put_index; // NOT an index, uses full range of type
} stm32_cans[SC_BOARD_CAN_COUNT];

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

static inline void dump_irq_prios(void)
{
	LOG("CAN_TX(%u)=%u\n", CAN_TX_IRQn, NVIC_GetPriority(CAN_TX_IRQn));
	LOG("CAN_RX0(%u)=%u\n", CAN_RX0_IRQn, NVIC_GetPriority(CAN_RX0_IRQn));
	LOG("CAN_SCE(%u)=%u\n", CAN_SCE_IRQn, NVIC_GetPriority(CAN_SCE_IRQn));
	LOG("USB_HP(%u)=%u\n", USB_HP_IRQn, NVIC_GetPriority(USB_HP_IRQn));
	LOG("USB_LP(%u)=%u\n", USB_LP_IRQn, NVIC_GetPriority(USB_LP_IRQn));
	LOG("USBWakeUp_RMP(%u)=%u\n", USBWakeUp_RMP_IRQn, NVIC_GetPriority(USBWakeUp_RMP_IRQn));
}


static void can_reset_state(uint8_t index)
{
	struct can* can = &stm32_cans[index];

	can->int_prev_bus_status = SC_CAN_STATUS_ERROR_ACTIVE;
	can->int_prev_rec = 0;
	can->int_prev_tec = 0;
	can->rx_get_index = 0;
	can->rx_put_index = 0;
	can->tx_get_index = 0;
	can->tx_put_index = 0;
	can->txr_get_index = 0;
	can->txr_put_index = 0;


	can->tx_mailbox_used_get_index = 0;
	can->tx_mailbox_used_put_index = 0;
	can->tx_mailbox_free_get_index = 0xfc;
	can->tx_mailbox_free_put_index = 0xff;


	for (uint8_t mb = 0; mb < HW_TX_MB_COUNT; ++mb) {
		can->tx_mailbox_free_fifo[mb] = mb;
	}



#if SUPERCAN_DEBUG
	can->txr = 0;
#endif

	__atomic_thread_fence(__ATOMIC_RELEASE); // int_*
}

static inline void can_init_hw(void)
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
		CAN_MCR_TXFP /* fifo mode (by reqeust order) for TX */
		| CAN_MCR_RFLM /* discard messages in case of RX fifo overrun */
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

	// filter -> accept everything
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
	// NVIC_SetPriority(CAN_RX1_IRQn, SC_ISR_PRIORITY);
}


static inline void can_init(void)
{
	memset(stm32_cans, 0, sizeof(stm32_cans));


	// for (uint8_t index = 0; index < TU_ARRAY_SIZE(cans); ++index) {
	// 	struct can *can = &stm32_cans[index];

	// 	can->int_prev_bus_status = SC_CAN_STATUS_ERROR_ACTIVE;
	// 	can->tx_mailbox_free_get_index = 0;
	// 	can->tx_mailbox_free_put_index = 3;

	// 	for (uint8_t mb = 0; mb < 3; ++mb) {
	// 		can->tx_mailbox_free_fifo[mb] = mb;
	// 	}
	// }

	for (uint8_t index = 0; index < TU_ARRAY_SIZE(stm32_cans); ++index) {
		can_reset_state(index);
	}

	can_init_hw();
}

static inline void counter_1MHz_init(void)
{
	// TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	// prescaler: 48 MHz -> 1MHz
	TIM2->PSC = 47; /* yes, minus one */
	// count as far as possible
	TIM2->ARR = 0xffffffff;

	TIM2->CR1 =
		TIM_CR1_URS  /* only under/overflow, DMA */
		| TIM_CR1_CEN;

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
	board_init();

	leds_init();
	can_init();
	counter_1MHz_init();

	// // PA3
	// RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// GPIOA->MODER =
	//   	(GPIOA->MODER & ~(GPIO_MODER_MODER3))
	// 	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODER3_Pos);

	// GPIOA->OSPEEDR =
	//   	(GPIOA->OSPEEDR & ~(GPIO_OSPEEDER_OSPEEDR3))
	// 	| (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDER_OSPEEDR3_Pos);


	// TIM2->PSC = 47;
	// TIM2->DIER = TIM_DIER_UIE;
	// TIM2->ARR = 1000;

	// TIM2->CR1 =
	// 	TIM_CR1_URS  /* only under/overflow, DMA */
	// 	| TIM_CR1_CEN;


	// NVIC_SetPriority(TIM2_IRQn, SC_ISR_PRIORITY);
	// NVIC_EnableIRQ(TIM2_IRQn);

	// TIM3->PSC = (48000000 / 1000) - 1;
	// TIM3->DIER = TIM_DIER_UIE;
	// TIM3->ARR = 1000;

	// TIM3->CR1 =
	// 	TIM_CR1_URS  /* only under/overflow, DMA */
	// 	| TIM_CR1_CEN;


	// NVIC_SetPriority(TIM3_IRQn, SC_ISR_PRIORITY);
	// NVIC_EnableIRQ(TIM3_IRQn);
}

extern void sc_board_init_end(void)
{

	led_blink(0, 2000);
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	// dump_irq_prios();
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
	struct can *can = &stm32_cans[index];
	uint8_t const tx_gi = __atomic_load_n(&can->tx_get_index, __ATOMIC_ACQUIRE);
	uint8_t tx_pi = can->tx_put_index;
	uint8_t used = tx_pi - tx_gi;
	bool available = used < TU_ARRAY_SIZE(can->tx_fifo);

	if (available) {
		uint8_t const tx_pi_mod = tx_pi % TU_ARRAY_SIZE(can->tx_fifo);
		struct tx_frame *txf = &can->tx_fifo[tx_pi_mod];

#if SUPERCAN_DEBUG
		bool track_id_present = can->txr & (UINT32_C(1) << msg->track_id);

		if (unlikely(track_id_present)) {
			LOG("ch%u track id %u in TXR bitset %08x\n", index, msg->track_id, can->txr);
		}

		SC_DEBUG_ASSERT(!track_id_present);
		SC_DEBUG_ASSERT(msg->track_id < 32);
		can->txr |= (UINT32_C(1) << msg->track_id);
#endif

		// store
		txf->TDTR = msg->dlc;
		txf->TIR = CAN_TI0R_TXRQ; // mark ready for TX

		if (msg->flags & SC_CAN_FRAME_FLAG_EXT) {
			txf->TIR |= (msg->can_id << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE;
		} else {
			txf->TIR |= msg->can_id << CAN_TI0R_STID_Pos;
		}

		if (msg->flags & SC_CAN_FRAME_FLAG_RTR) {
			txf->TIR |= CAN_TI0R_RTR;
		} else {
			uint8_t *d = (uint8_t *)&txf->TDLR;

			memcpy(d, msg->data, msg->dlc);
		}

		txf->track_id = msg->track_id;

		// mark available
		__atomic_store_n(&can->tx_put_index, tx_pi + 1, __ATOMIC_RELEASE);

		// flag interrupt to handle actual transmit
		NVIC_SetPendingIRQ(CAN_TX_IRQn);
	}

	return available;
}


SC_RAMFUNC extern int sc_board_can_retrieve(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	struct can *can = &stm32_cans[index];
	int result = 0;
	bool have_data_to_place = false;

	for (bool done = false; !done; ) {
		uint8_t const txr_pi = __atomic_load_n(&can->txr_put_index, __ATOMIC_ACQUIRE);
		uint8_t txr_gi = can->txr_get_index;
		uint8_t const rx_pi = __atomic_load_n(&can->rx_put_index, __ATOMIC_ACQUIRE);
		uint8_t rx_gi = can->rx_get_index;

		done = true;

		if (txr_gi != txr_pi) {
			struct sc_msg_can_txr *txr = NULL;
			uint8_t const bytes = sizeof(*txr);

			have_data_to_place = true;


			if ((size_t)(tx_end - tx_ptr) >= bytes) {
				uint8_t const txr_gi_mod = txr_gi % TU_ARRAY_SIZE(can->txr_fifo);
				done = false;

				txr = (struct sc_msg_can_txr *)tx_ptr;

				tx_ptr += bytes;
				result += bytes;

				txr->flags = 0;
				txr->id = SC_MSG_CAN_TXR;
				txr->len = bytes;
				txr->track_id = can->txr_fifo[txr_gi_mod].track_id;
				txr->timestamp_us = can->txr_fifo[txr_gi_mod].ts;

				__atomic_store_n(&can->txr_get_index, txr_gi+1, __ATOMIC_RELEASE);

#if SUPERCAN_DEBUG
				SC_DEBUG_ASSERT(txr->track_id < 32);
				SC_DEBUG_ASSERT(can->txr & (UINT32_C(1) << txr->track_id));
				can->txr &= ~(UINT32_C(1) << txr->track_id);
#endif

				// LOG("ch%u retrievd TXR %u\n", index, txr->track_id);
			}
		}

		if (rx_gi != rx_pi) {
			struct sc_msg_can_rx *rx = NULL;
			uint8_t const rx_gi_mod = rx_gi % TU_ARRAY_SIZE(can->rx_fifo);
			struct rx_frame *rxf = &can->rx_fifo[rx_gi_mod];
			uint8_t const dlc = (rxf->RDTR & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos;
			uint8_t bytes = sizeof(*rx) + dlc;

			if (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1)) {
				bytes += SC_MSG_CAN_LEN_MULTIPLE - (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1));
			}

			have_data_to_place = true;


			if ((size_t)(tx_end - tx_ptr) >= bytes) {
				done = false;

				rx = (struct sc_msg_can_rx *)tx_ptr;

				tx_ptr += bytes;
				result += bytes;

				rx->id = SC_MSG_CAN_RX;
				rx->len = bytes;
				rx->dlc = dlc;
				rx->flags = 0;
				rx->timestamp_us = rxf->ts;

				if (rxf->RIR & CAN_RI0R_IDE) {
					rx->can_id = (rxf->RIR & CAN_RI0R_EXID_Msk) >> CAN_RI0R_EXID_Pos;
					rx->flags |= SC_CAN_FRAME_FLAG_EXT;
				} else {
					rx->can_id = (rxf->RIR & CAN_RI0R_STID_Msk) >> CAN_RI0R_STID_Pos;
				}

				if (rxf->RIR & CAN_RI0R_RTR) {
					rx->flags |= SC_CAN_FRAME_FLAG_RTR;
				} else {
					uint8_t const *d = (uint8_t const *)&rxf->RDLR;

					memcpy(rx->data, d, dlc);
				}

				__atomic_store_n(&can->rx_get_index, rx_gi+1, __ATOMIC_RELEASE);
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

static void can_off(uint8_t index)
{
	NVIC_DisableIRQ(CAN_TX_IRQn);
	NVIC_DisableIRQ(CAN_RX0_IRQn);
	NVIC_DisableIRQ(CAN_SCE_IRQn);
	// NVIC_DisableIRQ(CAN_RX1_IRQn);

	/* abort scheduled transmissions */
	CAN->TSR |= CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;

	/* go to initialization mode */
	CAN->MCR |= CAN_MCR_INRQ;

	can_reset_state(index);
}

extern void sc_board_can_go_bus(uint8_t index, bool on)
{
	if (on) {
		NVIC_EnableIRQ(CAN_TX_IRQn);
		NVIC_EnableIRQ(CAN_RX0_IRQn);
		NVIC_EnableIRQ(CAN_SCE_IRQn);
		// NVIC_EnableIRQ(CAN_RX1_IRQn);
		CAN->MCR &= ~CAN_MCR_INRQ;
	} else {
		can_off(index);
	}
}

extern void sc_board_can_nm_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	struct can* can = &stm32_cans[index];

	CAN->BTR =
		(CAN->BTR & ~(CAN_BTR_SJW | CAN_BTR_TS1 | CAN_BTR_TS2 | CAN_BTR_BRP))
		| ((bt->sjw-1) << CAN_BTR_SJW_Pos)
		| ((bt->tseg1-1) << CAN_BTR_TS1_Pos)
		| ((bt->tseg2-1) << CAN_BTR_TS2_Pos)
		| ((bt->brp-1) << CAN_BTR_BRP_Pos)
		;

	can->nm_us_per_bit = UINT32_C(1000000) / sc_bitrate(bt->brp, bt->tseg1, bt->tseg2);
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
	// struct can* can = &stm32_cans[index];

	// disable CAN units, reset configuration & status
	can_off(index);

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

static inline bool validate_mailbox_assignment(uint8_t index)
{
	struct can *can = &stm32_cans[index];
	unsigned used = 0;
	unsigned free = 0;
	bool valid = true;
	uint8_t const used_count = can->tx_mailbox_used_put_index - can->tx_mailbox_used_get_index;
	uint8_t const free_count = can->tx_mailbox_free_put_index - can->tx_mailbox_free_get_index;

	if (valid) {
		valid = used_count <= TU_ARRAY_SIZE(can->tx_mailbox_used_fifo);
	}

	if (valid) {
		valid = free_count <= TU_ARRAY_SIZE(can->tx_mailbox_free_fifo);
	}

	if (valid) {
		for (uint8_t mbqgi = can->tx_mailbox_used_get_index; mbqgi != can->tx_mailbox_used_put_index; ++mbqgi) {
			uint8_t mbqgi_mod = mbqgi % TU_ARRAY_SIZE(can->tx_mailbox_used_fifo);
			unsigned mb = can->tx_mailbox_used_fifo[mbqgi_mod];
			unsigned bit = 1u << mb;

			if (used & bit) {
				LOG("ch%u used mailbox %u dup\n", index, mb);
				valid = false;
			}

			used |= bit;
		}

		for (uint8_t mbqgi = can->tx_mailbox_free_get_index; mbqgi != can->tx_mailbox_free_put_index; ++mbqgi) {
			uint8_t mbqgi_mod = mbqgi % TU_ARRAY_SIZE(can->tx_mailbox_free_fifo);
			unsigned mb = can->tx_mailbox_free_fifo[mbqgi_mod];
			unsigned bit = 1u << mb;

			if (free & bit) {
				LOG("ch%u free mailbox %u dup\n", index, mb);
				valid = false;
			}

			free |= bit;
		}

		if (valid) {
			valid = ((used | free) == 0x7) && ((used & free) == 0x0);
		}
	}

	if (!valid) {
		LOG("ch%u used count=%u bits=%x free count=%u bits=%x\n", index, used_count, used, free_count, free);
	}

	return valid;
}



SC_RAMFUNC void CAN_TX_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &stm32_cans[index];
	uint32_t const tsc = sc_board_can_ts_wait(index);
	uint32_t tsr = CAN->TSR;
	unsigned count = 0;
	uint32_t events = 0;
	uint16_t ts[HW_TX_MB_COUNT];
	uint8_t indices[HW_TX_MB_COUNT];

	// LOG("TSR=%08x\n", tsr);

	// SC_DEBUG_ASSERT(validate_mailbox_assignment(index));

	/* forward scan for finished mailboxes */
	while (can->tx_mailbox_used_get_index != can->tx_mailbox_used_put_index) {
		uint8_t const used_gi_mod = can->tx_mailbox_used_get_index % TU_ARRAY_SIZE(can->tx_mailbox_used_fifo);
		uint8_t const mailbox_index = can->tx_mailbox_used_fifo[used_gi_mod];
		uint32_t const mailbox_finished_bit = UINT32_C(1) << (8 * mailbox_index);
		SC_DEBUG_ASSERT(mailbox_index < HW_TX_MB_COUNT);

		// LOG("used gi=%02x pi=%02x gi_mod=%02x mb=%x\n", can->tx_mailbox_used_get_index, can->tx_mailbox_used_put_index, used_gi_mod, mailbox_index);

		if (tsr & mailbox_finished_bit) {
			uint8_t const free_pi_mod = can->tx_mailbox_free_put_index % TU_ARRAY_SIZE(can->tx_mailbox_free_fifo);
			SC_DEBUG_ASSERT(free_pi_mod < TU_ARRAY_SIZE(can->tx_mailbox_free_fifo));

			// LOG("free gi=%02x pi=%02x pi_mod=%02x\n", can->tx_mailbox_free_get_index, can->tx_mailbox_free_put_index, free_pi_mod);

			// remember mailbox as free
			can->tx_mailbox_free_fifo[free_pi_mod] = mailbox_index;
			++can->tx_mailbox_used_get_index;
			++can->tx_mailbox_free_put_index;

			// LOG("%u%u%u", used_gi_mod, free_pi_mod, mailbox_index);

			// if (!validate_mailbox_assignment(index)) {
			// 	LOG("mb %u\n", mailbox_index);
			// 	SC_DEBUG_ASSERT(false);
			// }

			/* save time stamp for delta computation */
			ts[count] = (CAN->sTxMailBox[mailbox_index].TDTR & CAN_TDT0R_TIME_Msk) >> CAN_TDT0R_TIME_Pos;
			indices[count] = mailbox_index;

			++count;

			// LOG("ch%u free mb %u\n", index, mailbox_index);

			/* acknowledge hw finished */
			CAN->TSR |= mailbox_finished_bit;
		} else {
			break;
		}
	}

	// SC_DEBUG_ASSERT(validate_mailbox_assignment(index));

	if (likely(count)) {
		// queue TXRs
		uint16_t const t0_can = ts[0];
		uint8_t const txr_gi = __atomic_load_n(&can->txr_get_index, __ATOMIC_ACQUIRE);
		uint8_t txr_pi = can->txr_put_index;
		bool desync = false;

		for (unsigned i = 0; i < count; ++i) {
			uint8_t const mailbox_index = indices[i];
			uint8_t const txrs_used = txr_pi - txr_gi;

			SC_DEBUG_ASSERT(txrs_used <= TU_ARRAY_SIZE(can->txr_fifo));

			if (unlikely(txrs_used == TU_ARRAY_SIZE(can->txr_fifo))) {
				LOG("ch%u lost TXR track ID %02x\n", index, can->tx_track_id_boxes[mailbox_index]);
				desync = true;
			} else {
				uint8_t const txr_pi_mod = txr_pi % TU_ARRAY_SIZE(can->txr_fifo);

				can->txr_fifo[txr_pi_mod].ts = tsc + (ts[i] - t0_can) * can->nm_us_per_bit;
				can->txr_fifo[txr_pi_mod].track_id = can->tx_track_id_boxes[mailbox_index];
				// can->txr_fifo[txr_index].flags = can->flags_boxes[mailbox_index];

				++events;
				++txr_pi;
			}
		}

		__atomic_store_n(&can->txr_put_index, txr_pi, __ATOMIC_RELEASE);

		if (unlikely(desync)) {
			sc_can_status status;

			status.type = SC_CAN_STATUS_FIFO_TYPE_TXR_DESYNC;
			status.timestamp_us = tsc;

			sc_can_status_queue(index, &status);
			++events;
		}
	}

	/* move frames from queue into mailboxes */
	uint8_t tx_pi = __atomic_load_n(&can->tx_put_index, __ATOMIC_ACQUIRE);
	uint8_t tx_gi = can->tx_get_index;

	uint8_t mailboxes_free = can->tx_mailbox_free_put_index - can->tx_mailbox_free_get_index;

	SC_DEBUG_ASSERT(mailboxes_free <= TU_ARRAY_SIZE(can->tx_mailbox_free_fifo));

	while (tx_pi != tx_gi && mailboxes_free) {
		uint8_t const tx_gi_mod = tx_gi % TU_ARRAY_SIZE(can->tx_fifo);
		struct tx_frame *txf = &can->tx_fifo[tx_gi_mod];
		uint8_t const free_fifo_index_mod = can->tx_mailbox_free_get_index % TU_ARRAY_SIZE(can->tx_mailbox_free_fifo);
		uint8_t const mailbox_index = can->tx_mailbox_free_fifo[free_fifo_index_mod];

		SC_DEBUG_ASSERT(mailbox_index < HW_TX_MB_COUNT);
		SC_DEBUG_ASSERT((CAN->TSR & (UINT32_C(1) << (26 + mailbox_index))) && "hw mailbox should be free");

#if SUPERCAN_DEBUG
		/* verifiy track ID is marked in use */
		bool track_id_present = can->txr & (UINT32_C(1) << txf->track_id);

		if (unlikely(!track_id_present)) {
			LOG("ch%u track id %u NOT in TXR bitset %08x\n", index, txf->track_id, can->txr);
		}

		SC_ASSERT(track_id_present);
		SC_ASSERT(txf->track_id < 32);
#endif

		can->tx_track_id_boxes[mailbox_index] = txf->track_id;
		// can->flags_boxes[mailbox_index] = tx->flags;

		/* store frame in box and mark for transmission */
		CAN->sTxMailBox[mailbox_index].TDLR = txf->TDLR;
		CAN->sTxMailBox[mailbox_index].TDHR = txf->TDHR;
		CAN->sTxMailBox[mailbox_index].TDTR = txf->TDTR;
		CAN->sTxMailBox[mailbox_index].TIR = txf->TIR; // must be last, has 'start-hw-tx' flag

		/* remember this hw mailbox is now in use */
		SC_DEBUG_ASSERT((uint8_t)(can->tx_mailbox_used_put_index - can->tx_mailbox_used_get_index) < TU_ARRAY_SIZE(can->tx_mailbox_used_fifo));
		can->tx_mailbox_used_fifo[can->tx_mailbox_used_put_index++ % TU_ARRAY_SIZE(can->tx_mailbox_used_fifo)] = mailbox_index;


		/* update loop variables */
		++can->tx_mailbox_free_get_index;
		++tx_gi;
		mailboxes_free = can->tx_mailbox_free_put_index - can->tx_mailbox_free_get_index;

	}

	__atomic_store_n(&can->tx_get_index, tx_gi, __ATOMIC_RELEASE);

	// SC_DEBUG_ASSERT(validate_mailbox_assignment(index));

	if (likely(events)) {
		sc_can_notify_task_isr(index, events);
	}

	// CAN->TSR |=
	// 	CAN_TSR_TERR2
	// 	| CAN_TSR_ALST2
	// 	| CAN_TSR_TXOK2
	// 	| CAN_TSR_TERR1
	// 	| CAN_TSR_ALST1
	// 	| CAN_TSR_TXOK1
	// 	| CAN_TSR_TERR0
	// 	| CAN_TSR_ALST0
	// 	| CAN_TSR_TXOK0
	// 	;
}

SC_RAMFUNC void CAN_RX0_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &stm32_cans[index];
	uint32_t rf0r = CAN->RF0R;
	uint8_t lost = 0;
	uint32_t events = 0;
	uint32_t const tsc = sc_board_can_ts_wait(index);

	// LOG("RF0R=%08x\n", rf0r);

	uint8_t rx_pi = can->rx_put_index;

	for ( ; (rf0r & CAN_RF0R_FMP0_Msk) >> CAN_RF0R_FMP0_Pos; rf0r = CAN->RF0R) {
		uint8_t rx_gi = __atomic_load_n(&can->rx_get_index, __ATOMIC_ACQUIRE);
		uint8_t used = rx_pi - rx_gi;

		if (unlikely(used == TU_ARRAY_SIZE(can->rx_fifo))) {
			++lost;
		} else {
			uint8_t const rx_pi_mod = rx_pi++ % TU_ARRAY_SIZE(can->rx_fifo);
			struct rx_frame* rxf = &can->rx_fifo[rx_pi_mod];

			rxf->RIR = CAN->sFIFOMailBox[0].RIR;
			rxf->RDTR = CAN->sFIFOMailBox[0].RDTR;
			rxf->RDLR = CAN->sFIFOMailBox[0].RDLR;
			rxf->RDHR = CAN->sFIFOMailBox[0].RDHR;
			rxf->ts = tsc;

			++events;
		}

		// release hw mailbox
		CAN->RF0R |= CAN_RF0R_RFOM0;

		// wait for hw
		while (CAN->RF0R & CAN_RF0R_RFOM0);
	}

	__atomic_store_n(&can->rx_put_index, rx_pi, __ATOMIC_RELEASE);

	if (unlikely(rf0r & CAN_RF0R_FOVR0)) {
		// LOG("-");
		++lost;
	}

	CAN->RF0R = CAN_RF0R_FOVR0 | CAN_RF0R_FULL0;

	if (unlikely(lost)) {
		sc_can_status status;

		status.type = SC_CAN_STATUS_FIFO_TYPE_RX_LOST;
		status.timestamp_us = tsc;
		status.rx_lost = lost;

		sc_can_status_queue(index, &status);
		++events;

	}

	if (likely(events)) {
		sc_can_notify_task_isr(index, events);
	}
}

// SC_RAMFUNC void CAN_RX1_IRQHandler(void)
// {
// 	SC_ASSERT(false && "no messages expected in FIFO1");
// }

SC_RAMFUNC void CAN_SCE_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &stm32_cans[index];
	sc_can_status status;
	uint32_t const esr = CAN->ESR;
	uint32_t const msr = CAN->MSR;
	uint8_t current_bus_state = 0;
	uint32_t events = 0;
	uint32_t const tsc = sc_board_can_ts_wait(index);
	uint8_t rec, tec, lec;

	// LOG("ESR=%08x\n", esr);



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

	// ? necessary?
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

// SC_RAMFUNC void TIM2_IRQHandler(void)
// {
// 	static unsigned i = 0;
// 	const unsigned pin = 3;
// 	unsigned v = i++ & 1;

// 	uint16_t sr = TIM2->SR;

// 	TIM2->SR = 0;

// 	// LOG("%08x\n", TIM2->CNT);

// 	GPIOA->BSRR = UINT32_C(1) << (pin + (!v) * 16);
// }

// SC_RAMFUNC void TIM3_IRQHandler(void)
// {
// 	static unsigned i = 0;
// 	const unsigned pin = 3;
// 	unsigned v = i++ & 1;

// 	uint16_t sr = TIM3->SR;

// 	TIM3->SR = 0;

// 	LOG("SR=%08x CNT=%08x PSC=%08x ARR=%08x\n", sr, TIM3->CNT, TIM3->PSC, TIM3->ARR);

// 	GPIOA->BSRR = UINT32_C(1) << (pin + (!v) * 16);
// }

#endif // #if STM32F3DISCOVERY

