/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include <supercan_board.h>

#if D5035_04

#include <leds.h>
#include <tusb.h>


//#include <usb_descriptors.h> // DFU_USB_RESET_TIMEOUT_MS


#define PORT_STEP 0x00000400U

enum {
	CAN_FEAT_PERM = SC_FEATURE_FLAG_TXR,
	CAN_FEAT_CONF = (MSG_BUFFER_SIZE >= 128 ? SC_FEATURE_FLAG_FDF : 0)
					// | SC_FEATURE_FLAG_TXP
					| SC_FEATURE_FLAG_EHD
					// not yet implemented
					// | SC_FEATURE_FLAG_DAR
					| SC_FEATURE_FLAG_MON_MODE
					// | SC_FEATURE_FLAG_RES_MODE
					// | SC_FEATURE_FLAG_EXT_LOOP_MODE,
};


static const sc_can_bit_timing_range nm_range = {
	.min = {
		.brp = 1,
		.tseg1 = 1,
		.tseg2 = 1,
		.sjw = 1,
	},
	.max = {
		.brp = 1023,
		.tseg1 = 128,
		.tseg2 = 32,
		.sjw = 32,
	},
};

static const sc_can_bit_timing_range dt_range = {
	.min = {
		.brp = 1,
		.tseg1 = 1,
		.tseg2 = 1,
		.sjw = 1,
	},
	.max = {
		.brp = 255, /* actually 1024 but Linux can't handle that */
		.tseg1 = 16,
		.tseg2 = 8,
		.sjw = 8,
	},
};

struct tx_frame {
	uint32_t tmi;
	uint32_t tmp;
	uint8_t track_id;
	uint8_t words;
	uint32_t data[16];
};

struct can_frame {
	uint32_t reg0;
	uint32_t reg1;
	uint32_t data[16];
};

struct txr {
	uint32_t ts;
	uint8_t track_id;
	uint8_t flags;
};

struct can {
	// CFG_TUSB_MEM_ALIGN struct can_tx_fifo_element tx_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];
	// CFG_TUSB_MEM_ALIGN struct can_tx_event_fifo_element tx_event_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];
	// CFG_TUSB_MEM_ALIGN struct can_rx_fifo_element rx_fifo[SC_BOARD_CAN_RX_FIFO_SIZE];
	struct can_frame rx_queue[SC_BOARD_CAN_RX_FIFO_SIZE];
	struct tx_frame tx_queue[SC_BOARD_CAN_TX_FIFO_SIZE];
	struct txr txr_queue[SC_BOARD_CAN_TX_FIFO_SIZE];
	sc_can_bit_timing nm;
	sc_can_bit_timing dt;
	uint32_t const regs;
	IRQn_Type const interrupt_id;
	uint8_t track_id_queue[3];
	uint16_t features;
	uint8_t const led_status_green;
	uint8_t const led_status_red;
	uint8_t const led_traffic;
	uint8_t rx_get_index; // NOT an index, uses full range of type
	uint8_t rx_put_index; // NOT an index, uses full range of type
	uint8_t tx_get_index; // NOT an index, uses full range of type
	uint8_t tx_put_index; // NOT an index, uses full range of type
	uint8_t tx_mailbox_get_index; // NOT an index, uses full range of type
	uint8_t tx_mailbox_put_index; // NOT an index, uses full range of type
	uint8_t txr_get_index; // NOT an index, uses full range of type
	uint8_t txr_put_index; // NOT an index, uses full range of type
} cans[SC_BOARD_CAN_COUNT] = {
	{
		.led_status_green = LED_CAN0_STATUS_GREEN,
		.led_status_red = LED_CAN0_STATUS_RED,
		.led_traffic = LED_DEBUG_1,
	},
};

static uint32_t device_identifier;

struct led {
	uint8_t port_pin_mux;
};

#define LED_STATIC_INITIALIZER(name, pin) \
	{ pin }



static const struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", (1 << 4) | 14),      // PB14
	LED_STATIC_INITIALIZER("red", 4),                    // PA04
	LED_STATIC_INITIALIZER("orange", 5),                 // PA05
	LED_STATIC_INITIALIZER("green", 6),                  // PA06
	LED_STATIC_INITIALIZER("blue", 7),                   // PA07
	LED_STATIC_INITIALIZER("can0_green", (2 << 4) | 13), // PC13
	LED_STATIC_INITIALIZER("can0_red", (2 << 4) | 14),   // PC14
	LED_STATIC_INITIALIZER("can1_green", (2 << 4) | 15), // PC15
	LED_STATIC_INITIALIZER("can1_red", (1 << 4) | 7),    // PB07
};

static inline void leds_init(void)
{
	/* configure clocks  */
	RCU_APB2EN |=
		RCU_APB2EN_PAEN |
		RCU_APB2EN_PBEN |
		RCU_APB2EN_PCEN;

	for (size_t i = 0 ; i < TU_ARRAY_SIZE(leds); ++i) {
		unsigned port = leds[i].port_pin_mux >> 4;
		unsigned pin = leds[i].port_pin_mux & 0xf;
		uint32_t regs = GPIO_BASE + port * PORT_STEP;

		GPIO_CTL1(regs) = (GPIO_CTL1(regs) & ~GPIO_MODE_MASK(pin)) | GPIO_MODE_SET(pin, GPIO_MODE_OUT_PP | GPIO_OSPEED_2MHZ);
	}
}

#define POWER_LED LED_DEBUG_0
#define USB_LED LED_DEBUG_3


extern void sc_board_led_set(uint8_t index, bool on)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(leds));

	unsigned port = leds[index].port_pin_mux >> 4;
	unsigned pin = leds[index].port_pin_mux & 0xf;
	uint32_t regs = GPIO_BASE + port * PORT_STEP;

	GPIO_BOP(regs) = UINT32_C(1) << (pin + (!on) * 16);
}

extern void sc_board_leds_on_unsafe(void)
{
	for (uint8_t i = 0; i < TU_ARRAY_SIZE(leds); ++i) {
		sc_board_led_set(i, 1);
	}
}


// extern uint32_t _svectors;
// extern uint32_t _evectors;

// static void move_vector_table_to_ram(void)
// {
// 	uint8_t* svectors = (void*)&_svectors;
// 	uint8_t* evectors = (void*)&_evectors;

// 	memcpy(svectors, (void*)SCB->VTOR, evectors - svectors);

// 	SCB->VTOR = (uint32_t)svectors;
// }

static void gd32_can_init(void)
{
	/* CAN0 */
	/* configure pins PB08 (RX), PB09 (TX) */
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(0)) | GPIO_MODE_SET(0, GPIO_MODE_AF_PP | GPIO_OSPEED_10MHZ);
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(1)) | GPIO_MODE_SET(1, GPIO_MODE_AF_PP | GPIO_OSPEED_10MHZ);

	/* enable clock */
	RCU_APB2EN |= RCU_APB2EN_PBEN | RCU_APB2EN_AFEN;
	RCU_APB1EN |= RCU_APB1EN_CAN0EN;

	/* reset CAN */
	CAN_CTL(CAN0) |= CAN_CTL_SWRST;
	/* wait for reset */
	while (CAN_CTL(CAN0) & CAN_CTL_SWRST);

	/* initial configuration */
	CAN_CTL(CAN0) =
	 (CAN_CTL(CAN0) & ~(CAN_CTL_DFZ | CAN_CTL_TTC | CAN_CTL_ABOR | CAN_CTL_AWU | CAN_CTL_RFOD | CAN_CTL_TFO | CAN_CTL_SLPWMOD | CAN_CTL_IWMOD)) |
	 (CAN_CTL_TFO | CAN_CTL_RFOD | CAN_CTL_IWMOD);

	/* interrupts */
	CAN_INTEN(CAN0) |=
		CAN_INTEN_ERRIE | /* error interrupt enable */
		CAN_INTEN_ERRNIE | /* error numer interrupt enable */
		CAN_INTEN_BOIE | /* bus-off interrupt enable */
		CAN_INTEN_PERRIE | /* passive error interrupt enable */
		CAN_INTEN_WERRIE | /* warning error interrupt enable */
		CAN_INTEN_RFOIE0 | /* receive FIFO0 overfull interrupt enable */
		CAN_INTEN_RFOIE0 | /* receive FIFO0 full interrupt enable */
		CAN_INTEN_RFNEIE0 | /* receive FIFO0 not empty interrupt enable */
		CAN_INTEN_TMEIE; /* transmit mailbox empty interrupt enable */


	/* disable filters */
	CAN_FCTL(CAN0) =
		(CAN_FCTL(CAN0) & ~(CAN_FCTL_FLD | CAN_FCTL_HBC1F)) |
		(FCTL_HBC1F(0) | CAN_FCTL_FLD);

	for (uint8_t i = 0; i < TU_ARRAY_SIZE(cans); ++i) {
		// struct can *can = &cans[i];

		sc_board_can_reset(i);
	}

	NVIC_SetPriority(CAN0_TX_IRQn, SC_ISR_PRIORITY);
	NVIC_SetPriority(CAN0_RX0_IRQn, SC_ISR_PRIORITY);
	NVIC_SetPriority(CAN0_EWMC_IRQn, SC_ISR_PRIORITY);
}

static inline void device_id_init(void)
{
	device_identifier =
		(*(uint32_t*)0x1FFFF7E8) ^
		(*(uint32_t*)0x1FFFF7EC) ^
		(*(uint32_t*)0x1FFFF7F0);

}

extern void sc_board_init_begin(void)
{
	__disable_irq();
	board_init();
	NVIC_SetPriority(USBFS_IRQn, SC_ISR_PRIORITY);
	__enable_irq();

	// LOG("Vectors ROM @ %p\n", (void*)SCB->VTOR);
	// move_vector_table_to_ram();
	// LOG("Vectors RAM @ %p\n", (void*)SCB->VTOR);

	device_id_init();
	leds_init();
	gd32_can_init();


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
}


SC_RAMFUNC extern void sc_board_led_can_status_set(uint8_t index, int status)
{
	struct can *can = &cans[index];

	switch (status) {
	case SC_CAN_LED_STATUS_DISABLED:
		led_set(can->led_status_green, 0);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_OFF_BUS:
		led_set(can->led_status_green, 1);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_PASSIVE:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_PASSIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ACTIVE:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		led_set(can->led_status_red, 0);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_PASSIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_PASSIVE_MS);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_ACTIVE:
		led_set(can->led_status_green, 0);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
		break;
	case SC_CAN_LED_STATUS_ENABLED_ON_BUS_BUS_OFF:
		led_set(can->led_status_green, 0);
		led_set(can->led_status_red, 1);
		break;
	default:
		led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
		led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
		break;
	}
}



__attribute__((noreturn)) extern void sc_board_reset(void)
{
	NVIC_SystemReset();
	__unreachable();
}

uint32_t sc_board_identifier(void)
{
	return device_identifier;
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

extern void sc_board_can_feat_set(uint8_t index, uint16_t features)
{
	struct can *can = &cans[index];

	can->features = features;
}

extern sc_can_bit_timing_range const* sc_board_can_nm_bit_timing_range(uint8_t index)
{
	(void)index;

	return &nm_range;
}

extern sc_can_bit_timing_range const* sc_board_can_dt_bit_timing_range(uint8_t index)
{
	(void)index;

	return &dt_range;
}

extern void sc_board_can_nm_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	struct can *can = &cans[index];

	can->nm = *bt;
}

extern void sc_board_can_dt_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	struct can *can = &cans[index];

	can->dt = *bt;
}

static void can_configure(uint8_t index)
{
	struct can *can = &cans[index];

	CAN_BT(can->regs) =
		(CAN_BT(can->regs) & ~(CAN_BT_SCMOD | CAN_BT_LCMOD | CAN_BT_BAUDPSC | CAN_BT_SJW | CAN_BT_BS1_6_4 | CAN_BT_BS1_3_0 | CAN_BT_BS2_4_3 | CAN_BT_BS2_2_0)) |
		(((can->features & SC_FEATURE_FLAG_MON_MODE) ? CAN_BT_SCMOD : 0) | BT_BAUDPSC(can->nm.brp) | BT_SJW(can->nm.sjw) | BT_BS1(can->nm.tseg1) | BT_BS2(can->nm.tseg2));

	if (can->features & SC_FEATURE_FLAG_FDF) {
		uint8_t tdcf = tu_min8((1 + can->dt.tseg1 + can->dt.tseg2 / 2), 128) - 1;
		uint8_t tdco = tu_min8((1 + can->dt.tseg1 - can->dt.tseg2 / 2), 128) - 1;

		CAN_FDCTL(can->regs) =
			(CAN_FDCTL(can->regs) & ~(CAN_FDCTL_FDEN | CAN_FDCTL_PRED | CAN_FDCTL_TDCEN | CAN_FDCTL_TDCMOD | CAN_FDCTL_ESIMOD | CAN_FDCTL_NISO)) |
			(CAN_FDCTL_FDEN | ((can->features & SC_FEATURE_FLAG_EHD) ? 0 : CAN_FDCTL_PRED) | ((sc_bitrate(can->dt.brp, can->dt.tseg1, can->dt.tseg2) >= 1000000) * CAN_FDCTL_TDCEN));


		/* transmitter delay compensation */
		CAN_FDTDC(can->regs) =
			(CAN_FDTDC(can->regs) & ~(CAN_FDTDC_TDCF | CAN_FDTDC_TDCO)) |
			(FDTDC_TDCF(tdcf) | FDTDC_TDCO(tdco));

	} else {
		CAN_FDCTL(can->regs) &= ~CAN_FDCTL_FDEN;
	}
}

static inline void can_clear_queues(uint8_t index)
{
	struct can *can = &cans[index];

	can->rx_get_index = 0;
	can->rx_put_index = 0;
	can->tx_get_index = 0;
	can->tx_put_index = 0;
	can->tx_mailbox_get_index = 0;
	can->tx_mailbox_put_index = 0;
}

static inline void can_off(uint8_t index)
{
	struct can *can = &cans[index];

	NVIC_DisableIRQ(CAN0_TX_IRQn);
	NVIC_DisableIRQ(CAN0_RX0_IRQn);
	NVIC_DisableIRQ(CAN0_EWMC_IRQn);

	/* abort scheduled transmissions */
	CAN_TSTAT(can->regs) |= CAN_TSTAT_MST0 | CAN_TSTAT_MST1 | CAN_TSTAT_MST2;
	/* initial working mode */
	CAN_CTL(can->regs) |= CAN_CTL_IWMOD;

}

void sc_board_can_go_bus(uint8_t index, bool on)
{
	struct can *can = &cans[index];

	if (on) {
		NVIC_EnableIRQ(CAN0_TX_IRQn);
		NVIC_EnableIRQ(CAN0_RX0_IRQn);
		NVIC_EnableIRQ(CAN0_EWMC_IRQn);

		can_configure(index);

		CAN_CTL(can->regs) &= ~CAN_CTL_IWMOD;
	} else {
		can_off(index);
		can_clear_queues(index);
	}
}

SC_RAMFUNC bool sc_board_can_tx_queue(uint8_t index, struct sc_msg_can_tx const * msg)
{
	return false;
}


void sc_board_can_reset(uint8_t index)
{
	struct can *can = &cans[index];

	can_off(index);

	can->features = CAN_FEAT_PERM;
	can->nm = nm_range.min;
	can->dt = dt_range.min;



	can_clear_queues(index);
}

SC_RAMFUNC int sc_board_can_retrieve(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	struct can *can = &cans[index];
	int result = 0;
	bool have_data_to_place = false;

	for (bool done = false; !done; ) {
		done = true;

		uint8_t pi = __atomic_load_n(&can->txr_put_index, __ATOMIC_ACQUIRE);

		if (can->txr_get_index != pi) {
			have_data_to_place = true;

			struct sc_msg_can_txr *msg = NULL;
			if ((size_t)(tx_end - tx_ptr) >= sizeof(*msg)) {
				uint8_t const get_index = can->txr_get_index % TU_ARRAY_SIZE(can->txr_queue);

				done = false;
				msg = (struct sc_msg_can_txr *)tx_ptr;
				tx_ptr += sizeof(*msg);
				result += sizeof(*msg);

				msg->id = SC_MSG_CAN_TXR;
				msg->len = sizeof(*msg);
				msg->track_id = can->txr_queue[get_index].track_id;
				msg->timestamp_us = can->txr_queue[get_index].ts;
				msg->flags = 0;
			}
		}
	}

	if (result > 0) {
		return result;
	}

	return have_data_to_place - 1;
}


SC_RAMFUNC static void can_int0(void)
{
	uint8_t const index = 0;
	struct can *can = &cans[index];
}

void CAN0_TX_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &cans[index];
	uint32_t inten = CAN_INTEN(can->regs);
	unsigned count = 0;
	uint32_t events = 0;
	uint16_t ts[3];
	uint8_t indices[3];

	LOG("CAN0_TX_IRQHandler\n");
	LOG("ch%u INTEN=%08x\n", index, inten);

	SC_DEBUG_ASSERT(inten & CAN_INTEN_TMEIE);

	CAN_INTEN(can->regs) &= ~CAN_INTEN_TMEIE;

	/* forward scan for finished mailboxes */
	for (bool done = false; !done; ) {
		uint8_t mailbox_index = can->tx_mailbox_get_index % 3;

		switch (mailbox_index) {
		case 0:
			if (CAN_TSTAT(can->regs) & CAN_TSTAT_MTF0) {
				ts[count] = (CAN_TMP(can->regs, mailbox_index) >> 16);
				indices[count] = mailbox_index;
				++count;
			} else {
				done = true;
			}
			break;
		case 1:
			if (CAN_TSTAT(can->regs) & CAN_TSTAT_MTF1) {
				ts[count] = (CAN_TMP(can->regs, mailbox_index) >> 16);
				indices[count] = mailbox_index;
				++count;
			} else {
				done = true;
			}
			break;
		case 2:
			if (CAN_TSTAT(can->regs) & CAN_TSTAT_MTF2) {
				ts[count] = (CAN_TMP(can->regs, mailbox_index) >> 16);
				indices[count] = mailbox_index;
				++count;
			} else {
				done = true;
			}
			break;
		default:
			__unreachable();
			break;
		}

		++can->tx_mailbox_get_index;
	}

	if (likely(count)) {
		// queue TXRs
		uint8_t gi = __atomic_load_n(&can->txr_get_index, __ATOMIC_ACQUIRE);
		uint8_t pi = can->txr_put_index;

		for (unsigned i = 0; i < count; ++i) {
			uint8_t mailbox_index = indices[i];
			uint8_t used = pi - gi;

			if (used == TU_ARRAY_SIZE(can->txr_queue)) {
				LOG("ch%u lost TXR track ID %02x\n", index, can->track_id_queue[mailbox_index]);
			} else {
				uint8_t const txr_index = pi++ % TU_ARRAY_SIZE(can->txr_queue);

				can->txr_queue[txr_index].ts = ts[i];
				can->txr_queue[txr_index].track_id = can->track_id_queue[mailbox_index];

				++events;
			}
		}

		__atomic_store_n(&can->txr_put_index, pi, __ATOMIC_RELEASE);
	}

	/* move frames from queue into mailboxes */
	uint8_t pi = __atomic_load_n(&can->tx_put_index, __ATOMIC_ACQUIRE);
	uint8_t gi = can->tx_get_index;

	while (gi != pi && can->tx_mailbox_put_index - can->tx_mailbox_get_index < 3) {
		uint8_t tx_index = gi % TU_ARRAY_SIZE(can->tx_queue);
		uint8_t tx_mailbox_index = can->tx_mailbox_put_index++ % 3;
		struct tx_frame *tx = &can->tx_queue[tx_index];

		can->track_id_queue[tx_mailbox_index] = tx->track_id;

		/* write properties so that the mailbox knows about FD */
		CAN_TMP(can->regs, tx_mailbox_index) = can->tx_queue[tx_index].tmp;

		for (unsigned j = 0; j < tx->words; ++j) {
			REG32((can->regs) + 0x188U + 0x10 * tx_mailbox_index) = tx->data[j];
		}

		CAN_TMI(can->regs, tx_mailbox_index) = can->tx_queue[tx_index].tmi;

		++gi;
	}

	__atomic_store_n(&can->tx_get_index, gi, __ATOMIC_RELEASE);

	if (likely(events)) {
		sc_can_notify_task_isr(index, events);
	}

	CAN_INTEN(can->regs) |= CAN_INTEN_TMEIE;
}

void CAN0_RX0_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &cans[index];
	uint32_t inten = CAN_INTEN(can->regs);

	LOG("CAN0_RX0_IRQHandler\n");
	LOG("ch%u INTEN=%08x\n", index, inten);

	SC_DEBUG_ASSERT(inten & (CAN_INTEN_RFNEIE0 | CAN_INTEN_RFFIE0 | CAN_INTEN_RFOIE0));

	CAN_INTEN(can->regs) &= ~(CAN_INTEN_RFNEIE0 | CAN_INTEN_RFFIE0 | CAN_INTEN_RFOIE0);
	CAN_INTEN(can->regs) |= (CAN_INTEN_RFNEIE0 | CAN_INTEN_RFFIE0 | CAN_INTEN_RFOIE0);
}

void CAN0_EWMC_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &cans[index];
	uint32_t inten = CAN_INTEN(can->regs);
	LOG("CAN0_EWMC_IRQHandler\n");
	LOG("ch%u INTEN=%08x\n", index, inten);


	SC_DEBUG_ASSERT(inten & (CAN_INTEN_WERRIE | CAN_INTEN_PERRIE | CAN_INTEN_BOIE | CAN_INTEN_BOIE | CAN_INTEN_ERRNIE | CAN_INTEN_ERRIE));

	CAN_INTEN(can->regs) &= ~(CAN_INTEN_WERRIE | CAN_INTEN_PERRIE | CAN_INTEN_BOIE | CAN_INTEN_BOIE | CAN_INTEN_ERRNIE | CAN_INTEN_ERRIE);
	CAN_INTEN(can->regs) |= (CAN_INTEN_WERRIE | CAN_INTEN_PERRIE | CAN_INTEN_BOIE | CAN_INTEN_BOIE | CAN_INTEN_ERRNIE | CAN_INTEN_ERRIE);
}


#endif // #if D5035_04
