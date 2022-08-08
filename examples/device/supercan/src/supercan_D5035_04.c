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
					| SC_FEATURE_FLAG_EHD
					/* not implemented */
					// | SC_FEATURE_FLAG_DAR
					| SC_FEATURE_FLAG_MON_MODE,
};


static const sc_can_bit_timing_range nm_range = {
	.min = {
		.brp = 1,
		.tseg1 = 1,
		.tseg2 = 1,
		.sjw = 1,
	},
	.max = {
		.brp = 1024,
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
	uint32_t mi;
	uint32_t mp;
	uint8_t track_id;
	uint8_t flags;
	uint8_t words;
	uint32_t data[16];
};

struct rx_frame {
	uint32_t mi;
	uint32_t mp;
	uint32_t data[16];
};

struct txr {
	uint32_t ts;
	uint8_t track_id;
	uint8_t flags;
};

#define IRQ_SENTINEL 0xff

static const uint8_t can0_irqs[] = { CAN0_TX_IRQn, CAN0_RX0_IRQn, CAN0_EWMC_IRQn, IRQ_SENTINEL };
static const uint8_t can1_irqs[] = { CAN1_TX_IRQn, CAN1_RX0_IRQn, CAN1_EWMC_IRQn, IRQ_SENTINEL };

struct can {
	struct rx_frame rx_queue[SC_BOARD_CAN_RX_FIFO_SIZE];
	struct tx_frame tx_queue[SC_BOARD_CAN_TX_FIFO_SIZE];
	struct txr txr_queue[SC_BOARD_CAN_TX_FIFO_SIZE];
	sc_can_bit_timing nm;
	sc_can_bit_timing dt;
	const uint8_t *const irqs;
	uint32_t const regs;
	uint8_t const tx_irq;
	uint16_t features;
	uint8_t track_id_boxes[3];
	uint8_t flags_boxes[3];
	uint8_t mailbox_queue[3];
	uint8_t const led_status_green;
	uint8_t const led_status_red;
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
		.led_status_green = LED_CAN1_STATUS_GREEN,
		.led_status_red = LED_CAN1_STATUS_RED,
		.regs = CAN1,
		.irqs = can1_irqs,
		.tx_irq = CAN1_TX_IRQn,
	},
};

static uint32_t device_identifier;

struct led {
	uint8_t port_pin_mux;
};

#define LED_STATIC_INITIALIZER(name, pin) \
	{ pin }

static uint16_t top_1us;

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
		uint32_t regs = GPIO_BASE + port * PORT_STEP + 4 * (pin >= 8);

		GPIO_CTL0(regs) = (GPIO_CTL0(regs) & ~GPIO_MODE_MASK(pin & 0x7)) | GPIO_MODE_SET(pin & 0x7, GPIO_MODE_OUT_PP | GPIO_OSPEED_2MHZ);
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


extern uint32_t _svectors;
extern uint32_t _evectors;

static void move_vector_table_to_ram(void)
{
	uint8_t* svectors = (void*)&_svectors;
	uint8_t* evectors = (void*)&_evectors;

	memcpy(svectors, (void*)SCB->VTOR, evectors - svectors);

	SCB->VTOR = (uint32_t)svectors;
	__DSB();
}

static inline void dump_can_regs(uint8_t index)
{
	struct can *can = &cans[index];

	(void)can;

	LOG("ch%u:\nCTL=%08x\nSTAT=%08x\nFDCTL=%08x\nFDSTAT=%08x\nBT=%08x\nDBT=%08x\nFDTDC=%08x\n",
		index, CAN_CTL(can->regs), CAN_STAT(can->regs),
		CAN_FDCTL(can->regs), CAN_FDSTAT(can->regs),
		CAN_BT(can->regs), CAN_DBT(can->regs), CAN_FDTDC(can->regs));

	LOG("RCU_CTL=%08x\nRCU_CFG0=%08x\nRCU_CFG1=%08x\nRCU_AHBEN=%08x\nRCU_APB1EN=%08x\nRCU_APB2EN=%08x\n",
		RCU_CTL, RCU_CFG0, RCU_CFG1, RCU_AHBEN, RCU_APB1EN, RCU_APB2EN);

	LOG("GPIO_CTL0(A)=%08x\nGPIO_CTL1(A)=%08x\nGPIO_CTL0(B)=%08x\nGPIO_CTL1(B)=%08x\n",
		GPIO_CTL0(GPIOA), GPIO_CTL1(GPIOA), GPIO_CTL0(GPIOB), GPIO_CTL1(GPIOB));
	LOG("GPIO_CTL0(C)=%08x\nGPIO_CTL1(C)=%08x\nGPIO_CTL0(D)=%08x\nGPIO_CTL1(D)=%08x\n",
		GPIO_CTL0(GPIOC), GPIO_CTL1(GPIOC), GPIO_CTL0(GPIOD), GPIO_CTL1(GPIOD));

}

static void gd32_can_init(void)
{
	/* enable clock GPIO port and AF clock */
	RCU_APB2EN |= RCU_APB2EN_PBEN | RCU_APB2EN_PDEN | RCU_APB2EN_AFEN;
	/* enable CAN clocks */
	RCU_APB1EN |= RCU_APB1EN_CAN0EN | RCU_APB1EN_CAN1EN;

	/* CAN0 configure pins PB08 (RX), PB09 (TX) */
	GPIO_BOP(GPIOB) = UINT32_C(1) << 8;
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(0)) | GPIO_MODE_SET(0, GPIO_MODE_IPU | GPIO_OSPEED_50MHZ);
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(1)) | GPIO_MODE_SET(1, GPIO_MODE_AF_PP | GPIO_OSPEED_50MHZ);


	/* CAN1 configure pins PB12 (RX), PB13 (TX) */
	GPIO_BOP(GPIOB) = UINT32_C(1) << 12;
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(4)) | GPIO_MODE_SET(4, GPIO_MODE_IPU | GPIO_OSPEED_50MHZ);
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(5)) | GPIO_MODE_SET(5, GPIO_MODE_AF_PP | GPIO_OSPEED_50MHZ);


	/* remap CAN0 partially */
	AFIO_PCF0 =
		(AFIO_PCF0 & ~(AFIO_PCF0_CAN0_REMAP | AFIO_PCF0_CAN1_REMAP)) |
		(PCF0_CAN_REMAP(2) | AFIO_PCF0_CAN1_REMAP);

	/* reset CANs */
	RCU_APB1RST |= RCU_APB1RST_CAN0RST | RCU_APB1RST_CAN1RST;
	RCU_APB1RST &= ~(RCU_APB1RST_CAN0RST | RCU_APB1RST_CAN1RST);



	for (uint8_t i = 0; i < TU_ARRAY_SIZE(cans); ++i) {
		struct can *can = &cans[i];

		// /* reset CAN */
		// CAN_CTL(can->regs) |= CAN_CTL_SWRST;
		// /* wait for reset */
		// while (CAN_CTL(can->regs) & CAN_CTL_SWRST);

			// CAN_CTL(can->regs) &= ~(CAN_CTL_SLPWMOD | CAN_CTL_IWMOD);
			// while (CAN_STAT(can->regs) & CAN_STAT_IWS);



		CAN_CTL(can->regs) &= ~CAN_CTL_SLPWMOD;
		CAN_CTL(can->regs) |= CAN_CTL_IWMOD;

		/* wait for initial working mode */
		while (!(CAN_STAT(can->regs) & CAN_STAT_IWS));

		CAN_CTL(can->regs) =
			(CAN_CTL(can->regs) & ~(CAN_CTL_DFZ | CAN_CTL_TTC | CAN_CTL_ABOR | CAN_CTL_ARD | CAN_CTL_AWU | CAN_CTL_RFOD | CAN_CTL_TFO | CAN_CTL_SLPWMOD | CAN_CTL_IWMOD)) |
			(CAN_CTL_IWMOD | CAN_CTL_RFOD | CAN_CTL_DFZ | CAN_CTL_ARD);

		// while (!(CAN_STAT(can->regs) & CAN_STAT_IWS));
		LOG("IWS\n");

		CAN_STAT(can->regs) |= CAN_STAT_WUIF;
		// CAN_BT(can->regs) = 0x00000027;

		// // dump_can_regs(i);

		// CAN_CTL(can->regs) &= ~CAN_CTL_IWMOD;

		// while ((CAN_STAT(can->regs) & CAN_STAT_IWS));

		// LOG("WS\n");

		/* interrupts */
		CAN_INTEN(can->regs) |=
			CAN_INTEN_ERRIE | /* error interrupt enable */
			CAN_INTEN_ERRNIE | /* error numer interrupt enable */
			CAN_INTEN_BOIE | /* bus-off interrupt enable */
			CAN_INTEN_PERRIE | /* passive error interrupt enable */
			CAN_INTEN_WERRIE | /* warning error interrupt enable */
			CAN_INTEN_RFOIE0 | /* receive FIFO0 overfull interrupt enable */
			CAN_INTEN_RFOIE0 | /* receive FIFO0 full interrupt enable */
			CAN_INTEN_RFNEIE0 | /* receive FIFO0 not empty interrupt enable */
			CAN_INTEN_TMEIE; /* transmit mailbox empty interrupt enable */


		/* setup filter for FIFO0 (assign ALL, use 1) */
		CAN_FCTL(can->regs) =
			(CAN_FCTL(can->regs) & ~(CAN_FCTL_FLD | CAN_FCTL_HBC1F)) |
			(FCTL_HBC1F(28) | CAN_FCTL_FLD);

		/* disable filters */
		CAN_FW(can->regs) = 0;

		/* configure 32 bit filter on index 0 */
		CAN_FSCFG(can->regs) = BIT(0);
		/* reset default: mask mode */
		CAN_FMCFG(can->regs) = 0;
		/* reset default: all filters assigned to fifo 0 */
		CAN_FAFIFO(can->regs) = 0;
		/* set pass everything mask */
		CAN_F0DATA0(can->regs) = 0;
		CAN_F1DATA0(can->regs) = 0;

		/* enable filter 0 */
		CAN_FW(can->regs) = BIT(0);

		CAN_FCTL(can->regs) &= ~CAN_FCTL_FLD;


		sc_board_can_reset(i);

		for (unsigned j = 0; can->irqs[j] != IRQ_SENTINEL; ++j) {
			NVIC_SetPriority(can->irqs[j], SC_ISR_PRIORITY);
		}
	}
}

static inline void device_id_init(void)
{
	device_identifier =
		(*(uint32_t*)0x1FFFF7E8) ^
		(*(uint32_t*)0x1FFFF7EC) ^
		(*(uint32_t*)0x1FFFF7F0);

}

static inline void timer_1mhz_init(void)
{
	RCU_APB1EN |= RCU_APB1EN_TIMER5EN;

	NVIC_SetPriority(TIMER5_IRQn, SC_ISR_PRIORITY);
	NVIC_EnableIRQ(TIMER5_IRQn);

	TIMER_PSC(TIMER5) = 119;
	TIMER_DMAINTEN(TIMER5) = TIMER_DMAINTEN_UPIE;
	TIMER_CAR(TIMER5) = 0xffff;
	TIMER_CTL0(TIMER5) = TIMER_CTL0_CEN;
}

extern void sc_board_init_begin(void)
{
	__disable_irq();
	board_init();

	/* change interrupt prio */
	NVIC_SetPriority(USBFS_IRQn, SC_ISR_PRIORITY);

	__enable_irq();

	LOG("Vectors ROM @ %p\n", (void*)SCB->VTOR);
	move_vector_table_to_ram();
	LOG("Vectors RAM @ %p\n", (void*)SCB->VTOR);


	device_id_init();
	leds_init();
	gd32_can_init();
	timer_1mhz_init();

	LOG("sc_board_init_begin exit\n");
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

	if (can->features & SC_FEATURE_FLAG_DAR) {
		CAN_CTL(can->regs) |= CAN_CTL_ARD;
	} else {
		CAN_CTL(can->regs) &= ~CAN_CTL_ARD;
	}

	/* nominal bitrate */
	CAN_BT(can->regs) =
		(CAN_BT(can->regs) & ~(CAN_BT_SCMOD | CAN_BT_LCMOD | CAN_BT_BAUDPSC | CAN_BT_SJW | CAN_BT_BS1_6_4 | CAN_BT_BS1_3_0 | CAN_BT_BS2_4_3 | CAN_BT_BS2_2_0)) |
		(
			((can->features & SC_FEATURE_FLAG_MON_MODE) ? CAN_BT_SCMOD : 0) |
			BT_BAUDPSC(can->nm.brp-1) |
			BT_SJW(can->nm.sjw-1) |
			BT_BS1(can->nm.tseg1-1) |
			BT_BS2(can->nm.tseg2-1) |
			CAN_BT_SCMOD | CAN_BT_LCMOD
		);

	if (can->features & SC_FEATURE_FLAG_FDF) {
		uint8_t tdcf = tu_min8((1 + can->dt.tseg1 + can->dt.tseg2 / 2), 128);
		uint8_t tdco = tu_min8((1 + can->dt.tseg1 - can->dt.tseg2 / 2), 128);

		CAN_FDCTL(can->regs) =
			(CAN_FDCTL(can->regs) & ~(CAN_FDCTL_FDEN | CAN_FDCTL_PRED | CAN_FDCTL_TDCEN | CAN_FDCTL_TDCMOD | CAN_FDCTL_ESIMOD | CAN_FDCTL_NISO)) |
			(
				CAN_FDCTL_FDEN |
				((can->features & SC_FEATURE_FLAG_EHD) ? 0 : CAN_FDCTL_PRED) |
				((sc_bitrate(can->dt.brp, can->dt.tseg1, can->dt.tseg2) >= 1000000) * CAN_FDCTL_TDCEN) |
				CAN_FDCTL_ESIMOD
			);


		/* transmitter delay compensation */
		CAN_FDTDC(can->regs) =
			(CAN_FDTDC(can->regs) & ~(CAN_FDTDC_TDCF | CAN_FDTDC_TDCO)) |
			(FDTDC_TDCF(tdcf) | FDTDC_TDCO(tdco));

		/* data bitrate */
		CAN_DBT(can->regs) =
			(CAN_DBT(can->regs) & ~(CAN_DBT_DBAUDPSC | CAN_DBT_DBS1 | CAN_DBT_DBS2 | CAN_DBT_DSJW)) |
			(BT_BAUDPSC(can->dt.brp-1) | BT_DSJW(can->dt.sjw-1) | BT_DBS1(can->dt.tseg1-1) | BT_DBS2(can->dt.tseg2-1));

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

	for (unsigned i = 0; can->irqs[i] != IRQ_SENTINEL; ++i) {
		NVIC_DisableIRQ(can->irqs[i]);
	}

	/* abort scheduled transmissions */
	CAN_TSTAT(can->regs) |= CAN_TSTAT_MST0 | CAN_TSTAT_MST1 | CAN_TSTAT_MST2;
	/* initial working mode */
	CAN_CTL(can->regs) |= CAN_CTL_IWMOD;
}

void sc_board_can_go_bus(uint8_t index, bool on)
{
	struct can *can = &cans[index];

	if (on) {
		can_configure(index);

		for (unsigned i = 0; can->irqs[i] != IRQ_SENTINEL; ++i) {
			NVIC_EnableIRQ(can->irqs[i]);
		}

		// dump_can_regs(index);

		CAN_CTL(can->regs) &= ~CAN_CTL_IWMOD;

		// dump_can_regs(index);

		/* wait for normal working mode */
		while (CAN_STAT(can->regs) & CAN_STAT_IWS);

		// dump_can_regs(index);
	} else {
		can_off(index);
		can_clear_queues(index);
	}
}

SC_RAMFUNC bool sc_board_can_tx_queue(uint8_t index, struct sc_msg_can_tx const * msg)
{
	struct can *can = &cans[index];
	uint8_t pi = can->tx_put_index;
	uint8_t gi = __atomic_load_n(&can->tx_get_index, __ATOMIC_ACQUIRE);
	uint8_t used = pi - gi;
	struct tx_frame *tx;

	if (unlikely(used == TU_ARRAY_SIZE(can->tx_queue))) {
		return false;
	}

	tx = &can->tx_queue[pi % TU_ARRAY_SIZE(can->tx_queue)];

	tx->track_id = msg->track_id;
	tx->flags = msg->flags;

	if (msg->flags & SC_CAN_FRAME_FLAG_EXT) {
		tx->mi = CAN_TMI_TEN | TMI_EFID(msg->can_id) | CAN_FF_EXTENDED;
	} else {
		tx->mi = CAN_TMI_TEN | TMI_SFID(msg->can_id);
	}

	if (msg->flags & SC_CAN_FRAME_FLAG_FDF) {
		tx->words = (dlc_to_len(msg->dlc) + 3) / 4;

		tx->mp = CAN_TMP_FDF | msg->dlc;
		if (likely(msg->flags & SC_CAN_FRAME_FLAG_BRS)) {
			tx->mp |= CAN_TMP_BRS;
		}

		if (unlikely(msg->flags & SC_CAN_FRAME_FLAG_ESI)) {
			tx->mp |= CAN_TMP_ESI;
		}

	} else if (unlikely(msg->flags & SC_CAN_FRAME_FLAG_RTR)) {
		tx->mi |= CAN_FT_REMOTE;
		tx->words = 0;
		tx->mp = msg->dlc;
	} else {
		tx->words = (dlc_to_len(msg->dlc) + 3) / 4;
		tx->mp = msg->dlc;
	}

	if (likely(tx->words)) {
		memcpy(tx->data, tx+1, tx->words * 4);
	}

	__atomic_store_n(&can->tx_put_index, pi + 1, __ATOMIC_RELEASE);

	/* trigger interrupt handler */
	NVIC_SetPendingIRQ(can->tx_irq);

	return true;
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

	// LOG("1\n");

	for (bool done = false; !done; ) {
		done = true;

		uint8_t pi = __atomic_load_n(&can->txr_put_index, __ATOMIC_ACQUIRE);

		if (can->txr_get_index != pi) {
			struct sc_msg_can_txr *msg = NULL;

			have_data_to_place = true;

			if ((size_t)(tx_end - tx_ptr) >= sizeof(*msg)) {
				uint8_t const get_index = can->txr_get_index % TU_ARRAY_SIZE(can->txr_queue);
				struct txr *txr = &can->txr_queue[get_index];

				done = false;
				msg = (struct sc_msg_can_txr *)tx_ptr;
				tx_ptr += sizeof(*msg);
				result += sizeof(*msg);

				msg->id = SC_MSG_CAN_TXR;
				msg->len = sizeof(*msg);
				msg->track_id = txr->track_id;
				msg->timestamp_us = txr->ts;
				msg->flags = txr->flags;

				__atomic_store_n(&can->txr_get_index, can->txr_get_index + 1, __ATOMIC_RELEASE);


				// LOG("2\n");
			}
		}
	}

	// LOG("3\n");

	if (result > 0) {
		return result;
	}

	return have_data_to_place - 1;
}



SC_RAMFUNC void CAN1_TX_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &cans[index];
	uint32_t inten = CAN_INTEN(can->regs);
	unsigned count = 0;
	uint32_t events = 0;
	uint16_t ts[3];
	uint8_t indices[3];

	// LOG("T");

	LOG("ch%u TSTAT=%08x\n", index, CAN_TSTAT(can->regs));

	/* forward scan for finished mailboxes */
	for (bool done = false; !done && can->tx_mailbox_get_index != can->tx_mailbox_put_index; ) {
		uint8_t mailbox_queue_index = can->tx_mailbox_get_index % TU_ARRAY_SIZE(can->mailbox_queue);
		uint8_t mailbox_index = can->mailbox_queue[mailbox_queue_index];

		switch (mailbox_index) {
		case 0:
			if (CAN_TSTAT(can->regs) & CAN_TSTAT_MTF0) {
				ts[count] = (CAN_TMP(can->regs, mailbox_index) >> 16);
				indices[count] = mailbox_index;
				++count;
				++can->tx_mailbox_get_index;

				CAN_TSTAT(can->regs) |= CAN_TSTAT_MTF0 | CAN_TSTAT_MTFNERR0 | CAN_TSTAT_MAL0 | CAN_TSTAT_MTE0;
			} else {
				done = true;
			}
			break;
		case 1:
			if (CAN_TSTAT(can->regs) & CAN_TSTAT_MTF1) {
				ts[count] = (CAN_TMP(can->regs, mailbox_index) >> 16);
				indices[count] = mailbox_index;
				++count;
				++can->tx_mailbox_get_index;

				CAN_TSTAT(can->regs) |= CAN_TSTAT_MTF1 | CAN_TSTAT_MTFNERR1 | CAN_TSTAT_MAL1 | CAN_TSTAT_MTE1;
			} else {
				done = true;
			}
			break;
		case 2:
			if (CAN_TSTAT(can->regs) & CAN_TSTAT_MTF2) {
				ts[count] = (CAN_TMP(can->regs, mailbox_index) >> 16);
				indices[count] = mailbox_index;
				++count;
				++can->tx_mailbox_get_index;

				CAN_TSTAT(can->regs) |= CAN_TSTAT_MTF2 | CAN_TSTAT_MTFNERR2 | CAN_TSTAT_MAL2 | CAN_TSTAT_MTE2;
			} else {
				done = true;
			}
			break;
		default:
			__unreachable();
			break;
		}
	}


	if (likely(count)) {
		// queue TXRs
		uint8_t gi = __atomic_load_n(&can->txr_get_index, __ATOMIC_ACQUIRE);
		uint8_t pi = can->txr_put_index;

		for (unsigned i = 0; i < count; ++i) {
			uint8_t mailbox_index = indices[i];
			uint8_t used = pi - gi;

			if (used == TU_ARRAY_SIZE(can->txr_queue)) {
				LOG("ch%u lost TXR track ID %02x\n", index, can->track_id_boxes[mailbox_index]);
			} else {
				uint8_t const txr_index = pi++ % TU_ARRAY_SIZE(can->txr_queue);

				can->txr_queue[txr_index].ts = ts[i];
				can->txr_queue[txr_index].track_id = can->track_id_boxes[mailbox_index];
				can->txr_queue[txr_index].flags = can->flags_boxes[mailbox_index];

				++events;
			}
		}

		__atomic_store_n(&can->txr_put_index, pi, __ATOMIC_RELEASE);
	}

	/* move frames from queue into mailboxes */
	uint8_t pi = __atomic_load_n(&can->tx_put_index, __ATOMIC_ACQUIRE);
	uint8_t gi = can->tx_get_index;

	while (gi != pi && (can->tx_mailbox_put_index - can->tx_mailbox_get_index) < TU_ARRAY_SIZE(can->mailbox_queue)) {
		uint8_t tx_index = gi % TU_ARRAY_SIZE(can->tx_queue);
		uint8_t mailbox_index = (CAN_TSTAT(can->regs)  >> 24) & 3;
		struct tx_frame *tx = &can->tx_queue[tx_index];

		can->track_id_boxes[mailbox_index] = tx->track_id;
		can->flags_boxes[mailbox_index] = tx->flags;

		/* write properties so that the mailbox knows about FD */
		CAN_TMP(can->regs, mailbox_index) = tx->mp;

		for (unsigned j = 0; j < tx->words; ++j) {
			REG32((can->regs) + 0x188U + 0x10 * mailbox_index) = tx->data[j];
		}

		CAN_TMI(can->regs, mailbox_index) = tx->mi;

		can->mailbox_queue[can->tx_mailbox_put_index++ % TU_ARRAY_SIZE(can->mailbox_queue)] = mailbox_index;

		++gi;
	}

	__atomic_store_n(&can->tx_get_index, gi, __ATOMIC_RELEASE);

	if (likely(events)) {
		sc_can_notify_task_isr(index, events);
	}
}

SC_RAMFUNC extern uint32_t sc_board_can_ts_fetch_isr(void)
{
	uint16_t bottom = TIMER_CNT(TIMER5);
	uint16_t top = __atomic_load_n(&top_1us, __ATOMIC_ACQUIRE);
	__ISB();

	if (unlikely(TIMER_INTF(TIMER5) & TIMER_INTF_UPIF)) {
		++top;
	}

	return  (((uint32_t)top) << 16) | bottom;
}

SC_RAMFUNC void CAN1_RX0_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &cans[index];
	uint32_t inten = CAN_INTEN(can->regs);
	uint32_t rfifo0 = CAN_RFIFO0(can->regs);
	uint32_t events = 0;
	uint32_t tsc = sc_board_can_ts_fetch_isr();
	uint16_t rx_lost = 0;


	LOG("CAN1_RX0_IRQHandler\n");
	LOG("R");

	SC_DEBUG_ASSERT(inten & (CAN_INTEN_RFNEIE0 | CAN_INTEN_RFFIE0 | CAN_INTEN_RFOIE0));

	uint8_t pi = can->rx_put_index;
	uint8_t gi = __atomic_load_n(&can->rx_get_index, __ATOMIC_ACQUIRE);

	do {
		uint8_t used = pi - gi;

		if (likely(used < TU_ARRAY_SIZE(can->rx_queue))) {
			uint8_t put_index = pi % TU_ARRAY_SIZE(can->rx_queue);
			struct rx_frame *rx = &can->rx_queue[put_index];
			uint8_t words;


			rx->mi = CAN_RFIFOMI0(can->regs);
			rx->mp = CAN_RFIFOMP0(can->regs);

			words = (dlc_to_len(rx->mp & 0xf) + 3) / 4;

			for (uint8_t i = 0; i < words; i += 2) {
				rx->data[i] = CAN_RFIFOMDATA00(can->regs);
				rx->data[i+1] = CAN_RFIFOMDATA01(can->regs);
			}
		} else {
			++rx_lost;
		}

		++pi;
		++events;

		// dequeue done
		CAN_RFIFO0(can->regs) |= CAN_RFIFO0_RFD0;
	} while (CAN_RFIFO0(can->regs) & 3);

	__atomic_store_n(&can->rx_put_index, pi, __ATOMIC_RELEASE);

	if (unlikely(rfifo0 & CAN_RFIFO0_RFO0)) {
		/* RX fifo overflow */
		++rx_lost;
	}

	if (unlikely(rx_lost)) {
		sc_can_status status;

		status.type = SC_CAN_STATUS_FIFO_TYPE_RX_LOST;
		status.timestamp_us = tsc;
		status.rx_lost = rx_lost;

		sc_can_status_queue(index, &status);
		++ events;
	}

	if (likely(events)) {
		sc_can_notify_task_isr(index, events);
	}
}

SC_RAMFUNC void CAN1_EWMC_IRQHandler(void)
{
	uint8_t const index = 0;
	struct can *can = &cans[index];
	uint32_t inten = CAN_INTEN(can->regs);
	LOG("CAN0_EWMC_IRQHandler\n");
	LOG("E");

	SC_DEBUG_ASSERT(inten & (CAN_INTEN_WERRIE | CAN_INTEN_PERRIE | CAN_INTEN_BOIE | CAN_INTEN_BOIE | CAN_INTEN_ERRNIE | CAN_INTEN_ERRIE));

	// CAN_INTEN(can->regs) &= ~(CAN_INTEN_WERRIE | CAN_INTEN_PERRIE | CAN_INTEN_BOIE | CAN_INTEN_BOIE | CAN_INTEN_ERRNIE | CAN_INTEN_ERRIE);
	// CAN_INTEN(can->regs) |= (CAN_INTEN_WERRIE | CAN_INTEN_PERRIE | CAN_INTEN_BOIE | CAN_INTEN_BOIE | CAN_INTEN_ERRNIE | CAN_INTEN_ERRIE);

	CAN_STAT(can->regs) |= CAN_STAT_ERRIF;
}

SC_RAMFUNC void TIMER5_IRQHandler(void)
{
	// LOG("TIMER5_IRQHandler\n");

	TIMER_INTF(TIMER5) &= ~TIMER_INTF_UPIF;

	// static bool on = false;

	// // GPIO_BOP(GPIOB) = UINT32_C(1) << (9 + (!on) * 16);
	// GPIO_BOP(GPIOA) = (UINT32_C(1) << (5 + (!on) * 16)) | (UINT32_C(1) << (6 + (!on) * 16));

	// on = !on;

	__atomic_store_n(&top_1us, top_1us + 1, __ATOMIC_RELEASE);
}

#endif // #if D5035_04
