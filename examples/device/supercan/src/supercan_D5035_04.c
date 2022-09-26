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
	uint32_t timestamp_us;
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
// static const uint8_t can1_irqs[] = { CAN1_TX_IRQn, CAN1_RX0_IRQn, CAN1_EWMC_IRQn, IRQ_SENTINEL };

struct can {
	struct rx_frame rx_queue[SC_BOARD_CAN_RX_FIFO_SIZE];
	struct tx_frame tx_queue[SC_BOARD_CAN_TX_FIFO_SIZE];
	struct txr txr_queue[SC_BOARD_CAN_TX_FIFO_SIZE];
	sc_can_bit_timing nm;
	sc_can_bit_timing dt;
	const uint8_t *const irqs;
	uint32_t const regs;
	uint32_t nm_us_per_bit;
#if SUPERCAN_DEBUG
	uint32_t txr;
#endif
	uint16_t features;
	uint8_t const tx_irq;
	uint8_t track_id_boxes[3];
	uint8_t flags_boxes[3];
	uint8_t mailbox_queue[3];
	uint8_t const led_status_green;
	uint8_t const led_status_red;
	uint8_t rx_get_index; // NOT an index, uses full range of type
	uint8_t rx_put_index; // NOT an index, uses full range of type
	uint8_t tx_get_index; // NOT an index, uses full range of type
	uint8_t tx_put_index; // NOT an index, uses full range of type
	uint8_t tx_mailbox_queue_get_index; // NOT an index, uses full range of type
	uint8_t tx_mailbox_queue_put_index; // NOT an index, uses full range of type
	uint8_t txr_get_index; // NOT an index, uses full range of type
	uint8_t txr_put_index; // NOT an index, uses full range of type
	uint8_t notify; // keep notifying main loop for busy light
	uint8_t int_prev_rx_errors;
	uint8_t int_prev_tx_errors;
	uint8_t int_prev_bus_state;
} cans[SC_BOARD_CAN_COUNT] = {
	{
		.led_status_green = LED_CAN0_STATUS_GREEN,
		.led_status_red = LED_CAN0_STATUS_RED,
		.regs = CAN0,
		.irqs = can0_irqs,
		.tx_irq = CAN0_TX_IRQn,
		.notify = 0,
	},
	// {
	// 	.led_status_green = LED_CAN1_STATUS_GREEN,
	// 	.led_status_red = LED_CAN1_STATUS_RED,
	// 	.regs = CAN1,
	// 	.irqs = can1_irqs,
	// 	.tx_irq = CAN1_TX_IRQn,
	// 	.notify = 0,
	// },
};

static const uint8_t sc_can_bus_error_map[8] = {
	SC_CAN_ERROR_NONE,
	SC_CAN_ERROR_STUFF,
	SC_CAN_ERROR_FORM,
	SC_CAN_ERROR_ACK,
	SC_CAN_ERROR_BIT1,
	SC_CAN_ERROR_BIT0,
	SC_CAN_ERROR_CRC,
	SC_CAN_ERROR_NONE
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

	// assume IRQs are disable (or peripheral doesn't generate any)
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
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(0)) | GPIO_MODE_SET(0, GPIO_MODE_IPU);
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(1)) | GPIO_MODE_SET(1, GPIO_MODE_AF_PP | GPIO_OSPEED_50MHZ);


	/* CAN1 configure pins PB12 (RX), PB13 (TX) */
	GPIO_BOP(GPIOB) = UINT32_C(1) << 12;
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(4)) | GPIO_MODE_SET(4, GPIO_MODE_AF_PP);
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(5)) | GPIO_MODE_SET(5, GPIO_MODE_AF_PP | GPIO_OSPEED_50MHZ);


	/* remap CAN0 partially */
	AFIO_PCF0 =
		(AFIO_PCF0 & ~(AFIO_PCF0_CAN0_REMAP | AFIO_PCF0_CAN1_REMAP)) |
		(PCF0_CAN_REMAP(2));

	// /* reset CANs */
	// RCU_APB1RST |= RCU_APB1RST_CAN0RST | RCU_APB1RST_CAN1RST;
	// RCU_APB1RST &= ~(RCU_APB1RST_CAN0RST | RCU_APB1RST_CAN1RST);


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
		// LOG("IWS\n");

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

		/* configure 32 bit filter 0 */
		CAN_FSCFG(can->regs) = 1;
		/* reset default: mask mode */
		CAN_FMCFG(can->regs) = 0;
		/* reset default: all filters assigned to fifo 0 */
		CAN_FAFIFO(can->regs) = 0;

		/* set pass everything mask */
		CAN_F0DATA0(can->regs) = 0;
		CAN_F0DATA1(can->regs) = 0;

		/* enable filter 0 */
		CAN_FW(can->regs) = 1;

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
			BT_BS2(can->nm.tseg2-1)
			// | CAN_BT_SCMOD | CAN_BT_LCMOD
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

	// We must enable FD mode to get the extra bits for TSEG1, TSEG2, sigh
	CAN_FDCTL(can->regs) |= CAN_FDCTL_FDEN;

	can->nm_us_per_bit = UINT32_C(1000000) / sc_bitrate(can->nm.brp, can->nm.tseg1, can->nm.tseg2);
}

static inline void can_clear_queues(uint8_t index)
{
	struct can *can = &cans[index];

	can->rx_get_index = 0;
	can->rx_put_index = 0;
	can->tx_get_index = 0;
	can->tx_put_index = 0;
	can->tx_mailbox_queue_get_index = 0;
	can->tx_mailbox_queue_put_index = 0;
	can->txr_get_index = 0;
	can->txr_put_index = 0;

#if SUPERCAN_DEBUG
	can->txr = 0;
#endif
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


	__atomic_store_n(&can->notify, 0, __ATOMIC_RELEASE);

	can->int_prev_rx_errors = 0;
	can->int_prev_tx_errors = 0;
	can->int_prev_bus_state = SC_CAN_STATUS_ERROR_ACTIVE;

	__atomic_thread_fence(__ATOMIC_RELEASE); // int_*
}

void sc_board_can_go_bus(uint8_t index, bool on)
{
	struct can *can = &cans[index];

	if (on) {
		can_configure(index);

		for (unsigned i = 0; can->irqs[i] != IRQ_SENTINEL; ++i) {
			NVIC_EnableIRQ(can->irqs[i]);
		}

		CAN_CTL(can->regs) &= ~CAN_CTL_IWMOD;

		/* wait for normal working mode */
		while (CAN_STAT(can->regs) & CAN_STAT_IWS);

		// dump_can_regs(index);

		__atomic_store_n(&can->notify, 1, __ATOMIC_RELEASE);
	} else {
		can_off(index);
		can_clear_queues(index);
	}
}

SC_RAMFUNC bool sc_board_can_tx_queue(uint8_t index, struct sc_msg_can_tx const * msg)
{
	struct can *can = &cans[index];
	uint8_t pi = can->tx_put_index;
	uint8_t const gi = __atomic_load_n(&can->tx_get_index, __ATOMIC_ACQUIRE);
	uint8_t const used = pi - gi;
	uint8_t put_index = 0;
	struct tx_frame *tx;

	if (unlikely(used == TU_ARRAY_SIZE(can->tx_queue))) {
		return false;
	}

#if SUPERCAN_DEBUG
	SC_DEBUG_ASSERT(!(can->txr & (UINT32_C(1) << msg->track_id)));
	can->txr |= (UINT32_C(1) << msg->track_id);
#endif

	put_index = pi % TU_ARRAY_SIZE(can->tx_queue);
	tx = &can->tx_queue[put_index];

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

	SC_DEBUG_ASSERT(tx->words <= 16);

	if (likely(tx->words)) {
		memcpy(tx->data, tx+1, tx->words * 4);
	}

	__atomic_store_n(&can->tx_put_index, pi + 1, __ATOMIC_RELEASE);

	LOG("ch%u tx i=%u\n", index, put_index);

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

SC_RAMFUNC int sc_board_can_retrieve(uint8_t index, uint8_t *tx_ptr, uint8_t * const tx_end)
{
	struct can *can = &cans[index];
	int result = 0;
	bool have_data_to_place = false;
	// uint8_t * const tx_start = tx_ptr;

	for (bool done = false; !done; ) {
		done = true;

		uint8_t const txr_pi = __atomic_load_n(&can->txr_put_index, __ATOMIC_ACQUIRE);

		if (can->txr_get_index != txr_pi) {
			struct sc_msg_can_txr *msg = NULL;

			_Static_assert(sizeof(*msg) == 8);

			have_data_to_place = true;

			if ((size_t)(tx_end - tx_ptr) >= sizeof(*msg)) {
				uint8_t const txr_index = can->txr_get_index % TU_ARRAY_SIZE(can->txr_queue);
				struct txr const *txre = &can->txr_queue[txr_index];

				// LOG("TXR %02x offset=%u\n", txre->track_id, tx_ptr - tx_start);

				done = false;
				msg = (struct sc_msg_can_txr *)tx_ptr;
				tx_ptr += sizeof(*msg);
				result += sizeof(*msg);

				msg->id = SC_MSG_CAN_TXR;
				msg->len = sizeof(*msg);
				msg->track_id = txre->track_id;
				msg->timestamp_us = txre->ts;
				msg->flags = txre->flags;

				// sc_dump_mem(msg, 8);

				__atomic_store_n(&can->txr_get_index, can->txr_get_index + 1, __ATOMIC_RELEASE);


#if SUPERCAN_DEBUG
				SC_DEBUG_ASSERT(can->txr & (UINT32_C(1) << txre->track_id));
				can->txr &= ~(UINT32_C(1) << txre->track_id);
#endif
			}
		}


		uint8_t rx_pi = __atomic_load_n(&can->rx_put_index, __ATOMIC_ACQUIRE);

		if (can->rx_get_index != rx_pi) {
			struct rx_frame *rxe = NULL;
			struct sc_msg_can_rx *msg = NULL;
			uint8_t bytes = sizeof(*msg);
			uint8_t dlc;
			uint8_t len;

			SC_DEBUG_ASSERT((uint8_t)(rx_pi - can->rx_get_index) <= TU_ARRAY_SIZE(can->rx_queue));

			have_data_to_place = true;
			rxe = &can->rx_queue[can->rx_get_index % TU_ARRAY_SIZE(can->rx_queue)];

			dlc = rxe->mp & 0xf;
			len = dlc_to_len(dlc);

			if (!(rxe->mi & CAN_FT_REMOTE)) {
				bytes += len;
			}

			// align
			if (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1)) {
				bytes += SC_MSG_CAN_LEN_MULTIPLE - (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1));
			}

			if ((size_t)(tx_end - tx_ptr) >= bytes) {
				done = false;
				msg = (struct sc_msg_can_rx *)tx_ptr;
				tx_ptr += bytes;
				result += bytes;

				msg->id = SC_MSG_CAN_RX;
				msg->len = bytes;
				msg->dlc = dlc;
				msg->timestamp_us = rxe->timestamp_us;
				msg->flags = 0;

				if (rxe->mi & CAN_RFIFOMI_FF) {
					msg->can_id = rxe->mi >> 3;
					msg->flags |= SC_CAN_FRAME_FLAG_EXT;
				} else {
					msg->can_id = rxe->mi >> 21;
				}

				if (rxe->mp & CAN_RFIFOMP_FDF) {
					msg->flags |= SC_CAN_FRAME_FLAG_FDF;
					memcpy(msg + 1, rxe->data, len);

					if (rxe->mp & CAN_RFIFOMP_BRS) {
						msg->flags |= SC_CAN_FRAME_FLAG_BRS;
					}

					if (rxe->mp & CAN_RFIFOMP_ESI) {
						msg->flags |= SC_CAN_FRAME_FLAG_ESI;
					}
				} else {
					if (rxe->mi & CAN_FT_REMOTE) {
						msg->flags |= SC_CAN_FRAME_FLAG_RTR;
					} else {
						memcpy(msg + 1, rxe->data, len);
					}
				}

				__atomic_store_n(&can->rx_get_index, can->rx_get_index + 1, __ATOMIC_RELEASE);
			}
		}


		// // generate bogus but easy to validate data
		// size_t left = tx_end - tx_ptr;

		// if (left >= 4) {
		// 	static uint8_t count = 0;
		// 	struct sc_msg_header *hdr = tx_ptr;

		// 	left -= sizeof(*hdr);

		// 	hdr->id = 0x42;
		// 	hdr->len = 4;

		// 	tx_ptr += 2;
		// 	result += hdr->len;

		// 	*tx_ptr++ = count++;
		// 	*tx_ptr++ = count++;

		// 	done = false;
		// }
	}

	if (result > 0) {
		return result;
	}

	return have_data_to_place - 1;
}

SC_RAMFUNC static inline void timer_1mhz_on_overflow(void)
{
	SC_DEBUG_ASSERT(TIMER_INTF(TIMER5) & TIMER_INTF_UPIF);

	TIMER_INTF(TIMER5) &= ~TIMER_INTF_UPIF;

	NVIC_ClearPendingIRQ(TIMER5_IRQn);

	__atomic_store_n(&top_1us, top_1us + 1, __ATOMIC_RELEASE);

	// notify CAN task periodically (status lights)
	for (uint8_t i = 0; i < TU_ARRAY_SIZE(cans); ++i) {
		struct can *can = &cans[i];

		if (__atomic_load_n(&can->notify, __ATOMIC_ACQUIRE)) {
			sc_can_notify_task_isr(i, 1);
		}
	}
}

SC_RAMFUNC extern uint32_t sc_board_can_ts_fetch_isr(void)
{
	uint16_t top, bottom;

	for (;;) {
		bottom = TIMER_CNT(TIMER5);
		top = __atomic_load_n(&top_1us, __ATOMIC_ACQUIRE);
		__ISB();

		if (unlikely(TIMER_INTF(TIMER5) & TIMER_INTF_UPIF)) {
			LOG("o");

			timer_1mhz_on_overflow();
		} else {
			break;
		}
	}


	return  (((uint32_t)top) << 16) | bottom;
}

SC_RAMFUNC static void can_tx_irq(uint8_t index)
{
	struct can *can = &cans[index];
	uint32_t tsc = sc_board_can_ts_fetch_isr();
	unsigned count = 0;
	uint32_t events = 0;
	uint16_t ts[3];
	uint8_t indices[3];
	const uint32_t mailbox_clear_mask = 0xf;



	// LOG("ch%u TSTAT=%08x\n", index, CAN_TSTAT(can->regs));

	/* forward scan for finished mailboxes */
	while (can->tx_mailbox_queue_get_index != can->tx_mailbox_queue_put_index) {
		uint8_t const mailbox_queue_index = can->tx_mailbox_queue_get_index % TU_ARRAY_SIZE(can->mailbox_queue);
		uint8_t const mailbox_index = can->mailbox_queue[mailbox_queue_index];
		uint32_t const mailbox_finished_bit = UINT32_C(1) << (8 * mailbox_index);

		if (CAN_TSTAT(can->regs) & mailbox_finished_bit) {
			ts[count] = (CAN_TMP(can->regs, mailbox_index) >> 16);
			indices[count] = mailbox_index;
			++count;
			++can->tx_mailbox_queue_get_index;
			CAN_TSTAT(can->regs) |= mailbox_clear_mask << (8 * mailbox_index);
		} else {
			break;
		}
	}

	if (likely(count)) {
		// queue TXRs
		uint16_t const t0_can = ts[0];
		uint8_t const gi = __atomic_load_n(&can->txr_get_index, __ATOMIC_ACQUIRE);
		uint8_t pi = can->txr_put_index;
		bool desync = false;

		for (unsigned i = 0; i < count; ++i) {
			uint8_t const mailbox_index = indices[i];
			uint8_t const txrs_used = pi - gi;

			SC_DEBUG_ASSERT(txrs_used <= TU_ARRAY_SIZE(can->txr_queue));

			if (unlikely(txrs_used == TU_ARRAY_SIZE(can->txr_queue))) {
				LOG("ch%u lost TXR track ID %02x\n", index, can->track_id_boxes[mailbox_index]);
				desync = true;
			} else {
				uint8_t const txr_index = pi % TU_ARRAY_SIZE(can->txr_queue);

				can->txr_queue[txr_index].ts = tsc + (ts[i] - t0_can) * can->nm_us_per_bit;
				can->txr_queue[txr_index].track_id = can->track_id_boxes[mailbox_index];
				can->txr_queue[txr_index].flags = can->flags_boxes[mailbox_index];

				++events;
				++pi;
			}
		}

		__atomic_store_n(&can->txr_put_index, pi, __ATOMIC_RELEASE);

		if (unlikely(desync)) {
			sc_can_status status;

			status.type = SC_CAN_STATUS_FIFO_TYPE_TXR_DESYNC;
			status.timestamp_us = tsc;

			sc_can_status_queue(index, &status);
		}
	}

	/* move frames from queue into mailboxes */
	uint8_t pi = __atomic_load_n(&can->tx_put_index, __ATOMIC_ACQUIRE);
	uint8_t gi = can->tx_get_index;
	uint8_t mailboxes_used = can->tx_mailbox_queue_put_index - can->tx_mailbox_queue_get_index;

	SC_DEBUG_ASSERT(mailboxes_used <= TU_ARRAY_SIZE(can->mailbox_queue));

	while (gi != pi && mailboxes_used < TU_ARRAY_SIZE(can->mailbox_queue)) {
		uint8_t const tx_index = gi % TU_ARRAY_SIZE(can->tx_queue);
		uint8_t const mailbox_index = (CAN_TSTAT(can->regs) >> 24) & 3;
		struct tx_frame *tx = &can->tx_queue[tx_index];

		// LOG("ch%u tx i=%02u -> mb=%u\n", index, tx_index, mailbox_index);

#if SUPERCAN_DEBUG
		/* verifiy mailbox not in use */
		for (uint8_t mbqgi = can->tx_mailbox_queue_get_index; mbqgi != can->tx_mailbox_queue_put_index; ++mbqgi) {
			uint8_t mbq_index = mbqgi % TU_ARRAY_SIZE(can->mailbox_queue);
			SC_ASSERT(can->mailbox_queue[mbq_index] != mailbox_index);
		}
#endif

		can->track_id_boxes[mailbox_index] = tx->track_id;
		can->flags_boxes[mailbox_index] = tx->flags;

		/* write properties so that the mailbox knows about FD */
		CAN_TMP(can->regs, mailbox_index) = tx->mp;

		for (unsigned j = 0; j < tx->words; ++j) {
			REG32((can->regs) + 0x188U + 0x10 * mailbox_index) = tx->data[j];
		}

		CAN_TMI(can->regs, mailbox_index) = tx->mi;

		can->mailbox_queue[can->tx_mailbox_queue_put_index++ % TU_ARRAY_SIZE(can->mailbox_queue)] = mailbox_index;

		mailboxes_used = can->tx_mailbox_queue_put_index - can->tx_mailbox_queue_get_index;
		++gi;
	}

	__atomic_store_n(&can->tx_get_index, gi, __ATOMIC_RELEASE);

	if (likely(events)) {
		sc_can_notify_task_isr(index, events);
	}

	// LOG("ch%u tx exit TSTAT=%08x\n", index, CAN_TSTAT(can->regs));
}

SC_RAMFUNC static void can_rx_irq(uint8_t index)
{
	struct can *can = &cans[index];
	uint32_t events = 0;
	uint32_t tsc = sc_board_can_ts_fetch_isr();
	uint16_t rx_lost = 0;
	int32_t t0_can = -1;

	// LOG("ch%u rx\n", index);

	SC_DEBUG_ASSERT(CAN_INTEN(can->regs) & (CAN_INTEN_RFNEIE0 | CAN_INTEN_RFFIE0 | CAN_INTEN_RFOIE0));
	SC_DEBUG_ASSERT(CAN_RFIFO0(can->regs) & 3 /* rx fifo not empty */);

	uint8_t pi = can->rx_put_index;
	uint8_t gi = __atomic_load_n(&can->rx_get_index, __ATOMIC_ACQUIRE);

	do {
		uint8_t used = pi - gi;

		SC_DEBUG_ASSERT(used <= TU_ARRAY_SIZE(can->rx_queue));

		if (likely(used < TU_ARRAY_SIZE(can->rx_queue))) {
			struct rx_frame *rxe = &can->rx_queue[pi % TU_ARRAY_SIZE(can->rx_queue)];
			uint8_t words;
			uint16_t t_can;

			rxe->mi = CAN_RFIFOMI0(can->regs);
			rxe->mp = CAN_RFIFOMP0(can->regs);

			words = (dlc_to_len(rxe->mp & 0xf) + 3) / 4;

			for (uint8_t i = 0; i < words; ++i) {
				rxe->data[i] = CAN_RFIFOMDATA00(can->regs);
			}

			t_can = (rxe->mp >> 16) & 0xffff;
			rxe->timestamp_us = tsc;

			/* convert time stamp */
			if (-1 == t0_can) {
				t0_can = t_can;
			} else {
				rxe->timestamp_us += (t_can - (uint16_t)t0_can) * can->nm_us_per_bit;
			}

			++pi;
		} else {
			++rx_lost;
		}

		++events;

		// dequeue done
		CAN_RFIFO0(can->regs) |= CAN_RFIFO0_RFD0;
	} while (CAN_RFIFO0(can->regs) & 3);

	__atomic_store_n(&can->rx_put_index, pi, __ATOMIC_RELEASE);

	if (unlikely(CAN_RFIFO0(can->regs) & CAN_RFIFO0_RFO0)) {
		/* RX fifo overflow */
		++rx_lost;
	}

	if (unlikely(rx_lost)) {
		sc_can_status status;

		status.type = SC_CAN_STATUS_FIFO_TYPE_RX_LOST;
		status.timestamp_us = tsc;
		status.rx_lost = rx_lost;

		sc_can_status_queue(index, &status);
		++events;
	}

	if (likely(events)) {
		sc_can_notify_task_isr(index, events);
	}
}

static SC_RAMFUNC void can_ewmc_irq(uint8_t index)
{
	sc_can_status status;
	struct can *can = &cans[index];
	uint32_t const can_stat = CAN_STAT(can->regs);
	uint32_t const can_err = CAN_ERR(can->regs);
	status.timestamp_us = sc_board_can_ts_fetch_isr();
	unsigned events = 0;
	uint8_t rec = (can_err >> 24) & 0xff;
	uint8_t tec = (can_err >> 16) & 0xff;
	uint8_t current_bus_state = SC_CAN_STATUS_ERROR_ACTIVE;
	uint8_t const bus_error = sc_can_bus_error_map[(can_err >> 4) & 0x7];


	// LOG("ch%u STAT=%08x ERR=%08x\n", index, CAN_STAT(can->regs), CAN_ERR(can->regs));

	if (likely(rec != can->int_prev_rx_errors || tec != can->int_prev_tx_errors)) {
		can->int_prev_tx_errors = tec;
		can->int_prev_rx_errors = rec;

		LOG("CAN%u REC=%u TEC=%u\n", index, can->int_prev_rx_errors, can->int_prev_tx_errors);

		status.type = SC_CAN_STATUS_FIFO_TYPE_RXTX_ERRORS;
		status.counts.rx = rec;
		status.counts.tx = tec;

		sc_can_status_queue(index, &status);
		++events;
	}


	if (can_err & CAN_ERR_BOERR) {
		current_bus_state = SC_CAN_STATUS_BUS_OFF;
	} else if (can_err & CAN_ERR_PERR) {
		current_bus_state = SC_CAN_STATUS_ERROR_PASSIVE;
	} else if (can_err & CAN_ERR_WERR) {
		current_bus_state = SC_CAN_STATUS_ERROR_WARNING;
	}

	if (unlikely(can->int_prev_bus_state != current_bus_state)) {
		can->int_prev_bus_state = current_bus_state;

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_STATUS;
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

	if (unlikely(SC_CAN_ERROR_NONE != bus_error)) {
		bool is_tx_error = (can_stat & CAN_STAT_TS) == CAN_STAT_TS;

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		status.bus_error.tx = is_tx_error;
		status.bus_error.code = bus_error,
		status.bus_error.data_part = 0;

		sc_can_status_queue(index, &status);
		++events;
	}


	CAN_STAT(can->regs) |= CAN_STAT_ERRIF;

	if (likely(events)) {
		// LOG(">");
		sc_can_notify_task_isr(index, events);
	}
}

SC_RAMFUNC void CAN0_TX_IRQHandler(void)
{
	can_tx_irq(0);
}

SC_RAMFUNC void CAN0_RX0_IRQHandler(void)
{
	can_rx_irq(0);
}

SC_RAMFUNC void CAN0_EWMC_IRQHandler(void)
{
	can_ewmc_irq(0);
}

SC_RAMFUNC void CAN1_TX_IRQHandler(void)
{
	can_tx_irq(1);
}

SC_RAMFUNC void CAN1_RX0_IRQHandler(void)
{
	can_rx_irq(1);
}

SC_RAMFUNC void CAN1_EWMC_IRQHandler(void)
{
	can_ewmc_irq(1);
}

SC_RAMFUNC void TIMER5_IRQHandler(void)
{
	timer_1mhz_on_overflow();
}


#endif // #if D5035_04
