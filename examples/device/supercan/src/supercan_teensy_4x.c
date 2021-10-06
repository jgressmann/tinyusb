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
 * Note: parts of this file are copyright NXP and available under BSD-3-Clause.
 */


/*
 * CCM_CSCMR2:
 * CAN_CLK_SEL
 *
 * 00 derive clock from pll3_sw_clk divided clock (60M)
 * 01 derive clock from osc_clk (24M)
 * 10 derive clock from pll3_sw_clk divided clock (80M)
 * 11 Disable FlexCAN clock
 *
 * reset default 11
 *
 * CAN_CLK_PODF
 * 000000 devide by 1
 * ...
 * 000111 devide by 8
 * ...
 * 111111 devide by 2^6
 *
 * reset default 000001
 *
 *
 * Handler: CAN3_IRQHandler
 *
 * kCLOCK_Can3
 * kCLOCK_Can3S
 *
 * IOMUXC_GPIO_EMC_36_FLEXCAN3_TX
 * IOMUXC_GPIO_EMC_37_FLEXCAN3_RX
 *
 * Message buffer address offset 0x80, 14 for 64 byte message
 * i.MX RT1060 Processor Reference Manual, Rev. 2, 12/2019 p. 2715
 * offset	message buffer
 * 0080 	MB0
 * 00C8 MB1
 * 0110 MB2
 * 0158 MB3
 * 01A0 MB4
 * 01E8 MB5
 * 0230 MB6
 * 0280 MB7
 * 02C8 MB8
 * 0310 MB9
 * 0358 MB10
 * 03A0 MB11
 * 03E8 MB12
 * 0430 MB13
 *
 */
#ifdef TEENSY_4X

#include <bsp/board.h>
#include <supercan_board.h>
#include <leds.h>
#include <fsl_clock.h>
#include <fsl_iomuxc.h>

enum {
	TX_MAILBOX_COUNT = 1,
	RX_MAILBOX_COUNT = 14-TX_MAILBOX_COUNT,
	MB_RX_INACTIVE = 0b0000,
	MB_RX_EMPTY = 0b0100,
	MB_RX_FULL = 0b0010,
	MB_RX_OVERRUN = 0b0110,
	MB_RX_RANSWER = 0b1010,
	MB_TX_INACTIVE = 0b1000,
	MB_TX_ABORT = 0b1001,
	MB_TX_DATA = 0b1100,
	MB_TX_REMOTE = 0b1100,
	MB_TX_ANSWER = 0b1110,

	MB_STEP_CAN = 0x10,
	MB_STEP_CANFD = 0x48,
};

static const uint8_t flexcan_mb_step_size[2] = {
	MB_STEP_CAN,
	MB_STEP_CANFD,
};

struct flexcan_mailbox {
	volatile uint32_t CS;
    volatile uint32_t ID;
	volatile uint32_t WORD[16];
};

struct tx_fifo_element {
	struct flexcan_mailbox box;
	volatile uint8_t track_id;
	volatile uint8_t len;
};

struct txr_fifo_element {
	volatile uint32_t timestamp_us;
	volatile uint8_t track_id;
	volatile uint8_t flags;
};

struct rx_fifo_element {
	struct flexcan_mailbox box;
	volatile uint32_t timestamp_us;
};

struct can {
	struct rx_fifo_element rx_fifo[SC_BOARD_CAN_RX_FIFO_SIZE];
	struct tx_fifo_element tx_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];
	struct txr_fifo_element txr_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];


	CAN_Type* const flex_can;
	const IRQn_Type flex_can_irq;
	const IRQn_Type tx_queue_irq;
	uint32_t int_prev_esr1;
	uint8_t int_prev_bus_state;
	uint8_t int_prev_rx_errors;
	uint8_t int_prev_tx_errors;
	uint8_t int_tx_track_id;
	const bool fd_capable;
	bool fd_enabled;
	bool enabled;
	bool int_tx_box_busy;


	volatile uint8_t rx_pi; // not index, uses full range of type
	volatile uint8_t rx_gi; // not index, uses full range of type
	volatile uint8_t tx_pi; // not index, uses full range of type
	volatile uint8_t tx_gi; // not index, uses full range of type
	volatile uint8_t txr_pi; // not index, uses full range of type
	volatile uint8_t txr_gi; // not index, uses full range of type

};

static struct can cans[] = {
	{
		.flex_can = CAN3,
		.flex_can_irq = CAN3_IRQn,
		.tx_queue_irq = ADC1_IRQn,
		.fd_capable = true,
	},
	{
		.flex_can = CAN1,
		.flex_can_irq = CAN1_IRQn,
		.tx_queue_irq = ADC2_IRQn,
		.fd_capable = false,
	},
};

static uint32_t device_identifier;


// SC_RAMFUNC static void tx_task(void* param);

__attribute__((noreturn)) extern void sc_board_reset(void)
{
	NVIC_SystemReset();
	__unreachable();
}

SC_RAMFUNC extern void sc_board_led_set(uint8_t index, bool on)
{
	(void)index;
	(void)on;
}

extern void sc_board_leds_on_unsafe(void)
{
	// for (size_t i = 0; i < ARRAY_SIZE(leds); ++i) {
	// 	gpio_set_pin_level(leds[i].pin, 1);
	// }
}

extern uint16_t sc_board_can_feat_perm(uint8_t index)
{
	(void)index;
	return SC_FEATURE_FLAG_TXR;
}



extern uint16_t sc_board_can_feat_conf(uint8_t index)
{
	switch (index) {
	case 0: // FlexCAN3
		return (MSG_BUFFER_SIZE >= 128 ? SC_FEATURE_FLAG_FDF : 0)
					// | SC_FEATURE_FLAG_TXP
					| SC_FEATURE_FLAG_EHD
					| SC_FEATURE_FLAG_MON_MODE
					;
	case 1: // FlexCAN1
		return SC_FEATURE_FLAG_MON_MODE;
	}

	__unreachable();
	return 0;
}


static inline void init_mailboxes(uint8_t index)
{
	struct can *can = &cans[index];

	if (can->fd_enabled) {
		for (uint8_t i = 0; i < TX_MAILBOX_COUNT; ++i) {
			can->flex_can->MB_64B[i].ID = 0;
			can->flex_can->MB_64B[i].CS = CAN_CS_CODE(MB_TX_INACTIVE);
		}

		for (uint8_t i = TX_MAILBOX_COUNT; i < TX_MAILBOX_COUNT + RX_MAILBOX_COUNT; ++i) {
			can->flex_can->MB_64B[i].ID = 0;
			can->flex_can->MB_64B[i].CS = CAN_CS_CODE(MB_RX_EMPTY);
		}
	} else {
		for (uint8_t i = 0; i < TX_MAILBOX_COUNT; ++i) {
			can->flex_can->MB_8B[i].ID = 0;
			can->flex_can->MB_8B[i].CS = CAN_CS_CODE(MB_TX_INACTIVE);
		}

		for (uint8_t i = TX_MAILBOX_COUNT; i < TX_MAILBOX_COUNT + RX_MAILBOX_COUNT; ++i) {
			can->flex_can->MB_8B[i].ID = 0;
			can->flex_can->MB_8B[i].CS = CAN_CS_CODE(MB_RX_EMPTY);
		}
	}
}

static inline void freeze(uint8_t index)
{
	struct can *can = &cans[index];

	SC_DEBUG_ASSERT(!(can->flex_can->MCR & CAN_MCR_MDIS_MASK));

	// enter freeze mode
	can->flex_can->MCR |= CAN_MCR_FRZ(1) | CAN_MCR_HALT(1);
	while (!(can->flex_can->MCR & CAN_MCR_FRZACK_MASK));
}

static inline void thaw(uint8_t index)
{
	struct can *can = &cans[index];

	SC_DEBUG_ASSERT(!(can->flex_can->MCR & CAN_MCR_MDIS_MASK));

	// enter freeze mode
	can->flex_can->MCR &= ~CAN_MCR_HALT(1);
	while ((can->flex_can->MCR & CAN_MCR_FRZACK_MASK));
}

static inline void can_reset_state(uint8_t index)
{
	struct can *can = &cans[index];

	can->rx_gi = 0;
	can->rx_pi = 0;
	can->tx_gi = 0;
	can->tx_pi = 0;
	can->txr_gi = 0;
	can->txr_pi = 0;
	can->fd_enabled = false;
	can->enabled = false;
	can->int_tx_box_busy = false;
	can->int_tx_track_id = 0;
	can->int_prev_esr1 = 0;
	can->int_prev_bus_state = SC_CAN_STATUS_ERROR_ACTIVE;
	can->int_prev_rx_errors = 0;
	can->int_prev_tx_errors = 0;
}

extern void sc_board_can_reset(uint8_t index)
{
	struct can *can = &cans[index];

	freeze(index);

	// LOG("CAN ch%u soft reset\n", (unsigned)i);
	can->flex_can->MCR |= CAN_MCR_SOFTRST_MASK;
	while (can->flex_can->MCR & CAN_MCR_SOFTRST_MASK);

	LOG("CAN ch%u ", (unsigned)index);

	can->flex_can->MCR = 0
			// | CAN_MCR_MDIS(1)  // module disable
			| CAN_MCR_SUPV(1) // This bit configures the FlexCAN to be either in Supervisor or User mode
			| CAN_MCR_WRNEN(1) // Warning Interrupt Enable
			| CAN_MCR_IRMQ(1) // Individual Rx Masking And Queue Enable
			| CAN_MCR_SRXDIS(1) // disable self resception
			// | CAN_MCR_MAXMB(can->fd_capable ? 14 : 64)
			| CAN_MCR_MAXMB((TX_MAILBOX_COUNT + RX_MAILBOX_COUNT - 1))
			| CAN_MCR_FRZ(1) // halt / debug should freeze
			| CAN_MCR_HALT(1) // halt
			// | CAN_MCR_AEN(1)
			// | CAN_MCR_LPRIO_EN(1)
			;

	LOG("MCR=%lx ", can->flex_can->MCR);


	can->flex_can->CTRL1 = CAN_CTRL1_LBUF(1)  // Lowest Buffer Transmitted First
			| CAN_CTRL1_BOFFREC(1) // bus off interrupt enable
			| CAN_CTRL1_ERRMSK(1) // Error interrupt enabled
			| CAN_CTRL1_TWRNMSK(1) // tx warning interrupt enabled
			| CAN_CTRL1_RWRNMSK(1) // rx warning interrupt enabled
			| (can->fd_capable ? CAN_CTRL1_CLKSRC(1) : 0) // use peripheral clock not oscillator
			;

	LOG("CTRL1=%lx ", can->flex_can->CTRL1);

	can->flex_can->CTRL2 = CAN_CTRL2_BOFFDONEMSK(1) // Bus Off Done Interrupt Mask
							| CAN_CTRL2_RRS(1) // store remote request frames
							| CAN_CTRL2_MRP(1) // mailboxes first
							| (can->fd_capable ? CAN_CTRL2_ERRMSK_FAST(1) : 0)  // Error Interrupt Mask for errors detected in the data phase of fast CAN FD frames
							| (can->fd_capable ? CAN_CTRL2_ISOCANFDEN(1) : 0) // CAN-FD in ISO mode
							| (can->fd_capable ? CAN_CTRL2_PREXCEN(1) : 0) // CAN-FD protocol exception handling
							;

	LOG("CTRL2=%lx ", can->flex_can->CTRL2);



	// enable all message buffer interrupts
	can->flex_can->IFLAG1 = ~0;
	// can->flex_can->IFLAG2 = ~0;

	if (can->fd_capable) {
		can->flex_can->CBT = CAN_CBT_BTF(1); // mooar arbitration bits

		can->flex_can->FDCTRL = CAN_FDCTRL_FDRATE(1) // enable BRS
								| CAN_FDCTRL_MBDSR0(3) // 64 byte per box
								| CAN_FDCTRL_MBDSR1(3) // 64 byte per box
								| CAN_FDCTRL_TDCEN(1) // enable transmitter delay compensation
								;

		LOG("CBT=%lx FDCTRL=%lx ", can->flex_can->CBT, can->flex_can->FDCTRL);
	}

	LOG("MCR=%lx ", can->flex_can->MCR);

	LOG("\n");

	// can->flex_can->RXMGMASK = 0;

	// enable buffer interrupts
	can->flex_can->IMASK1 = (((uint32_t)1) << (TX_MAILBOX_COUNT + RX_MAILBOX_COUNT)) - 1;
	can->flex_can->IMASK2 = 0;

	// set rx individual mask to 'don't care'
	memset((void*)can->flex_can->RXIMR, 0, sizeof(can->flex_can->RXIMR));

	can_reset_state(index);

}

static inline void timer_1MHz_init(void)
{
	/* CCSR[pll3_sw_clk_se]
	 * 0 pll3_main_clk _reset default_ -> 480MHz
	 * 1 pll3 bypass clock
	 *
	 * CCM_CSCMR1[PERCLK_CLK_SEL]
	 * 0 derive clock from ipg clk root _reset default_
	 * 1 derive clock from osc_clk
	 *
	 * CCM_CBCMR[PRE_PERIPH_CLK_SEL]
	 * 00 derive clock from PLL2
	 * 01 derive clock from PLL2 PFD2
	 * 10 derive clock from PLL2 PFD0
	 * 11 derive clock from divided PLL1 _reset default_
	 *
	 * PERIPH_CLK2_SEL
	 * 00 derive clock from pll3_sw_clk _reset default_
	 * 01 derive clock from osc_clk (pll1_ref_clk)
	 * 10 derive clock from pll2_bypass_clk
	 * 11 reserved
	 */


	CLOCK_EnableClock(kCLOCK_Gpt2);
	// CLOCK_EnableClock(kCLOCK_Gpt2S);

	// LOG("CSCMR1[PERCLK_PODF]=%x\n", (CCM->CSCMR1 & CCM_CSCMR1_PERCLK_PODF_MASK) >> CCM_CSCMR1_PERCLK_PODF_SHIFT);
	// LOG("CSCMR1[PERCLK_CLK_SEL]=%x\n", (CCM->CSCMR1 & CCM_CSCMR1_PERCLK_CLK_SEL_MASK) >> CCM_CSCMR1_PERCLK_CLK_SEL_SHIFT);
	// LOG("CBCDR[IPG_PODF]=%x\n", (CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT);
	// LOG("CBCDR[PERIPH_CLK_SEL]=%x\n", (CCM->CBCDR & CCM_CBCDR_PERIPH_CLK_SEL_MASK) >> CCM_CBCDR_PERIPH_CLK_SEL_SHIFT);
	// LOG("CBCMR[PRE_PERIPH_CLK_SEL]=%x\n", (CCM->CBCMR & CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK) >> CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT);
	// LOG("CACRR[ARM_PODF]=%x\n", (CCM->CACRR & CCM_CACRR_ARM_PODF_MASK) >> CCM_CACRR_ARM_PODF_SHIFT);
	// LOG("ipg clock f=%lu [Hz]\n", CLOCK_GetIpgFreq());
	// LOG("per clock f=%lu [Hz]\n", CLOCK_GetPerClkFreq());
	// LOG("cpu clock f=%lu [Hz]\n", CLOCK_GetCpuClkFreq());
	// LOG("arm clock f=%lu [Hz]\n", CLOCK_GetFreq(kCLOCK_ArmPllClk));


	// reset
	GPT2->CR |= GPT_CR_SWR_MASK;
	while (GPT2->CR & GPT_CR_SWR_MASK);

	/* CLKSRC
	 * 000 No clock _reset default_
	 * 001 Peripheral Clock (ipg_clk)
	 * 010 High Frequency Reference Clock (ipg_clk_highfreq)
	 * 011 External Clock
	 * 100 Low Frequency Reference Clock (ipg_clk_32k)
	 * 101 Crystal oscillator as Reference Clock (ipg_clk_24M)
	 * others reserved
	 */


	GPT2->CR |= 0
		// | GPT_CR_IM1(0b11) // both edges
		| GPT_CR_FRR_MASK // free run
		// | GPT_CR_CLKSRC(0b101) // 24M xtal
		| GPT_CR_CLKSRC(0b001) // ipg_clk
		// | GPT_CR_CLKSRC(0b010) // ipg_clk_highfreq
		;

	// GPT2->PR = GPT_PR_PRESCALER(23); // 24MHz -> 1MHz
	// GPT2->PR = GPT_PR_PRESCALER(479); // 480MHz -> 1MHz
	// GPT2->PR = GPT_PR_PRESCALER(107); // 108MHz -> 1MHz
	GPT2->PR = GPT_PR_PRESCALER(74); // 75MHz -> 1MHz
	// GPT2->PR = GPT_PR_PRESCALER(2); // 3MHz -> 1MHz


	// start timer
	GPT2->CR |= GPT_CR_EN_MASK;
}

static inline void can_init_once(void)
{
	device_identifier = OCOTP->CFG0 | OCOTP->CFG1; // 64 bit DEVICE_ID

	for (size_t i = 0; i < TU_ARRAY_SIZE(cans); ++i) {
		struct can *can = &cans[i];

		NVIC_SetPriority(can->flex_can_irq, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
		NVIC_SetPriority(can->tx_queue_irq, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
		// should be no need to disable this one
		NVIC_EnableIRQ(can->tx_queue_irq);
	}

	LOG("CAN root clock f=%lu [Hz]\n", CLOCK_GetClockRootFreq(kCLOCK_CanClkRoot));


	// Grün TX = GPIO_EMC_36 (CAN3) = Board PIN 31
	//           AD_B1_08 (CAN1) = Board PIN 22
	// Blau RX = GPIO_EMC_37 (CAN3) = Board PIN 30
	//           AD_B1_09 (CAN1) = Board PIN 23


	IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_36_FLEXCAN3_TX, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_37_FLEXCAN3_RX, 0);
	// 0xC8 fastest, lowest drive strength
	// 5–3 DSE 0 == off, 001 = low , 010 = better, ...111 best
	// 7–6 SPEED 00 = low .. 11 fastest
	// 0 slew mode 0 = slow, 1 = fast
	IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_36_FLEXCAN3_TX, 0b11001000);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_37_FLEXCAN3_RX, 0b11000000);
	// IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_36_FLEXCAN3_TX, 0x10B0);
	// IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_37_FLEXCAN3_RX, 0x10B0);


	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_08_FLEXCAN1_TX, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_09_FLEXCAN1_RX, 0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_08_FLEXCAN1_TX, 0b11001000);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_09_FLEXCAN1_RX, 0b11000000);
	// IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_08_FLEXCAN1_TX, 0x10B0u);
  	// IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_09_FLEXCAN1_RX, 0x10B0u);


	CLOCK_EnableClock(kCLOCK_Can3);
	CLOCK_EnableClock(kCLOCK_Can3S);
	CLOCK_EnableClock(kCLOCK_Can1);
	CLOCK_EnableClock(kCLOCK_Can1S);
}

extern void sc_board_init_begin(void)
{
	board_init();

	can_init_once();

	timer_1MHz_init();

	for (uint8_t i = 0; i < TU_ARRAY_SIZE(cans); ++i) {
		struct can *can = &cans[i];

		// enable the module
		can->flex_can->MCR &= ~CAN_MCR_MDIS_MASK;

		sc_board_can_reset(i);
	}
}



extern void sc_board_init_end(void)
{

}


extern sc_can_bit_timing_range const* sc_board_can_nm_bit_timing_range(uint8_t index)
{
	static const sc_can_bit_timing_range ranges[] = {
		{
			.min = {
				.brp = 1,
				.tseg1 = 2,
				.tseg2 = 1,
				.sjw = 1,
			},
			.max = {
				.brp = 1024,
				.tseg1 = 64 + 32,
				.tseg2 = 32,
				.sjw = 32,
			},
		},
		{
			.min = {
				.brp = 1,
				.tseg1 = 2,
				.tseg2 = 1,
				.sjw = 1,
			},
			.max = {
				.brp = 256,
				.tseg1 = 8 + 8,
				.tseg2 = 8,
				.sjw = 4,
			},
		}
	};

	return &ranges[index];
}

extern sc_can_bit_timing_range const* sc_board_can_dt_bit_timing_range(uint8_t index)
{
	(void)index;

	static const sc_can_bit_timing_range range = {

		.min = {
			.brp = 1,
			.tseg1 = 2,
			.tseg2 = 1,
			.sjw = 1,
		},
		.max = {
			.brp = 0xff, // 1024 is what the device supports, Linux seems to handle one byte only
			.tseg1 = 32 + 8,
			.tseg2 = 8,
			.sjw = 8,
		},
	};

	return &range;
}

extern void sc_board_can_nm_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	struct can* can = &cans[index];

	if (can->fd_capable) {
		uint16_t prop_seg = 0;
		uint16_t tseg1 = 0;

		if (bt->tseg1 > 64) {
			prop_seg = 64;
			tseg1 = bt->tseg1 - prop_seg;
		} else {
			prop_seg = bt->tseg1 - 1;
			tseg1 = 1;
		}

		can->flex_can->CBT = CAN_CBT_BTF(1)
								| CAN_CBT_EPRESDIV(bt->brp - 1)
								| CAN_CBT_ERJW(bt->sjw - 1)
								| CAN_CBT_EPROPSEG(prop_seg - 1)
								| CAN_CBT_EPSEG1(tseg1 - 1)
								| CAN_CBT_EPSEG2(bt->tseg2 - 1)
								;

		LOG("ch%u CBT brp=%u sjw=%u propseg=%u tseg1=%u tseg2=%u\n",
			index, bt->brp, bt->sjw, prop_seg, tseg1, bt->tseg2);
	} else {
		uint32_t reg = can->flex_can->CTRL1 & ~(
			CAN_CTRL1_PROPSEG_MASK |
			CAN_CTRL1_PSEG1_MASK |
			CAN_CTRL1_PSEG2_MASK |
			CAN_CTRL1_RJW_MASK);

		uint16_t prop_seg = 0;
		uint16_t tseg1 = 0;

		if (bt->tseg1 > 8) {
			prop_seg = 8;
			tseg1 = bt->tseg1 - prop_seg;
		} else {
			prop_seg = bt->tseg1 - 1;
			tseg1 = 1;
		}

		can->flex_can->CTRL1 = reg
								| CAN_CTRL1_PRESDIV(bt->brp - 1)
								| CAN_CTRL1_RJW(bt->sjw - 1)
								| CAN_CTRL1_PROPSEG(prop_seg - 1)
								| CAN_CTRL1_PSEG1(tseg1 - 1)
								| CAN_CTRL1_PSEG2(bt->tseg2 - 1)
								;


		LOG("ch%u brp=%u sjw=%u propseg=%u tseg1=%u tseg2=%u CTRL1=%lx\n",
			index, bt->brp, bt->sjw, prop_seg, tseg1, bt->tseg2, can->flex_can->CTRL1);

		// can->flex_can->CTRL1 = 0x9624002;
	}

}

extern void sc_board_can_dt_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	struct can* can = &cans[index];

	uint16_t prop_seg = 0;
	uint16_t tseg1 = 0;

	if (bt->tseg1 > 32) {
		prop_seg = 32;
		tseg1 = bt->tseg1 - prop_seg;
	} else {
		prop_seg = bt->tseg1 - 1;
		tseg1 = 1;
	}

	can->flex_can->FDCBT = CAN_FDCBT_FPRESDIV(bt->brp - 1)
							| CAN_FDCBT_FRJW(bt->sjw - 1)
							| CAN_FDCBT_FPROPSEG(prop_seg - 1)
							| CAN_FDCBT_FPSEG1(tseg1 - 1)
							| CAN_FDCBT_FPSEG2(bt->tseg2 - 1)
							;

	LOG("ch%u FDCBT brp=%u sjw=%u propseg=%u tseg1=%u tseg2=%u\n",
		index, bt->brp, bt->sjw, prop_seg, tseg1, bt->tseg2);
}

extern void sc_board_can_go_bus(uint8_t index, bool on)
{
	struct can *can = &cans[index];

	can->enabled = on;

	// LOG("CNT=%lx\n", GPT2->CNT);

	if (on) {
		init_mailboxes(index);

		NVIC_EnableIRQ(can->flex_can_irq);

		thaw(index);


		// if (can->fd_enabled) {
		// 	can->flex_can->MB_64B[0].WORD[0] = 0xdeadbeef;
		// 	can->flex_can->MB_64B[0].ID = CAN_ID_STD(0x123 << 11);
		// 	can->flex_can->MB_64B[0].CS =
		// 		// CAN_CS_SRR(1) |
		// 		CAN_CS_DLC(4) | CAN_CS_CODE(MB_TX_DATA);
		// } else {
		// 	can->flex_can->MB_8B[0].WORD[0] = 0xdeadbeef;
		// 	can->flex_can->MB_8B[0].ID = CAN_ID_STD(0x123 << 11);
		// 	can->flex_can->MB_8B[0].CS =
		// 		// CAN_CS_SRR(1) |
		// 		CAN_CS_DLC(4) | CAN_CS_CODE(MB_TX_DATA);
		// }




	} else {
		freeze(index);

		LOG("go off bus MCR=%lx\n", can->flex_can->MCR);
		NVIC_DisableIRQ(can->flex_can_irq);

		// clear interrupts
		can->flex_can->IFLAG1 = can->flex_can->IFLAG1;
		// can->flex_can->IFLAG2 = can->flex_can->IFLAG2;

		// clear error flags
		can->flex_can->ESR1 = ~0;

		can_reset_state(index);
	}
}


SC_RAMFUNC extern bool sc_board_can_tx_queue(uint8_t index, struct sc_msg_can_tx const * msg)
{
	// LOG("> ch%u tx queue\n", index);

	struct can *can = &cans[index];
	struct tx_fifo_element *e = NULL;
	uint8_t gi = __atomic_load_n(&can->tx_gi, __ATOMIC_ACQUIRE);
	uint8_t pi = can->tx_pi;
	uint8_t used = pi - gi;
	uint8_t mailbox_index = 0;
	uint32_t id = 0;
	uint32_t cs = CAN_CS_DLC(msg->dlc);
	unsigned bytes = dlc_to_len(msg->dlc);

	if (unlikely(used == TU_ARRAY_SIZE(can->tx_fifo))) {
		// LOG("< ch%u tx queue\n", index);
		return false;
	}

	mailbox_index = pi % TU_ARRAY_SIZE(can->tx_fifo);
	SC_ASSERT(mailbox_index < TU_ARRAY_SIZE(can->tx_fifo));
	e = &can->tx_fifo[mailbox_index];
	e->track_id = msg->track_id;

	if (can->fd_enabled && (msg->flags & SC_CAN_FRAME_FLAG_FDF)) {
		memcpy((void*)e->box.WORD, msg->data, bytes);
		e->len = 0;
		cs |= CAN_CS_CODE(MB_TX_DATA);
		if (msg->flags & SC_CAN_FRAME_FLAG_BRS) {
			cs |= CAN_CS_BRS_MASK;
		}
		if (msg->flags & SC_CAN_FRAME_FLAG_ESI) {
			cs |= CAN_CS_ESI_MASK;
		}
	} else if (unlikely(msg->flags & SC_CAN_FRAME_FLAG_RTR)) {
		// RTR
		e->len = 0;
		cs |= CAN_CS_CODE(MB_TX_REMOTE) | CAN_CS_RTR_MASK;
	} else {
		memcpy((void*)e->box.WORD, msg->data, bytes);
		e->len = bytes;
		cs |= CAN_CS_CODE(MB_TX_DATA);
	}

	if (msg->flags & SC_CAN_FRAME_FLAG_EXT) {
		id = CAN_ID_EXT(msg->can_id);
		cs |= CAN_CS_EDL_MASK;
	} else {
		id = CAN_ID_STD(msg->can_id);
	}

	e->box.ID = id;
	e->box.CS = cs;

	__atomic_store_n(&can->tx_pi, pi + 1, __ATOMIC_RELEASE);

	// trigger interrupt
	NVIC->STIR = can->tx_queue_irq;

	// LOG("< ch%u tx queue\n", index);

	return true;
}

SC_RAMFUNC extern int sc_board_can_place_msgs(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	struct can *can = &cans[index];

	int result = 0;
	bool have_data_to_send = false;
	uint8_t txr_gi = can->txr_gi;
	uint8_t rx_gi = can->rx_gi;
	// size_t step = flexcan_mb_step_size[can->fd_enabled];

	for (;;) {
		const uint8_t txr_pi = __atomic_load_n(&can->txr_pi, __ATOMIC_ACQUIRE);

		if (txr_pi != txr_gi) {
			struct sc_msg_can_txr *txr = (struct sc_msg_can_txr *)tx_ptr;
			have_data_to_send = true;
			// LOG("ch%u have txr\n", index);

			if (tx_ptr + sizeof(*txr) <= tx_end) {
				const uint8_t txr_index = txr_gi % TU_ARRAY_SIZE(can->txr_fifo);
				SC_DEBUG_ASSERT(txr_index < TU_ARRAY_SIZE(can->txr_fifo));
				struct txr_fifo_element *e = &can->txr_fifo[txr_index];

				// LOG("ch%u place txr %u\n", index, e->track_id);
				result += sizeof(*txr);
				tx_ptr += sizeof(*txr);

				txr->id = SC_MSG_CAN_TXR;
				txr->len = sizeof(*txr);
				txr->flags = e->flags;
				txr->track_id = e->track_id;
				txr->timestamp_us = e->timestamp_us;

				++txr_gi;
			} else {
				break;
			}
		} else {
			break;
		}
	}

	__atomic_store_n(&can->txr_gi, txr_gi, __ATOMIC_RELEASE);

	for (;;) {
		const uint8_t rx_pi = __atomic_load_n(&can->rx_pi, __ATOMIC_ACQUIRE);

		if (rx_gi != rx_pi) {
			have_data_to_send = true;

			const uint8_t rx_index = rx_gi % TU_ARRAY_SIZE(can->rx_fifo);
			SC_DEBUG_ASSERT(rx_index < TU_ARRAY_SIZE(can->rx_fifo));
			struct rx_fifo_element *e = &can->rx_fifo[rx_index];


			const uint32_t cs = e->box.CS;
			const uint8_t dlc = (cs & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT;
			const bool rtr = (cs & CAN_CS_RTR_MASK) >> CAN_CS_RTR_SHIFT;
			const uint8_t can_frame_len = dlc_to_len(dlc);
			struct sc_msg_can_rx *msg = (struct sc_msg_can_rx *)tx_ptr;
			uint8_t bytes = sizeof(*msg);

			if (!rtr) {
				bytes += can_frame_len;
			}

			// align
			if (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1)) {
				bytes += SC_MSG_CAN_LEN_MULTIPLE - (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1));
			}

			if ((size_t)(tx_end - tx_ptr) >= bytes) {
				uint32_t id = e->box.ID;

				// usb_can->tx_offsets[usb_can->tx_bank] += bytes;
				tx_ptr += bytes;
				result += bytes;

				msg->id = SC_MSG_CAN_RX;
				msg->len = bytes;
				msg->dlc = dlc;
				msg->flags = 0;

				if (cs & CAN_CS_IDE_MASK) {
					msg->flags |= SC_CAN_FRAME_FLAG_EXT;
				} else {
					id >>= 18;
				}
				msg->can_id = id;
				msg->timestamp_us = e->timestamp_us;

				if (cs & CAN_CS_EDL_MASK) {
					msg->flags |= SC_CAN_FRAME_FLAG_FDF;

					if (cs & CAN_CS_BRS_MASK) {
						msg->flags |= SC_CAN_FRAME_FLAG_BRS;
					}

					if (cs & CAN_CS_ESI_MASK) {
						msg->flags |= SC_CAN_FRAME_FLAG_ESI;
					}

					memcpy(msg->data, (void*)e->box.WORD, can_frame_len);
				} else {
					if (rtr) {
						msg->flags |= SC_CAN_FRAME_FLAG_RTR;
					} else {
						memcpy(msg->data, (void*)e->box.WORD, can_frame_len);
					}
				}

				++rx_gi;
			} else {
				break;
			}
		} else {
			break;
		}
	}

	__atomic_store_n(&can->rx_gi, rx_gi, __ATOMIC_RELEASE);


	if (result > 0) {
		return result;
	}

	return have_data_to_send - 1;
}


SC_RAMFUNC static inline void service_tx_box(uint8_t index, uint32_t *events, uint32_t tsc)
{
	struct can *can = &cans[index];
	uint8_t* box_mem = ((uint8_t*)can->flex_can) + 0x80;
	struct flexcan_mailbox *box = (struct flexcan_mailbox *)box_mem;
	uint32_t cs = box->CS;
	unsigned code = (cs & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT;
	const uint8_t tx_fifo_gi = can->tx_gi;
	const uint8_t tx_fifo_pi = __atomic_load_n(&can->tx_pi, __ATOMIC_ACQUIRE);

	// LOG("> srv tx mb\n");

	if (MB_RX_EMPTY == code) {
		// remove rx empty for RTR frames
		// LOG("ch%u RTR cleanup\n", index);
		code = MB_TX_INACTIVE;
		cs &= ~CAN_CS_CODE_MASK;
		cs |= CAN_CS_CODE(code);
		box->CS = cs;
	}

	if (can->int_tx_box_busy && MB_TX_INACTIVE == code) {
		// tx done
		const uint8_t txr_pi = can->txr_pi;
		const uint8_t txr_gi = __atomic_load_n(&can->txr_gi, __ATOMIC_ACQUIRE);
		const uint8_t used = txr_pi - txr_gi;
		SC_ISR_ASSERT(used <= TU_ARRAY_SIZE(can->txr_fifo));

		if (unlikely(used == TU_ARRAY_SIZE(can->txr_fifo))) {
			sc_can_status status;

			LOG("ch%u txr desync\n", index);
			status.type = SC_CAN_STATUS_FIFO_TYPE_TXR_DESYNC;
			sc_can_status_queue(index, &status);

			++*events;
		} else {
			const uint8_t txr_index = txr_pi % TU_ARRAY_SIZE(can->txr_fifo);
			struct txr_fifo_element *e = &can->txr_fifo[txr_index];
			uint8_t flags = 0;

			e->track_id = can->int_tx_track_id;
			e->timestamp_us = tsc;


			if (cs & CAN_CS_IDE_MASK) {
				flags |= SC_CAN_FRAME_FLAG_EXT;
			}

			if (cs & CAN_CS_EDL_MASK) {
				flags |= SC_CAN_FRAME_FLAG_FDF;

				if (cs & CAN_CS_ESI_MASK) {
					flags |= SC_CAN_FRAME_FLAG_ESI;
				}

				if (cs & CAN_CS_BRS_MASK) {
					flags |= SC_CAN_FRAME_FLAG_BRS;
				}
			} else {
				if (cs & CAN_CS_RTR_MASK) {
					flags |= SC_CAN_FRAME_FLAG_RTR;
				}
			}

			e->flags = flags;

			__atomic_store_n(&can->txr_pi, txr_pi + 1, __ATOMIC_RELEASE);

			++*events;

			// LOG("ch%u sending txr %u\n", index, can->isr_tx_track_id);
		}

		can->int_tx_box_busy = false;
	}


	if (!can->int_tx_box_busy && tx_fifo_pi != tx_fifo_gi) {
		const uint8_t tx_mailbox_index = tx_fifo_gi % TU_ARRAY_SIZE(can->tx_fifo);
		const struct tx_fifo_element *e = &can->tx_fifo[tx_mailbox_index];
		SC_ISR_ASSERT(tx_mailbox_index < TU_ARRAY_SIZE(can->tx_fifo));

		// LOG("ch%u tx fifo pi=%u gi=%u tx_mailbox_index=%u\n", index, tx_fifo_pi, tx_fifo_gi, tx_mailbox_index);

		switch (code) {
		case MB_TX_INACTIVE: {
			// LOG("ch%u tranfer %u bytes of data to mailbox\n", index, e->len);
			can->int_tx_track_id = e->track_id;

			// LOG("ch%u 1\n", index);
			box->ID = e->box.ID; // write ID first according to manual

			// LOG("ch%u 2\n", index);
			for (uint8_t i = 0, o = 0; o < e->len; ++i, o += 4) {
				box->WORD[i] = e->box.WORD[i];
			}

			// LOG("ch%u 3\n", index);
			box->CS = e->box.CS;
			// LOG("ch%u 4\n", index);

			can->int_tx_box_busy = true;

			__atomic_store_n(&can->tx_gi, tx_fifo_gi + 1, __ATOMIC_RELEASE);

		} break;
		default:
			// LOG("ch%u no mailbox found\n", index);
			break;
		}
	}

	// LOG("< srv tx mb\n");
}


SC_RAMFUNC static void can_int_update_status(
	uint8_t index, uint32_t* events, uint32_t tsc)
{
	SC_DEBUG_ASSERT(events);

	struct can *can = &cans[index];
	uint8_t current_bus_state = 0;
	const uint32_t current_esr1 = can->flex_can->ESR1;
	const uint32_t ecr = can->flex_can->ECR;
	const uint8_t rxe = (ecr & CAN_ECR_RXERRCNT_MASK) >> CAN_ECR_RXERRCNT_SHIFT;
	const uint8_t txe = (ecr & CAN_ECR_TXERRCNT_MASK) >> CAN_ECR_TXERRCNT_SHIFT;
	// uint32_t prev_esr1 = can->int_prev_esr1;
	sc_can_status status;
	const uint8_t confinement = (current_esr1 & CAN_ESR1_FLTCONF_MASK) >> CAN_ESR1_FLTCONF_SHIFT;

	// clear error flags
	can->flex_can->ESR1 = ~0;


	if (unlikely(can->int_prev_rx_errors != rxe || can->int_prev_tx_errors != txe)) {
		can->int_prev_rx_errors = rxe;
		can->int_prev_tx_errors = txe;

		status.type = SC_CAN_STATUS_FIFO_TYPE_RXTX_ERRORS;
		status.timestamp_us = tsc;
		status.counts.rx = rxe;
		status.counts.tx = txe;

		sc_can_status_queue(index, &status);
		++*events;
	}

	switch (confinement) {
	case 0b00:
		current_bus_state = SC_CAN_STATUS_ERROR_ACTIVE;
		break;
	case 0b01:
		current_bus_state = SC_CAN_STATUS_ERROR_PASSIVE;
		break;
	case 0b10:
	case 0b11:
		current_bus_state = SC_CAN_STATUS_BUS_OFF;
		break;
	}

	// store updated reg
	// can->int_prev_esr1 = current_esr1;

	if (unlikely(can->int_prev_bus_state != current_bus_state)) {
		can->int_prev_bus_state = current_bus_state;

		LOG("ch%u bus state ", index);
		switch (current_bus_state) {
		case SC_CAN_STATUS_ERROR_ACTIVE:
			LOG("active");
			break;
		case SC_CAN_STATUS_ERROR_WARNING:
			LOG("warning");
			break;
		case SC_CAN_STATUS_ERROR_PASSIVE:
			LOG("passive");
			break;
		case SC_CAN_STATUS_BUS_OFF:
			LOG("off");
			break;
		default:
			LOG("unknown");
			break;
		}

		LOG("\n");

		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_STATUS;
		status.bus_state = current_bus_state;
		status.timestamp_us = tsc;

		sc_can_status_queue(index, &status);
		++*events;
	}

	if (current_esr1 & CAN_ESR1_ERRINT_FAST_MASK) {
		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		status.timestamp_us = tsc;
		status.bus_error.data_part = 1;
		status.bus_error.tx = (current_esr1 & CAN_ESR1_TX_MASK) == CAN_ESR1_TX_MASK;
		status.bus_error.code = SC_CAN_ERROR_NONE;

		if (current_esr1 & CAN_ESR1_BIT1ERR_FAST_MASK) {
			status.bus_error.code = SC_CAN_ERROR_BIT1;
		} else if (current_esr1 & CAN_ESR1_BIT0ERR_FAST_MASK) {
			status.bus_error.code = SC_CAN_ERROR_BIT0;
		} else if (current_esr1 & CAN_ESR1_CRCERR_FAST_MASK) {
			status.bus_error.code = SC_CAN_ERROR_CRC;
		} else if (current_esr1 & CAN_ESR1_FRMERR_FAST_MASK) {
			status.bus_error.code = SC_CAN_ERROR_FORM;
		} else if (current_esr1 & CAN_ESR1_STFERR_FAST_MASK) {
			status.bus_error.code = SC_CAN_ERROR_STUFF;
		}

		sc_can_status_queue(index, &status);
		++*events;
	}

	if (current_esr1 & CAN_ESR1_ERRINT_MASK) {
		status.type = SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR;
		status.timestamp_us = tsc;
		status.bus_error.data_part = 0;
		status.bus_error.tx = (current_esr1 & CAN_ESR1_TX_MASK) == CAN_ESR1_TX_MASK;
		status.bus_error.code = SC_CAN_ERROR_NONE;

		if (current_esr1 & CAN_ESR1_BIT1ERR_MASK) {
			status.bus_error.code = SC_CAN_ERROR_BIT1;
		} else if (current_esr1 & CAN_ESR1_BIT0ERR_MASK) {
			status.bus_error.code = SC_CAN_ERROR_BIT0;
		} else if (current_esr1 & CAN_ESR1_CRCERR_MASK) {
			status.bus_error.code = SC_CAN_ERROR_CRC;
		} else if (current_esr1 & CAN_ESR1_FRMERR_MASK) {
			status.bus_error.code = SC_CAN_ERROR_FORM;
		} else if (current_esr1 & CAN_ESR1_STFERR_MASK) {
			status.bus_error.code = SC_CAN_ERROR_STUFF;
		} else if (current_esr1 & CAN_ESR1_ACKERR_MASK) {
			status.bus_error.code = SC_CAN_ERROR_ACK;
		}

		sc_can_status_queue(index, &status);
		++*events;
	}
}

SC_RAMFUNC static void can_int(uint8_t index)
{
	// LOG("> ch%u int\n", index);

	struct can *can = &cans[index];
	uint32_t iflag1 = can->flex_can->IFLAG1;
	// uint32_t iflag2 = can->flex_can->IFLAG2;
	uint32_t events = 0;
	uint32_t tsc = GPT2->CNT;
	uint8_t* box_mem = ((uint8_t*)can->flex_can) + 0x80;
	const size_t step = flexcan_mb_step_size[can->fd_enabled];
	uint8_t rx_lost = 0;
	uint8_t rx_pi = can->rx_pi;


	// clear interrupts
	can->flex_can->IFLAG1 = iflag1;
	// can->flex_can->IFLAG2 = iflag2;

	// if (iflag1) {
	// 	LOG("IFLAG1=%lx\n", iflag1);
	// }

	// if (iflag2) {
	// 	LOG("IFLAG1=%lx\n", iflag2);
	// }

	if (iflag1 & 1) {
		// TX mailbox
		iflag1 &= ~(uint32_t)1;

		service_tx_box(index, &events, tsc);
	}

	if (iflag1) { // rx frames
		uint16_t rx_timestamps[RX_MAILBOX_COUNT];
		uint8_t rx_indices[RX_MAILBOX_COUNT];
		uint8_t rx_count = 0;

		// LOG("IFLAG1=%lx\n", iflag1);

		for (uint32_t i = 0, mask = ((uint32_t)1) << TX_MAILBOX_COUNT, k = TX_MAILBOX_COUNT; i < RX_MAILBOX_COUNT; ++i, ++k, mask <<= 1) {
			if (iflag1 & mask) {
				struct flexcan_mailbox *box = (struct flexcan_mailbox *)(box_mem + step * k);
				unsigned code = (box->CS & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT;

				if (likely(code == MB_RX_FULL || code == MB_RX_OVERRUN)) {
					rx_indices[rx_count] = k;
					rx_timestamps[rx_count] = (box->CS & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT;

					++rx_count;
					rx_lost += code == MB_RX_OVERRUN;

					// box->CS = CAN_CS_CODE(MB_RX_EMPTY);
				}
			}
		}

		// SC_ISR_ASSERT(__builtin_popcount(iflag1) == rx_count);
		unsigned start = 0;

		if (rx_count > 1) {
			// LOG("unsorted\n");
			// for (unsigned i = 0; i < rx_count; ++i) {
			// 	LOG("bit=%u ts=%x\n", rx_indices[i], rx_timestamps[i]);
			// }

			// sort
			for (unsigned i = 0; i < rx_count; ++i) {
				for (unsigned j = i + 1; j < rx_count; ++j) {
					if (rx_timestamps[j] < rx_timestamps[i]) {
						uint16_t tmp_ts = rx_timestamps[j];
						rx_timestamps[j] = rx_timestamps[i];
						rx_timestamps[i] = tmp_ts;

						uint8_t tmp_index = rx_indices[j];
						rx_indices[j] = rx_indices[i];
						rx_indices[i] = tmp_index;
					}
				}
			}

			// LOG("sorted\n");
			// for (unsigned i = 0; i < rx_count; ++i) {
			// 	LOG("bit=%u ts=%x\n", rx_indices[i], rx_timestamps[i]);
			// }

			// detect wrap around
			for (unsigned i = 1, j = 0; i < rx_count; ++i, ++j) {
				uint16_t ts_i = rx_timestamps[i];
				uint16_t ts_j = rx_timestamps[j];
				uint16_t ts_diff = ts_i - ts_j;

				if (ts_diff >= 0x8000) {
					// LOG("i=%u diff=%lx\n", i, ts_diff);
					start = i;
					break;
				}
			}

			// LOG("start=%u\n", start);
		}

		for (unsigned i = 0; i < rx_count; ++i) {
			const unsigned mailbox_index = rx_indices[i];
			struct flexcan_mailbox *box = (struct flexcan_mailbox *)(box_mem + step * mailbox_index);
			SC_ISR_ASSERT(mailbox_index < TX_MAILBOX_COUNT + RX_MAILBOX_COUNT);


			uint8_t rx_gi = __atomic_load_n(&can->rx_gi, __ATOMIC_ACQUIRE);
			uint8_t used = rx_pi - rx_gi;

			if (likely(used < TU_ARRAY_SIZE(can->rx_fifo))) {
				const uint8_t rx_index = rx_gi % TU_ARRAY_SIZE(can->rx_fifo);
				struct rx_fifo_element *e = &can->rx_fifo[rx_index];

				SC_ISR_ASSERT(rx_index < TU_ARRAY_SIZE(can->rx_fifo));
				e->timestamp_us = tsc - (rx_timestamps[rx_count-1] - rx_timestamps[i]);
				e->box = *box;

				++rx_pi;
				++events;
			} else {
				++rx_lost;
			}

			box->CS = CAN_CS_CODE(MB_RX_EMPTY);
		}

		__atomic_store_n(&can->rx_pi, rx_pi, __ATOMIC_RELEASE);

		if (unlikely(rx_lost)) {
			sc_can_status status;
			status.type = SC_CAN_STATUS_FIFO_TYPE_RX_LOST;
			status.timestamp_us = tsc;
			status.rx_lost = rx_lost;

			sc_can_status_queue(index, &status);
			++events;
		}
	}

	can_int_update_status(index, &events, tsc);


	if (likely(events)) {
		// LOG("/");
		sc_can_notify_task_isr(index, events);
	}

	// LOG("< ch%u int\n", index);
}


SC_RAMFUNC static inline void tx_frame_submitted(uint8_t index)
{
	const uint32_t tsc = GPT2->CNT;
	uint32_t events = 0;

	service_tx_box(index, &events, tsc);

	if (likely(events)) {
		// LOG("\\");
		sc_can_notify_task_isr(index, events);
	}
}



SC_RAMFUNC void CAN1_IRQHandler(void)
{
	can_int(1);
}

// SC_RAMFUNC void CAN2_IRQHandler(void)
// {
// 	LOG("+");
// }

SC_RAMFUNC void CAN3_IRQHandler(void)
{
	can_int(0);
}


SC_RAMFUNC void ADC1_IRQHandler(void)
{
	// LOG("> ADC1_IRQHandler\n");

	tx_frame_submitted(0);

	// LOG("< ADC1_IRQHandler\n");
}

SC_RAMFUNC void ADC2_IRQHandler(void)
{
	// LOG("> ADC2_IRQHandler\n");

	tx_frame_submitted(1);

	// LOG("< ADC2_IRQHandler\n");
}

extern uint32_t sc_board_identifier(void)
{
	return device_identifier;
}

extern void sc_board_can_feat_set(uint8_t index, uint16_t features)
{
	struct can *can = &cans[index];

	if (can->fd_capable) {
		if (features & SC_FEATURE_FLAG_FDF) {
			can->fd_enabled = true;
			can->flex_can->MCR |= CAN_MCR_FDEN_MASK;
		} else {
			can->fd_enabled = false;
			can->flex_can->MCR &= ~CAN_MCR_FDEN_MASK;
		}

		if (features & SC_FEATURE_FLAG_EHD) {
			can->flex_can->CTRL2 &= ~CAN_CTRL2_PREXCEN_MASK;
		} else {
			can->flex_can->CTRL2 |= CAN_CTRL2_PREXCEN_MASK;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////

/*
 * Copyright 2018-2020 ,2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "clock_config.h"
#include "fsl_iomuxc.h"


/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockRUN
called_from_default_init: true
outputs:
- {id: AHB_CLK_ROOT.outFreq, value: 600 MHz}
- {id: CAN_CLK_ROOT.outFreq, value: 80 MHz}
- {id: CKIL_SYNC_CLK_ROOT.outFreq, value: 32.768 kHz}
- {id: CLK_1M.outFreq, value: 1 MHz}
- {id: CLK_24M.outFreq, value: 24 MHz}
- {id: CSI_CLK_ROOT.outFreq, value: 12 MHz}
- {id: ENET1_TX_CLK.outFreq, value: 2.4 MHz}
- {id: ENET2_125M_CLK.outFreq, value: 1.2 MHz}
- {id: ENET2_TX_CLK.outFreq, value: 1.2 MHz}
- {id: ENET_125M_CLK.outFreq, value: 2.4 MHz}
- {id: ENET_25M_REF_CLK.outFreq, value: 1.2 MHz}
- {id: FLEXIO1_CLK_ROOT.outFreq, value: 30 MHz}
- {id: FLEXIO2_CLK_ROOT.outFreq, value: 30 MHz}
- {id: FLEXSPI2_CLK_ROOT.outFreq, value: 1440/11 MHz}
- {id: FLEXSPI_CLK_ROOT.outFreq, value: 1440/11 MHz}
- {id: GPT1_ipg_clk_highfreq.outFreq, value: 75 MHz}
- {id: GPT2_ipg_clk_highfreq.outFreq, value: 75 MHz}
- {id: IPG_CLK_ROOT.outFreq, value: 150 MHz}
- {id: LCDIF_CLK_ROOT.outFreq, value: 67.5 MHz}
- {id: LPI2C_CLK_ROOT.outFreq, value: 60 MHz}
- {id: LPSPI_CLK_ROOT.outFreq, value: 105.6 MHz}
- {id: LVDS1_CLK.outFreq, value: 1.2 GHz}
- {id: MQS_MCLK.outFreq, value: 1080/17 MHz}
- {id: PERCLK_CLK_ROOT.outFreq, value: 75 MHz}
- {id: PLL7_MAIN_CLK.outFreq, value: 24 MHz}
- {id: SAI1_CLK_ROOT.outFreq, value: 1080/17 MHz}
- {id: SAI1_MCLK1.outFreq, value: 1080/17 MHz}
- {id: SAI1_MCLK2.outFreq, value: 1080/17 MHz}
- {id: SAI1_MCLK3.outFreq, value: 30 MHz}
- {id: SAI2_CLK_ROOT.outFreq, value: 1080/17 MHz}
- {id: SAI2_MCLK1.outFreq, value: 1080/17 MHz}
- {id: SAI2_MCLK3.outFreq, value: 30 MHz}
- {id: SAI3_CLK_ROOT.outFreq, value: 1080/17 MHz}
- {id: SAI3_MCLK1.outFreq, value: 1080/17 MHz}
- {id: SAI3_MCLK3.outFreq, value: 30 MHz}
- {id: SEMC_CLK_ROOT.outFreq, value: 75 MHz}
- {id: SPDIF0_CLK_ROOT.outFreq, value: 30 MHz}
- {id: TRACE_CLK_ROOT.outFreq, value: 132 MHz}
- {id: UART_CLK_ROOT.outFreq, value: 80 MHz}
- {id: USDHC1_CLK_ROOT.outFreq, value: 198 MHz}
- {id: USDHC2_CLK_ROOT.outFreq, value: 198 MHz}
settings:
- {id: CCM.AHB_PODF.scale, value: '1', locked: true}
- {id: CCM.ARM_PODF.scale, value: '2', locked: true}
- {id: CCM.FLEXSPI2_PODF.scale, value: '2', locked: true}
- {id: CCM.FLEXSPI2_SEL.sel, value: CCM_ANALOG.PLL3_PFD0_CLK}
- {id: CCM.FLEXSPI_PODF.scale, value: '2', locked: true}
- {id: CCM.FLEXSPI_SEL.sel, value: CCM_ANALOG.PLL3_PFD0_CLK}
- {id: CCM.LPSPI_PODF.scale, value: '5', locked: true}
- {id: CCM.PERCLK_PODF.scale, value: '2', locked: true}
- {id: CCM.SEMC_PODF.scale, value: '8'}
- {id: CCM.TRACE_CLK_SEL.sel, value: CCM_ANALOG.PLL2_MAIN_CLK}
- {id: CCM.TRACE_PODF.scale, value: '4', locked: true}
- {id: CCM_ANALOG.PLL1_BYPASS.sel, value: CCM_ANALOG.PLL1}
- {id: CCM_ANALOG.PLL1_PREDIV.scale, value: '1', locked: true}
- {id: CCM_ANALOG.PLL1_VDIV.scale, value: '50', locked: true}
- {id: CCM_ANALOG.PLL2.denom, value: '1', locked: true}
- {id: CCM_ANALOG.PLL2.num, value: '0', locked: true}
- {id: CCM_ANALOG.PLL2_BYPASS.sel, value: CCM_ANALOG.PLL2_OUT_CLK}
- {id: CCM_ANALOG.PLL2_PFD0_BYPASS.sel, value: CCM_ANALOG.PLL2_PFD0}
- {id: CCM_ANALOG.PLL2_PFD1_BYPASS.sel, value: CCM_ANALOG.PLL2_PFD1}
- {id: CCM_ANALOG.PLL2_PFD2_BYPASS.sel, value: CCM_ANALOG.PLL2_PFD2}
- {id: CCM_ANALOG.PLL2_PFD3_BYPASS.sel, value: CCM_ANALOG.PLL2_PFD3}
- {id: CCM_ANALOG.PLL3_BYPASS.sel, value: CCM_ANALOG.PLL3}
- {id: CCM_ANALOG.PLL3_PFD0_BYPASS.sel, value: CCM_ANALOG.PLL3_PFD0}
- {id: CCM_ANALOG.PLL3_PFD0_DIV.scale, value: '33', locked: true}
- {id: CCM_ANALOG.PLL3_PFD0_MUL.scale, value: '18', locked: true}
- {id: CCM_ANALOG.PLL3_PFD1_BYPASS.sel, value: CCM_ANALOG.PLL3_PFD1}
- {id: CCM_ANALOG.PLL3_PFD2_BYPASS.sel, value: CCM_ANALOG.PLL3_PFD2}
- {id: CCM_ANALOG.PLL3_PFD3_BYPASS.sel, value: CCM_ANALOG.PLL3_PFD3}
- {id: CCM_ANALOG.PLL4.denom, value: '50'}
- {id: CCM_ANALOG.PLL4.div, value: '47'}
- {id: CCM_ANALOG.PLL5.denom, value: '1'}
- {id: CCM_ANALOG.PLL5.div, value: '31', locked: true}
- {id: CCM_ANALOG.PLL5.num, value: '0'}
- {id: CCM_ANALOG.PLL5_BYPASS.sel, value: CCM_ANALOG.PLL5_POST_DIV}
- {id: CCM_ANALOG.PLL5_POST_DIV.scale, value: '2'}
- {id: CCM_ANALOG.VIDEO_DIV.scale, value: '4'}
- {id: CCM_ANALOG_PLL_ENET_POWERDOWN_CFG, value: 'Yes'}
- {id: CCM_ANALOG_PLL_USB1_POWER_CFG, value: 'Yes'}
- {id: CCM_ANALOG_PLL_VIDEO_POWERDOWN_CFG, value: 'No'}
sources:
- {id: XTALOSC24M.RTC_OSC.outFreq, value: 32.768 kHz, enabled: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockRUN configuration
 ******************************************************************************/


void clock_init(void)
{
	static const clock_arm_pll_config_t sc_armPllConfig_BOARD_BootClockRUN =
    {
        .loopDivider = 100,                       /* PLL loop divider, Fout = Fin * 50 */
        .src = 0,                                 /* Bypass clock source, 0 - OSC 24M, 1 - CLK1_P and CLK1_N */
    };
static const clock_sys_pll_config_t sc_sysPllConfig_BOARD_BootClockRUN =
    {
        .loopDivider = 1,                         /* PLL loop divider, Fout = Fin * ( 20 + loopDivider*2 + numerator / denominator ) */
        .numerator = 0,                           /* 30 bit numerator of fractional loop divider */
        .denominator = 1,                         /* 30 bit denominator of fractional loop divider */
        .src = 0,                                 /* Bypass clock source, 0 - OSC 24M, 1 - CLK1_P and CLK1_N */
    };
static const clock_usb_pll_config_t sc_usb1PllConfig_BOARD_BootClockRUN =
    {
        .loopDivider = 0,                         /* PLL loop divider, Fout = Fin * 20 */
        .src = 0,                                 /* Bypass clock source, 0 - OSC 24M, 1 - CLK1_P and CLK1_N */
    };
static const clock_video_pll_config_t sc_videoPllConfig_BOARD_BootClockRUN =
    {
        .loopDivider = 31,                        /* PLL loop divider, Fout = Fin * ( loopDivider + numerator / denominator ) */
        .postDivider = 8,                         /* Divider after PLL */
        .numerator = 0,                           /* 30 bit numerator of fractional loop divider, Fout = Fin * ( loopDivider + numerator / denominator ) */
        .denominator = 1,                         /* 30 bit denominator of fractional loop divider, Fout = Fin * ( loopDivider + numerator / denominator ) */
        .src = 0,                                 /* Bypass clock source, 0 - OSC 24M, 1 - CLK1_P and CLK1_N */
    };

    /* Init RTC OSC clock frequency. */
    CLOCK_SetRtcXtalFreq(32768U);
    /* Enable 1MHz clock output. */
    XTALOSC24M->OSC_CONFIG2 |= XTALOSC24M_OSC_CONFIG2_ENABLE_1M_MASK;
    /* Use free 1MHz clock output. */
    XTALOSC24M->OSC_CONFIG2 &= ~XTALOSC24M_OSC_CONFIG2_MUX_1M_MASK;
    /* Set XTAL 24MHz clock frequency. */
    CLOCK_SetXtalFreq(24000000U);
    /* Enable XTAL 24MHz clock source. */
    CLOCK_InitExternalClk(0);
    /* Enable internal RC. */
    CLOCK_InitRcOsc24M();
    /* Switch clock source to external OSC. */
    CLOCK_SwitchOsc(kCLOCK_XtalOsc);
    /* Set Oscillator ready counter value. */
    CCM->CCR = (CCM->CCR & (~CCM_CCR_OSCNT_MASK)) | CCM_CCR_OSCNT(127);
    /* Setting PeriphClk2Mux and PeriphMux to provide stable clock before PLLs are initialed */
    CLOCK_SetMux(kCLOCK_PeriphClk2Mux, 1); /* Set PERIPH_CLK2 MUX to OSC */
    CLOCK_SetMux(kCLOCK_PeriphMux, 1);     /* Set PERIPH_CLK MUX to PERIPH_CLK2 */
    /* Setting the VDD_SOC to 1.275V. It is necessary to config AHB to 600Mhz. */
    DCDC->REG3 = (DCDC->REG3 & (~DCDC_REG3_TRG_MASK)) | DCDC_REG3_TRG(0x13);
    /* Waiting for DCDC_STS_DC_OK bit is asserted */
    while (DCDC_REG0_STS_DC_OK_MASK != (DCDC_REG0_STS_DC_OK_MASK & DCDC->REG0))
    {
    }
    /* Set AHB_PODF. */
    CLOCK_SetDiv(kCLOCK_AhbDiv, 0);
    /* Disable IPG clock gate. */
    CLOCK_DisableClock(kCLOCK_Adc1);
    CLOCK_DisableClock(kCLOCK_Adc2);
    CLOCK_DisableClock(kCLOCK_Xbar1);
    CLOCK_DisableClock(kCLOCK_Xbar2);
    CLOCK_DisableClock(kCLOCK_Xbar3);
    /* Set IPG_PODF. */
    CLOCK_SetDiv(kCLOCK_IpgDiv, 3);
    /* Set ARM_PODF. */
    CLOCK_SetDiv(kCLOCK_ArmDiv, 1);
    /* Set PERIPH_CLK2_PODF. */
    CLOCK_SetDiv(kCLOCK_PeriphClk2Div, 0);
    /* Disable PERCLK clock gate. */
    CLOCK_DisableClock(kCLOCK_Gpt1);
    CLOCK_DisableClock(kCLOCK_Gpt1S);
    CLOCK_DisableClock(kCLOCK_Gpt2);
    CLOCK_DisableClock(kCLOCK_Gpt2S);
    CLOCK_DisableClock(kCLOCK_Pit);
    /* Set PERCLK_PODF. */
    CLOCK_SetDiv(kCLOCK_PerclkDiv, 1);
    /* Disable USDHC1 clock gate. */
    CLOCK_DisableClock(kCLOCK_Usdhc1);
    /* Set USDHC1_PODF. */
    CLOCK_SetDiv(kCLOCK_Usdhc1Div, 1);
    /* Set Usdhc1 clock source. */
    CLOCK_SetMux(kCLOCK_Usdhc1Mux, 0);
    /* Disable USDHC2 clock gate. */
    CLOCK_DisableClock(kCLOCK_Usdhc2);
    /* Set USDHC2_PODF. */
    CLOCK_SetDiv(kCLOCK_Usdhc2Div, 1);
    /* Set Usdhc2 clock source. */
    CLOCK_SetMux(kCLOCK_Usdhc2Mux, 0);
    /* In SDK projects, SDRAM (configured by SEMC) will be initialized in either debug script or dcd.
     * With this macro SKIP_SYSCLK_INIT, system pll (selected to be SEMC source clock in SDK projects) will be left unchanged.
     * Note: If another clock source is selected for SEMC, user may want to avoid changing that clock as well.*/
#ifndef SKIP_SYSCLK_INIT
    /* Disable Semc clock gate. */
    CLOCK_DisableClock(kCLOCK_Semc);
    /* Set SEMC_PODF. */
    CLOCK_SetDiv(kCLOCK_SemcDiv, 7);
    /* Set Semc alt clock source. */
    CLOCK_SetMux(kCLOCK_SemcAltMux, 0);
    /* Set Semc clock source. */
    CLOCK_SetMux(kCLOCK_SemcMux, 0);
#endif
    /* In SDK projects, external flash (configured by FLEXSPI) will be initialized by dcd.
     * With this macro XIP_EXTERNAL_FLASH, usb1 pll (selected to be FLEXSPI clock source in SDK projects) will be left unchanged.
     * Note: If another clock source is selected for FLEXSPI, user may want to avoid changing that clock as well.*/
#if !(defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1))
    /* Disable Flexspi clock gate. */
    CLOCK_DisableClock(kCLOCK_FlexSpi);
    /* Set FLEXSPI_PODF. */
    CLOCK_SetDiv(kCLOCK_FlexspiDiv, 1);
    /* Set Flexspi clock source. */
    CLOCK_SetMux(kCLOCK_FlexspiMux, 3);
#endif
    /* Disable Flexspi2 clock gate. */
    CLOCK_DisableClock(kCLOCK_FlexSpi2);
    /* Set FLEXSPI2_PODF. */
    CLOCK_SetDiv(kCLOCK_Flexspi2Div, 1);
    /* Set Flexspi2 clock source. */
    CLOCK_SetMux(kCLOCK_Flexspi2Mux, 1);
    /* Disable CSI clock gate. */
    CLOCK_DisableClock(kCLOCK_Csi);
    /* Set CSI_PODF. */
    CLOCK_SetDiv(kCLOCK_CsiDiv, 1);
    /* Set Csi clock source. */
    CLOCK_SetMux(kCLOCK_CsiMux, 0);
    /* Disable LPSPI clock gate. */
    CLOCK_DisableClock(kCLOCK_Lpspi1);
    CLOCK_DisableClock(kCLOCK_Lpspi2);
    CLOCK_DisableClock(kCLOCK_Lpspi3);
    CLOCK_DisableClock(kCLOCK_Lpspi4);
    /* Set LPSPI_PODF. */
    CLOCK_SetDiv(kCLOCK_LpspiDiv, 4);
    /* Set Lpspi clock source. */
    CLOCK_SetMux(kCLOCK_LpspiMux, 2);
    /* Disable TRACE clock gate. */
    CLOCK_DisableClock(kCLOCK_Trace);
    /* Set TRACE_PODF. */
    CLOCK_SetDiv(kCLOCK_TraceDiv, 3);
    /* Set Trace clock source. */
    CLOCK_SetMux(kCLOCK_TraceMux, 0);
    /* Disable SAI1 clock gate. */
    CLOCK_DisableClock(kCLOCK_Sai1);
    /* Set SAI1_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, 3);
    /* Set SAI1_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Sai1Div, 1);
    /* Set Sai1 clock source. */
    CLOCK_SetMux(kCLOCK_Sai1Mux, 0);
    /* Disable SAI2 clock gate. */
    CLOCK_DisableClock(kCLOCK_Sai2);
    /* Set SAI2_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Sai2PreDiv, 3);
    /* Set SAI2_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Sai2Div, 1);
    /* Set Sai2 clock source. */
    CLOCK_SetMux(kCLOCK_Sai2Mux, 0);
    /* Disable SAI3 clock gate. */
    CLOCK_DisableClock(kCLOCK_Sai3);
    /* Set SAI3_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Sai3PreDiv, 3);
    /* Set SAI3_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Sai3Div, 1);
    /* Set Sai3 clock source. */
    CLOCK_SetMux(kCLOCK_Sai3Mux, 0);
    /* Disable Lpi2c clock gate. */
    CLOCK_DisableClock(kCLOCK_Lpi2c1);
    CLOCK_DisableClock(kCLOCK_Lpi2c2);
    CLOCK_DisableClock(kCLOCK_Lpi2c3);
    /* Set LPI2C_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, 0);
    /* Set Lpi2c clock source. */
    CLOCK_SetMux(kCLOCK_Lpi2cMux, 0);
    /* Disable CAN clock gate. */
    CLOCK_DisableClock(kCLOCK_Can1);
    CLOCK_DisableClock(kCLOCK_Can2);
    CLOCK_DisableClock(kCLOCK_Can3);
    CLOCK_DisableClock(kCLOCK_Can1S);
    CLOCK_DisableClock(kCLOCK_Can2S);
    CLOCK_DisableClock(kCLOCK_Can3S);
    /* Set CAN_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_CanDiv, 0);
    /* Set Can clock source. */
    CLOCK_SetMux(kCLOCK_CanMux, 2);
    /* Disable UART clock gate. */
    CLOCK_DisableClock(kCLOCK_Lpuart1);
    CLOCK_DisableClock(kCLOCK_Lpuart2);
    CLOCK_DisableClock(kCLOCK_Lpuart3);
    CLOCK_DisableClock(kCLOCK_Lpuart4);
    CLOCK_DisableClock(kCLOCK_Lpuart5);
    CLOCK_DisableClock(kCLOCK_Lpuart6);
    CLOCK_DisableClock(kCLOCK_Lpuart7);
    CLOCK_DisableClock(kCLOCK_Lpuart8);
    /* Set UART_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_UartDiv, 0);
    /* Set Uart clock source. */
    CLOCK_SetMux(kCLOCK_UartMux, 0);
    /* Disable LCDIF clock gate. */
    CLOCK_DisableClock(kCLOCK_LcdPixel);
    /* Set LCDIF_PRED. */
    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 1);
    /* Set LCDIF_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_LcdifDiv, 3);
    /* Set Lcdif pre clock source. */
    CLOCK_SetMux(kCLOCK_LcdifPreMux, 5);
    /* Disable SPDIF clock gate. */
    CLOCK_DisableClock(kCLOCK_Spdif);
    /* Set SPDIF0_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Spdif0PreDiv, 1);
    /* Set SPDIF0_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Spdif0Div, 7);
    /* Set Spdif clock source. */
    CLOCK_SetMux(kCLOCK_SpdifMux, 3);
    /* Disable Flexio1 clock gate. */
    CLOCK_DisableClock(kCLOCK_Flexio1);
    /* Set FLEXIO1_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Flexio1PreDiv, 1);
    /* Set FLEXIO1_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Flexio1Div, 7);
    /* Set Flexio1 clock source. */
    CLOCK_SetMux(kCLOCK_Flexio1Mux, 3);
    /* Disable Flexio2 clock gate. */
    CLOCK_DisableClock(kCLOCK_Flexio2);
    /* Set FLEXIO2_CLK_PRED. */
    CLOCK_SetDiv(kCLOCK_Flexio2PreDiv, 1);
    /* Set FLEXIO2_CLK_PODF. */
    CLOCK_SetDiv(kCLOCK_Flexio2Div, 7);
    /* Set Flexio2 clock source. */
    CLOCK_SetMux(kCLOCK_Flexio2Mux, 3);
    /* Set Pll3 sw clock source. */
    CLOCK_SetMux(kCLOCK_Pll3SwMux, 0);
    /* Init ARM PLL. */
    CLOCK_InitArmPll(&sc_armPllConfig_BOARD_BootClockRUN);
    /* In SDK projects, SDRAM (configured by SEMC) will be initialized in either debug script or dcd.
     * With this macro SKIP_SYSCLK_INIT, system pll (selected to be SEMC source clock in SDK projects) will be left unchanged.
     * Note: If another clock source is selected for SEMC, user may want to avoid changing that clock as well.*/
#ifndef SKIP_SYSCLK_INIT
#if defined(XIP_BOOT_HEADER_DCD_ENABLE) && (XIP_BOOT_HEADER_DCD_ENABLE == 1)
    #warning "SKIP_SYSCLK_INIT should be defined to keep system pll (selected to be SEMC source clock in SDK projects) unchanged."
#endif
    /* Init System PLL. */
    CLOCK_InitSysPll(&sc_sysPllConfig_BOARD_BootClockRUN);
    /* Init System pfd0. */
    CLOCK_InitSysPfd(kCLOCK_Pfd0, 27);
    /* Init System pfd1. */
    CLOCK_InitSysPfd(kCLOCK_Pfd1, 16);
    /* Init System pfd2. */
    CLOCK_InitSysPfd(kCLOCK_Pfd2, 24);
    /* Init System pfd3. */
    CLOCK_InitSysPfd(kCLOCK_Pfd3, 16);
#endif
    /* In SDK projects, external flash (configured by FLEXSPI) will be initialized by dcd.
     * With this macro XIP_EXTERNAL_FLASH, usb1 pll (selected to be FLEXSPI clock source in SDK projects) will be left unchanged.
     * Note: If another clock source is selected for FLEXSPI, user may want to avoid changing that clock as well.*/
#if !(defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1))
    /* Init Usb1 PLL. */
    CLOCK_InitUsb1Pll(&sc_usb1PllConfig_BOARD_BootClockRUN);
    /* Init Usb1 pfd0. */
    CLOCK_InitUsb1Pfd(kCLOCK_Pfd0, 33);
    /* Init Usb1 pfd1. */
    CLOCK_InitUsb1Pfd(kCLOCK_Pfd1, 16);
    /* Init Usb1 pfd2. */
    CLOCK_InitUsb1Pfd(kCLOCK_Pfd2, 17);
    /* Init Usb1 pfd3. */
    CLOCK_InitUsb1Pfd(kCLOCK_Pfd3, 19);
    /* Disable Usb1 PLL output for USBPHY1. */
    CCM_ANALOG->PLL_USB1 &= ~CCM_ANALOG_PLL_USB1_EN_USB_CLKS_MASK;
#endif
    /* DeInit Audio PLL. */
    CLOCK_DeinitAudioPll();
    /* Bypass Audio PLL. */
    CLOCK_SetPllBypass(CCM_ANALOG, kCLOCK_PllAudio, 1);
    /* Set divider for Audio PLL. */
    CCM_ANALOG->MISC2 &= ~CCM_ANALOG_MISC2_AUDIO_DIV_LSB_MASK;
    CCM_ANALOG->MISC2 &= ~CCM_ANALOG_MISC2_AUDIO_DIV_MSB_MASK;
    /* Enable Audio PLL output. */
    CCM_ANALOG->PLL_AUDIO |= CCM_ANALOG_PLL_AUDIO_ENABLE_MASK;
    /* Init Video PLL. */
    uint32_t pllVideo;
    /* Disable Video PLL output before initial Video PLL. */
    CCM_ANALOG->PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_ENABLE_MASK;
    /* Bypass PLL first */
    CCM_ANALOG->PLL_VIDEO = (CCM_ANALOG->PLL_VIDEO & (~CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_MASK)) |
                            CCM_ANALOG_PLL_VIDEO_BYPASS_MASK | CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC(0);
    CCM_ANALOG->PLL_VIDEO_NUM = CCM_ANALOG_PLL_VIDEO_NUM_A(0);
    CCM_ANALOG->PLL_VIDEO_DENOM = CCM_ANALOG_PLL_VIDEO_DENOM_B(1);
    pllVideo = (CCM_ANALOG->PLL_VIDEO & (~(CCM_ANALOG_PLL_VIDEO_DIV_SELECT_MASK | CCM_ANALOG_PLL_VIDEO_POWERDOWN_MASK))) |
               CCM_ANALOG_PLL_VIDEO_ENABLE_MASK |CCM_ANALOG_PLL_VIDEO_DIV_SELECT(31);
    pllVideo |= CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(1);
    CCM_ANALOG->MISC2 = (CCM_ANALOG->MISC2 & (~CCM_ANALOG_MISC2_VIDEO_DIV_MASK)) | CCM_ANALOG_MISC2_VIDEO_DIV(3);
    CCM_ANALOG->PLL_VIDEO = pllVideo;
    while ((CCM_ANALOG->PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_LOCK_MASK) == 0)
    {
    }
    /* Disable bypass for Video PLL. */
    CLOCK_SetPllBypass(CCM_ANALOG, kCLOCK_PllVideo, 0);
    /* DeInit Enet PLL. */
    CLOCK_DeinitEnetPll();
    /* Bypass Enet PLL. */
    CLOCK_SetPllBypass(CCM_ANALOG, kCLOCK_PllEnet, 1);
    /* Set Enet output divider. */
    CCM_ANALOG->PLL_ENET = (CCM_ANALOG->PLL_ENET & (~CCM_ANALOG_PLL_ENET_DIV_SELECT_MASK)) | CCM_ANALOG_PLL_ENET_DIV_SELECT(1);
    /* Enable Enet output. */
    CCM_ANALOG->PLL_ENET |= CCM_ANALOG_PLL_ENET_ENABLE_MASK;
    /* Set Enet2 output divider. */
    CCM_ANALOG->PLL_ENET = (CCM_ANALOG->PLL_ENET & (~CCM_ANALOG_PLL_ENET_ENET2_DIV_SELECT_MASK)) | CCM_ANALOG_PLL_ENET_ENET2_DIV_SELECT(0);
    /* Enable Enet2 output. */
    CCM_ANALOG->PLL_ENET |= CCM_ANALOG_PLL_ENET_ENET2_REF_EN_MASK;
    /* Enable Enet25M output. */
    CCM_ANALOG->PLL_ENET |= CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN_MASK;
    /* DeInit Usb2 PLL. */
    CLOCK_DeinitUsb2Pll();
    /* Bypass Usb2 PLL. */
    CLOCK_SetPllBypass(CCM_ANALOG, kCLOCK_PllUsb2, 1);
    /* Enable Usb2 PLL output. */
    CCM_ANALOG->PLL_USB2 |= CCM_ANALOG_PLL_USB2_ENABLE_MASK;
    /* Set preperiph clock source. */
    CLOCK_SetMux(kCLOCK_PrePeriphMux, 3);
    /* Set periph clock source. */
    CLOCK_SetMux(kCLOCK_PeriphMux, 0);
    /* Set periph clock2 clock source. */
    CLOCK_SetMux(kCLOCK_PeriphClk2Mux, 0);
    /* Set per clock source. */
    CLOCK_SetMux(kCLOCK_PerclkMux, 0);
    /* Set lvds1 clock source. */
    CCM_ANALOG->MISC1 = (CCM_ANALOG->MISC1 & (~CCM_ANALOG_MISC1_LVDS1_CLK_SEL_MASK)) | CCM_ANALOG_MISC1_LVDS1_CLK_SEL(0);
    /* Set clock out1 divider. */
    CCM->CCOSR = (CCM->CCOSR & (~CCM_CCOSR_CLKO1_DIV_MASK)) | CCM_CCOSR_CLKO1_DIV(0);
    /* Set clock out1 source. */
    CCM->CCOSR = (CCM->CCOSR & (~CCM_CCOSR_CLKO1_SEL_MASK)) | CCM_CCOSR_CLKO1_SEL(1);
    /* Set clock out2 divider. */
    CCM->CCOSR = (CCM->CCOSR & (~CCM_CCOSR_CLKO2_DIV_MASK)) | CCM_CCOSR_CLKO2_DIV(0);
    /* Set clock out2 source. */
    CCM->CCOSR = (CCM->CCOSR & (~CCM_CCOSR_CLKO2_SEL_MASK)) | CCM_CCOSR_CLKO2_SEL(18);
    /* Set clock out1 drives clock out1. */
    CCM->CCOSR &= ~CCM_CCOSR_CLK_OUT_SEL_MASK;
    /* Disable clock out1. */
    CCM->CCOSR &= ~CCM_CCOSR_CLKO1_EN_MASK;
    /* Disable clock out2. */
    CCM->CCOSR &= ~CCM_CCOSR_CLKO2_EN_MASK;
    /* Set SAI1 MCLK1 clock source. */
    IOMUXC_SetSaiMClkClockSource(IOMUXC_GPR, kIOMUXC_GPR_SAI1MClk1Sel, 0);
    /* Set SAI1 MCLK2 clock source. */
    IOMUXC_SetSaiMClkClockSource(IOMUXC_GPR, kIOMUXC_GPR_SAI1MClk2Sel, 0);
    /* Set SAI1 MCLK3 clock source. */
    IOMUXC_SetSaiMClkClockSource(IOMUXC_GPR, kIOMUXC_GPR_SAI1MClk3Sel, 0);
    /* Set SAI2 MCLK3 clock source. */
    IOMUXC_SetSaiMClkClockSource(IOMUXC_GPR, kIOMUXC_GPR_SAI2MClk3Sel, 0);
    /* Set SAI3 MCLK3 clock source. */
    IOMUXC_SetSaiMClkClockSource(IOMUXC_GPR, kIOMUXC_GPR_SAI3MClk3Sel, 0);
    /* Set MQS configuration. */
    IOMUXC_MQSConfig(IOMUXC_GPR,kIOMUXC_MqsPwmOverSampleRate32, 0);
    /* Set ENET1 Tx clock source. */
    IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1RefClkMode, false);
    /* Set ENET2 Tx clock source. */
#if defined(FSL_IOMUXC_DRIVER_VERSION) && (FSL_IOMUXC_DRIVER_VERSION != (MAKE_VERSION(2, 0, 0)))
    IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET2RefClkMode, false);
#else
    IOMUXC_EnableMode(IOMUXC_GPR, IOMUXC_GPR_GPR1_ENET2_CLK_SEL_MASK, false);
#endif
    /* Set GPT1 High frequency reference clock source. */
    IOMUXC_GPR->GPR5 &= ~IOMUXC_GPR_GPR5_VREF_1M_CLK_GPT1_MASK;
    /* Set GPT2 High frequency reference clock source. */
    IOMUXC_GPR->GPR5 &= ~IOMUXC_GPR_GPR5_VREF_1M_CLK_GPT2_MASK;
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
}


//////////////////////////////////////////////////////////////////////////////


#endif // defined(TEENSY_4X)
