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

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>

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
};

struct flexcan_mailbox {
	volatile uint32_t CS;
    volatile uint32_t ID;
	uint32_t WORD[16];
};

struct tx_fifo_element {
	struct flexcan_mailbox box;
	volatile uint8_t track_id;
	volatile uint8_t len;
};

struct txr_fifo_element {
	volatile uint32_t timestamp_us;
	volatile uint8_t track_id;
};

struct can {
	struct flexcan_mailbox rx_fifo[SC_BOARD_CAN_RX_FIFO_SIZE];
	struct tx_fifo_element tx_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];
	struct txr_fifo_element txr_fifo[SC_BOARD_CAN_TX_FIFO_SIZE];
	// uint8_t tx_lens[SC_BOARD_CAN_TX_FIFO_SIZE];
	// StackType_t tx_task_stack_mem[configMINIMAL_STACK_SIZE];
	// StaticTask_t tx_task_mem;
	// TaskHandle_t tx_task_handle;
	// StaticSemaphore_t tx_mutex_mem;
	// SemaphoreHandle_t tx_mutex_handle;

	CAN_Type* const flex_can;
	const IRQn_Type irq;
	volatile uint8_t isr_tx_track_id;
	volatile bool desync;
	const bool fd_capable;
	bool fd_enabled;
	bool enabled;
	volatile bool tx_box_busy;


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
		.irq = CAN3_IRQn,
		.fd_capable = true,
		.fd_enabled = false,
	},
	{
		.flex_can = CAN1,
		.irq = CAN1_IRQn,
		.fd_capable = false,
		.fd_enabled = false,
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
					| SC_FEATURE_FLAG_EHD;
	case 1: // FlexCAN1
		return 0;
	}

	__unreachable();
	return 0;
}


static void init_mailboxes(uint8_t index)
{
	struct can *can = &cans[index];

	if (can->fd_capable) {
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

extern void sc_board_can_reset(uint8_t index)
{
	struct can *can = &cans[index];

	// LOG("CAN ch%u soft reset\n", (unsigned)i);
	// can->flex_can->MCR = CAN_MCR_SOFTRST(1);
	// while (can->flex_can->MCR & CAN_MCR_SOFTRST_MASK);

	freeze(index);

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
	can->flex_can->IFLAG2 = ~0;

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

	// enable all buffer interrupts
	can->flex_can->IMASK1 = ~0;
	can->flex_can->IMASK2 = ~0;

	// set rx individual mask to 'don't care'
	memset((void*)can->flex_can->RXIMR, 0, sizeof(can->flex_can->RXIMR));


	init_mailboxes(index);

	can->rx_gi = 0;
	can->rx_pi = 0;
	can->tx_gi = 0;
	can->tx_pi = 0;

	can->txr_gi = 0;
	can->txr_pi = 0;
	can->desync = false;
	can->fd_enabled = false;
	can->enabled = false;
	can->tx_box_busy = false;
}

static inline void timer_1MHz_init(void)
{
	// GPT2 capture, input on field => GPIO1_IO19
	// IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_03_GPT2_CAPTURE1, 1);
	// IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_03_GPT2_CAPTURE1, 0b10001000);

	// gpio_pin_config_t led_config = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode };
  	// GPIO_PinInit(LED_PORT, LED_PIN, &led_config);




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

	// LOG("GPT2 div=%u, GPT2S div=%u\n", CLOCK_GetDiv(kCLOCK_Gpt2), CLOCK_GetDiv(kCLOCK_Gpt2S));

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
		;

	// GPT2->PR = GPT_PR_PRESCALER(23); // 24MHz -> 1MHz
	GPT2->PR = GPT_PR_PRESCALER(479); // 480MHz -> 1MHz


	// start timer
	GPT2->CR |= GPT_CR_EN_MASK;
}

static inline void can_init_once(void)
{
	device_identifier = OCOTP->CFG0 | OCOTP->CFG1; // 64 bit DEVICE_ID

	NVIC_SetPriority(CAN1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(CAN2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(CAN3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);


	uint32_t cscmr2 = CCM->CSCMR2 & ~(
		CCM_CSCMR2_CAN_CLK_PODF_MASK |
		CCM_CSCMR2_CAN_CLK_SEL_MASK);



	CCM->CSCMR2 = cscmr2
					// | CCM_CSCMR2_CAN_CLK_SEL(0b00)  // derive clock from pll3_sw_clk divided clock (60M)
					| CCM_CSCMR2_CAN_CLK_SEL(0b10) // derive clock from pll3_sw_clk divided clock (80M)
					| CCM_CSCMR2_CAN_CLK_PODF(0b000000) // prescaler of 1
					// | CCM_CSCMR2_CAN_CLK_PODF(0b000001) // prescaler of 2
					;

	LOG("CSCMR2=%lx\n", CCM->CSCMR2);

	// CCM->CCGR0 &= ~(
	// 	CCM_CCGR0_CG8_MASK | // CAN1 serial clock
	// 	CCM_CCGR0_CG7_MASK); // CAN1 clock




	// Grün TX = GPIO_EMC_36 (CAN3) = Board PIN 31
	//           AD_B1_08 (CAN1) = Board PIN 22


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

	LOG("CGR0=%lx\n", CCM->CCGR0);
}

extern void sc_board_init_begin(void)
{
	board_init();


	can_init_once();

	timer_1MHz_init();


	CLOCK_EnableClock(kCLOCK_Gpio1);

	// Pin 15, Output
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_03_GPIO1_IO19, 0);
	// IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_03_GPIO1_IO19, 0b11001000);
	GPIO1->GDIR |= ((uint32_t)1) << 19;
	GPIO1->DR_SET |= ((uint32_t)1) << 19;


	// Pin 14, Input
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_02_GPIO1_IO18, 0);
	// 100K Ohm Pull Up, disable pull up keeper, use input from pull up
	//                                                       32109
	IOMUXC_SetPinConfig(
		IOMUXC_GPIO_AD_B1_02_GPIO1_IO18,
		0
		| IOMUXC_SW_PAD_CTL_PAD_PUS(0b10)
		| IOMUXC_SW_PAD_CTL_PAD_SPEED(0b11) // fastest
		// | IOMUXC_SW_PAD_CTL_PAD_HYS(1)
		);
	GPIO1->GDIR &= ~(((uint32_t)1) << 18);
	// GPIO1->ICR2 |= GPIO_ICR2_ICR18(0b00); // low sensitive
	// GPIO1->ICR2 |= GPIO_ICR2_ICR18(0b01); // high sensitive
	// GPIO1->ICR2 |= GPIO_ICR2_ICR18(0b10); // rising edge sensitive
	GPIO1->ICR2 |= GPIO_ICR2_ICR18(0b11); // falling edge sensitive
	GPIO1->IMR |= ((uint32_t)1) << 18;

	// LOG("GPIO1->PSR=%lx\n", GPIO1->PSR);


	NVIC_SetPriority(GPIO1_Combined_16_31_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(GPIO1_Combined_16_31_IRQn);


	for (uint8_t i = 0; i < TU_ARRAY_SIZE(cans); ++i) {
		struct can *can = &cans[i];

		// enable the module
		can->flex_can->MCR &= ~CAN_MCR_MDIS_MASK;

		sc_board_can_reset(i);

		// can->tx_mutex_handle = xSemaphoreCreateMutexStatic(&can->tx_mutex_mem);
		// can->tx_task_handle = xTaskCreateStatic(&tx_task, NULL, TU_ARRAY_SIZE(can->tx_task_stack_mem), (void*)(uintptr_t)i, configMAX_PRIORITIES-1, can->tx_task_stack_mem, &can->tx_task_mem);
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
			.brp = 1024,
			.tseg1 = 256,
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
	(void)index;
	(void)bt;
}

extern void sc_board_can_go_bus(uint8_t index, bool on)
{
	struct can *can = &cans[index];

	can->enabled = on;

	LOG("CNT=%lx\n", GPT2->CNT);

	// GPIO1->DR_TOGGLE |= ((uint32_t)1) << 19;
	// GPIO1->DR_CLEAR |= ((uint32_t)1) << 19;


	// LOG("GPIO1->PSR=%lx\n", GPIO1->PSR);

	// LOG("GPIO1->GDIR=%lx\n", GPIO1->GDIR);

	if (on) {
		NVIC_EnableIRQ(can->irq);
		// can->flex_can->MCR &= ~CAN_MCR_HALT(1);
		// LOG("go on bus pre MCR=%lx\n", can->flex_can->MCR);
		// can->flex_can->MCR &= ~CAN_MCR_HALT_MASK;
		// LOG("go on bus post MCR=%lx\n", can->flex_can->MCR);
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
		// can->flex_can->MCR |= CAN_MCR_HALT(1);
		// can->flex_can->MCR |= CAN_MCR_HALT_MASK;
		freeze(index);
		LOG("go off bus MCR=%lx\n", can->flex_can->MCR);
		NVIC_DisableIRQ(can->irq);

		// clear interrupts
		can->flex_can->IFLAG1 = can->flex_can->IFLAG1;
		can->flex_can->IFLAG2 = can->flex_can->IFLAG2;

		// clear error flags
		can->flex_can->ESR1 = ~0;

		// reset mailboxes
		init_mailboxes(index);
	}
}

SC_RAMFUNC static inline void transfer_to_box(
	struct can *can,
	struct flexcan_mailbox *box,
	struct sc_msg_can_tx const * msg)
{

}

SC_RAMFUNC extern bool sc_board_can_tx_queue(uint8_t index, struct sc_msg_can_tx const * msg)
{
	struct can *can = &cans[index];
	struct tx_fifo_element *e = NULL;
	uint8_t gi = __atomic_load_n(&can->tx_gi, __ATOMIC_ACQUIRE);
	uint8_t pi = can->tx_pi;
	uint8_t used = pi = gi;
	uint8_t mailbox_index = 0;
	uint32_t id = 0;
	uint32_t cs = CAN_CS_DLC(msg->dlc);
	unsigned bytes = dlc_to_len(msg->dlc);

	if (unlikely(used == TU_ARRAY_SIZE(can->tx_fifo))) {
		return false;
	}

	mailbox_index = pi % TU_ARRAY_SIZE(can->tx_fifo);
	e = &can->tx_fifo[mailbox_index];

	// can->tx_track_ids[mailbox_index] = msg->track_id;
	e->track_id = msg->track_id;

	if (can->fd_enabled && (msg->flags & SC_CAN_FRAME_FLAG_FDF)) {
		memcpy(e->box.WORD, msg->data, bytes);
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
		memcpy(e->box.WORD, msg->data, bytes);
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

	// LOG("ch%u notify tx task\n", index);
	// xTaskNotifyGive(can->tx_task_handle);
	GPIO1->DR_CLEAR |= ((uint32_t)1) << 19;

	return true;
}

SC_RAMFUNC extern void sc_board_can_status_fill(uint8_t index, struct sc_msg_can_status *msg)
{
	struct can *can = &cans[index];
	uint32_t ecr = can->flex_can->ECR;


	msg->flags = 0;
	if (unlikely(can->desync)) {
		msg->flags = SC_CAN_STATUS_FLAG_TXR_DESYNC;
	}
	msg->rx_errors = (ecr & CAN_ECR_RXERRCNT_MASK) >> CAN_ECR_RXERRCNT_SHIFT;
	msg->tx_errors = (ecr & CAN_ECR_TXERRCNT_MASK) >> CAN_ECR_TXERRCNT_SHIFT;
}

SC_RAMFUNC extern int sc_board_can_place_msgs(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	struct can *can = &cans[index];

	unsigned txrs_removed = 0;
	int bytes = 0;
	bool have_data_to_send = false;
	uint8_t txr_gi = can->txr_gi;

	for (;;) {
		uint8_t txr_pi = __atomic_load_n(&can->txr_pi, __ATOMIC_ACQUIRE);

		if (txr_pi != txr_gi) {

			LOG("ch%u have txr\n", index);;
			struct sc_msg_can_txr *txr = (struct sc_msg_can_txr *)tx_ptr;
			have_data_to_send = true;

			if (tx_ptr + sizeof(*txr) <= tx_end) {
				struct txr_fifo_element *e = &can->txr_fifo[txr_gi % sizeof(can->txr_fifo)];

				LOG("ch%u place txr %u\n", index, e->track_id);
				bytes += sizeof(*txr);
				tx_ptr += sizeof(*txr);

				txr->id = SC_MSG_CAN_TXR;
				txr->len = sizeof(*txr);
				txr->flags = 0;
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

	if (txr_gi != can->txr_gi) {
		__atomic_store_n(&can->txr_gi, txr_gi, __ATOMIC_RELEASE);
	}

	if (bytes > 0) {
		return bytes;
	}

	return have_data_to_send ? 0 : -1;
}


SC_RAMFUNC static inline void service_tx_box(uint8_t index)
{
	struct can *can = &cans[index];
	uint8_t* box_mem = ((uint8_t*)can->flex_can) + 0x80;
	struct flexcan_mailbox *box = (struct flexcan_mailbox *)box_mem;
	uint32_t cs = box->CS;
	unsigned code = (cs & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT;
	uint8_t tx_fifo_gi = can->tx_gi;
	uint8_t tx_fifo_pi = __atomic_load_n(&can->tx_pi, __ATOMIC_ACQUIRE);

	if (MB_RX_EMPTY == code) {
		// remove rx empty for RTR frames
		LOG("ch%u RTR cleanup\n", index);
		code = MB_TX_INACTIVE;
		cs &= ~CAN_CS_CODE_MASK;
		cs |= CAN_CS_CODE(code);
		box->CS = cs;
	}

	if (can->tx_box_busy && MB_TX_INACTIVE == code) {
		// tx done
		uint8_t txr_pi = can->txr_pi;
		uint8_t txr_gi = __atomic_load_n(&can->txr_gi, __ATOMIC_ACQUIRE);
		uint8_t used = txr_pi - txr_gi;

		if (unlikely(used == TU_ARRAY_SIZE(can->txr_fifo))) {
			LOG("ch%u txr desync\n", index);
			can->desync = true;
		} else {
			struct txr_fifo_element *e = &can->txr_fifo[txr_pi % TU_ARRAY_SIZE(can->txr_fifo)];

			e->track_id = can->isr_tx_track_id;
			e->timestamp_us = GPT2->CNT;
			__atomic_store_n(&can->txr_pi, txr_pi + 1, __ATOMIC_RELEASE);

			sc_can_notify_task_isr(index, 1);
		}

		can->tx_box_busy = false;
	}


	if (!can->tx_box_busy && tx_fifo_pi != tx_fifo_gi) {
		uint8_t tx_mailbox_index = tx_fifo_gi % TU_ARRAY_SIZE(can->tx_fifo);

		LOG("ch%u tx fifo pi=%u gi=%u\n", index, tx_fifo_pi, tx_fifo_gi);

		switch (code) {
		case MB_TX_INACTIVE: {
			LOG("ch%u empty FlexCAN mailbox found\n", index);
			can->isr_tx_track_id = can->tx_fifo[tx_mailbox_index].track_id;
			memcpy(box->WORD, can->tx_fifo[tx_mailbox_index].box.WORD, can->tx_fifo[tx_mailbox_index].len);
			box->ID = can->tx_fifo[tx_mailbox_index].box.ID;
			box->CS = can->tx_fifo[tx_mailbox_index].box.CS;

			__atomic_store_n(&can->tx_gi, tx_fifo_gi + 1, __ATOMIC_RELEASE);

			can->tx_box_busy = true;
		} break;
		default:
			LOG("ch%u no mailbox found\n", index);
			break;
		}
	}
}

SC_RAMFUNC static void can_int(uint8_t index)
{
	struct can *can = &cans[index];
	uint32_t esr1 = can->flex_can->ESR1;
	uint32_t iflag1 = can->flex_can->IFLAG1;
	uint32_t iflag2 = can->flex_can->IFLAG2;
	// unsigned give = 0;

	if (iflag1) {
		LOG("IFLAG1=%lx\n", iflag1);
	}

	if (iflag2) {
		LOG("IFLAG1=%lx\n", iflag2);
	}

	// for (uint32_t i = 0, j = 1, k = 0;
	// 	k < 32 && i < TX_MAILBOX_COUNT; ++i, j <<= 1, ++k) {
	// 	if (iflag1 & j) {
	// 		++give;
	// 	}
	// }

	// for (uint32_t i = 32, j = 1, k = 0;
	// 	k < 32 && i < TX_MAILBOX_COUNT; ++i, j <<= 1, ++k) {
	// 	if (iflag2 & j) {
	// 		++give;
	// 	}
	// }




	// LOG("ESR1=%lx\n", esr1);


	// clear interrupts
	can->flex_can->IFLAG1 = iflag1;
	can->flex_can->IFLAG2 = iflag2;

	// clear errors
	can->flex_can->ESR1 = ~0;

	if (iflag1 & 1) {
		// TX mailbox

	}

	service_tx_box(index);

	// for (unsigned i = 0; i < give; ++i) {
	// 	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// 	vTaskNotifyGiveFromISR(can->tx_task_handle, &xHigherPriorityTaskWoken);
	// 	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	// }
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

SC_RAMFUNC void GPT2_IRQHandler(void)
{
	LOG("GPT2 capture\n");

	// clear status flags
	GPT2->SR = ~0;
}

SC_RAMFUNC void GPIO1_Combined_16_31_IRQHandler(void)
{
	// LOG("GPIO1_Combined_16_31_IRQHandler\n");

	uint32_t isr = GPIO1->ISR;

	// clear interrupts
	GPIO1->ISR = isr;

	// reset output pin
	GPIO1->DR_SET |= ((uint32_t)1) << 19;

	service_tx_box(1); // FIX ME
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

// SC_RAMFUNC static void tx_task(void* param)
// {
// 	const uint8_t index = (uint8_t)(uintptr_t)param;
// 	SC_ASSERT(index < TU_ARRAY_SIZE(cans));

// 	LOG("ch%u tx task start\n", index);

// 	struct can *can = &cans[index];
// 	uint8_t* box_mem = ((uint8_t*)can->flex_can) + 0x80;

// 	while (42) {
// 		(void)ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

// 		// LOG("ch%u tx task take\n", index);

// 		if (unlikely(!can->enabled)) {
// 			LOG("ch%u tx task CAN disabled\n", index);
// 			continue;
// 		}

// 		const size_t mailbox_step = can->fd_enabled ? 0x48 : 0x10;

// 		while (pdTRUE != xSemaphoreTake(can->tx_mutex_handle, portMAX_DELAY));

// 		LOG("ch%u tx task enter work\n", index);

// 		for (bool done = false; !done; ) {
// 			uint8_t gi = can->tx_gi;
// 			uint8_t pi = __atomic_load_n(&can->tx_pi, __ATOMIC_ACQUIRE);
// 			uint8_t tx_mailbox_index = gi % TU_ARRAY_SIZE(can->tx_fifo);

// 			if (likely(pi != gi)) {
// 				LOG("ch%u pi=%u gi=%u\n", index, pi, gi);
// 				struct flexcan_mailbox *box = (struct flexcan_mailbox *)box_mem;
// 				unsigned code = (box->CS & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT;

// 				switch (code) {
// 				case MB_TX_INACTIVE:
// 				case MB_RX_EMPTY:
// 					LOG("ch%u empty FlexCAN mailbox found\n", index);
// 					memcpy(box->WORD, can->tx_fifo[tx_mailbox_index].WORD, can->tx_lens[tx_mailbox_index]);
// 					box->ID = can->tx_fifo[tx_mailbox_index].ID;
// 					box->CS = can->tx_fifo[tx_mailbox_index].CS;

// 					__atomic_store_n(&can->tx_gi, gi + 1, __ATOMIC_RELEASE);
// 					break;
// 				default:
// 					done = true;
// 					LOG("ch%u no mailbox found\n", index);
// 					break;
// 				}
// 			} else {
// 				LOG("ch%u tx fifo empty\n", index);
// 				done = true;
// 			}
// 		}

// 		LOG("ch%u tx task exit work\n", index);

// 		xSemaphoreGive(can->tx_mutex_handle);
// 	}
// }

#endif // defined(TEENSY_4X)
