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
	TX_MAILBOX_COUNT = 7,
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

struct can {
	CAN_Type* const flex_can;
	IRQn_Type irq;
	// const uint8_t rx_mailbox_count;
	bool fd_capable;
};

static struct can cans[] = {
	{
		.flex_can = CAN3,
		.irq = CAN3_IRQn,
		// .rx_mailbox_count = 14 - TX_MAILBOX_COUNT,
		.fd_capable = true,
	},
	{
		.flex_can = CAN1,
		.irq = CAN1_IRQn,
		// .rx_mailbox_count = 64 - TX_MAILBOX_COUNT,
		.fd_capable = false,
	},
};

static uint32_t device_identifier;

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
					| SC_FEATURE_FLAG_TXP
					| SC_FEATURE_FLAG_EHD;
	case 1: // FlexCAN1
		return SC_FEATURE_FLAG_TXP;
	}

	__unreachable();
	return 0;
}

extern void sc_board_can_reset(uint8_t index)
{
	(void)index;
}

static void init_mailboxes(uint8_t index)
{
	struct can *can = &cans[index];

	if (can->fd_capable) {
		uint8_t rx_count = 14 - TX_MAILBOX_COUNT;

		for (uint8_t i = 0; i < TX_MAILBOX_COUNT; ++i) {
			can->flex_can->MB_64B[i].ID = 0;
			can->flex_can->MB_64B[i].CS = CAN_CS_CODE(MB_TX_INACTIVE);
		}

		for (uint8_t i = TX_MAILBOX_COUNT; i < TX_MAILBOX_COUNT + rx_count; ++i) {
			can->flex_can->MB_64B[i].ID = 0;
			can->flex_can->MB_64B[i].CS = CAN_CS_CODE(MB_RX_EMPTY);
		}
	} else {
		uint8_t rx_count = 64 - TX_MAILBOX_COUNT;

		for (uint8_t i = 0; i < TX_MAILBOX_COUNT; ++i) {
			can->flex_can->MB_8B[i].ID = 0;
			can->flex_can->MB_8B[i].CS = CAN_CS_CODE(MB_TX_INACTIVE);
		}

		for (uint8_t i = TX_MAILBOX_COUNT; i < TX_MAILBOX_COUNT + rx_count; ++i) {
			can->flex_can->MB_8B[i].ID = 0;
			can->flex_can->MB_8B[i].CS = CAN_CS_CODE(MB_RX_EMPTY);
		}
	}
}

extern void sc_board_init_begin(void)
{
	NVIC_SetPriority(CAN1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(CAN2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(CAN3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

	board_init();



	//CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
	CLOCK_EnableClock(kCLOCK_Can3);
	CLOCK_EnableClock(kCLOCK_Can3S);
	CLOCK_EnableClock(kCLOCK_Can1);
	CLOCK_EnableClock(kCLOCK_Can1S);
	CLOCK_EnableClock(kCLOCK_Ocotp);

	device_identifier = OCOTP->CFG0 | OCOTP->CFG1; // 64 bit DEVICE_ID

	uint32_t cscmr2 = CCM->CSCMR2 & ~(
		(CCM_CSCMR2_CAN_CLK_PODF_MASK << CCM_CSCMR2_CAN_CLK_PODF_SHIFT) |
		(CCM_CSCMR2_CAN_CLK_SEL_MASK << CCM_CSCMR2_CAN_CLK_SEL_SHIFT));




	CCM->CSCMR2 = cscmr2
					| CCM_CSCMR2_CAN_CLK_SEL(0b10)  // 80 MHz
					| CCM_CSCMR2_CAN_CLK_PODF(0b000000) // prescaler of 1
					;




  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_36_FLEXCAN3_TX, 0);
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_37_FLEXCAN3_RX, 0);
  IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_36_FLEXCAN3_TX, 0b11001000);
  IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_37_FLEXCAN3_RX, 0b11000000);


	for (size_t i = 0; i < TU_ARRAY_SIZE(cans); ++i) {
		struct can *can = &cans[i];

		// LOG("CAN ch%u soft reset\n", (unsigned)i);
		// can->flex_can->MCR = CAN_MCR_SOFTRST(1);
		// while (can->flex_can->MCR & CAN_MCR_SOFTRST_MASK);

		LOG("CAN ch%u ", (unsigned)i);

		can->flex_can->MCR = CAN_MCR_MDIS(1)  // module disable
				| CAN_MCR_SUPV(0) // This bit configures the FlexCAN to be either in Supervisor or User mode
				| CAN_MCR_WRNEN(1) // Warning Interrupt Enable
				| CAN_MCR_IRMQ(1) // Individual Rx Masking And Queue Enable
				| CAN_MCR_SRXDIS(1) // disable self resception
				| CAN_MCR_MAXMB(can->fd_capable ? 14 : 64)
				| CAN_MCR_FRZ(1) // halt / debug should freeze
				| CAN_MCR_HALT(1) // halt
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

		can->flex_can->CTRL2 =  CAN_CTRL2_BOFFDONEMSK(1) // Bus Off Done Interrupt Mask
								| CAN_CTRL2_RRS(1) // store remote request frames
								| (can->fd_capable ? CAN_CTRL2_ERRMSK_FAST(1) : 0)  // Error Interrupt Mask for errors detected in the data phase of fast CAN FD frames
								| (can->fd_capable ? CAN_CTRL2_ISOCANFDEN(1) : 0) // CAN-FD in ISO mode
								;

		LOG("CTRL2=%lx ", can->flex_can->CTRL2);



		// enable all message buffer interrupts
		can->flex_can->IFLAG1 = ~0;
		can->flex_can->IFLAG2 = ~0;

		if (can->fd_capable) {
			can->flex_can->CBT = CAN_CBT_BTF(1); // mooar arbitration bits

			can->flex_can->FDCTRL = CAN_FDCTRL_MBDSR0(3) // 64 byte per box
									| CAN_FDCTRL_MBDSR1(3) // 64 byte per box
									| CAN_FDCTRL_TDCEN(1) // enable transmitter delay compensation
									;


		} else {

		}

		// LOG("CAN ch0 MCR=%lx CTRL1=%lx CTRL2=%lx",
		// 	can->flex_can->MCR,
		// 	can->flex_can->CTRL1,
		// 	can->flex_can->CTRL2);

		if (can->fd_capable) {
			LOG("CBT=%lx FDCTRL=%lx ", can->flex_can->CBT, can->flex_can->FDCTRL);
		}


		can->flex_can->MCR &= ~CAN_MCR_MDIS(1);  // module enable

		LOG("MCR=%lx ", can->flex_can->MCR);

		init_mailboxes(i);

		// set rx individual mask to 'don't care'
		memset((void*)can->flex_can->RXIMR, 0, sizeof(can->flex_can->RXIMR));

		LOG("\n");
	}


}


extern void sc_board_init_end(void)
{

}

// extern void sc_board_fill_can_info(uin8_t index, struct sc_msg_can_info* msg)
// {
// 	msg->can_clk_hz = SC_BOARD_CAN_CLK_HZ;
// 	msg->nmbt_brp_min = nm_bt->min.brp;
// 	msg->nmbt_brp_max = nm_bt->max.brp;
// 	msg->nmbt_sjw_max = nm_bt->max.sjw;
// 	msg->nmbt_tseg1_min = nm_bt->min.tseg1;
// 	msg->nmbt_tseg1_max = nm_bt->max.tseg1;
// 	msg->nmbt_tseg2_min = nm_bt->min.tseg2;
// 	msg->nmbt_tseg2_max = nm_bt->max.tseg2;
// 	msg->dtbt_brp_min = dt_bt->min.brp;
// 	msg->dtbt_brp_max = dt_bt->max.brp;
// 	msg->dtbt_sjw_max = dt_bt->max.sjw;
// 	msg->dtbt_tseg1_min = dt_bt->min.tseg1;
// 	msg->dtbt_tseg1_max = dt_bt->max.tseg1;
// 	msg->dtbt_tseg2_min = dt_bt->min.tseg2;
// 	msg->dtbt_tseg2_max = dt_bt->max.tseg2;
// 	msg->tx_fifo_size = 32;
// 	msg->rx_fifo_size = 32;
// }

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
				.tseg1 = 2048,
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
				.tseg1 = 64,
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
			prop_seg = bt->tseg1;
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
			(CAN_CTRL1_PROPSEG_MASK << CAN_CTRL1_PROPSEG_SHIFT) |
			(CAN_CTRL1_PSEG1_MASK << CAN_CTRL1_PSEG1_SHIFT) |
			(CAN_CTRL1_PSEG2_MASK << CAN_CTRL1_PSEG2_SHIFT) |
			(CAN_CTRL1_RJW_MASK << CAN_CTRL1_RJW_SHIFT));

		uint16_t prop_seg = 0;
		uint16_t tseg1 = 0;

		if (bt->tseg1 > 8) {
			prop_seg = 8;
			tseg1 = bt->tseg1 - prop_seg;
		} else {
			prop_seg = bt->tseg1;
		}

		can->flex_can->CTRL1 = reg
								| CAN_CTRL1_PRESDIV(bt->brp - 1)
								| CAN_CTRL1_RJW(bt->sjw - 1)
								| CAN_CTRL1_PROPSEG(prop_seg - 1)
								| CAN_CTRL1_PSEG1(tseg1 - 1)
								| CAN_CTRL1_PSEG2(bt->tseg2 - 1)
								;

		LOG("ch%u CTRL1 brp=%u sjw=%u propseg=%u tseg1=%u tseg2=%u\n",
			index, bt->brp, bt->sjw, prop_seg, tseg1, bt->tseg2);
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

	if (on) {
		NVIC_EnableIRQ(can->irq);
		can->flex_can->MCR &= ~CAN_MCR_HALT(1);
		LOG("go on bus MCR=%lx\n", can->flex_can->MCR);
	} else {
		can->flex_can->MCR |= CAN_MCR_HALT(1);
		LOG("go off bus MCR=%lx\n", can->flex_can->MCR);
		NVIC_DisableIRQ(can->irq);

		// clear interrupts
		can->flex_can->IFLAG1 = can->flex_can->IFLAG1;
		can->flex_can->IFLAG2 = can->flex_can->IFLAG2;

		// clear error flags
		can->flex_can->ESR1 = can->flex_can->ESR1;

		// reset mailboxes
		init_mailboxes(index);
	}
}

SC_RAMFUNC extern bool sc_board_can_tx_queue(uint8_t index, struct sc_msg_can_tx const * msg)
{
	(void)index;
	(void)msg;

	return true;
}

SC_RAMFUNC extern void sc_board_can_status_fill(uint8_t index, struct sc_msg_can_status *msg)
{
	struct can *can = &cans[index];
	uint32_t ecr = can->flex_can->ECR;


	msg->rx_errors = (ecr & CAN_ECR_RXERRCNT_MASK) >> CAN_ECR_RXERRCNT_SHIFT;
	msg->tx_errors = (ecr & CAN_ECR_TXERRCNT_MASK) >> CAN_ECR_TXERRCNT_SHIFT;
}

SC_RAMFUNC extern int sc_board_can_place_msgs(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	(void)index;
	(void)tx_ptr;
	(void)tx_end;

	return -1;
}

SC_RAMFUNC void CAN1_IRQHandler(void)
{
	LOG(".");
}

SC_RAMFUNC void CAN2_IRQHandler(void)
{
	LOG("+");
}

SC_RAMFUNC void CAN3_IRQHandler(void)
{
	LOG("\\");
}



extern uint32_t sc_board_identifier(void)
{
	return device_identifier;
}

extern void sc_board_can_feat_set(uint8_t index, uint16_t features)
{
	(void)index;
	(void)features;
}

#endif // defined(TEENSY_4X)
