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
#include "supercan_board.h"
#include "leds.h"

enum {
	TX_MAILBOX_COUNT = 2
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

extern void sc_board_init_begin(void)
{
	NVIC_SetPriority(CAN1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(CAN2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(CAN3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

	board_init();

	// TODO pins & mux setup

	for (size_t i = 0; i < TU_ARRAY_SIZE(cans); ++i) {
		struct can *can = &cans[i];

		can->flex_can->MCR = CAN_MCR_MDIS(0)  // module disable
				| CAN_MCR_SUPV(0) // This bit configures the FlexCAN to be either in Supervisor or User mode
				| CAN_MCR_WRNEN(1) // Warning Interrupt Enable
				| CAN_MCR_SOFTRST(1) // soft reset
				| CAN_MCR_SRXDIS(1) // disable self resception
				| CAN_MCR_MAXMB(TX_MAILBOX_COUNT)
				;

		can->flex_can->CTRL1 = CAN_CTRL1_LBUF(1)  // Lowest Buffer Transmitted First
				| CAN_CTRL1_BOFFREC(1) // bus off interrupt enable
				| CAN_CTRL1_ERRMSK(1) // Error interrupt enabled
				| CAN_CTRL1_TWRNMSK(1) // tx warning interrupt enabled
				| CAN_CTRL1_RWRNMSK(1) // rx warning interrupt enabled
				| CAN_CTRL1_LBUF(1) // lowest number buffer is transmitted first
				;

		can->flex_can->CTRL2 = CAN_CTRL2_ERRMSK_FAST(1)  // Error Interrupt Mask for errors detected in the data phase of fast CAN FD frames
								| CAN_CTRL2_BOFFDONEMSK(1) // Bus Off Done Interrupt Mask
								;

		// enable all message buffer interrupts
		can->flex_can->IFLAG1 = ~0;
		can->flex_can->IFLAG2 = ~0;

		if (can->fd_capable) {
			can->flex_can->CBT = CAN_CBT_BTF(1); // mooar arbitration bits

			can->flex_can->FDCTRL = CAN_FDCTRL_MBDSR0(3) // 64 byte per box
									| CAN_FDCTRL_MBDSR1(3) // 64 byte per box
									| CAN_FDCTRL_TDCEN(1) // enable transmitter delay compensation
									;
		}
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
				.tseg2 = 32,
				.sjw = 1,
			},
			.max = {
				.brp = 1024,
				.tseg1 = 2048,
				.tseg2 = 8,
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
	(void)index;
	(void)bt;
}

extern void sc_board_can_dt_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{
	(void)index;
	(void)bt;
}

extern void sc_board_can_go_bus(uint8_t index, bool on)
{
	if (on) {
		cans[index].flex_can->MCR &= ~CAN_MCR_FRZ(1);
	} else {
		cans[index].flex_can->MCR |= CAN_MCR_FRZ(1);
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
	(void)index;
	(void)msg;
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

}

SC_RAMFUNC void CAN2_IRQHandler(void)
{

}

SC_RAMFUNC void CAN3_IRQHandler(void)
{

}

#endif // defined(TEENSY_4X)
