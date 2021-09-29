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
	// NVIC_SetPriority(USB_PHY1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	// NVIC_SetPriority(USB_PHY2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  	// NVIC_SetPriority(USB_OTG2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	// NVIC_SetPriority(USB_OTG1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

	board_init();
}



extern void sc_board_init_end(void)
{

}

extern sc_can_bit_timing_range const* sc_board_can_nm_bit_timing_range(uint8_t index)
{
	static const sc_can_bit_timing_range ranges[] = {
		{
			.min = {
				.brp = 0,
				.tseg1 = 0,
				.tseg2 = 0,
				.sjw = 0,
			},
		}
	};

	return &ranges[index];
}

extern sc_can_bit_timing_range const* sc_board_can_dt_bit_timing_range(uint8_t index)
{
	static const sc_can_bit_timing_range ranges[] = {
		{
			.min = {
				.brp = 0,
				.tseg1 = 0,
				.tseg2 = 0,
				.sjw = 0,
			},
		}
	};

	return &ranges[index];
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
	(void)index;
	(void)on;
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



#endif // defined(TEENSY_4X)
