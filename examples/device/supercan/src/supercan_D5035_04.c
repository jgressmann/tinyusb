/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */



#ifdef D5035_04

#include <supercan_debug.h>
#include <supercan_board.h>

#include <leds.h>
#include <tusb.h>


#include <usb_descriptors.h> // DFU_USB_RESET_TIMEOUT_MS


#define CONF_CPU_FREQUENCY 120000000L
#define CONF_USB_FREQUENCY 48000000L
#define USART_BAUDRATE     115200L
#define PORT_STEP 0x00000400U


struct led {
	uint8_t port_pin_mux;
};

#define LED_STATIC_INITIALIZER(name, pin) \
	{ pin }



static const struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", (1 << 4) | 14),  // PB14
	LED_STATIC_INITIALIZER("red", 4),                // PA04
	LED_STATIC_INITIALIZER("orange", 5),             // PA05
	LED_STATIC_INITIALIZER("green", 6),              // PA06
	LED_STATIC_INITIALIZER("blue", 7),               // PA07
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


extern void sc_board_init_begin(void)
{
	board_init();
	// LOG("Vectors ROM @ %p\n", (void*)SCB->VTOR);
	// move_vector_table_to_ram();
	// LOG("Vectors RAM @ %p\n", (void*)SCB->VTOR);






	leds_init();

	// can_init_pins();
	// can_init_clock();
	// can_init_module();

	// init_usb();
	// rev_read();

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
	// struct same5x_can *can = &same5x_cans[index];

	// switch (status) {
	// case SC_CAN_LED_STATUS_DISABLED:
	// 	led_set(can->led_status_green, 0);
	// 	led_set(can->led_status_red, 0);
	// 	break;
	// case SC_CAN_LED_STATUS_ENABLED_OFF_BUS:
	// 	led_set(can->led_status_green, 1);
	// 	led_set(can->led_status_red, 0);
	// 	break;
	// case SC_CAN_LED_STATUS_ENABLED_ON_BUS_PASSIVE:
	// 	led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_PASSIVE_MS);
	// 	led_set(can->led_status_red, 0);
	// 	break;
	// case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ACTIVE:
	// 	led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
	// 	led_set(can->led_status_red, 0);
	// 	break;
	// case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_PASSIVE:
	// 	led_set(can->led_status_green, 0);
	// 	led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_PASSIVE_MS);
	// 	break;
	// case SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_ACTIVE:
	// 	led_set(can->led_status_green, 0);
	// 	led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS);
	// 	break;
	// case SC_CAN_LED_STATUS_ENABLED_ON_BUS_BUS_OFF:
	// 	led_set(can->led_status_green, 0);
	// 	led_set(can->led_status_red, 1);
	// 	break;
	// default:
	// 	led_blink(can->led_status_green, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
	// 	led_blink(can->led_status_red, SC_CAN_LED_BLINK_DELAY_ACTIVE_MS / 2);
	// 	break;
	// }
}



__attribute__((noreturn)) extern void sc_board_reset(void)
{
	NVIC_SystemReset();
	__unreachable();
}

uint32_t sc_board_identifier(void)
{
	return 0xdeadbeef;
}

uint16_t sc_board_can_feat_perm(uint8_t index)
{
	return 0;
}

uint16_t sc_board_can_feat_conf(uint8_t index)
{
	return 0;
}

void sc_board_can_feat_set(uint8_t index, uint16_t features)
{

}

sc_can_bit_timing_range const* sc_board_can_nm_bit_timing_range(uint8_t index)
{
	return NULL;
}

sc_can_bit_timing_range const* sc_board_can_dt_bit_timing_range(uint8_t index)
{
	return NULL;
}

void sc_board_can_nm_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{

}

void sc_board_can_dt_bit_timing_set(uint8_t index, sc_can_bit_timing const *bt)
{

}

void sc_board_can_go_bus(uint8_t index, bool on)
{

}

SC_RAMFUNC bool sc_board_can_tx_queue(uint8_t index, struct sc_msg_can_tx const * msg)
{
	return false;
}


void sc_board_can_reset(uint8_t index)
{

}

SC_RAMFUNC int sc_board_can_retrieve(uint8_t index, uint8_t *tx_ptr, uint8_t *tx_end)
{
	return 0;
}



#endif // #ifdef D5035_04
