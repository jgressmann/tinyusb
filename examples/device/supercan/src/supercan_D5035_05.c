/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#include <supercan_board.h>

#if D5035_05

#include <FreeRTOS.h>
#include <timers.h>

#include <supercan_debug.h>
#include <leds.h>
#include <tusb.h>
#include <hw/bsp/board.h>


#define CONF_CPU_FREQUENCY 64000000L
#define USART_BAUDRATE     115200L


#define PORT_SHIFT 4
#define PIN_MASK 15
#define MAKE_PIN(port, pin) (((port) << PORT_SHIFT) | (pin))

#define PIN_PA03 MAKE_PIN(0, 3)
#define PIN_PA04 MAKE_PIN(0, 4)
#define PIN_PA05 MAKE_PIN(0, 5)
#define PIN_PA06 MAKE_PIN(0, 6)

#define PIN_PB04 MAKE_PIN(1, 4)
#define PIN_PB05 MAKE_PIN(1, 5)
#define PIN_PB06 MAKE_PIN(1, 6)
#define PIN_PB07 MAKE_PIN(1, 7)
#define PIN_PB12 MAKE_PIN(1, 12)

#define  GPIO_MODE_OUTPUT_PP 0x00000001U



// controller and hardware specific setup of i/o pins for CAN
static inline void can_init(void)
{
	// PINS, CAN module

	mcan_can_init();

	mcan_cans[0].m_can = (MCanX*)FDCAN1;
	mcan_cans[1].m_can = (MCanX*)FDCAN2;
	mcan_cans[0].interrupt_id = TIM16_FDCAN_IT0_IRQn;
	mcan_cans[1].interrupt_id = TIM17_FDCAN_IT1_IRQn;
	// mcan_cans[0].led_traffic = CAN0_TRAFFIC_LED;
	// mcan_cans[1].led_traffic = CAN1_TRAFFIC_LED;
	mcan_cans[0].led_status_green = LED_CAN0_STATUS_GREEN;
	mcan_cans[1].led_status_green = LED_CAN1_STATUS_GREEN;
	mcan_cans[0].led_status_red = LED_CAN0_STATUS_RED;
	mcan_cans[1].led_status_red = LED_CAN1_STATUS_RED;

	for (size_t j = 0; j < TU_ARRAY_SIZE(mcan_cans); ++j) {
		struct mcan_can *can = &mcan_cans[j];
		can->features = CAN_FEAT_PERM;
	}

	m_can_init_begin(mcan_cans[0].m_can);
	m_can_init_begin(mcan_cans[1].m_can);

	mcan_cans[0].m_can->MRCFG.reg = MCANX_MRCFG_QOS_HIGH;
	mcan_cans[1].m_can->MRCFG.reg = MCANX_MRCFG_QOS_HIGH;

	NVIC_SetPriority(mcan_cans[0].interrupt_id, SC_ISR_PRIORITY);
	NVIC_SetPriority(mcan_cans[1].interrupt_id, SC_ISR_PRIORITY);

	LOG("M_CAN release %u.%u.%u (%lx)\n", mcan_cans[0].m_can->CREL.bit.REL, mcan_cans[0].m_can->CREL.bit.STEP, mcan_cans[0].m_can->CREL.bit.SUBSTEP, mcan_cans[0].m_can->CREL.reg);
}

static inline void counter_1mhz_init(void)
{

}

struct led {
	uint8_t port_pin_mux;
};

#define LED_STATIC_INITIALIZER(name, mux) \
	{ mux }


static const struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PB12), // board led
	LED_STATIC_INITIALIZER("red", PIN_PA03),
	LED_STATIC_INITIALIZER("orange", PIN_PA04),
	LED_STATIC_INITIALIZER("green", PIN_PA05),
	LED_STATIC_INITIALIZER("blue", PIN_PA06),
	LED_STATIC_INITIALIZER("can0_green", PIN_PB04),
	LED_STATIC_INITIALIZER("can0_red", PIN_PB05),
	LED_STATIC_INITIALIZER("can1_green", PIN_PB06),
	LED_STATIC_INITIALIZER("can1_red", PIN_PB07),
};

static inline void leds_init(void)
{
	// enable clock to GPIO block A, B
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

	// disable output
	GPIOA->BSRR =
		GPIO_BSRR_BR3 |
		GPIO_BSRR_BR4 |
		GPIO_BSRR_BR5 |
		GPIO_BSRR_BR6;

	GPIOB->BSRR =
		GPIO_BSRR_BR4 |
		GPIO_BSRR_BR5 |
		GPIO_BSRR_BR6 |
		GPIO_BSRR_BR7 |
		GPIO_BSRR_BR12;

  // switch output type is push-pull on reset

  // switch mode to output
  GPIOA->MODER =
  	(GPIOA->MODER & ~(
		GPIO_MODER_MODE3 |
		GPIO_MODER_MODE4 |
		GPIO_MODER_MODE5 |
		GPIO_MODER_MODE6))
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE3_Pos)
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE4_Pos)
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE5_Pos)
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE6_Pos);

  GPIOB->MODER =
  	(GPIOB->MODER & ~(
		GPIO_MODER_MODE4 |
		GPIO_MODER_MODE5 |
		GPIO_MODER_MODE6 |
		GPIO_MODER_MODE7 |
		GPIO_MODER_MODE12))
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE4_Pos)
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE5_Pos)
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE6_Pos)
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE7_Pos)
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE12_Pos);
}

#define POWER_LED LED_DEBUG_0
#define USB_LED LED_DEBUG_3


extern void sc_board_led_set(uint8_t index, bool on)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(leds));

	unsigned port = leds[index].port_pin_mux >> PORT_SHIFT;
	unsigned pin = leds[index].port_pin_mux & PIN_MASK;

	GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x00000400UL * port));

	gpio->BSRR = UINT32_C(1) << (pin + (!on) * 16);
}

extern void sc_board_leds_on_unsafe(void)
{
	for (size_t i = 0; i < TU_ARRAY_SIZE(leds); ++i) {
		sc_board_led_set(i, 1);
	}
}


extern void sc_board_init_begin(void)
{
	board_init();

	leds_init();

	can_init();

	counter_1mhz_init();
}

extern void sc_board_init_end(void)
{
	led_blink(0, 2000);
	led_set(POWER_LED, 1);
}

// extern char const* sc_board_name(void)
// {
// 	return SC_BOARD_NAME;
// }

SC_RAMFUNC extern void sc_board_led_can_status_set(uint8_t index, int status)
{
	struct mcan_can *can = &mcan_cans[index];

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

extern uint32_t sc_board_identifier(void)
{
	uint32_t *id_ptr = (uint32_t *)UID_BASE;
	uint32_t id = 0;

	// 96 bit unique device ID
	id ^= *id_ptr++;
	id ^= *id_ptr++;
	id ^= *id_ptr++;

	return id;
}

SC_RAMFUNC void CAN0_Handler(void)
{
	// LOG("CAN0 int\n");

	mcan_can_int(0);
}


SC_RAMFUNC void CAN1_Handler(void)
{
	// LOG("CAN1 int\n");

	mcan_can_int(1);
}


#endif // #if D5035_05