/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#include <supercan_board.h>

#if STM32H7A3NUCLEO

#include <supercan_debug.h>
#include <leds.h>
#include <tusb.h>
#include <bsp/board.h>

#include <m_can.h>

#include <tusb.h>
#include <class/dfu/dfu_rt_device.h>

#define CONF_CPU_FREQUENCY 280000000L



#define PORT_SHIFT 4
#define PIN_MASK 15
#define MAKE_PIN(port, pin) (((port) << PORT_SHIFT) | (pin))

#define PIN_PA03 MAKE_PIN(0, 3)
#define PIN_PA04 MAKE_PIN(0, 4)
#define PIN_PA05 MAKE_PIN(0, 5)
#define PIN_PA06 MAKE_PIN(0, 6)

#define PIN_PB00 MAKE_PIN(1, 0)
#define PIN_PB14 MAKE_PIN(1, 4)

#define PIN_PE01 MAKE_PIN(4, 1)

static const uint32_t GPIO_MODE_OUTPUT_PP = 0x00000001U;
static const uint32_t GPIO_SPEED_FREQ_HIGH = 0x00000003U;
static const uint32_t GPIO_MODE_AF_PP = 0x00000002U;

#if CFG_TUD_DFU_RUNTIME
SC_RAMFUNC void tud_dfu_runtime_reboot_to_dfu_cb(uint16_t ms)
{
	__disable_irq();

	(void)ms;

	LOG("activating ST bootloader, remove CAN bus connection and replug device after firmware download :)\n\n");

	// pretend may flash is empty, flag is held during power-on-reset
	FLASH->ACR |= FLASH_ACR_PROGEMPTY;

	NVIC_SystemReset();
}
#endif // #if CFG_TUD_DFU_RT

static inline void m_can_can_init_board(struct mcan_can* c)
{
	m_can_conf_begin(c->m_can);





	// tx fifo
	c->m_can->TXBC.reg = MCANX_TXBC_TBSA((uint32_t) c->hw_tx_fifo_ram) | MCANX_TXBC_TFQS(MCAN_HW_TX_FIFO_SIZE);
	// tx event fifo
	c->m_can->TXEFC.reg = MCANX_TXEFC_EFSA((uint32_t) c->hw_txe_fifo_ram) | MCANX_TXEFC_EFS(MCAN_HW_TX_FIFO_SIZE);
	// rx fifo0
	c->m_can->RXF0C.reg = MCANX_RXF0C_F0SA((uint32_t) c->hw_rx_fifo_ram) | MCANX_RXF0C_F0S(MCAN_HW_RX_FIFO_SIZE);

	// configure for max message size
	c->m_can->TXESC.reg = MCANX_TXESC_TBDS_DATA64;
	//  | MCANX_RXF0C_F0OM; // FIFO 0 overwrite mode
	c->m_can->RXESC.reg = MCANX_RXESC_RBDS_DATA64 + MCANX_RXESC_F0DS_DATA64;

	m_can_conf_end(c->m_can);

	LOG("TXBC=%08lx TXEFC=%08lx RXF0C=%08lx TXESC=%08lx RXESC=%08lx\n",
		c->m_can->TXBC.reg,
		c->m_can->TXEFC.reg,
		c->m_can->RXF0C.reg,
		c->m_can->TXESC.reg,
		c->m_can->RXESC.reg);
}

// controller and hardware specific setup of i/o pins for CAN
static inline void can_init(void)
{
	/* FDCAN1_RX PD0, FDCAN1_TX PD1 */
	const uint32_t GPIO_MODE_AF_FDCAN = 0x9; // DS13195 - Rev 8 page 71/23

	/* CAN RAM Layout:
	 	- RX fifo (base)
		- TX fifo
		- TX event fifo
	*/
	uint32_t const FDCAN1_RAM = SRAMCAN_BASE;
	uint32_t const FDCAN1_RAM_RX_FIFO = FDCAN1_RAM;
	uint32_t const FDCAN1_RAM_TX_FIFO = FDCAN1_RAM_RX_FIFO + sizeof(struct mcan_rx_fifo_element) * MCAN_HW_RX_FIFO_SIZE;
	uint32_t const FDCAN1_RAM_TXE_FIFO = FDCAN1_RAM_TX_FIFO + sizeof(struct mcan_tx_fifo_element) * MCAN_HW_TX_FIFO_SIZE;


	LOG("FDCAN1 RAM RX=%08lx TX=%08lx TXE=%08lx\n", FDCAN1_RAM_RX_FIFO, FDCAN1_RAM_TX_FIFO, FDCAN1_RAM_TXE_FIFO);
	// enable clock to GPIO block D
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;


	// // high speed output
	// GPIOD->OSPEEDR =
	// 	(GPIOD->OSPEEDR & ~(
	// 	GPIO_OSPEEDR_OSPEED0
	// 	| GPIO_OSPEEDR_OSPEED1))
	// 	| (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED0_Pos)
	// 	| (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED1_Pos);


	// alternate function to CAN
	GPIOD->AFR[0] =
		(GPIOB->AFR[0] & ~(GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1))
		| (GPIO_MODE_AF_FDCAN << GPIO_AFRL_AFSEL0_Pos)
		| (GPIO_MODE_AF_FDCAN << GPIO_AFRL_AFSEL1_Pos);

	// switch mode to alternate function
	GPIOD->MODER =
		(GPIOD->MODER & ~(
			GPIO_MODER_MODE0
			| GPIO_MODER_MODE1))
		| (GPIO_MODE_AF_PP << GPIO_MODER_MODE0_Pos)
		| (GPIO_MODE_AF_PP << GPIO_MODER_MODE1_Pos);


	// set clock for FDCAN to PLL2 Q
	RCC->CDCCIP1R =
		(RCC->CDCCIP1R &
		~(RCC_CDCCIP1R_FDCANSEL))
		| (0x1 << RCC_CDCCIP1R_FDCANSEL_Pos);

	// enable clock
	RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;

	LOG("M_CAN CCU release %lx\n", FDCAN_CCU->CREL);

	//m_can_init_begin(mcan_cans[0].m_can);

	// // setup clock calibration unit (CCU) for bypass
	// FDCAN_CCU->CCFG = FDCANCCU_CCFG_SWR;

	// // wait for reset
	// while (FDCAN_CCU->CCFG & FDCANCCU_CCFG_SWR);

	// // set bypass with divider of 1
	// FDCAN_CCU->CCFG = FDCANCCU_CCFG_BCC;
	// LOG("CCU CCFG=%08lx\n", FDCAN_CCU->CCFG);


	mcan_can_init();

	mcan_cans[0].m_can = (MCanX*)FDCAN1;
	mcan_cans[0].interrupt_id = FDCAN1_IT0_IRQn;
	mcan_cans[0].led_traffic = 0;
	mcan_cans[0].led_status_green = LED_CAN0_STATUS_GREEN;
	mcan_cans[0].led_status_red = LED_CAN0_STATUS_RED;
	mcan_cans[0].hw_tx_fifo_ram = (struct mcan_tx_fifo_element *)(FDCAN1_RAM_TX_FIFO);
	mcan_cans[0].hw_txe_fifo_ram = (struct mcan_txe_fifo_element *)(FDCAN1_RAM_TXE_FIFO);
	mcan_cans[0].hw_rx_fifo_ram = (struct mcan_rx_fifo_element *)(FDCAN1_RAM_RX_FIFO);



	for (size_t j = 0; j < TU_ARRAY_SIZE(mcan_cans); ++j) {
		struct mcan_can *can = &mcan_cans[j];
		can->features = CAN_FEAT_PERM;
	}

	m_can_init_begin(mcan_cans[0].m_can);

	m_can_can_init_board(&mcan_cans[0]);

	NVIC_SetPriority(mcan_cans[0].interrupt_id, SC_ISR_PRIORITY);

	LOG("M_CAN release %u.%u.%u (%lx)\n", mcan_cans[0].m_can->CREL.bit.REL, mcan_cans[0].m_can->CREL.bit.STEP, mcan_cans[0].m_can->CREL.bit.SUBSTEP, mcan_cans[0].m_can->CREL.reg);
}

static inline void counter_1mhz_init(void)
{
	// enable clock
	RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;

	// 1 MHz
	TIM2->PSC = (CONF_CPU_FREQUENCY / 1000000UL) - 1; /* yes, minus one */

	/* reset value of TIM2->ARR is 0xffffffff which is what we want */

	// clear counter to load prescaler (RM0444 Rev 5 p. 689/1390)
	TIM2->EGR |= TIM_EGR_UG;


	// // 1 second reload
	// TIM2->ARR = 1000000UL;

	// TIM2->DIER = TIM_DIER_UIE;

	// NVIC_SetPriority(TIM2_IRQn, SC_ISR_PRIORITY);
	// NVIC_EnableIRQ(TIM2_IRQn);

	// start timer
	TIM2->CR1 =
		TIM_CR1_URS /* only under/overflow, DMA */
		| TIM_CR1_CEN;
}

struct led {
	uint8_t port_pin_mux;
};

#define LED_STATIC_INITIALIZER(name, mux) \
	{ mux }


static const struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PE01), // board led
	LED_STATIC_INITIALIZER("can0_green", PIN_PB00),
	LED_STATIC_INITIALIZER("can0_red", PIN_PB14),

};

static inline void leds_init(void)
{
	// enable clock to GPIO block B, E
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOEEN;

	// disable output
	GPIOB->BSRR =
		GPIO_BSRR_BR0 |
		GPIO_BSRR_BR14;

	GPIOE->BSRR =
		GPIO_BSRR_BR1;

  // switch output type is push-pull on reset

  // switch mode to output
  GPIOB->MODER =
  	(GPIOB->MODER & ~(
		GPIO_MODER_MODE0 |
		GPIO_MODER_MODE14))
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE0_Pos)
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE14_Pos);

  GPIOE->MODER =
  	(GPIOE->MODER & ~(
		GPIO_MODER_MODE1))
	| (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE1_Pos);
}

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

	// LOG("FLASH_SECR.BOOT_LOCK=%u FLASH_ACR.EMPTY=%u FLASH_OPTR_nBOOT_SEL=%u FLASH_OPTR_nBOOT1=%u FLASH_OPTR_nBOOT0=%u\n",
	// 	(FLASH->SECR & FLASH_SECR_BOOT_LOCK) == FLASH_SECR_BOOT_LOCK,
	// 	(FLASH->ACR & FLASH_ACR_PROGEMPTY) == FLASH_ACR_PROGEMPTY,
	// 	(FLASH->OPTR & FLASH_OPTR_nBOOT_SEL) == FLASH_OPTR_nBOOT_SEL,
	// 	(FLASH->OPTR & FLASH_OPTR_nBOOT1) == FLASH_OPTR_nBOOT1,
	// 	(FLASH->OPTR & FLASH_OPTR_nBOOT0) == FLASH_OPTR_nBOOT0);

	leds_init();

	can_init();

	counter_1mhz_init();
}

extern void sc_board_init_end(void)
{
	led_blink(0, 2000);
}

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

SC_RAMFUNC void FDCAN1_IT0_IRQHandler(void)
{
	LOG("FDCAN1_IT0 int\n");

	mcan_can_int(0);
}

SC_RAMFUNC void FDCAN1_IT1_IRQHandler(void)
{
	LOG("FDCAN1_IT1 int\n");

	mcan_can_int(0);
}

SC_RAMFUNC void FDCAN2_IT0_IRQHandler(void)
{
	LOG("FDCAN2_IT0 int\n");

	mcan_can_int(0);
}

SC_RAMFUNC void FDCAN2_IT1_IRQHandler(void)
{
	LOG("FDCAN2_IT1 int\n");

	mcan_can_int(0);
}




// SC_RAMFUNC void TIM2_IRQHandler(void)
// {
// 	LOG("SR=%04x\n", TIM2->SR);

// 	// clear interrupts
// 	TIM2->SR = 0;
// }

#endif // #if STM32H7A3NUCLEO
