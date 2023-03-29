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

#include <m_can.h>

#include <tusb.h>
#include <class/dfu/dfu_rt_device.h>

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

static const uint32_t GPIO_MODE_OUTPUT_PP = 0x00000001U;
static const uint32_t GPIO_SPEED_FREQ_HIGH = 0x00000003U;
static const uint32_t GPIO_MODE_AF_PP = 0x00000002U;

#if CFG_TUD_DFU_RUNTIME

__attribute__((noreturn, section(SC_RAMFUNC_SECTION_NAME))) static inline void start_app_jump(uint32_t addr)
{
	// Load the vector table address of the user application into SCB->VTOR register. Make sure the address meets the alignment requirements.

	SCB->VTOR = addr;

	__DSB();
	__ISB();


	// A few device families, like the NXP 4300 series, will also have a "shadow pointer" to the VTOR, which also needs to be updated with the new address. Review the device datasheet to see if one exists.
	// The final part is to set the MSP to the value found in the user application vector table and then load the PC with the reset vector value of the user application. This can't be done in C, as it is always possible, that the compiler uses the current SP. But that would be gone after setting the new MSP. So, a call to a small assembler function is done.

	uint32_t* base = (uint32_t*)(uintptr_t)addr;
	// LOG("stack @ %p reset @ %p\n", (void*)base[0], (void*)base[1]);

	__asm__ (
		"MSR MSP, %0\n"
		"BX  %1\n"
		: : "r" (base[0]), "r" (base[1])
	);

	__unreachable();
}


SC_RAMFUNC void tud_dfu_runtime_reboot_to_dfu_cb(uint16_t ms)
{
	__disable_irq();

	(void)ms;

	LOG("activating ST bootloader, remove CAN bus connection and replug device after firmware download :)\n\n");

	FLASH->ACR |= FLASH_ACR_PROGEMPTY;

	NVIC_SystemReset();

	// // unlock flash
	// FLASH->KEYR = 0x45670123;
	// FLASH->KEYR = 0xCDEF89AB;
	// // unlock option bytes
	// FLASH->OPTKEYR = 0x08192A3B;
	// FLASH->OPTKEYR = 0x4C5D6E7F;






// (RCC_)
	RCC->AHBRSTR = 0xffffffff;
	RCC->APBRSTR1 = 0xffffffff;
	RCC->APBRSTR2 = 0xffffffff;

	// enable power to SYSCFG
	RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;

	LOG("CFGR1=%08x\n", SYSCFG->CFGR1);

	SYSCFG->CFGR1 =
		(SYSCFG->CFGR1 & ~(SYSCFG_CFGR1_MEM_MODE))
		| SYSCFG_CFGR1_MEM_MODE_0;


	LOG("CFGR1=%08x\n", SYSCFG->CFGR1);

	typedef void (*pFunction)(void);
	pFunction Jump_To_Application;
	uint32_t JumpAddress;
	static uint32_t APP_START_ADDRESS = 0;


   /* Get the application stack pointer (First entry in the application vector table) */
   JumpAddress = (uint32_t) *((__IO uint32_t*)APP_START_ADDRESS);

   /* Get the application entry point (Second entry in the application vector table) */
   Jump_To_Application = (pFunction) *(__IO uint32_t*) (APP_START_ADDRESS + 4);

   /* Reconfigure vector table offset register to match the application location */
   SCB->VTOR = JumpAddress;

   /* Set the application stack pointer */
   __set_MSP(JumpAddress);

   /* Start the application */
   Jump_To_Application();



	// start_app_jump(0x1FFF0000);
	// start_app_jump(0);

	// arm-none-eabi-gcc 4.9.0 does not correctly inline this
    // MSP function, so we write it out explicitly here.
    //__set_MSP(*((uint32_t*) 0x00000000));
    // __ASM volatile ("movs r3, #0\nldr r3, [r3, #0]\nMSR msp, r3\n" : : : "r3", "sp");
	// // __set_MSP(*((uint32_t*) 0x00000000));

    // ((void (*)(void)) *((uint32_t*) 0x00000004))();
	// #define PROGRAM_VECTOR_TABLE ((uint32_t *) 0)


    // static void (*go_to_app)(void) = 0;
    // go_to_app = (void (*)(void))(PROGRAM_VECTOR_TABLE[1]);
    // SCB->VTOR = (uint32_t)PROGRAM_VECTOR_TABLE;
    // __set_MSP(PROGRAM_VECTOR_TABLE[0]);
    // __set_PSP(PROGRAM_VECTOR_TABLE[0]);
    // go_to_app();


    while (1);
}
#endif // #if CFG_TUD_DFU_RT


// controller and hardware specific setup of i/o pins for CAN
static inline void can_init(void)
{
	/* FDCAN2_RX PB0, FDCAN2_TX PB1, FDCAN1_RX PB8, FDCAN1_TX PB9 */
	const uint32_t GPIO_MODE_AF3_FDCAN = 0x3; // DS13560 Rev 4, p. 58
	uint32_t FDCAN1_RAM = SRAMCAN_BASE;
	uint32_t FDCAN2_RAM = FDCAN1_RAM + 0x400;
	// const unsigned FLSSA_BASE = 0x0000;
	// const unsigned FLESA_BASE = 0x0070;
	const unsigned F0SA_BASE = 0x00B0;
	// const unsigned F1SA_BASE = 0x188;
	const unsigned EFSA_BASE = 0x0260; // 0x4000B660
	const unsigned TBSA_BASE = 0x0278; // 0x4000B678

	// enable clock to GPIO block B
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

	// high speed output
	GPIOB->OSPEEDR =
		(GPIOB->OSPEEDR & ~(
		GPIO_OSPEEDR_OSPEED0
		| GPIO_OSPEEDR_OSPEED1
		| GPIO_OSPEEDR_OSPEED8
		| GPIO_OSPEEDR_OSPEED9))
		| (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED0_Pos)
		| (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED1_Pos)
		| (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED8_Pos)
		| (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED9_Pos);


	// alternate function to CAN
	GPIOB->AFR[0] =
		(GPIOB->AFR[0] & ~(GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1))
		| (GPIO_MODE_AF3_FDCAN << GPIO_AFRL_AFSEL0_Pos)
		| (GPIO_MODE_AF3_FDCAN << GPIO_AFRL_AFSEL1_Pos);
	GPIOB->AFR[1] =
		(GPIOB->AFR[1] & ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9))
		| (GPIO_MODE_AF3_FDCAN << GPIO_AFRH_AFSEL8_Pos)
		| (GPIO_MODE_AF3_FDCAN << GPIO_AFRH_AFSEL9_Pos);

	// switch mode to alternate function
	GPIOB->MODER =
		(GPIOB->MODER & ~(
			GPIO_MODER_MODE0
			| GPIO_MODER_MODE1
			| GPIO_MODER_MODE8
			| GPIO_MODER_MODE9))
		| (GPIO_MODE_AF_PP << GPIO_MODER_MODE0_Pos)
		| (GPIO_MODE_AF_PP << GPIO_MODER_MODE1_Pos)
		| (GPIO_MODE_AF_PP << GPIO_MODER_MODE8_Pos)
		| (GPIO_MODE_AF_PP << GPIO_MODER_MODE9_Pos);


	// set clock to PLL Q
	RCC->CCIPR2 =
		(RCC->CCIPR2 & ~(RCC_CCIPR2_FDCANSEL_Msk))
		| (UINT32_C(1) << RCC_CCIPR2_FDCANSEL_Pos);

	// enable clock
	RCC->APBENR1 |= RCC_APBENR1_FDCANEN;

	mcan_can_init();

	mcan_cans[0].m_can = (MCanX*)FDCAN1;
	mcan_cans[1].m_can = (MCanX*)FDCAN2;
	mcan_cans[0].interrupt_id = TIM16_FDCAN_IT0_IRQn;
	mcan_cans[1].interrupt_id = TIM17_FDCAN_IT1_IRQn;
	mcan_cans[0].led_traffic = CAN0_TRAFFIC_LED;
	mcan_cans[1].led_traffic = CAN1_TRAFFIC_LED;
	mcan_cans[0].led_status_green = LED_CAN0_STATUS_GREEN;
	mcan_cans[1].led_status_green = LED_CAN1_STATUS_GREEN;
	mcan_cans[0].led_status_red = LED_CAN0_STATUS_RED;
	mcan_cans[1].led_status_red = LED_CAN1_STATUS_RED;
	mcan_cans[0].hw_tx_fifo_ram = (struct mcan_tx_fifo_element *)(FDCAN1_RAM + TBSA_BASE);
	mcan_cans[0].hw_txe_fifo_ram = (struct mcan_txe_fifo_element *)(FDCAN1_RAM + EFSA_BASE);
	mcan_cans[0].hw_rx_fifo_ram = (struct mcan_rx_fifo_element *)(FDCAN1_RAM + F0SA_BASE);
	mcan_cans[1].hw_tx_fifo_ram = (struct mcan_tx_fifo_element *)(FDCAN2_RAM + TBSA_BASE);
	mcan_cans[1].hw_txe_fifo_ram = (struct mcan_txe_fifo_element *)(FDCAN2_RAM + EFSA_BASE);
	mcan_cans[1].hw_rx_fifo_ram = (struct mcan_rx_fifo_element *)(FDCAN2_RAM + F0SA_BASE);

	SC_ASSERT(0x4000B678 == (uintptr_t)mcan_cans[0].hw_tx_fifo_ram);
	SC_ASSERT(0x4000B660 == (uintptr_t)mcan_cans[0].hw_txe_fifo_ram);



	for (size_t j = 0; j < TU_ARRAY_SIZE(mcan_cans); ++j) {
		struct mcan_can *can = &mcan_cans[j];
		can->features = CAN_FEAT_PERM;
	}

	m_can_init_begin(mcan_cans[0].m_can);
	m_can_init_begin(mcan_cans[1].m_can);

	/* interrupts, map
	 * FDCAN1 to TIM16_FDCAN_IT0_Handler,
	 * FDCAN2 to TIM17_FDCAN_IT1_Handler
	 */


	// map interrupts to line 0, enable line
	mcan_cans[0].m_can->ILS = 0;
	mcan_cans[0].m_can->ILE.reg = MCANX_ILE_EINT0;

	// map interrupts to line 1, enable line
	mcan_cans[1].m_can->ILS = UINT32_C(127);
	mcan_cans[1].m_can->ILE.reg = MCANX_ILE_EINT1;

	// mcan_cans[0].m_can->MRCFG.reg = MCANX_MRCFG_QOS_HIGH;
	// mcan_cans[1].m_can->MRCFG.reg = MCANX_MRCFG_QOS_HIGH;

	NVIC_SetPriority(mcan_cans[0].interrupt_id, SC_ISR_PRIORITY);
	NVIC_SetPriority(mcan_cans[1].interrupt_id, SC_ISR_PRIORITY);

	LOG("M_CAN release %u.%u.%u (%lx)\n", mcan_cans[0].m_can->CREL.bit.REL, mcan_cans[0].m_can->CREL.bit.STEP, mcan_cans[0].m_can->CREL.bit.SUBSTEP, mcan_cans[0].m_can->CREL.reg);
}

static inline void counter_1mhz_init(void)
{
	// enable clock
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN;

	// 1 MHz
	TIM2->PSC = (CONF_CPU_FREQUENCY / 1000000UL) - 1; /* yes, minus one */

	/* reset value of TIM2->ARR is 0xffffffff which is what we want */

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

	LOG("FLASH_SECR.BOOT_LOCK=%u FLASH_ACR.EMPTY=%u FLASH_OPTR_nBOOT_SEL=%u FLASH_OPTR_nBOOT1=%u FLASH_OPTR_nBOOT0=%u\n",
		(FLASH->SECR & FLASH_SECR_BOOT_LOCK) == FLASH_SECR_BOOT_LOCK,
		(FLASH->ACR & FLASH_ACR_PROGEMPTY) == FLASH_ACR_PROGEMPTY,
		(FLASH->OPTR & FLASH_OPTR_nBOOT_SEL) == FLASH_OPTR_nBOOT_SEL,
		(FLASH->OPTR & FLASH_OPTR_nBOOT1) == FLASH_OPTR_nBOOT1,
		(FLASH->OPTR & FLASH_OPTR_nBOOT0) == FLASH_OPTR_nBOOT0);

	leds_init();

	can_init();

	counter_1mhz_init();
}

extern void sc_board_init_end(void)
{
	led_blink(0, 2000);
	led_set(POWER_LED, 1);
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

SC_RAMFUNC void TIM16_FDCAN_IT0_IRQHandler(void)
{
	// LOG("CAN0 int\n");

	mcan_can_int(0);
}


SC_RAMFUNC void TIM17_FDCAN_IT1_IRQHandler(void)
{
	LOG("CAN1 int\n");

	mcan_can_int(1);
}

// SC_RAMFUNC void TIM2_IRQHandler(void)
// {
// 	LOG("SR=%04x\n", TIM2->SR);

// 	// clear interrupts
// 	TIM2->SR = 0;
// }

#endif // #if D5035_05
