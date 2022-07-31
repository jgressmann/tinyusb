/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include "drv_usb_hw.h"
#include "drv_usb_dev.h"

#include <bsp/board.h>
#include <gd32c10x.h>

#if !defined(HWREV)
# error Define "HWREV"
#endif

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

void USBFS_IRQHandler(void) { tud_int_handler(0); }

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
//#define CONF_CPU_FREQUENCY 120000000
#define CONF_CPU_FREQUENCY 120000000
// #define CONF_CPU_FREQUENCY 8000000
#define CONF_APB2_FREQUENCY 120000000
#define USART_BAUDRATE 115200

/* The board has a 16MHz external crystal
 * Configure for 120 MHz core clock, 48 MHz USB clock.
 */
void board_init_xtal(void)
{
	/* CFG1:
	 * - PREDV0 to 4 to get 4 MHz out of 16 MHz xtal
	 * - select XTAL / IRC48M as PLL input
	 */
	RCU_CFG1 =
		(RCU_CFG1 & ~(RCU_CFG1_PREDV0 | RCU_CFG1_PREDV0SEL | RCU_CFG1_PLLPRESEL)) |
		(RCU_PREDV0_DIV2 | RCU_PREDV0SRC_HXTAL_IRC48M | RCU_PLLPRESRC_HXTAL);

	/* CFG0:
	 * - AHB div by 1 => 120 MHz
	 * - APB1 div by 2 => 60 MHz (which is max allowd for bus)
	 * - APB2 div by 1 => 120 MHz (which is max allowd for bus)
	 * - USB prediv to 2,5 (120->48)
	 * - PLL multiplcation factor to 30 (4->120)
	 * - PLL from XTAL or IRC48M
	 */
	RCU_CFG0 =
		(RCU_CFG0 & ~(RCU_CFG0_AHBPSC | RCU_CFG0_APB1PSC | RCU_CFG0_APB2PSC | RCU_CFG0_USBFSPSC_2 | RCU_CFG0_USBFSPSC | RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0_LSB)) |
		(RCU_AHB_CKSYS_DIV1 | RCU_APB1_CKAHB_DIV2 | RCU_APB2_CKAHB_DIV1 | RCU_CKUSB_CKPLL_DIV2_5 | RCU_PLL_MUL30 | RCU_PLLSRC_HXTAL_IRC48M | RCU_CFG0_PREDV0_LSB );




	/* select PLL for USB clock */
	RCU_ADDCTL =
		(RCU_ADDCTL & ~(RCU_ADDCTL_CK48MSEL)) |
		(RCU_CK48MSRC_CKPLL);

	/* enable crystal, PLL */
	RCU_CTL |= RCU_CTL_HXTALEN | RCU_CTL_PLLEN;

	/* wait for crystal, PLL */
	while ((RCU_CTL_HXTALSTB | RCU_CTL_PLLSTB) != (RCU_CTL & (RCU_CTL_HXTALSTB | RCU_CTL_PLLSTB)));

	/* set wait state for flash to 3 (for 120 MHz), section 2.3.2. Read operations , GD32C10x User Manual */
	FMC_WS = (FMC_WS & ~(FMC_WS_WSCNT)) | (WS_WSCNT(3));

	/* switch to PLL */
	RCU_CFG0 =
		(RCU_CFG0 & ~(RCU_CFG0_SCS)) |
		(RCU_CKSYSSRC_PLL);


	/* wait for clock switch */
	while (RCU_SCSS_PLL != (RCU_CFG0 & RCU_CFG0_SCSS));
}

void board_init_uart(void)
{
	/* reset */
	RCU_APB2RST |= RCU_USART0RST;
	RCU_APB2RST &= ~RCU_USART0RST;

	/* configure clocks  */
	RCU_APB2EN |=
		RCU_APB2EN_PAEN |
		RCU_APB2EN_AFEN |
		RCU_APB2EN_USART0EN;

	/* configure pin PA9 (USART0_TX) */
	GPIO_CTL1(GPIOA) = (GPIO_CTL1(GPIOA) & ~GPIO_MODE_MASK(1)) | GPIO_MODE_SET(1, GPIO_MODE_AF_PP | GPIO_OSPEED_50MHZ);

	 /* disable USART */
	USART_CTL0(USART0) = 0;

	/* wait for disable */
	while (USART_CTL0(USART0) & USART_CTL0_UEN);

	/* default values */
	USART_CTL1(USART0) = 0;

	/* default values */
	USART_CTL2(USART0) = 0;

	/* default values */
	USART_CTL3(USART0) = 0;

	/* baud rate */
	USART_BAUD(USART0) = ((UINT32_C(CONF_APB2_FREQUENCY) + UINT32_C(USART_BAUDRATE) / 2)) / (UINT32_C(USART_BAUDRATE));

	USART_CTL0(USART0) =
		USART_TRANSMIT_ENABLE |
		USART_CTL0_UEN;

	while (!(USART_CTL0(USART0) & USART_CTL0_UEN));
}

void uart_send_buffer(uint8_t const *text, size_t len)
{
	// for (size_t i = 0; i < len; ++i) {
	// 	BOARD_SERCOM->USART.DATA.reg = text[i];
	// 	while((BOARD_SERCOM->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC) == 0);
	// }
}

void uart_send_str(const char* text)
{
	while (*text) {
		USART_DATA(USART0) = *text++;
		while(!(USART_STAT0(USART0) & USART_STAT0_TBE));
	}
}

void board_init(void)
{
	/* enable PB pin group for debug LED */
	RCU_APB2EN |= RCU_APB2EN_PBEN;

	/* debug LED on PB14, set push-pull output */
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(6)) | GPIO_MODE_SET(6, GPIO_MODE_OUT_PP | GPIO_OSPEED_2MHZ);

	board_init_xtal();

	// GPIO_BOP(GPIOB) = GPIO_BOP_BOP14;

	board_init_uart();

	// GPIO_BOP(GPIOB) = GPIO_BOP_CR14;


	GPIO_BOP(GPIOB) = GPIO_BOP_BOP14;

	// unsigned i = 0;

	while (1) {
		// while(!(USART_STAT0(USART0) & USART_STAT0_TBE));
		// USART_DATA(USART0) = 0xA5;
		// while(!(USART_STAT0(USART0) & USART_STAT0_TBE));
		// while(!(USART_STAT0(USART0) & USART_STAT0_TC));
		uart_send_str("Hello, GD32 world!\n");

		// if (i++ & 1) {
		// 	GPIO_BOP(GPIOB) = GPIO_BOP_BOP14;
		// } else {
		// 	GPIO_BOP(GPIOB) = GPIO_BOP_CR14;
		// }
	}

	// GPIO_BOP(GPIOB) = GPIO_BOP_BOP14;

#if CFG_TUSB_OS == OPT_OS_NONE
	/* GD32C10x User Manual, p. 120:
	 *
	 * The SysTick calibration value is 15000 and SysTick clock frequency is fixed to HCLK*0.125.
	 * 1ms SysTick interrupt will be generated when HCLK is configured to 120MHz.
	 */
	SysTick_Config(120000);
#endif




	// /* configure USB pins */
	// GPIO_CTL1)GPIOA) = )GPIO_CTL1)GPIOA) & ~(GPIO_MODE_MASK(2) | GPIO_MODE_MASK(3))) | GPIO_MODE_SET(6, GPIO_MODE_AF_PP | GPIO_OSPEED_50MHZ);

	/* reset USB */
	RCU_AHBRST |= RCU_AHBRST_USBFSRST;

	/* enable USB clock */
	RCU_AHBEN |= RCU_AHBEN_USBFSEN;





//   /* Disable interrupts during init */
//   __disable_irq();



//   rcu_periph_clock_enable(RCU_GPIOA);
//   rcu_periph_clock_enable(RCU_GPIOB);
//   rcu_periph_clock_enable(RCU_GPIOC);
//   rcu_periph_clock_enable(RCU_GPIOD);
//   rcu_periph_clock_enable(RCU_AF);


// #ifdef LED_PIN
//   gd_led_init(LED_PIN);
// #endif

// #if defined(UART_DEV)
//   gd_com_init(UART_DEV);
// #endif

//   /* USB D+ and D- pins don't need to be configured. */
//   /* Configure VBUS Pin */
// #ifndef USB_NO_VBUS_PIN
//   gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
// #endif

//   /* This for ID line debug */
//   // gpio_init(GPIOA, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

//   /* Enable USB OTG clock */
//   usb_rcu_config();

//   /* Reset USB OTG peripheral */
//   rcu_periph_reset_enable(RCU_USBFSRST);
//   rcu_periph_reset_disable(RCU_USBFSRST);

//   /* Configure USBFS IRQ */
//   ECLIC_Register_IRQ(USBFS_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
//                      ECLIC_POSTIVE_EDGE_TRIGGER, 3, 0, NULL);

//   /* Retrieve otg core registers */
//   usb_gr* otg_core_regs = (usb_gr*)(USBFS_REG_BASE + USB_REG_OFFSET_CORE);

// #ifdef USB_NO_VBUS_PIN
//   /* Disable VBUS sense*/
//   otg_core_regs->GCCFG |= GCCFG_VBUSIG | GCCFG_PWRON | GCCFG_VBUSBCEN;
// #else
//   /* Enable VBUS sense via pin PA9 */
//   otg_core_regs->GCCFG |= GCCFG_VBUSIG | GCCFG_PWRON | GCCFG_VBUSBCEN;
//   otg_core_regs->GCCFG &= ~GCCFG_VBUSIG;
// #endif

  /* Enable interrupts globaly */
//   __enable_irq();
}



//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
	GPIO_BOP(GPIOB) = BIT(14 + state * 16);
}

uint32_t board_button_read(void)
{
	// this board has no button
	return 0;
}

int board_uart_read(uint8_t* buf, int len)
{
	(void) buf; (void) len;
	return 0;
}

int board_uart_write(void const * buf, int len)
{
	if (len < 0) {
		uart_send_str(buf);
	} else {
		uart_send_buffer(buf, len);
	}
	return len;
}

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void)
{
	system_ticks++;
}

uint32_t board_millis(void)
{
	return system_ticks;
}
#endif

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}
