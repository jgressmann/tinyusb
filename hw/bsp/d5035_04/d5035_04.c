/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include <bsp/board.h>
#include <gd32c10x.h>
#include "startup_gd32c10x.h"
#include <synopsys_common.h>


#if !defined(HWREV)
# error Define "HWREV"
#endif

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

__attribute__((section(".ramfunc"))) void USBFS_IRQHandler(void) { tud_int_handler(0); }

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
#define CONF_CPU_FREQUENCY 120000000
#define CONF_SYSTICK_FREQUENCY (CONF_CPU_FREQUENCY / 8)
#define CONF_APB2_FREQUENCY 120000000
#define USART_BAUDRATE 115200
#define BOARD_NAME "D5035-04"

/* The board has a 25Hz external crystal
 * Configure for 120 MHz core clock, 48 MHz USB clock.
 */
void board_init_xtal(void)
{
	/* CFG0:
	 * - AHB div by 1 => 120 MHz
	 * - APB1 div by 2 => 60 MHz (which is max allowed for bus)
	 * - APB2 div by 1 => 120 MHz (which is max allowed for bus)
	 * - USB prediv to 2,5 (120->48)
	 * - PLL multiplcation factor to 24 (5->120)
	 * - PLL from XTAL or IRC48M
	 */
	RCU_CFG0 =
		(RCU_CFG0 & ~(RCU_CFG0_AHBPSC | RCU_CFG0_APB1PSC | RCU_CFG0_APB2PSC | RCU_CFG0_USBFSPSC_2 | RCU_CFG0_USBFSPSC | RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0_LSB)) |
		(RCU_AHB_CKSYS_DIV1 | RCU_APB1_CKAHB_DIV2 | RCU_APB2_CKAHB_DIV1 | RCU_CKUSB_CKPLL_DIV2_5 | RCU_PLL_MUL24 | RCU_PLLSRC_HXTAL_IRC48M);


	/* CFG1:
	 * - PREDV0 to 5 to get 5 MHz out of xtal
	 * - select XTAL / IRC48M as PLL input
	 */
	RCU_CFG1 =
		(RCU_CFG1 & ~(RCU_CFG1_PREDV0 | RCU_CFG1_PREDV0SEL | RCU_CFG1_PLLPRESEL)) |
		(RCU_PREDV0_DIV5 | RCU_PREDV0SRC_HXTAL_IRC48M | RCU_PLLPRESRC_HXTAL);


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

	SystemCoreClock = CONF_CPU_FREQUENCY;
}

void board_init_uart(void)
{
	// /* reset */
	// RCU_APB2RST |= RCU_USART0RST;
	// RCU_APB2RST &= ~RCU_USART0RST;

	/* configure clocks  */
	RCU_APB2EN |=
		RCU_APB2EN_PAEN |
		RCU_APB2EN_AFEN |
		RCU_APB2EN_USART0EN;

	/* configure pin PA9 (USART0_TX) */
	GPIO_CTL1(GPIOA) = (GPIO_CTL1(GPIOA) & ~GPIO_MODE_MASK(1)) | GPIO_MODE_SET(1, GPIO_MODE_AF_PP | GPIO_OSPEED_10MHZ);

	/* baud rate */
	USART_BAUD(USART0) = ((UINT32_C(CONF_APB2_FREQUENCY) + UINT32_C(USART_BAUDRATE) / 2)) / (UINT32_C(USART_BAUDRATE));

	USART_CTL0(USART0) =
		USART_TRANSMIT_ENABLE |
		USART_CTL0_UEN;

	while (!(USART_CTL0(USART0) & USART_CTL0_UEN));
}

void uart_send_buffer(uint8_t const *text, size_t len)
{
	for (size_t i = 0; i < len; ++i) {
		USART_DATA(USART0) = text[i];
		while(!(USART_STAT0(USART0) & USART_STAT0_TBE));
	}
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
	/* enable
	 * - PB pin group for debug LED
	 */
	RCU_APB2EN |=
		RCU_APB2EN_PBEN;

	/* debug LED on PB14, set push-pull output */
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(6)) | GPIO_MODE_SET(6, GPIO_MODE_OUT_PP | GPIO_OSPEED_2MHZ);
	// board_led_off();

	board_init_xtal();

	board_init_uart();

#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " UART initialized\n");
#endif

#if CFG_TUSB_OS == OPT_OS_NONE
	SysTick_Config(CONF_SYSTICK_FREQUENCY / 1000);
#endif

#if CFG_TUSB_OS == OPT_OS_FREERTOS
	// If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
	NVIC_SetPriority(USBFS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

	/* select PLL for USB clock */
	RCU_ADDCTL =
		(RCU_ADDCTL & ~(RCU_ADDCTL_CK48MSEL)) |
		(RCU_CK48MSRC_CKPLL);

	/* enable USB clock */
	RCU_AHBEN |= RCU_AHBEN_USBFSEN;

	// /* reset USB */
	RCU_AHBRST |= RCU_AHBRST_USBFSRST;
	RCU_AHBRST &= ~RCU_AHBRST_USBFSRST;

	// delay 3us
	for (uint32_t millis = board_millis(); millis == board_millis(); ) {

	}

	/* Retrieve USB OTG core registers */
	USB_OTG_GlobalTypeDef* otg_core_regs = (USB_OTG_GlobalTypeDef*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_GLOBAL_BASE);
	otg_core_regs->GOTGCTL =
		(otg_core_regs->GOTGCTL & ~(USB_OTG_GOTGCTL_DHNPEN | USB_OTG_GOTGCTL_HSHNPEN | USB_OTG_GOTGCTL_HNPRQ)) |
		(USB_OTG_GOTGCTL_DHNPEN);

	otg_core_regs->GUSBCFG =
		(otg_core_regs->GUSBCFG & ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_SRPCAP | USB_OTG_GUSBCFG_HNPCAP)) |
		(USB_OTG_GUSBCFG_FDMOD);

#define USB_OTG_GCCFG_VBUSIG (UINT32_C(1) << 21)
	otg_core_regs->GCCFG |= USB_OTG_GCCFG_VBUSIG | USB_OTG_GCCFG_PWRDWN | USB_OTG_GCCFG_VBUSBSEN;
}



//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
	GPIO_BOP(GPIOB) = BIT(14 + (!state) * 16);
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
