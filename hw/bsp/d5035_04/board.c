/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include "drv_usb_hw.h"
#include "drv_usb_dev.h"

#include "bsp/board.h"

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

#define USB_NO_VBUS_PIN

// According to GD32VF103 user manual clock tree:
// Systick clock = AHB clock / 4.
#define TIMER_TICKS         ((SystemCoreClock / 4) / 1000)


void uart_send_buffer(uint8_t const *text, size_t len)
{
	// for (size_t i = 0; i < len; ++i) {
	// 	BOARD_SERCOM->USART.DATA.reg = text[i];
	// 	while((BOARD_SERCOM->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC) == 0);
	// }
}

void uart_send_str(const char* text)
{
	// while (*text) {
	// 	BOARD_SERCOM->USART.DATA.reg = *text++;
	// 	while((BOARD_SERCOM->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC) == 0);
	// }
}

void board_init(void)
{
  /* Disable interrupts during init */
  __disable_irq();

#if CFG_TUSB_OS == OPT_OS_NONE
  SysTick_Config(TIMER_TICKS);
#endif

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
  __enable_irq();
}



//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
	// gpio_set_pin_level(LED_PIN, state);
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
