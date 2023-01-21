/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "../board.h"
#include "stm32f3xx_hal.h"

#define CONF_CPU_FREQUENCY 48000000
#define USART_BAURATE      115200
#define BOARD_USART        USART1

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

// USB defaults to using interrupts 19, 20 and 42, however, this BSP sets the
// SYSCFG_CFGR1.USB_IT_RMP bit remapping interrupts to 74, 75 and 76.

// FIXME: Do all three need to be handled, or just the LP one?
// USB high-priority interrupt (Channel 74): Triggered only by a correct
// transfer event for isochronous and double-buffer bulk transfer to reach
// the highest possible transfer rate.
void USB_HP_IRQHandler(void)
{
  tud_int_handler(0);
}

// USB low-priority interrupt (Channel 75): Triggered by all USB events
// (Correct transfer, USB reset, etc.). The firmware has to check the
// interrupt source before serving the interrupt.
void USB_LP_IRQHandler(void)
{
  tud_int_handler(0);
}

// USB wakeup interrupt (Channel 76): Triggered by the wakeup event from the USB
// Suspend mode.
void USBWakeUp_RMP_IRQHandler(void)
{
  tud_int_handler(0);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

#define LED_PORT              GPIOE
#define LED_PIN               GPIO_PIN_9
#define LED_STATE_ON          1

#define BUTTON_PORT           GPIOA
#define BUTTON_PIN            GPIO_PIN_0
#define BUTTON_STATE_ACTIVE   1


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = RCC_PLL_MUL12 (12)
  *            Flash Latency(WS)              = 1
  *
  * Chip boots off of 8 MHz interal RC oscillator (HSI).
  * Configure for 48 MHz system clock.
  * HSI gets deviced by 2 before feed to PLL.
  *
  * @param  None
  * @retval None
  */
static void clock_init(void)
{
  // set flash for 1 wait state, DocID022558 Rev 8, p.78
  FLASH->ACR = (FLASH->ACR  & ~(FLASH_ACR_LATENCY)) | FLASH_ACR_LATENCY_1;

  // configure PLL input to HSI, no USB prediv
  RCC->CFGR = RCC_CFGR_PLLMUL12 | RCC_CFGR_USBPRE;

  // enable PLL
  RCC->CR |= RCC_CR_PLLON;

  // wait for PLL ready
  while ((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);

  // switch main clock to PLL
  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;

  // wait for clock switch
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

  SystemCoreClock = CONF_CPU_FREQUENCY;
}


/* configure UART1 on PC4 (TX) / PC5 (RX), available through ACM on ST-Link USB connection, 8N1 */
void uart_init(void)
{
  /* configure pins for UART */

  // enable clock to GPIO block C
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // pull up on RX pin
  GPIOC->PUPDR = (GPIOC->PUPDR & ~(GPIO_PUPDR_PUPDR5)) | (UINT32_C(0x1) << GPIO_PUPDR_PUPDR5_Pos);
  // high speed output on TX pin
  GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~(GPIO_OSPEEDER_OSPEEDR4)) | (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDER_OSPEEDR4_Pos);
  // alternate function to UART1
  GPIOC->AFR[0] = (GPIOC->AFR[0] & ~(GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5)) | (GPIO_AF7_USART1 << GPIO_AFRL_AFRL4_Pos) | (GPIO_AF7_USART1 << GPIO_AFRL_AFRL5_Pos);
  // switch mode to alternate function
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5)) | (GPIO_MODE_AF_PP << GPIO_MODER_MODER4_Pos) | (GPIO_MODE_AF_PP << GPIO_MODER_MODER5_Pos);

  /* configure UART */

  // enable clock to UART1
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  // configure baud rate
  BOARD_USART->BRR = CONF_CPU_FREQUENCY / USART_BAURATE;

  // enable uart (TX)
  BOARD_USART->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static inline void tx_char(uint8_t ch)
{
    BOARD_USART->TDR = ch;
		while((BOARD_USART->ISR & USART_ISR_TXE) != USART_ISR_TXE);
}

static inline void uart_send_buffer(uint8_t const *text, size_t len)
{
	for (size_t i = 0; i < len; ++i) {
    tx_char(text[i]);
	}
}

static inline void uart_send_str(const char* text)
{
	while (*text) {
    tx_char(*text++);
	}
}

void board_init(void)
{
  clock_init();
  uart_init();

  #if CFG_TUSB_OS  == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
  #endif

#if CFG_TUSB_OS == OPT_OS_FREERTOS
	// If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
	NVIC_SetPriority(USB_HP_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(USB_LP_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(USBWakeUp_RMP_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

  // Remap the USB interrupts
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_REMAPINTERRUPT_USB_ENABLE();

  // LED
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  // Button
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

  /* Configure USB DM and DP pins */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable USB clock
  __HAL_RCC_USB_CLK_ENABLE();


}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, state ? LED_STATE_ON : (1-LED_STATE_ON));
}

uint32_t board_button_read(void)
{
  return BUTTON_STATE_ACTIVE == HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
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
void SysTick_Handler (void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif

void HardFault_Handler (void)
{
  asm("bkpt");
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}
