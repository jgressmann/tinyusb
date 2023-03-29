/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#include "../board.h"
#include "stm32g0b1xx.h"

#define CONF_CPU_FREQUENCY 64000000
#define USART_BAURATE      115200
#define BOARD_USART        USART1

static const uint32_t GPIO_SPEED_FREQ_HIGH = 0x00000003U;
static const uint32_t GPIO_MODE_AF_PP = 0x00000002U;

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
__attribute__((section(".RamFunc"))) void USB_UCPD1_2_IRQHandler(void)
{
  tud_int_handler(0);
}

/* Configure for 64 Mhz system clock from HSE 16 MHz oscillator */
static void clock_init(void)
{

  // The system starts from the 16 MHz HSI clock.
  const uint32_t RCC_CFGR_SW_PLLSRC = UINT32_C(2) << RCC_CFGR_SW_Pos;
  const uint32_t RCC_CFGR_SWS_PLLSRC = UINT32_C(2) << RCC_CFGR_SWS_Pos;

  // enable HSE, HSI48 (USB)
  RCC->CR |= RCC_CR_HSEON | RCC_CR_HSI48ON;

  // wait for clock ready
  while ((RCC->CR & (RCC_CR_HSERDY | RCC_CR_HSI48RDY)) != (RCC_CR_HSERDY | RCC_CR_HSI48RDY));

  // configure PLL input to HSE
  RCC->PLLCFGR =
    RCC_PLLCFGR_PLLSRC_HSE
    // RCC_PLLCFGR_PLLSRC_HSI
    | (UINT32_C(0) << RCC_PLLCFGR_PLLM_Pos) /* no division of input clock (16 MHz) */
    | (UINT32_C(16) << RCC_PLLCFGR_PLLN_Pos) /* 256 MHz */
    | (UINT32_C(3) << RCC_PLLCFGR_PLLR_Pos)  /* R = 4, SYSCLK to 64 MHz = Fmax */
    | RCC_PLLCFGR_PLLREN
    //| (UINT32_C(1) << RCC_PLLCFGR_PLLQ_Pos)  /* Q = 2, 128 MHz */
    | (UINT32_C(3) << RCC_PLLCFGR_PLLQ_Pos)  /* Q = 4, 64 MHz */
    | RCC_PLLCFGR_PLLQEN;

  // enable PLL
  RCC->CR |= RCC_CR_PLLON;

  // wait for PLL ready
  while ((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);

  // set flash for 2 wait states, RM0444 Rev 5, p.72
  FLASH->ACR = (FLASH->ACR & ~(FLASH_ACR_LATENCY)) | (UINT32_C(2) << FLASH_ACR_LATENCY_Pos);

  // switch main clock to PLL (reset values of HPRE (HCLK) and PPRE (PCLK) setup division by 1 of SYSCLK)
  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_PLLSRC;

  // wait for clock switch
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLLSRC);

  SystemCoreClock = CONF_CPU_FREQUENCY;

  // configure HSI48 for USB
  RCC->CCIPR2 &= ~(RCC_CCIPR2_USBSEL);
}


/* configure UART1 on PA9 (TX) / PA10 (RX), 8N1 */
void uart_init(void)
{
  /* configure pins for UART */
  const uint32_t GPIO_AF1_USART1 = 1;

  // enable clock to GPIO block A
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  // pull up on RX pin
  GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPD10)) | (UINT32_C(0x1) << GPIO_PUPDR_PUPD10_Pos);
  // high speed output on TX pin
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED9)) | (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED9_Pos);
  // alternate function to UART1
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10)) | (GPIO_AF1_USART1 << GPIO_AFRH_AFSEL9_Pos) | (GPIO_AF1_USART1 << GPIO_AFRH_AFSEL10_Pos);
  // switch mode to alternate function
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10)) | (GPIO_MODE_AF_PP << GPIO_MODER_MODE9_Pos) | (GPIO_MODE_AF_PP << GPIO_MODER_MODE10_Pos);

  /* configure UART */

  // enable clock to UART1
  RCC->APBENR2 |= RCC_APBENR2_USART1EN;

  // configure baud rate
  BOARD_USART->BRR = CONF_CPU_FREQUENCY / USART_BAURATE;

  // enable uart (TX)
  BOARD_USART->CR1 = USART_CR1_UE | USART_CR1_TE;
}

/* LED is on PB12 */
static inline void led_init(void)
{
  const uint32_t GPIO_MODE_OUTPUT_PP = 0x00000001U;

  // enable clock to GPIO block B
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

  // disable output
	GPIOB->BSRR = GPIO_BSRR_BR12;

  // switch output type is push-pull on reset

  // switch mode to output
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE12)) | (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE12_Pos);
}

/* configure USB on PA11 (D-) and PA12 (D+) */
static inline void usb_init(void)
{
  // // enable clock to GPIO block A
  // RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  // high speed output
  GPIOA->OSPEEDR =
    (GPIOA->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED11
      | GPIO_OSPEEDR_OSPEED12))
      | (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED11_Pos)
      | (GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED12_Pos);


  // // alternate function to USB
  // GPIOA->AFR[1] =
  //   (GPIOA->AFR[1] & ~(GPIO_AFRH_AFSEL11 | GPIO_AFRH_AFSEL12))
  //     | (GPIO_AF14_USB << GPIO_AFRH_AFSEL11_Pos)
  //     | (GPIO_AF14_USB << GPIO_AFRH_AFSEL12_Pos);

  // // switch mode to alternate function
  // GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE11 | GPIO_MODER_MODE12)) | (GPIO_MODE_AF_PP << GPIO_MODER_MODE11_Pos) | (GPIO_MODE_AF_PP << GPIO_MODER_MODE12_Pos);



  PWR->CR2 |= PWR_CR2_USV;
  // | PWR_CR2_PVMEN_USB;


  // enable clock to USB
  RCC->APBENR1 |= RCC_APBENR1_USBEN;

}

static inline void tx_char(uint8_t ch)
{
  const uint32_t USART_ISR_TXE = USART_ISR_TXE_TXFNF; // alternate mode (no fifo) RM0444 Rev 5, p.1075

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
  led_init();
  clock_init();
  uart_init();
#if CFG_TUSB_DEBUG >= 2
  uart_send_str("D5035-05 UART initialized\n");
#endif

#if CFG_TUSB_OS  == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#endif

#if CFG_TUSB_OS == OPT_OS_FREERTOS
	// If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
	NVIC_SetPriority(USB_UCPD1_2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

  usb_init();
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  GPIOB->BSRR = UINT32_C(1) << (12 + (!state) * 16);
}

uint32_t board_button_read(void)
{
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
void SysTick_Handler (void)
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

void SystemInit(void)
{
}

uint32_t SystemCoreClock;
