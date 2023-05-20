/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019
 *    William D. Jones (thor0505@comcast.net),
 *    Ha Thach (tinyusb.org)
 *    Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de
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

#include "stm32h7xx_hal.h"
#include "bsp/board.h"
#include "board.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

// Despite being call USB2_OTG
// OTG_FS is marked as RHPort0 by TinyUSB to be consistent across stm32 port
void OTG_FS_IRQHandler(void)
{
  tud_int_handler(0);
  board_uart_write("fs\n", -1);
}

// Despite being call USB2_OTG
// OTG_HS is marked as RHPort1 by TinyUSB to be consistent across stm32 port
void OTG_HS_IRQHandler(void)
{
#if BOARD_FS_PHY_ON_HS_CORE
  board_uart_write("hs 0\n", -1);
  tud_int_handler(0);
#else
  board_uart_write("hs 1\n", -1);
  tud_int_handler(1);
#endif

}

// void OTG_HS_EP1_OUT_IRQHandler(void) {}
// void OTG_HS_EP1_IN_IRQHandler(void) {}
// void OTG_HS_WKUP_IRQHandler(void) {}
// void OTG_HS_IRQHandler(void) {}


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

UART_HandleTypeDef UartHandle;

static inline void uart_send_buffer(uint8_t const *text, size_t len)
{
	HAL_UART_Transmit(&UartHandle, (uint8_t*) text, len, 0xffff);
}

static inline void uart_send_str(const char* text)
{
	while (*text) {
    uint8_t c = *text++;

    uart_send_buffer(&c, 1);
	}
}

// static inline void board_led_init(void)
// {
//   GPIO_InitTypeDef  GPIO_InitStruct;

//   __HAL_RCC_GPIOB_CLK_ENABLE();


//   GPIO_InitStruct.Pin   = LED_PIN;
//   GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull  = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//   HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);



//   // // enable clock to GPIO block B
//   // RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;

//   // // disable output
// 	// GPIOB->BSRR = GPIO_BSRR_BR0;

//   // // switch output type is push-pull on reset

//   // // switch mode to output
//   // GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE0)) | (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE0_Pos);
// }

void board_init(void)
{
  // board_led_init();
  board_stm32h7_clock_init();


  // Enable All GPIOs clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE(); // USB ULPI NXT
  __HAL_RCC_GPIOC_CLK_ENABLE(); // USB ULPI NXT
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE(); // USB ULPI NXT
  __HAL_RCC_GPIOI_CLK_ENABLE(); // USB ULPI NXT
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  // Enable UART Clock
  UART_CLK_EN();

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);

#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // Explicitly disable systick to prevent its ISR runs before scheduler start
  SysTick->CTRL &= ~1U;

  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
#if !BOARD_FS_PHY_ON_HS_CORE
  // NVIC_SetPriority(OTG_FS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif
  NVIC_SetPriority(OTG_HS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  GPIO_InitTypeDef  GPIO_InitStruct;

  // LED
  GPIO_InitStruct.Pin   = LED_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  // Button
  GPIO_InitStruct.Pin   = BUTTON_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

  // Uart
  GPIO_InitStruct.Pin       = UART_TX_PIN | UART_RX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = UART_GPIO_AF;
  HAL_GPIO_Init(UART_GPIO_PORT, &GPIO_InitStruct);

  UartHandle.Instance        = UART_DEV;
  UartHandle.Init.BaudRate   = CFG_BOARD_UART_BAUDRATE;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&UartHandle);

#if BOARD_DEVICE_RHPORT_NUM == 0
  // Despite being call USB2_OTG
  // OTG_FS is marked as RHPort0 by TinyUSB to be consistent across stm32 port
  // PA9 VUSB, PA10 ID, PA11 DM, PA12 DP

  // Configure DM DP Pins
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG2_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // This for ID line debug
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG2_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // https://community.st.com/s/question/0D50X00009XkYZLSA3/stm32h7-nucleo-usb-fs-cdc
  // TODO: Board init actually works fine without this line.
  HAL_PWREx_EnableUSBVoltageDetector();
  __HAL_RCC_USB2_OTG_FS_CLK_ENABLE();

#if OTG_FS_VBUS_SENSE
  // Configure VBUS Pin
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable VBUS sense (B device) via pin PA9
  USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBDEN;
#else
  // Disable VBUS sense (B device) via pin PA9
  USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;

  // B-peripheral session valid override enable
  USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
#endif // vbus sense

#elif BOARD_DEVICE_RHPORT_NUM == 1
  // Despite being call USB2_OTG
  // OTG_HS is marked as RHPort1 by TinyUSB to be consistent across stm32 port

#if BOARD_FS_PHY_ON_HS_CORE
  // PA9 VUSB, PA10 ID, PA11 DM, PA12 DP

  // Configure DM DP Pins
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // This for ID line debug
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#else

  struct {
    GPIO_TypeDef* port;
    uint32_t pin;
  } const ulpi_pins[] =
  {
    ULPI_PINS
  };

  for (uint8_t i=0; i < sizeof(ulpi_pins)/sizeof(ulpi_pins[0]); i++)
  {
    GPIO_InitStruct.Pin       = ulpi_pins[i].pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG2_HS;
    HAL_GPIO_Init(ulpi_pins[i].port, &GPIO_InitStruct);
  }
  // Enable USB ULPI clock
  __HAL_RCC_USB1_OTG_HS_ULPI_CLK_ENABLE();
#endif

  // Enable USB HS clock
  __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

#if OTG_HS_VBUS_SENSE
  #error OTG HS VBUS Sense enabled is not implemented
#else
  // No VBUS sense
  USB_OTG_HS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;

  // B-peripheral session valid override enable
  USB_OTG_HS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  USB_OTG_HS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
#endif

  // Force device mode
  USB_OTG_HS->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
  USB_OTG_HS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;

  HAL_PWREx_EnableUSBVoltageDetector();

  // For waveshare openh743 ULPI PHY reset walkaround
  board_stm32h7_post_init();
#endif // rhport = 1

  board_led_write(1);

  board_uart_write("Hello, world!\n", -1);
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, state ? LED_STATE_ON : (1-LED_STATE_ON));
  // GPIOB->BSRR = UINT32_C(1) << (0 + (!state) * 16);
}

uint32_t board_button_read(void)
{
  return (BUTTON_STATE_ACTIVE == HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) ? 1 : 0;
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


#if CFG_TUSB_OS == OPT_OS_NONE
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

void HardFault_Handler(void)
{
  asm("bkpt");
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}
