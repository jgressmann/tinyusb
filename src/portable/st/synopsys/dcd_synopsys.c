/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Scott Shawcroft, 2019 William D. Jones for Adafruit Industries
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2020 Jan Duempelmann
 * Copyright (c) 2020 Reinhard Panhuber
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

#include "tusb_option.h"


#include "supercan_debug.h"
// #define LOG(...)
// #define sc_dump_mem(...)



// Since TinyUSB doesn't use SOF for now, and this interrupt too often (1ms interval)
// We disable SOF for now until needed later on
#define USE_SOF     0

#ifndef DCD_ST_SYN_CUSTOM_FIFO_SIZES
# define DCD_ST_SYN_CUSTOM_FIFO_SIZES 0
#else
# ifndef DCD_ST_SYN_RX_FIFO_SIZE_WORDS
#   error define DCD_ST_SYN_RX_FIFO_SIZE_WORDS
# endif
# ifndef DCD_ST_SYN_TX0_FIFO_SIZE_WORDS
#   error define DCD_ST_SYN_TX0_FIFO_SIZE_WORDS
# endif
# ifndef DCD_ST_SYN_TX1_FIFO_SIZE_WORDS
#   error define DCD_ST_SYN_TX1_FIFO_SIZE_WORDS
# endif
# ifndef DCD_ST_SYN_TX2_FIFO_SIZE_WORDS
#   error define DCD_ST_SYN_TX2_FIFO_SIZE_WORDS
# endif
# ifndef DCD_ST_SYN_TX3_FIFO_SIZE_WORDS
#   error define DCD_ST_SYN_TX3_FIFO_SIZE_WORDS
# endif
#endif

#if defined (STM32F105x8) || defined (STM32F105xB) || defined (STM32F105xC) || \
    defined (STM32F107xB) || defined (STM32F107xC)
#define STM32F1_SYNOPSYS
#endif

#if defined (STM32L475xx) || defined (STM32L476xx) ||                          \
    defined (STM32L485xx) || defined (STM32L486xx) || defined (STM32L496xx) || \
    defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || \
    defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)
#define STM32L4_SYNOPSYS
#endif

#if TUSB_OPT_DEVICE_ENABLED &&                                          \
    ( (CFG_TUSB_MCU == OPT_MCU_STM32F1 && defined(STM32F1_SYNOPSYS)) || \
       CFG_TUSB_MCU == OPT_MCU_STM32F2                               || \
       CFG_TUSB_MCU == OPT_MCU_STM32F4                               || \
       CFG_TUSB_MCU == OPT_MCU_STM32F7                               || \
       CFG_TUSB_MCU == OPT_MCU_STM32H7                               || \
      (CFG_TUSB_MCU == OPT_MCU_STM32L4 && defined(STM32L4_SYNOPSYS)  || \
       CFG_TUSB_MCU == OPT_MCU_GD32VF103 )                           || \
       CFG_TUSB_MCU == OPT_MCU_GD32C10X                              \
    )

// EP_MAX       : Max number of bi-directional endpoints including EP0
// EP_FIFO_SIZE : Size of dedicated USB SRAM
#if CFG_TUSB_MCU == OPT_MCU_STM32F1
#include "stm32f1xx.h"
#define EP_MAX_FS       4
#define EP_FIFO_SIZE_FS 1280

#elif CFG_TUSB_MCU == OPT_MCU_STM32F2
#include "stm32f2xx.h"
#define EP_MAX_FS       USB_OTG_FS_MAX_IN_ENDPOINTS
#define EP_FIFO_SIZE_FS USB_OTG_FS_TOTAL_FIFO_SIZE

#elif CFG_TUSB_MCU == OPT_MCU_STM32F4
#include "stm32f4xx.h"
#define EP_MAX_FS       USB_OTG_FS_MAX_IN_ENDPOINTS
#define EP_FIFO_SIZE_FS USB_OTG_FS_TOTAL_FIFO_SIZE
#define EP_MAX_HS       USB_OTG_HS_MAX_IN_ENDPOINTS
#define EP_FIFO_SIZE_HS USB_OTG_HS_TOTAL_FIFO_SIZE

#elif CFG_TUSB_MCU == OPT_MCU_STM32H7
#include "stm32h7xx.h"
#define EP_MAX_FS       9
#define EP_FIFO_SIZE_FS 4096
#define EP_MAX_HS       9
#define EP_FIFO_SIZE_HS 4096

#if BOARD_FS_PHY_ON_HS_CORE
  #define USB_OTG_FS_PERIPH_BASE USB1_OTG_HS_PERIPH_BASE
  #define OTG_FS_IRQn OTG_HS_IRQn
#endif


#elif CFG_TUSB_MCU == OPT_MCU_STM32F7
#include "stm32f7xx.h"
#define EP_MAX_FS       6
#define EP_FIFO_SIZE_FS 1280
#define EP_MAX_HS       9
#define EP_FIFO_SIZE_HS 4096

#elif CFG_TUSB_MCU == OPT_MCU_STM32L4
#include "stm32l4xx.h"
#define EP_MAX_FS       6
#define EP_FIFO_SIZE_FS 1280

#elif CFG_TUSB_MCU == OPT_MCU_GD32VF103
#include "synopsys_common.h"

// for remote wakeup delay
#define __NOP()   __asm volatile ("nop")

// These numbers are the same for the whole GD32VF103 family.
#define OTG_FS_IRQn     86
#define EP_MAX_FS       4
#define EP_FIFO_SIZE_FS 1280

// The GD32VF103 is a RISC-V MCU, which implements the ECLIC Core-Local
// Interrupt Controller by Nuclei. It is nearly API compatible to the
// NVIC used by ARM MCUs.
#define ECLIC_INTERRUPT_ENABLE_BASE 0xD2001001UL

#define NVIC_EnableIRQ __eclic_enable_interrupt
#define NVIC_DisableIRQ __eclic_disable_interrupt

static inline void __eclic_enable_interrupt (uint32_t irq) {
  *(volatile uint8_t*)(ECLIC_INTERRUPT_ENABLE_BASE + (irq * 4)) = 1;
}

static inline void __eclic_disable_interrupt (uint32_t irq){
  *(volatile uint8_t*)(ECLIC_INTERRUPT_ENABLE_BASE + (irq * 4)) = 0;
}


#elif CFG_TUSB_MCU == OPT_MCU_GD32C10X
#include "gd32c10x.h"
#include "synopsys_common.h"
// These numbers are the same for the whole GD32C10X family.
#define OTG_FS_IRQn     USBFS_IRQn
#define EP_MAX_FS       (CFG_TUSB_ENDPOINT_LIMIT < 0 ? 4 : CFG_TUSB_ENDPOINT_LIMIT)
#define EP_FIFO_SIZE_FS 1280

#else
#error "Unsupported MCUs"
#endif

#include "device/dcd.h"

#if DCD_ST_SYN_CUSTOM_FIFO_SIZES
TU_VERIFY_STATIC(4 * (DCD_ST_SYN_RX_FIFO_SIZE_WORDS + DCD_ST_SYN_TX0_FIFO_SIZE_WORDS + DCD_ST_SYN_TX2_FIFO_SIZE_WORDS + DCD_ST_SYN_TX3_FIFO_SIZE_WORDS) <= EP_FIFO_SIZE_FS);

static const uint16_t TX_FIFO_SIZES_WORDS[] = {
  DCD_ST_SYN_TX0_FIFO_SIZE_WORDS,
  DCD_ST_SYN_TX1_FIFO_SIZE_WORDS,
  DCD_ST_SYN_TX2_FIFO_SIZE_WORDS,
  DCD_ST_SYN_TX3_FIFO_SIZE_WORDS,
};

/* Start address at begin of fifo size, end of RAM -> no effect on size */
// static const uint16_t TX_FIFO_OFFSETS_WORDS[] = {
//   1 * (EP_FIFO_SIZE_FS / 4 - DCD_ST_SYN_TX0_FIFO_SIZE_WORDS),
//   1 * (EP_FIFO_SIZE_FS / 4 - DCD_ST_SYN_TX0_FIFO_SIZE_WORDS - DCD_ST_SYN_TX1_FIFO_SIZE_WORDS),
//   1 * (EP_FIFO_SIZE_FS / 4 - DCD_ST_SYN_TX0_FIFO_SIZE_WORDS - DCD_ST_SYN_TX1_FIFO_SIZE_WORDS - DCD_ST_SYN_TX2_FIFO_SIZE_WORDS),
//   1 * (EP_FIFO_SIZE_FS / 4 - DCD_ST_SYN_TX0_FIFO_SIZE_WORDS - DCD_ST_SYN_TX1_FIFO_SIZE_WORDS - DCD_ST_SYN_TX2_FIFO_SIZE_WORDS - DCD_ST_SYN_TX3_FIFO_SIZE_WORDS),
// };

/* Start address in bytes at begin of fifo size, end of RAM -> no effect on size */
// static const uint16_t TX_FIFO_OFFSETS_WORDS[] = {
//   4 * (EP_FIFO_SIZE_FS / 4 - DCD_ST_SYN_TX0_FIFO_SIZE_WORDS),
//   4 * (EP_FIFO_SIZE_FS / 4 - DCD_ST_SYN_TX0_FIFO_SIZE_WORDS - DCD_ST_SYN_TX1_FIFO_SIZE_WORDS),
//   4 * (EP_FIFO_SIZE_FS / 4 - DCD_ST_SYN_TX0_FIFO_SIZE_WORDS - DCD_ST_SYN_TX1_FIFO_SIZE_WORDS - DCD_ST_SYN_TX2_FIFO_SIZE_WORDS),
//   4 * (EP_FIFO_SIZE_FS / 4 - DCD_ST_SYN_TX0_FIFO_SIZE_WORDS - DCD_ST_SYN_TX1_FIFO_SIZE_WORDS - DCD_ST_SYN_TX2_FIFO_SIZE_WORDS - DCD_ST_SYN_TX3_FIFO_SIZE_WORDS),
// };


/* Start address at begin of fifo size -> no effect on size */
static const uint16_t TX_FIFO_OFFSETS_WORDS[] = {
  DCD_ST_SYN_RX_FIFO_SIZE_WORDS,
  DCD_ST_SYN_RX_FIFO_SIZE_WORDS + DCD_ST_SYN_TX0_FIFO_SIZE_WORDS,
  DCD_ST_SYN_RX_FIFO_SIZE_WORDS + DCD_ST_SYN_TX0_FIFO_SIZE_WORDS + DCD_ST_SYN_TX1_FIFO_SIZE_WORDS,
  DCD_ST_SYN_RX_FIFO_SIZE_WORDS + DCD_ST_SYN_TX0_FIFO_SIZE_WORDS + DCD_ST_SYN_TX1_FIFO_SIZE_WORDS + DCD_ST_SYN_TX2_FIFO_SIZE_WORDS,
};


/* Start address at end of fifo size -> no effect on size */
// static const uint16_t TX_FIFO_OFFSETS_WORDS[] = {
//   DCD_ST_SYN_RX_FIFO_SIZE_WORDS + DCD_ST_SYN_TX0_FIFO_SIZE_WORDS,
//   DCD_ST_SYN_RX_FIFO_SIZE_WORDS + DCD_ST_SYN_TX0_FIFO_SIZE_WORDS + DCD_ST_SYN_TX1_FIFO_SIZE_WORDS,
//   DCD_ST_SYN_RX_FIFO_SIZE_WORDS + DCD_ST_SYN_TX0_FIFO_SIZE_WORDS + DCD_ST_SYN_TX1_FIFO_SIZE_WORDS + DCD_ST_SYN_TX2_FIFO_SIZE_WORDS,
//   DCD_ST_SYN_RX_FIFO_SIZE_WORDS + DCD_ST_SYN_TX0_FIFO_SIZE_WORDS + DCD_ST_SYN_TX1_FIFO_SIZE_WORDS + DCD_ST_SYN_TX2_FIFO_SIZE_WORDS + DCD_ST_SYN_TX3_FIFO_SIZE_WORDS,
// };


#endif

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

// On STM32 we associate Port0 to OTG_FS, and Port1 to OTG_HS
#if TUD_OPT_RHPORT == 0
#define EP_MAX            EP_MAX_FS
#define EP_FIFO_SIZE      EP_FIFO_SIZE_FS
#define RHPORT_REGS_BASE  USB_OTG_FS_PERIPH_BASE
#define RHPORT_IRQn       OTG_FS_IRQn

#else
#define EP_MAX            EP_MAX_HS
#define EP_FIFO_SIZE      EP_FIFO_SIZE_HS
#define RHPORT_REGS_BASE  USB_OTG_HS_PERIPH_BASE
#define RHPORT_IRQn       OTG_HS_IRQn

#endif

#define GLOBAL_BASE(_port)     ((USB_OTG_GlobalTypeDef*) RHPORT_REGS_BASE)
#define DEVICE_BASE(_port)     (USB_OTG_DeviceTypeDef *) (RHPORT_REGS_BASE + USB_OTG_DEVICE_BASE)
#define OUT_EP_BASE(_port)     (USB_OTG_OUTEndpointTypeDef *) (RHPORT_REGS_BASE + USB_OTG_OUT_ENDPOINT_BASE)
#define IN_EP_BASE(_port)      (USB_OTG_INEndpointTypeDef *) (RHPORT_REGS_BASE + USB_OTG_IN_ENDPOINT_BASE)
#define FIFO_BASE(_port, _x)   ((volatile uint32_t *) (RHPORT_REGS_BASE + USB_OTG_FIFO_BASE + (_x) * USB_OTG_FIFO_SIZE))
#define FIFO_RAM_BASE(_port)  ((uint8_t *) (RHPORT_REGS_BASE + 0x2000))

enum
{
  DCD_HIGH_SPEED        = 0, // Highspeed mode
  DCD_FULL_SPEED_USE_HS = 1, // Full speed in Highspeed port (probably with internal PHY)
  DCD_FULL_SPEED        = 3, // Full speed with internal PHY
};

static TU_ATTR_ALIGNED(4) uint32_t _setup_packet[2];

typedef struct {
  uint8_t * buffer;
  tu_fifo_t * ff;
  uint16_t total_len;
  uint16_t max_size;
  uint8_t interval;
} xfer_ctl_t;

typedef volatile uint32_t * usb_fifo_t;

xfer_ctl_t xfer_status[EP_MAX][2];
#define XFER_CTL_BASE(_ep, _dir) &xfer_status[_ep][_dir]

// EP0 transfers are limited to 1 packet - larger sizes has to be split
static uint16_t ep0_pending[2];                   // Index determines direction as tusb_dir_t type

#if !DCD_ST_SYN_CUSTOM_FIFO_SIZES
// TX FIFO RAM allocation so far in words - RX FIFO size is readily available from usb_otg->GRXFSIZ
static uint16_t _allocated_fifo_words_tx;         // TX FIFO size in words (IN EPs)
static bool _out_ep_closed;                       // Flag to check if RX FIFO size needs an update (reduce its size)

// Calculate the RX FIFO size according to recommendations from reference manual
static inline uint16_t calc_rx_ff_size(uint16_t ep_size)
{
  return 15 + 2*(ep_size/4) + 2*EP_MAX;
}

static inline void update_grxfsiz(uint8_t rhport)
{
  (void) rhport;

  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);

  // Determine largest EP size for RX FIFO
  uint16_t max_epsize = 0;
  for (uint8_t epnum = 0; epnum < EP_MAX; epnum++)
  {
    max_epsize = tu_max16(max_epsize, xfer_status[epnum][TUSB_DIR_OUT].max_size);
  }

  // Update size of RX FIFO
  usb_otg->GRXFSIZ = calc_rx_ff_size(max_epsize);
}
#endif

static inline void dump_fifos(bool dump_mem)
{
  const uint8_t rhport = 0;
  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  // USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  // USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);
  uint8_t* fifo_ram = FIFO_RAM_BASE(rhport);


  (void)rhport;
  (void)usb_otg;
  (void)in_ep;
  (void)fifo_ram;

  for (unsigned i = 0; i < EP_MAX; ++i) {
      if (i == 0) {
        unsigned mps = (in_ep[i].DIEPCTL & 0x3);

        switch (mps) {
        case 0:
          mps = 64;
          break;
        case 1:
          mps = 32;
          break;
        case 2:
          mps = 16;
          break;
        case 3:
          mps = 8;
          break;
        }
        LOG("in ep=%u start(w)=%u start(h)=%03x size(w)=%u free(w)=%u mps=%u\n",
          i,
          (usb_otg->DIEPTXF0_HNPTXFSIZ & 0xffff),
          (usb_otg->DIEPTXF0_HNPTXFSIZ & 0xffff) * 4,
          ((usb_otg->DIEPTXF0_HNPTXFSIZ >> 16) & 0xffff),
          (in_ep[i].DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) >> USB_OTG_DTXFSTS_INEPTFSAV_Pos,
          mps);
      } else {
        LOG("in ep=%u start(w)=%u start(h)=%03x size(w)=%u free(w)=%u mps=%u\n",
          i,
          (usb_otg->DIEPTXF[i-1] & 0xffff),
          (usb_otg->DIEPTXF[i-1] & 0xffff) * 4,
          (usb_otg->DIEPTXF[i-1] >> 16) & 0xffff,
          (in_ep[i].DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) >> USB_OTG_DTXFSTS_INEPTFSAV_Pos,
          (in_ep[i].DIEPCTL & USB_OTG_DIEPCTL_MPSIZ_Msk) >> USB_OTG_DIEPCTL_MPSIZ_Pos);
      }
  }

  if (dump_mem) {
    //sc_dump_mem(fifo_ram + (usb_otg->DIEPTXF[1] & 0xffff), ((usb_otg->DIEPTXF[1] >> 16) & 0xffff) * 4);
    sc_dump_mem(fifo_ram, 1280);
  }
}

// Setup the control endpoint 0.
static void bus_reset(uint8_t rhport)
{
  (void) rhport;

  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  tu_memclr(xfer_status, sizeof(xfer_status));
#if !DCD_ST_SYN_CUSTOM_FIFO_SIZES
  _out_ep_closed = false;
#endif

  // clear device address
  dev->DCFG &= ~USB_OTG_DCFG_DAD_Msk;

  // 1. NAK for all OUT endpoints
  for(uint8_t n = 0; n < EP_MAX; n++) {
    out_ep[n].DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
    // in_ep[n].DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
  }

  // 2. Un-mask interrupt bits
  dev->DAINTMSK = (1 << USB_OTG_DAINTMSK_OEPM_Pos) | (1 << USB_OTG_DAINTMSK_IEPM_Pos);
  dev->DOEPMSK = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM;
  dev->DIEPMSK = USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM;

#if DCD_ST_SYN_CUSTOM_FIFO_SIZES
  usb_otg->GRXFSIZ = DCD_ST_SYN_RX_FIFO_SIZE_WORDS;
  // Control IN uses FIFO 0 with 64 bytes ( 16 32-bit word )
  usb_otg->DIEPTXF0_HNPTXFSIZ = (TX_FIFO_SIZES_WORDS[0] << USB_OTG_TX0FD_Pos) | (TX_FIFO_OFFSETS_WORDS[0] << USB_OTG_TX0FSA_Pos);
  // usb_otg->DIEPTXF0_HNPTXFSIZ = (DCD_ST_SYN_TX0_FIFO_SIZE_WORDS << USB_OTG_TX0FD_Pos) | ((4 * DCD_ST_SYN_RX_FIFO_SIZE_WORDS) << USB_OTG_TX0FSA_Pos);
  //usb_otg->DIEPTXF0_HNPTXFSIZ = (DCD_ST_SYN_TX0_FIFO_SIZE_WORDS << USB_OTG_TX0FD_Pos) | ((EP_FIFO_SIZE) << USB_OTG_TX0FSA_Pos);

  // Fixed control EP0 size to 64 bytes
  in_ep[0].DIEPCTL &= ~(0x03 << USB_OTG_DIEPCTL_MPSIZ_Pos);
  xfer_status[0][TUSB_DIR_OUT].max_size = xfer_status[0][TUSB_DIR_IN].max_size = 4 * DCD_ST_SYN_TX0_FIFO_SIZE_WORDS;
  LOG("rx fifo size=%u\n", usb_otg->GRXFSIZ & USB_OTG_GRXFSIZ_RXFD_Msk);



  for (size_t epnum = 1; epnum < 4; ++epnum) {
    usb_fifo_t tx_fifo = FIFO_BASE(rhport, epnum);
    //usb_otg->DIEPTXF[epnum-1] = (TX_FIFO_OFFSETS_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXSA_Pos);
    //usb_otg->DIEPTXF[epnum-1] = (TX_FIFO_SIZES_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXFD_Pos) | (TX_FIFO_OFFSETS_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXSA_Pos);
    // usb_otg->DIEPTXF[epnum-1] = (320 << USB_OTG_DIEPTXF_INEPTXSA_Pos);
    // usb_otg->DIEPTXF[epnum-1] = (TX_FIFO_SIZES_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXFD_Pos);

    usb_otg->DIEPTXF[epnum-1] = (TX_FIFO_SIZES_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXFD_Pos) | (TX_FIFO_OFFSETS_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXSA_Pos);

    for (size_t i = 0, e = TX_FIFO_SIZES_WORDS[epnum]; i < e; ++i) {
      *tx_fifo = 0x00;
    }
  }

#else

  // "USB Data FIFOs" section in reference manual
  // Peripheral FIFO architecture
  //
  // The FIFO is split up in a lower part where the RX FIFO is located and an upper part where the TX FIFOs start.
  // We do this to allow the RX FIFO to grow dynamically which is possible since the free space is located
  // between the RX and TX FIFOs. This is required by ISO OUT EPs which need a bigger FIFO than the standard
  // configuration done below.
  //
  // Dynamically FIFO sizes are of interest only for ISO EPs since all others are usually not opened and closed.
  // All EPs other than ISO are opened as soon as the driver starts up i.e. when the host sends a
  // configure interface command. Hence, all IN EPs other the ISO will be located at the top. IN ISO EPs are usually
  // opened when the host sends an additional command: setInterface. At this point in time
  // the ISO EP will be located next to the free space and can change its size. In case more IN EPs change its size
  // an additional memory
  //
  // --------------- 320 or 1024 ( 1280 or 4096 bytes )
  // | IN FIFO 0   |
  // --------------- (320 or 1024) - 16
  // | IN FIFO 1   |
  // --------------- (320 or 1024) - 16 - x
  // |   . . . .   |
  // --------------- (320 or 1024) - 16 - x - y - ... - z
  // | IN FIFO MAX |
  // ---------------
  // |    FREE     |
  // --------------- GRXFSIZ
  // | OUT FIFO    |
  // | ( Shared )  |
  // --------------- 0
  //
  // According to "FIFO RAM allocation" section in RM, FIFO RAM are allocated as follows (each word 32-bits):
  // - Each EP IN needs at least max packet size, 16 words is sufficient for EP0 IN
  //
  // - All EP OUT shared a unique OUT FIFO which uses
  //   - 13 for setup packets + control words (up to 3 setup packets).
  //   - 1 for global NAK (not required/used here).
  //   - Largest-EPsize / 4 + 1. ( FS: 64 bytes, HS: 512 bytes). Recommended is  "2 x (Largest-EPsize/4) + 1"
  //   - 2 for each used OUT endpoint
  //
  //   Therefore GRXFSIZ = 13 + 1 + 1 + 2 x (Largest-EPsize/4) + 2 x EPOUTnum
  //   - FullSpeed (64 Bytes ): GRXFSIZ = 15 + 2 x  16 + 2 x EP_MAX = 47  + 2 x EP_MAX
  //   - Highspeed (512 bytes): GRXFSIZ = 15 + 2 x 128 + 2 x EP_MAX = 271 + 2 x EP_MAX
  //
  //   NOTE: Largest-EPsize & EPOUTnum is actual used endpoints in configuration. Since DCD has no knowledge
  //   of the overall picture yet. We will use the worst scenario: largest possible + EP_MAX
  //
  //   For Isochronous, largest EP size can be 1023/1024 for FS/HS respectively. In addition if multiple ISO
  //   are enabled at least "2 x (Largest-EPsize/4) + 1" are recommended.  Maybe provide a macro for application to
  //   overwrite this.

  usb_otg->GRXFSIZ = calc_rx_ff_size(TUD_OPT_HIGH_SPEED ? 512 : 64);


  _allocated_fifo_words_tx = 16;

  // Control IN uses FIFO 0 with 64 bytes ( 16 32-bit word )
  usb_otg->DIEPTXF0_HNPTXFSIZ = (16 << USB_OTG_TX0FD_Pos) | (EP_FIFO_SIZE/4 - _allocated_fifo_words_tx);

  // Fixed control EP0 size to 64 bytes
  in_ep[0].DIEPCTL &= ~(0x03 << USB_OTG_DIEPCTL_MPSIZ_Pos);
  xfer_status[0][TUSB_DIR_OUT].max_size = xfer_status[0][TUSB_DIR_IN].max_size = 64;
#endif
  out_ep[0].DOEPTSIZ |= (3 << USB_OTG_DOEPTSIZ_STUPCNT_Pos);

  usb_otg->GINTMSK |= USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IEPINT;

//






  // for (size_t epnum = 0; epnum < 4; ++epnum) {
  //   in_ep[epnum].DIEPCTL =
  //         // (1 << USB_OTG_DOEPCTL_USBAEP_Pos)    |
  //         (epnum << USB_OTG_DIEPCTL_TXFNUM_Pos) |
  //         (64 << USB_OTG_DIEPCTL_MPSIZ_Pos);

  //   out_ep[epnum].DOEPCTL =
  //         // (1 << USB_OTG_DOEPCTL_USBAEP_Pos)    |
  //         // (epnum << USB_OTG_DIEPCTL_TXFNUM_Pos) |
  //         (64 << USB_OTG_DIEPCTL_MPSIZ_Pos);
  // }


}

// Set turn-around timeout according to link speed
extern uint32_t SystemCoreClock;
static void set_turnaround(USB_OTG_GlobalTypeDef * usb_otg, tusb_speed_t speed)
{
  usb_otg->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;

  if ( speed == TUSB_SPEED_HIGH )
  {
    // Use fixed 0x09 for Highspeed
    usb_otg->GUSBCFG |= (0x09 << USB_OTG_GUSBCFG_TRDT_Pos);
  }
  else
  {
    // Turnaround timeout depends on the MCU clock
    uint32_t turnaround;

    if ( SystemCoreClock >= 32000000U )
      turnaround = 0x6U;
    else if ( SystemCoreClock >= 27500000U )
      turnaround = 0x7U;
    else if ( SystemCoreClock >= 24000000U )
      turnaround = 0x8U;
    else if ( SystemCoreClock >= 21800000U )
      turnaround = 0x9U;
    else if ( SystemCoreClock >= 20000000U )
      turnaround = 0xAU;
    else if ( SystemCoreClock >= 18500000U )
      turnaround = 0xBU;
    else if ( SystemCoreClock >= 17200000U )
      turnaround = 0xCU;
    else if ( SystemCoreClock >= 16000000U )
      turnaround = 0xDU;
    else if ( SystemCoreClock >= 15000000U )
      turnaround = 0xEU;
    else
      turnaround = 0xFU;

    // Fullspeed depends on MCU clocks, but we will use 0x06 for 32+ Mhz
    usb_otg->GUSBCFG |= (turnaround << USB_OTG_GUSBCFG_TRDT_Pos);
  }
}

static tusb_speed_t get_speed(uint8_t rhport)
{
  (void) rhport;
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  uint32_t const enum_spd = (dev->DSTS & USB_OTG_DSTS_ENUMSPD_Msk) >> USB_OTG_DSTS_ENUMSPD_Pos;
  return (enum_spd == DCD_HIGH_SPEED) ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL;
}

static void set_speed(uint8_t rhport, tusb_speed_t speed)
{
  uint32_t bitvalue;

  if ( rhport == 1 )
  {
    bitvalue = ((TUSB_SPEED_HIGH == speed) ? DCD_HIGH_SPEED : DCD_FULL_SPEED_USE_HS);
  }
  else
  {
    bitvalue = DCD_FULL_SPEED;
  }

  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);

  // Clear and set speed bits
  dev->DCFG &= ~(3 << USB_OTG_DCFG_DSPD_Pos);
  dev->DCFG |= (bitvalue << USB_OTG_DCFG_DSPD_Pos);
}

#if defined(USB_HS_PHYC)
static bool USB_HS_PHYCInit(void)
{
  USB_HS_PHYC_GlobalTypeDef *usb_hs_phyc = (USB_HS_PHYC_GlobalTypeDef*) USB_HS_PHYC_CONTROLLER_BASE;

  // Enable LDO
  usb_hs_phyc->USB_HS_PHYC_LDO |= USB_HS_PHYC_LDO_ENABLE;

  // Wait until LDO ready
  while ( 0 == (usb_hs_phyc->USB_HS_PHYC_LDO & USB_HS_PHYC_LDO_STATUS) ) {}

  uint32_t phyc_pll = 0;

  // TODO Try to get HSE_VALUE from registers instead of depending CFLAGS
  switch ( HSE_VALUE )
  {
    case 12000000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_12MHZ   ; break;
    case 12500000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_12_5MHZ ; break;
    case 16000000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_16MHZ   ; break;
    case 24000000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_24MHZ   ; break;
    case 25000000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_25MHZ   ; break;
    case 32000000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_Msk     ; break; // Value not defined in header
    default:
      TU_ASSERT(0);
  }
  usb_hs_phyc->USB_HS_PHYC_PLL = phyc_pll;

  // Control the tuning interface of the High Speed PHY
  // Use magic value (USB_HS_PHYC_TUNE_VALUE) from ST driver
  usb_hs_phyc->USB_HS_PHYC_TUNE |= 0x00000F13U;

  // Enable PLL internal PHY
  usb_hs_phyc->USB_HS_PHYC_PLL |= USB_HS_PHYC_PLL_PLLEN;

  // Original ST code has 2 ms delay for PLL stabilization.
  // Primitive test shows that more than 10 USB un/replug cycle showed no error with enumeration

  return true;
}
#endif

static void write_fifo_packet(uint8_t rhport, uint8_t fifo_num, uint8_t * src, uint16_t len);
static void edpt_schedule_packets(uint8_t rhport, uint8_t const epnum, uint8_t const dir, uint16_t const num_packets, uint16_t total_bytes)
{
  (void) rhport;

  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  // EP0 is limited to one packet each xfer
  // We use multiple transaction of xfer->max_size length to get a whole transfer done
  if(epnum == 0) {
    xfer_ctl_t * const xfer = XFER_CTL_BASE(epnum, dir);
    total_bytes = tu_min16(ep0_pending[dir], xfer->max_size);
    ep0_pending[dir] -= total_bytes;
  }

  // IN and OUT endpoint xfers are interrupt-driven, we just schedule them here.
  if(dir == TUSB_DIR_IN) {
    xfer_ctl_t *xfer = XFER_CTL_BASE(epnum, TUSB_DIR_IN);

    // if (xfer->max_size) {
      // SC_ASSERT(0 == (in_ep[epnum].DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ_Msk) >> USB_OTG_DIEPTSIZ_XFRSIZ_Pos);
      // SC_ASSERT(in_ep[epnum].DIEPINT & USB_OTG_DIEPINT_TXFE);
      // if (epnum > 1) {
      //   LOG("ep=%u free=%u\n", epnum, (in_ep[epnum].DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) >> USB_OTG_DTXFSTS_INEPTFSAV_Pos);
      // }
      // switch (epnum) {
      // case 0:
      //   //SC_ASSERT((in_ep[epnum].DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) == DCD_ST_SYN_TX0_FIFO_SIZE_WORDS);
      //   break;
      // case 1:
      //   SC_ASSERT((in_ep[epnum].DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) == DCD_ST_SYN_TX1_FIFO_SIZE_WORDS);
      //   break;
      // case 2:
      //   SC_ASSERT((in_ep[epnum].DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) == DCD_ST_SYN_TX2_FIFO_SIZE_WORDS);
      //   break;
      // case 3:
      //   SC_ASSERT((in_ep[epnum].DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) == DCD_ST_SYN_TX3_FIFO_SIZE_WORDS);
      //   break;
      // }
    // }




    // LOG("IN ep=%02x p=%u b=%u\n", epnum, num_packets, total_bytes);
    // in_ep[epnum].DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    // A full IN transfer (multiple packets, possibly) triggers XFRC.
    in_ep[epnum].DIEPTSIZ = (num_packets << USB_OTG_DIEPTSIZ_PKTCNT_Pos) |
        ((total_bytes << USB_OTG_DIEPTSIZ_XFRSIZ_Pos) & USB_OTG_DIEPTSIZ_XFRSIZ_Msk);


    // For ISO endpoint set correct odd/even bit for next frame.
    if ((in_ep[epnum].DIEPCTL & USB_OTG_DIEPCTL_EPTYP) == USB_OTG_DIEPCTL_EPTYP_0 && (XFER_CTL_BASE(epnum, dir))->interval == 1)
    {
      // Take odd/even bit from frame counter.
      uint32_t const odd_frame_now = (dev->DSTS & (1u << USB_OTG_DSTS_FNSOF_Pos));
      in_ep[epnum].DIEPCTL |= (odd_frame_now ? USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Msk : USB_OTG_DIEPCTL_SODDFRM_Msk);
    }
    // // Enable fifo empty interrupt only if there are something to put in the fifo.
    // if(total_bytes != 0) {
    //   dev->DIEPEMPMSK |= (1 << epnum);
    // }
    TU_ASSERT(!xfer->ff,);


  // if (epnum == 2) {
  //   static char buf[16] = {0};
  //   static uint8_t x = 0xa5;

  //   ++x;

  //   memset(buf[2], x, sizeof(buf)-2);
  //   // //LOG("buffer\n");
  //   write_fifo_packet(rhport, epnum, buf, sizeof(buf));

  //   dump_fifos(true);

  // } else {
    // size_t offset = 0;

    // for (; offset < total_bytes; offset += 64) {
    //   write_fifo_packet(rhport, epnum, xfer->buffer + offset, 64);
    // }

    // if (offset < total_bytes) {
    //   write_fifo_packet(rhport, epnum, xfer->buffer + offset, xfer->total_len - offset);
    // }
    in_ep[epnum].DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    write_fifo_packet(rhport, epnum, xfer->buffer, xfer->total_len);
    // dump_fifos(false);

  // }





  } else {
    // A full OUT transfer (multiple packets, possibly) triggers XFRC.
    out_ep[epnum].DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT_Msk | USB_OTG_DOEPTSIZ_XFRSIZ);
    out_ep[epnum].DOEPTSIZ |= (num_packets << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
        ((total_bytes << USB_OTG_DOEPTSIZ_XFRSIZ_Pos) & USB_OTG_DOEPTSIZ_XFRSIZ_Msk);

    out_ep[epnum].DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
    if ((out_ep[epnum].DOEPCTL & USB_OTG_DOEPCTL_EPTYP) == USB_OTG_DOEPCTL_EPTYP_0 && (XFER_CTL_BASE(epnum, dir))->interval == 1)
    {
      // Take odd/even bit from frame counter.
      uint32_t const odd_frame_now = (dev->DSTS & (1u << USB_OTG_DSTS_FNSOF_Pos));
      out_ep[epnum].DOEPCTL |= (odd_frame_now ? USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Msk : USB_OTG_DOEPCTL_SODDFRM_Msk);
    }
  }
}

/*------------------------------------------------------------------*/
/* Controller API
 *------------------------------------------------------------------*/
void dcd_init (uint8_t rhport)
{
  // Programming model begins in the last section of the chapter on the USB
  // peripheral in each Reference Manual.

  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);

  // No HNP/SRP (no OTG support), program timeout later.
  if ( rhport == 1 )
  {
    // On selected MCUs HS port1 can be used with external PHY via ULPI interface
#if CFG_TUSB_RHPORT1_MODE & OPT_MODE_HIGH_SPEED
    // deactivate internal PHY
    usb_otg->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;

    // Init The UTMI Interface
    usb_otg->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);

    // Select default internal VBUS Indicator and Drive for ULPI
    usb_otg->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);
#else
    usb_otg->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
#endif

#if defined(USB_HS_PHYC)
    // Highspeed with embedded UTMI PHYC

    // Select UTMI Interface
    usb_otg->GUSBCFG &= ~USB_OTG_GUSBCFG_ULPI_UTMI_SEL;
    usb_otg->GCCFG |= USB_OTG_GCCFG_PHYHSEN;

    // Enables control of a High Speed USB PHY
    USB_HS_PHYCInit();
#endif
  } else
  {
    // Enable internal PHY
    usb_otg->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
  }

  // Reset core after selecting PHY
  // Wait AHB IDLE, reset then wait until it is cleared
  while ((usb_otg->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U) {}
  usb_otg->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
  while ((usb_otg->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST) {}

  // Restart PHY clock
  *((volatile uint32_t *)(RHPORT_REGS_BASE + USB_OTG_PCGCCTL_BASE)) = 0;

  // Clear all interrupts
  usb_otg->GINTSTS |= usb_otg->GINTSTS;

  // Required as part of core initialization.
  // TODO: How should mode mismatch be handled? It will cause
  // the core to stop working/require reset.
  usb_otg->GINTMSK |= USB_OTG_GINTMSK_OTGINT | USB_OTG_GINTMSK_MMISM;

  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);

  // If USB host misbehaves during status portion of control xfer
  // (non zero-length packet), send STALL back and discard.
  dev->DCFG |=  USB_OTG_DCFG_NZLSOHSK;

  set_speed(rhport, TUD_OPT_HIGH_SPEED ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL);

  // Enable internal USB transceiver, unless using HS core (port 1) with external PHY.
  if (!(rhport == 1 && (CFG_TUSB_RHPORT1_MODE & OPT_MODE_HIGH_SPEED))) usb_otg->GCCFG |= USB_OTG_GCCFG_PWRDWN;

  usb_otg->GINTMSK |= USB_OTG_GINTMSK_USBRST   | USB_OTG_GINTMSK_ENUMDNEM |
      USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM     |
      USB_OTG_GINTMSK_RXFLVLM  | (USE_SOF ? USB_OTG_GINTMSK_SOFM : 0);

  // Enable global interrupt
  usb_otg->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

  dcd_connect(rhport);
}

void dcd_int_enable (uint8_t rhport)
{
  (void) rhport;
  NVIC_EnableIRQ(RHPORT_IRQn);
}

void dcd_int_disable (uint8_t rhport)
{
  (void) rhport;
  NVIC_DisableIRQ(RHPORT_IRQn);
}

void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  dev->DCFG = (dev->DCFG & ~USB_OTG_DCFG_DAD_Msk) | (dev_addr << USB_OTG_DCFG_DAD_Pos);

  // Response with status after changing device address
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

static void remote_wakeup_delay(void)
{
  // try to delay for 1 ms
  uint32_t count = SystemCoreClock / 1000;
  while ( count-- )
  {
    __NOP();
  }
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;

  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);

  // set remote wakeup
  dev->DCTL |= USB_OTG_DCTL_RWUSIG;

  // enable SOF to detect bus resume
  usb_otg->GINTSTS = USB_OTG_GINTSTS_SOF;
  usb_otg->GINTMSK |= USB_OTG_GINTMSK_SOFM;

  // Per specs: remote wakeup signal bit must be clear within 1-15ms//usb_otg->DIEPTXF[epnum-1] = (TX_FIFO_SIZES_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXFD_Pos) | (TX_FIFO_OFFSETS_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXSA_Pos);
  remote_wakeup_delay();

  dev->DCTL &= ~USB_OTG_DCTL_RWUSIG;
}

void dcd_connect(uint8_t rhport)
{
  (void) rhport;
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  dev->DCTL &= ~USB_OTG_DCTL_SDIS;
}

void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  dev->DCTL |= USB_OTG_DCTL_SDIS;
}


/*------------------------------------------------------------------*/
/* DCD Endpoint port
 *------------------------------------------------------------------*/

bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * desc_edpt)
{
  (void) rhport;

  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  uint8_t const epnum = tu_edpt_number(desc_edpt->bEndpointAddress);
  uint8_t const dir   = tu_edpt_dir(desc_edpt->bEndpointAddress);

  TU_ASSERT(epnum < EP_MAX);

  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, dir);
  xfer->max_size = desc_edpt->wMaxPacketSize.size;
  xfer->interval = desc_edpt->bInterval;
#if !DCD_ST_SYN_CUSTOM_FIFO_SIZES
  uint16_t const fifo_size = (desc_edpt->wMaxPacketSize.size + 3) / 4; // Round up to next full word
#endif
  if(dir == TUSB_DIR_OUT)
  {
#if !DCD_ST_SYN_CUSTOM_FIFO_SIZES
    // Calculate required size of RX FIFO
    uint16_t const sz = calc_rx_ff_size(4*fifo_size);

    // If size_rx needs to be extended check if possible and if so enlarge it
    if (usb_otg->GRXFSIZ < sz)
    {
      TU_ASSERT(sz + _allocated_fifo_words_tx <= EP_FIFO_SIZE/4);

      // Enlarge RX FIFO
      usb_otg->GRXFSIZ = sz;
    }
#endif

    out_ep[epnum].DOEPCTL |=
        (1 << USB_OTG_DOEPCTL_USBAEP_Pos)        |
        (desc_edpt->bmAttributes.xfer << USB_OTG_DOEPCTL_EPTYP_Pos)   |
        (desc_edpt->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS ? USB_OTG_DOEPCTL_SD0PID_SEVNFRM : 0) |
        (desc_edpt->wMaxPacketSize.size << USB_OTG_DOEPCTL_MPSIZ_Pos)
        ;

    dev->DAINTMSK |= (1 << (USB_OTG_DAINTMSK_OEPM_Pos + epnum));
  }
  else
  {
#if DCD_ST_SYN_CUSTOM_FIFO_SIZES
  // usb_fifo_t tx_fifo = FIFO_BASE(rhport, epnum);

  //  uint32_t count = 10;
  // while ( count-- )
  // {
  //   __NOP();
  // }



    // // DIEPTXF starts at FIFO #1.
    // // Both TXFD and TXSA are in unit of 32-bit words.
    // usb_otg->DIEPTXF[epnum-1] = (TX_FIFO_SIZES_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXFD_Pos) | (TX_FIFO_OFFSETS_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXSA_Pos);

    // for (size_t i = 0, e = TX_FIFO_SIZES_WORDS[epnum]; i < e; ++i) {
    //   *tx_fifo = 0x00;
    // }
    // usb_otg->DIEPTXF[epnum] = (TX_FIFO_SIZES_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXFD_Pos) | (TX_FIFO_OFFSETS_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXSA_Pos);
    // usb_otg->DIEPTXF[epnum-1] = (tx_fifo_size_words << USB_OTG_DIEPTXF_INEPTXFD_Pos) | ((4*tx_fifo_offset_words) << USB_OTG_DIEPTXF_INEPTXSA_Pos);
    //LOG("free words ep=%u count=%u\n", epnum, (in_ep[epnum].DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) >> USB_OTG_DTXFSTS_INEPTFSAV_Pos);

    // for (size_t i = 1; i < TU_ARRAY_SIZE(TX_FIFO_SIZES_WORDS); ++i) {
    //   usb_otg->DIEPTXF[i-1] = (TX_FIFO_SIZES_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXFD_Pos) | (TX_FIFO_OFFSETS_WORDS[epnum] << USB_OTG_DIEPTXF_INEPTXSA_Pos);
    // }

  // uint32_t count = SystemCoreClock / 1000;
  // while ( count-- )
  // {
  //   __NOP();
  // }
#else
    // "USB Data FIFOs" section in reference manual
    // Peripheral FIFO architecture
    //
    // --------------- 320 or 1024 ( 1280 or 4096 bytes )
    // | IN FIFO 0   |
    // --------------- (320 or 1024) - 16
    // | IN FIFO 1   |
    // --------------- (320 or 1024) - 16 - x
    // |   . . . .   |
    // --------------- (320 or 1024) - 16 - x - y - ... - z
    // | IN FIFO MAX |
    // ---------------
    // |    FREE     |
    // --------------- GRXFSIZ
    // | OUT FIFO    |
    // | ( Shared )  |
    // --------------- 0
    //
    // In FIFO is allocated by following rules:
    // - IN EP 1 gets FIFO 1, IN EP "n" gets FIFO "n".

    // Check if free space is available
    TU_ASSERT(_allocated_fifo_words_tx + fifo_size + usb_otg->GRXFSIZ <= EP_FIFO_SIZE/4);

    _allocated_fifo_words_tx += fifo_size;

    TU_LOG(2, "    Allocated %u bytes at offset %u", fifo_size*4, EP_FIFO_SIZE-_allocated_fifo_words_tx*4);

    // DIEPTXF starts at FIFO #1.
    // Both TXFD and TXSA are in unit of 32-bit words.
    usb_otg->DIEPTXF[epnum - 1] = (fifo_size << USB_OTG_DIEPTXF_INEPTXFD_Pos) | (EP_FIFO_SIZE/4 - _allocated_fifo_words_tx);
#endif
    in_ep[epnum].DIEPCTL |=
        (1 << USB_OTG_DIEPCTL_USBAEP_Pos) |
        (epnum << USB_OTG_DIEPCTL_TXFNUM_Pos) |
        (desc_edpt->bmAttributes.xfer << USB_OTG_DIEPCTL_EPTYP_Pos) |
        (desc_edpt->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS ? USB_OTG_DIEPCTL_SD0PID_SEVNFRM : 0) |
        (desc_edpt->wMaxPacketSize.size << USB_OTG_DIEPCTL_MPSIZ_Pos)
        ;

    dev->DAINTMSK |= (1 << (USB_OTG_DAINTMSK_IEPM_Pos + epnum));

    // dump_fifos(false);
  }

  return true;
}

// Close all non-control endpoints, cancel all pending transfers if any.
void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;

//  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  // Disable non-control interrupt
  dev->DAINTMSK = (1 << USB_OTG_DAINTMSK_OEPM_Pos) | (1 << USB_OTG_DAINTMSK_IEPM_Pos);

  for(uint8_t n = 1; n < EP_MAX; n++)
  {
    // disable OUT endpoint
    out_ep[n].DOEPCTL = 0;
    xfer_status[n][TUSB_DIR_OUT].max_size = 0;

    // disable IN endpoint
    in_ep[n].DIEPCTL = 0;
    xfer_status[n][TUSB_DIR_IN].max_size = 0;
  }

#if !DCD_ST_SYN_CUSTOM_FIFO_SIZES
  // reset allocated fifo IN
  _allocated_fifo_words_tx = 16;
#endif
}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, dir);
  xfer->buffer      = buffer;
  xfer->ff          = NULL;
  xfer->total_len   = total_bytes;

  LOG("X ep=%02x bytes=%u\n", ep_addr, total_bytes);


  // EP0 can only handle one packet
  if(epnum == 0) {
    ep0_pending[dir] = total_bytes;
    // Schedule the first transaction for EP0 transfer
    edpt_schedule_packets(rhport, epnum, dir, 1, ep0_pending[dir]);
    return true;
  }

  uint16_t num_packets = (total_bytes / xfer->max_size);
  uint16_t const short_packet_size = total_bytes % xfer->max_size;

  // Zero-size packet is special case.
  if(short_packet_size > 0 || (total_bytes == 0)) {
    num_packets++;
  }

  // Schedule packets to be sent within interrupt
  edpt_schedule_packets(rhport, epnum, dir, num_packets, total_bytes);

  return true;
}

// The number of bytes has to be given explicitly to allow more flexible control of how many
// bytes should be written and second to keep the return value free to give back a boolean
// success message. If total_bytes is too big, the FIFO will copy only what is available
// into the USB buffer!
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
  // USB buffers always work in bytes so to avoid unnecessary divisions we demand item_size = 1
  TU_ASSERT(ff->item_size == 1);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, dir);
  xfer->buffer      = NULL;
  xfer->ff          = ff;
  xfer->total_len   = total_bytes;

  uint16_t num_packets = (total_bytes / xfer->max_size);
  uint16_t const short_packet_size = total_bytes % xfer->max_size;

  // Zero-size packet is special case.
  if(short_packet_size > 0 || (total_bytes == 0)) num_packets++;

  // Schedule packets to be sent within interrupt
  edpt_schedule_packets(rhport, epnum, dir, num_packets, total_bytes);

  return true;
}

static void dcd_edpt_disable (uint8_t rhport, uint8_t ep_addr, bool stall)
{
  (void) rhport;

  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  if(dir == TUSB_DIR_IN) {
    // Only disable currently enabled non-control endpoint
    if ( (epnum == 0) || !(in_ep[epnum].DIEPCTL & USB_OTG_DIEPCTL_EPENA) ){
      in_ep[epnum].DIEPCTL |= USB_OTG_DIEPCTL_SNAK | (stall ? USB_OTG_DIEPCTL_STALL : 0);
    } else {
      // Stop transmitting packets and NAK IN xfers.
      in_ep[epnum].DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
      while((in_ep[epnum].DIEPINT & USB_OTG_DIEPINT_INEPNE) == 0);

      // Disable the endpoint.
      in_ep[epnum].DIEPCTL |= USB_OTG_DIEPCTL_EPDIS | (stall ? USB_OTG_DIEPCTL_STALL : 0);
      while((in_ep[epnum].DIEPINT & USB_OTG_DIEPINT_EPDISD_Msk) == 0);
      in_ep[epnum].DIEPINT = USB_OTG_DIEPINT_EPDISD;
    }

    // Flush the FIFO, and wait until we have confirmed it cleared.
    usb_otg->GRSTCTL |= (epnum << USB_OTG_GRSTCTL_TXFNUM_Pos);
    usb_otg->GRSTCTL |= USB_OTG_GRSTCTL_TXFFLSH;
    while((usb_otg->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH_Msk) != 0);
  } else {
    // Only disable currently enabled non-control endpoint
    if ( (epnum == 0) || !(out_ep[epnum].DOEPCTL & USB_OTG_DOEPCTL_EPENA) ){
      out_ep[epnum].DOEPCTL |= stall ? USB_OTG_DOEPCTL_STALL : 0;
    } else {
      // Asserting GONAK is required to STALL an OUT endpoint.
      // Simpler to use polling here, we don't use the "B"OUTNAKEFF interrupt
      // anyway, and it can't be cleared by user code. If this while loop never
      // finishes, we have bigger problems than just the stack.
      dev->DCTL |= USB_OTG_DCTL_SGONAK;
      while((usb_otg->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF_Msk) == 0);

      // Ditto here- disable the endpoint.
      out_ep[epnum].DOEPCTL |= USB_OTG_DOEPCTL_EPDIS | (stall ? USB_OTG_DOEPCTL_STALL : 0);
      while((out_ep[epnum].DOEPINT & USB_OTG_DOEPINT_EPDISD_Msk) == 0);
      out_ep[epnum].DOEPINT = USB_OTG_DOEPINT_EPDISD;

      // Allow other OUT endpoints to keep receiving.
      dev->DCTL |= USB_OTG_DCTL_CGONAK;
    }
  }
}

/**
 * Close an endpoint.
 */
void dcd_edpt_close (uint8_t rhport, uint8_t ep_addr)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  dcd_edpt_disable(rhport, ep_addr, false);

  xfer_status[epnum][dir].max_size = 0;  // max_size = 0 marks a disabled EP - required for changing FIFO allocation

#if !DCD_ST_SYN_CUSTOM_FIFO_SIZES
  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);

  // Update max_size


  if (dir == TUSB_DIR_IN)
  {
    uint16_t const fifo_size = (usb_otg->DIEPTXF[epnum - 1] & USB_OTG_DIEPTXF_INEPTXFD_Msk) >> USB_OTG_DIEPTXF_INEPTXFD_Pos;
    uint16_t const fifo_start = (usb_otg->DIEPTXF[epnum - 1] & USB_OTG_DIEPTXF_INEPTXSA_Msk) >> USB_OTG_DIEPTXF_INEPTXSA_Pos;
    // For now only the last opened endpoint can be closed without fuss.
    TU_ASSERT(fifo_start == EP_FIFO_SIZE/4 - _allocated_fifo_words_tx,);
    _allocated_fifo_words_tx -= fifo_size;
  }
  else
  {
    _out_ep_closed = true;     // Set flag such that RX FIFO gets reduced in size once RX FIFO is empty
  }
#endif
}

void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  dcd_edpt_disable(rhport, ep_addr, true);
}

void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  // Clear stall and reset data toggle
  if(dir == TUSB_DIR_IN) {
    in_ep[epnum].DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
    in_ep[epnum].DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
  } else {
    out_ep[epnum].DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
    out_ep[epnum].DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
  }
}


/*------------------------------------------------------------------*/

// Read a single data packet from receive FIFO
static void read_fifo_packet(uint8_t rhport, uint8_t * dst, uint16_t len)
{
  (void) rhport;

  usb_fifo_t rx_fifo = FIFO_BASE(rhport, 0);
  uint_least16_t words = len / 4;

  len -= words * 4;

  if (((uintptr_t)dst) & 3) { // aligned to 4 bytes?
    // LOG("ro\n");
    // Reading full available 32 bit words from fifo
    for(uint_least16_t i = 0; i < words; i++) {
      uint32_t tmp = *rx_fifo;
      dst[0] = tmp & 0x000000FF;
      dst[1] = (tmp & 0x0000FF00) >> 8;
      dst[2] = (tmp & 0x00FF0000) >> 16;
      dst[3] = (tmp & 0xFF000000) >> 24;
      dst += 4;
    }
  } else {
    // LOG("re\n");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
    uint32_t* dst32 = (uint32_t*)dst;
#pragma GCC diagnostic pop

    for (uint_least16_t i = 0; i < words; ++i) {
      *dst32++ = *rx_fifo;
    }

    dst += words * 4;
  }

  // uint32_t *dst_word = (uint32_t *)dst;
  // uint_least16_t words = len / 4;
  // uint_least16_t rem = len - words * 4;
  // uint32_t be_word;

  // // Reading full available 32 bit words from fifo
  // uint_least16_t full_words = len >> 2;
  // for(uint16_t i = 0; i < full_words; i++) {
  //   uint32_t tmp = *rx_fifo;
  //   dst[0] = tmp & 0x000000FF;
  //   dst[1] = (tmp & 0x0000FF00) >> 8;
  //   dst[2] = (tmp & 0x00FF0000) >> 16;
  //   dst[3] = (tmp & 0xFF000000) >> 24;
  //   dst += 4;
  // }



  // Read the remaining 1-3 bytes from fifo
  if(len != 0) {
    uint32_t tmp = *rx_fifo;
    dst[0] = tmp & 0x000000FF;
    if(len > 1) {
      dst[1] = (tmp & 0x0000FF00) >> 8;
    }
    if(len > 2) {
      dst[2] = (tmp & 0x00FF0000) >> 16;
    }
  }
}

// Write a single data packet to EPIN FIFO
static void write_fifo_packet(uint8_t rhport, uint8_t fifo_num, uint8_t * src, uint16_t len)
{
  (void) rhport;

  usb_fifo_t tx_fifo = FIFO_BASE(rhport, fifo_num);
  uint_least16_t words = len / 4;

  len -= words * 4;

  if (((uintptr_t)src) & 3) { // aligned to 4 bytes?
    // LOG("wo\n");
    for(uint16_t i = 0; i < words; i++){
      *tx_fifo = (src[3] << 24) | (src[2] << 16) | (src[1] << 8) | src[0];
      src += 4;
    }
  } else {
    // LOG("we\n");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
    uint32_t const* src32 = (uint32_t*)src;
#pragma GCC diagnostic pop

    for (uint_least16_t i = 0; i < words; ++i) {
      *tx_fifo = *src32++;
    }

    src += words * 4;
  }

  // Write the remaining 1-3 bytes into fifo
  if(len){
    // LOG("rem\n");
    uint32_t tmp_word = 0;
    tmp_word |= src[0];
    if(len > 1){
      tmp_word |= src[1] << 8;
    }
    if(len > 2){
      tmp_word |= src[2] << 16;
    }
    *tx_fifo = tmp_word;
  }
}

static bool handle_rxflvl_ints(uint8_t rhport, USB_OTG_OUTEndpointTypeDef * out_ep) {
  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  usb_fifo_t rx_fifo = FIFO_BASE(rhport, 0);

  // Pop control word off FIFO
  uint32_t ctl_word = usb_otg->GRXSTSP;
  // LOG("ctrl word %08x\n", ctl_word);
  uint8_t pktsts = (ctl_word & USB_OTG_GRXSTSP_PKTSTS_Msk) >> USB_OTG_GRXSTSP_PKTSTS_Pos;
  uint8_t epnum = (ctl_word &  USB_OTG_GRXSTSP_EPNUM_Msk) >>  USB_OTG_GRXSTSP_EPNUM_Pos;
  uint16_t bcnt = (ctl_word & USB_OTG_GRXSTSP_BCNT_Msk) >> USB_OTG_GRXSTSP_BCNT_Pos;

  // if (!bcnt) {
  //   return false;
  // }

  switch(pktsts) {
    case 0x00: // possibly empty fifo? Happens after dir creation with msc_dual_lun demo
      // LOG("ctrl word %08x bcnt=%04x\n", ctl_word, bcnt);
      return false;
      break;
    case 0x01: // Global OUT NAK (Interrupt)
      break;

    case 0x02: // Out packet recvd
    {
      xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, TUSB_DIR_OUT);

      // Read packet off RxFIFO
      if (xfer->ff)
      {
        // Ring buffer
        tu_fifo_write_n_const_addr_full_words(xfer->ff, (const void *) rx_fifo, bcnt);
      }
      else
      {
        // Linear buffer
        read_fifo_packet(rhport, xfer->buffer, bcnt);

        // Increment pointer to xfer data
        xfer->buffer += bcnt;
      }

      // Truncate transfer length in case of short packet
      if(bcnt < xfer->max_size) {
        xfer->total_len -= (out_ep[epnum].DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ_Msk) >> USB_OTG_DOEPTSIZ_XFRSIZ_Pos;
        if(epnum == 0) {
          xfer->total_len -= ep0_pending[TUSB_DIR_OUT];
          ep0_pending[TUSB_DIR_OUT] = 0;
        }
      }
    }
    break;

    case 0x03: // Out packet done (Interrupt)
      break;

    case 0x04: // Setup packet done (Interrupt)
      out_ep[epnum].DOEPTSIZ |= (3 << USB_OTG_DOEPTSIZ_STUPCNT_Pos);
      break;

    case 0x06: // Setup packet recvd
      // We can receive up to three setup packets in succession, but
      // only the last one is valid.
      _setup_packet[0] = (* rx_fifo);
      _setup_packet[1] = (* rx_fifo);
      break;

    default: // Invalid
      // LOG("unhandled packet type %02x\n", pktsts);
      // TU_BREAKPOINT();
      return false;
      break;
  }

  return true;
}

static void handle_epout_ints(uint8_t rhport, USB_OTG_DeviceTypeDef * dev, USB_OTG_OUTEndpointTypeDef * out_ep) {
  // DAINT for a given EP clears when DOEPINTx is cleared.
  // OEPINT will be cleared when DAINT's out bits are cleared.
  for(uint8_t n = 0; n < EP_MAX; n++) {
    xfer_ctl_t * xfer = XFER_CTL_BASE(n, TUSB_DIR_OUT);

    if(dev->DAINT & (1 << (USB_OTG_DAINT_OEPINT_Pos + n))) {
      // SETUP packet Setup Phase done.
      if(out_ep[n].DOEPINT & USB_OTG_DOEPINT_STUP) {
        out_ep[n].DOEPINT =  USB_OTG_DOEPINT_STUP;
        dcd_event_setup_received(rhport, (uint8_t*) &_setup_packet[0], true);
      }

      // OUT XFER complete
      if(out_ep[n].DOEPINT & USB_OTG_DOEPINT_XFRC) {
        out_ep[n].DOEPINT = USB_OTG_DOEPINT_XFRC;

        // EP0 can only handle one packet
        if((n == 0) && ep0_pending[TUSB_DIR_OUT]) {
          // Schedule another packet to be received.
          edpt_schedule_packets(rhport, n, TUSB_DIR_OUT, 1, ep0_pending[TUSB_DIR_OUT]);
        } else {
          dcd_event_xfer_complete(rhport, n, xfer->total_len, XFER_RESULT_SUCCESS, true);
        }
      }
    }
  }
}

static void handle_epin_ints(uint8_t rhport, USB_OTG_DeviceTypeDef * dev, USB_OTG_INEndpointTypeDef * in_ep) {
  // DAINT for a given EP clears when DIEPINTx is cleared.
  // IEPINT will be cleared when DAINT's out bits are cleared.
  // /LOG("DAINT %08x\n", dev->DAINT);
  for ( uint8_t n = 0; n < EP_MAX; n++ )
  {
    xfer_ctl_t *xfer = XFER_CTL_BASE(n, TUSB_DIR_IN);

    if ( dev->DAINT & (1 << (USB_OTG_DAINT_IEPINT_Pos + n)) )
    {
      // LOG("DIEPINT %u %08x\n", n, in_ep[n].DIEPINT);
      // IN XFER complete (entire xfer).
      if ( in_ep[n].DIEPINT & USB_OTG_DIEPINT_XFRC )
      {
        in_ep[n].DIEPINT = USB_OTG_DIEPINT_XFRC;

        // EP0 can only handle one packet
        if((n == 0) && ep0_pending[TUSB_DIR_IN]) {
          // Schedule another packet to be transmitted.
          edpt_schedule_packets(rhport, n, TUSB_DIR_IN, 1, ep0_pending[TUSB_DIR_IN]);
        } else {
          // LOG("IN %02x done %u bytes\n", n | TUSB_DIR_IN_MASK, xfer->total_len);
          dcd_event_xfer_complete(rhport, n | TUSB_DIR_IN_MASK, xfer->total_len, XFER_RESULT_SUCCESS, true);
        }
      }

      // FIX ME proper
      // in_ep[n].DIEPINT = USB_OTG_DIEPINT_TOC;

      // // XFER FIFO empty
      // if ( (in_ep[n].DIEPINT & USB_OTG_DIEPINT_TXFE)) {
      //     TU_ASSERT(!xfer->ff,);
      //     // TU_ASSERT(xfer->total_len <= in_ep[n].DIEPS);

      //       //LOG("buffer\n");
      //       write_fifo_packet(rhport, n, xfer->buffer, xfer->total_len);

      //       // // Increment pointer to xfer data
      //       // xfer->buffer += packet_size;

      //       dev->DIEPEMPMSK &= ~(1 << n);
      // }

      // // XFER FIFO empty
      // if ( (in_ep[n].DIEPINT & USB_OTG_DIEPINT_TXFE) && (dev->DIEPEMPMSK & (1 << n)) )
      // {
        // DIEPINT's TXFE bit is read-only, software cannot clear it.
        // It will only be cleared by hardware when written bytes is more than
        // - 64 bytes or
        // - Half of TX FIFO size (configured by DIEPTXF)


        // uint16_t remaining_packets = (in_ep[n].DIEPTSIZ & USB_OTG_DIEPTSIZ_PKTCNT_Msk) >> USB_OTG_DIEPTSIZ_PKTCNT_Pos;




        // // Process every single packet (only whole packets can be written to fifo)
        // for(uint16_t i = 0; i < remaining_packets; i++)
        // {
        //   uint16_t const remaining_bytes = (in_ep[n].DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ_Msk) >> USB_OTG_DIEPTSIZ_XFRSIZ_Pos;

        //   // Packet can not be larger than ep max size
        //   uint16_t const packet_size = tu_min16(remaining_bytes, xfer->max_size);

        //   // LOG("IN rem ep=%02x, p=%u b=%u s=%u\n", n, remaining_packets, remaining_bytes, packet_size);

        //   // It's only possible to write full packets into FIFO. Therefore DTXFSTS register of current
        //   // EP has to be checked if the buffer can take another WHOLE packet
        //   if(packet_size > ((in_ep[n].DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) << 2)) {
        //     LOG("ep=%u packet size=%u left=%u\n", n, packet_size, ((in_ep[n].DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) << 2));
        //     break;
        //   }

        //   // Push packet to Tx-FIFO
        //   if (xfer->ff)
        //   {
        //     //LOG("ff\n");
        //     usb_fifo_t tx_fifo = FIFO_BASE(rhport, n);
        //     tu_fifo_read_n_const_addr_full_words(xfer->ff, (void *) tx_fifo, packet_size);
        //   }
        //   else
        //   {
        //     //LOG("buffer\n");
        //     write_fifo_packet(rhport, n, xfer->buffer, packet_size);

        //     // Increment pointer to xfer data
        //     xfer->buffer += packet_size;
        //   }
        // }

        // // Turn off TXFE if all bytes are written.
        // if (((in_ep[n].DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ_Msk) >> USB_OTG_DIEPTSIZ_XFRSIZ_Pos) == 0)
        // {
        //   dev->DIEPEMPMSK &= ~(1 << n);
        // }
      // }

    }
  }
}

// static uint32_t enter_count;

void dcd_int_handler(uint8_t rhport)
{
  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  uint32_t const int_status = usb_otg->GINTSTS & usb_otg->GINTMSK;

  // uint32_t ec = __atomic_fetch_add(&enter_count, 1, __ATOMIC_ACQ_REL);
  // if (ec > 1) {
  //   LOG("ec=%u\n", ec);
  // }

  // LOG("/ %08x\n", int_status);

  if(int_status & USB_OTG_GINTSTS_USBRST)
  {
    LOG("bus reset\n");
    // USBRST is start of reset.
    usb_otg->GINTSTS = USB_OTG_GINTSTS_USBRST;
    bus_reset(rhport);
  }

  if(int_status & USB_OTG_GINTSTS_ENUMDNE)
  {
    // ENUMDNE is the end of reset where speed of the link is detected

    usb_otg->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;

    tusb_speed_t const speed = get_speed(rhport);

    set_turnaround(usb_otg, speed);
    dcd_event_bus_reset(rhport, speed, true);
  }

  if(int_status & USB_OTG_GINTSTS_USBSUSP)
  {
    usb_otg->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
  }

  if(int_status & USB_OTG_GINTSTS_WKUINT)
  {
    usb_otg->GINTSTS = USB_OTG_GINTSTS_WKUINT;
    dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
  }

  // TODO check USB_OTG_GINTSTS_DISCINT for disconnect detection
  // if(int_status & USB_OTG_GINTSTS_DISCINT)

  if(int_status & USB_OTG_GINTSTS_OTGINT)
  {
    // OTG INT bit is read-only
    uint32_t const otg_int = usb_otg->GOTGINT;

    if (otg_int & USB_OTG_GOTGINT_SEDET)
    {
      dcd_event_bus_signal(rhport, DCD_EVENT_UNPLUGGED, true);
    }

    usb_otg->GOTGINT = otg_int;
  }

#if USE_SOF
  if(int_status & USB_OTG_GINTSTS_SOF)
  {
    usb_otg->GINTSTS = USB_OTG_GINTSTS_SOF;

    // Disable SOF interrupt since currently only used for remote wakeup detection
    usb_otg->GINTMSK &= ~USB_OTG_GINTMSK_SOFM;

    dcd_event_bus_signal(rhport, DCD_EVENT_SOF, true);
  }
#endif
  // RxFIFO non-empty interrupt handling.
  if(int_status & USB_OTG_GINTSTS_RXFLVL)
  {
    // RXFLVL bit is read-only

    // // Mask out RXFLVL while reading data from FIFO
    // usb_otg->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;

    // Loop until all available packets were handled
    handle_rxflvl_ints(rhport, out_ep);

#if !DCD_ST_SYN_CUSTOM_FIFO_SIZES
    // Manage RX FIFO size
    if (_out_ep_closed)
    {
      update_grxfsiz(rhport);

      // Disable flag
      _out_ep_closed = false;
    }
#endif
    // usb_otg->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
  }

  // OUT endpoint interrupt handling.
  if(int_status & USB_OTG_GINTSTS_OEPINT)
  {
    // OEPINT is read-only
    handle_epout_ints(rhport, dev, out_ep);
  }

  // IN endpoint interrupt handling.
  if(int_status & USB_OTG_GINTSTS_IEPINT)
  {
    // IEPINT bit read-only
    handle_epin_ints(rhport, dev, in_ep);
  }

  //  // Check for Incomplete isochronous IN transfer
  //  if(int_status & USB_OTG_GINTSTS_IISOIXFR) {
  //    printf("      IISOIXFR!\r\n");
  ////    TU_LOG2("      IISOIXFR!\r\n");
  //  }

  // __atomic_fetch_sub(&enter_count, 1, __ATOMIC_ACQ_REL);
}

#endif
