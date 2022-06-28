/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
 *
 */


#include <sllin_board.h>

#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// defined by compiler flags for flexibility
#ifndef CFG_TUSB_MCU
  #error CFG_TUSB_MCU must be defined
#endif

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE

#define CFG_TUSB_OS OPT_OS_FREERTOS


// CFG_TUSB_DEBUG is defined by compiler in DEBUG build
// #define CFG_TUSB_DEBUG           0


/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#define CFG_TUSB_SOF_CALLBACK     0
#define CFG_TUSB_LEAN_AND_MEAN    0

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUD_ENDPOINT0_SIZE    64

//------------- CLASS -------------//
#define CFG_TUD_CDC               SLLIN_BOARD_LIN_COUNT
#define CFG_TUD_DFU_RUNTIME       SUPERDFU_APP

#define CFG_TUSB_ENDPOINT_LIMIT   5

#if D5035_51
  // CDC FIFO size of TX and RX
  #define CFG_TUD_CDC_RX_BUFSIZE   (64)
  #define CFG_TUD_CDC_TX_BUFSIZE   (64)
  // CDC Endpoint transfer buffer size, more is faster
  #define CFG_TUD_CDC_EP_BUFSIZE   (64)
#else
  // CDC FIFO size of TX and RX
  #define CFG_TUD_CDC_RX_BUFSIZE   (512)
  #define CFG_TUD_CDC_TX_BUFSIZE   (512)
  // CDC Endpoint transfer buffer size, more is faster
  #define CFG_TUD_CDC_EP_BUFSIZE   (512)
#endif




#ifdef __cplusplus
} // extern "C" {
#endif

