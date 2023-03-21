/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#ifndef D5035_05
#	error "Only include this file for D5035-05 boards"
#endif

#define HWREV 1

#ifndef HWREV
#	error "Define HWREV"
#endif

#define SC_BOARD_USB_BCD_DEVICE (HWREV << 8)
#define SC_BOARD_USB_MANUFACTURER_STRING "2guys"
#define SC_BOARD_CAN_COUNT 2
#define SC_BOARD_NAME "D5035-05"
#define SC_BOARD_CAN_CLK_HZ 64000000


enum {
	SC_BOARD_DEBUG_DEFAULT,
	LED_DEBUG_0,
	LED_DEBUG_1,
	LED_DEBUG_2,
	LED_DEBUG_3,
	LED_CAN0_STATUS_GREEN,
	LED_CAN0_STATUS_RED,
	LED_CAN1_STATUS_GREEN,
	LED_CAN1_STATUS_RED,
	SC_BOARD_LED_COUNT
};

#define CAN0_TRAFFIC_LED LED_DEBUG_1
#define CAN1_TRAFFIC_LED LED_DEBUG_2


#define sc_board_led_usb_burst() led_burst(LED_DEBUG_3, SC_LED_BURST_DURATION_MS)
#define sc_board_led_can_traffic_burst(index) \
	do { \
		switch (index) { \
		case 0: led_burst(CAN0_TRAFFIC_LED, SC_LED_BURST_DURATION_MS); break; \
		case 1: led_burst(CAN1_TRAFFIC_LED, SC_LED_BURST_DURATION_MS); break; \
		default: break; \
		} \
	} while (0)


SC_RAMFUNC extern void sc_board_led_can_status_set(uint8_t index, int status);

// #undef sc_board_name
// extern char const* sc_board_name(void);

#define sc_board_can_ts_request(index)
#define sc_board_can_ts_wait(index) (TIM2->CNT)

#include <stm32g0b1xx.h>

#define SUPERCAN_MCAN 1
#define MCAN_MESSAGE_RAM_CONFIGURABLE 0
#define MCAN_HW_RX_FIFO_SIZE 3
#define MCAN_HW_TX_FIFO_SIZE 3

#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef volatile       uint32_t RoReg;   /**< Read only 32-bit register (volatile const unsigned int) */
typedef volatile       uint16_t RoReg16; /**< Read only 16-bit register (volatile const unsigned int) */
typedef volatile       uint8_t  RoReg8;  /**< Read only  8-bit register (volatile const unsigned int) */
// typedef volatile       uint32_t WoReg;   /**< Write only 32-bit register (volatile unsigned int) */
// typedef volatile       uint16_t WoReg16; /**< Write only 16-bit register (volatile unsigned int) */
// typedef volatile       uint8_t  WoReg8;  /**< Write only  8-bit register (volatile unsigned int) */
// typedef volatile       uint32_t RwReg;   /**< Read-Write 32-bit register (volatile unsigned int) */
// typedef volatile       uint16_t RwReg16; /**< Read-Write 16-bit register (volatile unsigned int) */
// typedef volatile       uint8_t  RwReg8;  /**< Read-Write  8-bit register (volatile unsigned int) */
#endif


#include <supercan_mcan.h>


struct MCanX {
  __I  MCANX_CREL_Type             CREL;        /**< \brief Offset: 0x00 (R/  32) Core Release */
  __I  MCANX_ENDN_Type             ENDN;        /**< \brief Offset: 0x04 (R/  32) Endian */
       uint8_t                     Reserved0[0x4];
  __IO MCANX_DBTP_Type             DBTP;        /**< \brief Offset: 0x0C (R/W 32) Fast Bit Timing and Prescaler */
  __IO MCANX_TEST_Type             TEST;        /**< \brief Offset: 0x10 (R/W 32) Test */
  __IO MCANX_RWD_Type              RWD;         /**< \brief Offset: 0x14 (R/W 32) RAM Watchdog */
  __IO MCANX_CCCR_Type             CCCR;        /**< \brief Offset: 0x18 (R/W 32) CC Control */
  __IO MCANX_NBTP_Type             NBTP;        /**< \brief Offset: 0x1C (R/W 32) Nominal Bit Timing and Prescaler */
  __IO MCANX_TSCC_Type             TSCC;        /**< \brief Offset: 0x20 (R/W 32) Timestamp Counter Configuration */
  __I  MCANX_TSCV_Type             TSCV;        /**< \brief Offset: 0x24 (R/  32) Timestamp Counter Value */
  __IO MCANX_TOCC_Type             TOCC;        /**< \brief Offset: 0x28 (R/W 32) Timeout Counter Configuration */
  __IO MCANX_TOCV_Type             TOCV;        /**< \brief Offset: 0x2C (R/W 32) Timeout Counter Value */
       uint8_t                     Reserved1[0x10];
  __I  MCANX_ECR_Type              ECR;         /**< \brief Offset: 0x40 (R/  32) Error Counter */
  __I  MCANX_PSR_Type              PSR;         /**< \brief Offset: 0x44 (R/  32) Protocol Status */
  __IO MCANX_TDCR_Type             TDCR;        /**< \brief Offset: 0x48 (R/W 32) Extended ID Filter Configuration */
       uint8_t                     Reserved2[0x4];
  __IO MCANX_IR_Type               IR;          /**< \brief Offset: 0x50 (R/W 32) Interrupt */
  __IO MCANX_IE_Type               IE;          /**< \brief Offset: 0x54 (R/W 32) Interrupt Enable */
  __IO MCANX_ILS_Type              ILS;         /**< \brief Offset: 0x58 (R/W 32) Interrupt Line Select */
  __IO MCANX_ILE_Type              ILE;         /**< \brief Offset: 0x5C (R/W 32) Interrupt Line Enable */
       uint8_t                     Reserved3[0x20];
  __IO MCANX_GFC_Type              GFC;         /**< \brief Offset: 0x80 (R/W 32) Global Filter Configuration */
  __IO MCANX_XIDAM_Type            XIDAM;       /**< \brief Offset: 0x84 (R/W 32) Extended ID AND Mask */
  __I  MCANX_HPMS_Type             HPMS;        /**< \brief Offset: 0x88 (R/  32) High Priority Message Status */
       uint8_t                     Reserved31[0x4];
  __I  MCANX_RXF0S_Type            RXF0S;       /**< \brief Offset: 0x90 (R/  32) Rx FIFO 0 Status */
  __IO MCANX_RXF0A_Type            RXF0A;       /**< \brief Offset: 0x94 (R/W 32) Rx FIFO 0 Acknowledge */
  __I  MCANX_RXF1S_Type            RXF1S;       /**< \brief Offset: 0x98 (R/  32) Rx FIFO 1 Status */
  __IO MCANX_RXF1A_Type            RXF1A;       /**< \brief Offset: 0x9C (R/W 32) Rx FIFO 1 Acknowledge */
       uint8_t                     Reserved4[0x20];
  __IO MCANX_TXBC_Type             TXBC;        /**< \brief Offset: 0xC0 (R/W 32) Tx Buffer Configuration */
  __I  MCANX_TXFQS_Type            TXFQS;       /**< \brief Offset: 0xC4 (R/  32) Tx FIFO / Queue Status */
  __I  MCANX_TXBRP_Type            TXBRP;       /**< \brief Offset: 0xC8 (R/  32) Tx Buffer Request Pending */
  __IO MCANX_TXBAR_Type            TXBAR;       /**< \brief Offset: 0xCC (R/W 32) Tx Buffer Add Request */
  __IO MCANX_TXBCR_Type            TXBCR;       /**< \brief Offset: 0xD0 (R/W 32) Tx Buffer Cancellation Request */
  __I  MCANX_TXBTO_Type            TXBTO;       /**< \brief Offset: 0xD4 (R/  32) Tx Buffer Transmission Occurred */
  __I  MCANX_TXBCF_Type            TXBCF;       /**< \brief Offset: 0xD8 (R/  32) Tx Buffer Cancellation Finished */
  __IO MCANX_TXBTIE_Type           TXBTIE;      /**< \brief Offset: 0xDC (R/W 32) Tx Buffer Transmission Interrupt Enable */
  __IO MCANX_TXBCIE_Type           TXBCIE;      /**< \brief Offset: 0xE0 (R/W 32) Tx Buffer Cancellation Finished Interrupt Enable */
  __I  MCANX_TXEFS_Type            TXEFS;       /**< \brief Offset: 0xE4 (R/  32) Tx Event FIFO Status */
  __IO MCANX_TXEFA_Type            TXEFA;       /**< \brief Offset: 0xE8 (R/W 32) Tx Event FIFO Acknowledge */
       uint8_t                     Reserved5[0x14];
  __IO uint32_t                    CKDIV;       /**< \brief Offset: 0x100 (R/W 32) FDCAN CFG clock divider register (FDCAN_CKDIV) */
};

