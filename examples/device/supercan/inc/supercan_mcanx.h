/**
 * \file
 *
 * \brief Component description for CAN
 *
 * Copyright (c) 2017 Microchip Technology Inc.
 *
 * \asf_license_start
 *
 * \page License
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the Licence at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * \asf_license_stop
 *
 * Adapted for use in SuperCAN (renamed symbols). Thiese changes are under
 * PDX-License-Identifier: MIT
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */
#pragma once

#if !defined(SKIP_INTEGER_LITERALS)
#if defined(_U_) || defined(_L_) || defined(_UL_)
  #error "Integer Literals macros already defined elsewhere"
#endif

#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
/* Macros that deal with adding suffixes to integer literal constants for C/C++ */
#define _U_(x)         x ## U            /**< C code: Unsigned integer literal constant value */
#define _L_(x)         x ## L            /**< C code: Long integer literal constant value */
#define _UL_(x)        x ## UL           /**< C code: Unsigned Long integer literal constant value */
#else /* Assembler */
#define _U_(x)         x                 /**< Assembler: Unsigned integer literal constant value */
#define _L_(x)         x                 /**< Assembler: Long integer literal constant value */
#define _UL_(x)        x                 /**< Assembler: Unsigned Long integer literal constant value */
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* SKIP_INTEGER_LITERALS */

#define MCANX_U2003
#define REV_CAN                     0x321

/* -------- MCANX_CREL : (CAN Offset: 0x00) (R/  32) Core Release -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t :20;              /*!< bit:  0..19  Reserved                           */
    uint32_t SUBSTEP:4;        /*!< bit: 20..23  Sub-step of Core Release           */
    uint32_t STEP:4;           /*!< bit: 24..27  Step of Core Release               */
    uint32_t REL:4;            /*!< bit: 28..31  Core Release                       */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_CREL_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_CREL_OFFSET             0x00         /**< \brief (MCANX_CREL offset) Core Release */
#define MCANX_CREL_RESETVALUE         _U_(0x32100000) /**< \brief (MCANX_CREL reset_value) Core Release */

#define MCANX_CREL_SUBSTEP_Pos        20           /**< \brief (MCANX_CREL) Sub-step of Core Release */
#define MCANX_CREL_SUBSTEP_Msk        (_U_(0xF) << MCANX_CREL_SUBSTEP_Pos)
#define MCANX_CREL_SUBSTEP(value)     (MCANX_CREL_SUBSTEP_Msk & ((value) << MCANX_CREL_SUBSTEP_Pos))
#define MCANX_CREL_STEP_Pos           24           /**< \brief (MCANX_CREL) Step of Core Release */
#define MCANX_CREL_STEP_Msk           (_U_(0xF) << MCANX_CREL_STEP_Pos)
#define MCANX_CREL_STEP(value)        (MCANX_CREL_STEP_Msk & ((value) << MCANX_CREL_STEP_Pos))
#define MCANX_CREL_REL_Pos            28           /**< \brief (MCANX_CREL) Core Release */
#define MCANX_CREL_REL_Msk            (_U_(0xF) << MCANX_CREL_REL_Pos)
#define MCANX_CREL_REL(value)         (MCANX_CREL_REL_Msk & ((value) << MCANX_CREL_REL_Pos))
#define MCANX_CREL_MASK               _U_(0xFFF00000) /**< \brief (MCANX_CREL) MASK Register */

/* -------- MCANX_ENDN : (CAN Offset: 0x04) (R/  32) Endian -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t ETV:32;           /*!< bit:  0..31  Endianness Test Value              */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_ENDN_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_ENDN_OFFSET             0x04         /**< \brief (MCANX_ENDN offset) Endian */
#define MCANX_ENDN_RESETVALUE         _U_(0x87654321) /**< \brief (MCANX_ENDN reset_value) Endian */

#define MCANX_ENDN_ETV_Pos            0            /**< \brief (MCANX_ENDN) Endianness Test Value */
#define MCANX_ENDN_ETV_Msk            (_U_(0xFFFFFFFF) << MCANX_ENDN_ETV_Pos)
#define MCANX_ENDN_ETV(value)         (MCANX_ENDN_ETV_Msk & ((value) << MCANX_ENDN_ETV_Pos))
#define MCANX_ENDN_MASK               _U_(0xFFFFFFFF) /**< \brief (MCANX_ENDN) MASK Register */

/* -------- MCANX_MRCFG : (CAN Offset: 0x08) (R/W 32) Message RAM Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t QOS:2;            /*!< bit:  0.. 1  Quality of Service                 */
    uint32_t :30;              /*!< bit:  2..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_MRCFG_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_MRCFG_OFFSET            0x08         /**< \brief (MCANX_MRCFG offset) Message RAM Configuration */
#define MCANX_MRCFG_RESETVALUE        _U_(0x00000002) /**< \brief (MCANX_MRCFG reset_value) Message RAM Configuration */

#define MCANX_MRCFG_QOS_Pos           0            /**< \brief (MCANX_MRCFG) Quality of Service */
#define MCANX_MRCFG_QOS_Msk           (_U_(0x3) << MCANX_MRCFG_QOS_Pos)
#define MCANX_MRCFG_QOS(value)        (MCANX_MRCFG_QOS_Msk & ((value) << MCANX_MRCFG_QOS_Pos))
#define   MCANX_MRCFG_QOS_DISABLE_Val       _U_(0x0)   /**< \brief (MCANX_MRCFG) Background (no sensitive operation) */
#define   MCANX_MRCFG_QOS_LOW_Val           _U_(0x1)   /**< \brief (MCANX_MRCFG) Sensitive Bandwidth */
#define   MCANX_MRCFG_QOS_MEDIUM_Val        _U_(0x2)   /**< \brief (MCANX_MRCFG) Sensitive Latency */
#define   MCANX_MRCFG_QOS_HIGH_Val          _U_(0x3)   /**< \brief (MCANX_MRCFG) Critical Latency */
#define MCANX_MRCFG_QOS_DISABLE       (MCANX_MRCFG_QOS_DISABLE_Val     << MCANX_MRCFG_QOS_Pos)
#define MCANX_MRCFG_QOS_LOW           (MCANX_MRCFG_QOS_LOW_Val         << MCANX_MRCFG_QOS_Pos)
#define MCANX_MRCFG_QOS_MEDIUM        (MCANX_MRCFG_QOS_MEDIUM_Val      << MCANX_MRCFG_QOS_Pos)
#define MCANX_MRCFG_QOS_HIGH          (MCANX_MRCFG_QOS_HIGH_Val        << MCANX_MRCFG_QOS_Pos)
#define MCANX_MRCFG_MASK              _U_(0x00000003) /**< \brief (MCANX_MRCFG) MASK Register */

/* -------- MCANX_DBTP : (CAN Offset: 0x0C) (R/W 32) Fast Bit Timing and Prescaler -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t DSJW:4;           /*!< bit:  0.. 3  Data (Re)Synchronization Jump Width */
    uint32_t DTSEG2:4;         /*!< bit:  4.. 7  Data time segment after sample point */
    uint32_t DTSEG1:5;         /*!< bit:  8..12  Data time segment before sample point */
    uint32_t :3;               /*!< bit: 13..15  Reserved                           */
    uint32_t DBRP:5;           /*!< bit: 16..20  Data Baud Rate Prescaler           */
    uint32_t :2;               /*!< bit: 21..22  Reserved                           */
    uint32_t TDC:1;            /*!< bit:     23  Tranceiver Delay Compensation      */
    uint32_t :8;               /*!< bit: 24..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_DBTP_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_DBTP_OFFSET             0x0C         /**< \brief (MCANX_DBTP offset) Fast Bit Timing and Prescaler */
#define MCANX_DBTP_RESETVALUE         _U_(0x00000A33) /**< \brief (MCANX_DBTP reset_value) Fast Bit Timing and Prescaler */

#define MCANX_DBTP_DSJW_Pos           0            /**< \brief (MCANX_DBTP) Data (Re)Synchronization Jump Width */
#define MCANX_DBTP_DSJW_Msk           (_U_(0xF) << MCANX_DBTP_DSJW_Pos)
#define MCANX_DBTP_DSJW(value)        (MCANX_DBTP_DSJW_Msk & ((value) << MCANX_DBTP_DSJW_Pos))
#define MCANX_DBTP_DTSEG2_Pos         4            /**< \brief (MCANX_DBTP) Data time segment after sample point */
#define MCANX_DBTP_DTSEG2_Msk         (_U_(0xF) << MCANX_DBTP_DTSEG2_Pos)
#define MCANX_DBTP_DTSEG2(value)      (MCANX_DBTP_DTSEG2_Msk & ((value) << MCANX_DBTP_DTSEG2_Pos))
#define MCANX_DBTP_DTSEG1_Pos         8            /**< \brief (MCANX_DBTP) Data time segment before sample point */
#define MCANX_DBTP_DTSEG1_Msk         (_U_(0x1F) << MCANX_DBTP_DTSEG1_Pos)
#define MCANX_DBTP_DTSEG1(value)      (MCANX_DBTP_DTSEG1_Msk & ((value) << MCANX_DBTP_DTSEG1_Pos))
#define MCANX_DBTP_DBRP_Pos           16           /**< \brief (MCANX_DBTP) Data Baud Rate Prescaler */
#define MCANX_DBTP_DBRP_Msk           (_U_(0x1F) << MCANX_DBTP_DBRP_Pos)
#define MCANX_DBTP_DBRP(value)        (MCANX_DBTP_DBRP_Msk & ((value) << MCANX_DBTP_DBRP_Pos))
#define MCANX_DBTP_TDC_Pos            23           /**< \brief (MCANX_DBTP) Tranceiver Delay Compensation */
#define MCANX_DBTP_TDC                (_U_(0x1) << MCANX_DBTP_TDC_Pos)
#define MCANX_DBTP_MASK               _U_(0x009F1FFF) /**< \brief (MCANX_DBTP) MASK Register */

/* -------- MCANX_TEST : (CAN Offset: 0x10) (R/W 32) Test -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t :4;               /*!< bit:  0.. 3  Reserved                           */
    uint32_t LBCK:1;           /*!< bit:      4  Loop Back Mode                     */
    uint32_t TX:2;             /*!< bit:  5.. 6  Control of Transmit Pin            */
    uint32_t RX:1;             /*!< bit:      7  Receive Pin                        */
    uint32_t :24;              /*!< bit:  8..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TEST_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TEST_OFFSET             0x10         /**< \brief (MCANX_TEST offset) Test */
#define MCANX_TEST_RESETVALUE         _U_(0x00000000) /**< \brief (MCANX_TEST reset_value) Test */

#define MCANX_TEST_LBCK_Pos           4            /**< \brief (MCANX_TEST) Loop Back Mode */
#define MCANX_TEST_LBCK               (_U_(0x1) << MCANX_TEST_LBCK_Pos)
#define MCANX_TEST_TX_Pos             5            /**< \brief (MCANX_TEST) Control of Transmit Pin */
#define MCANX_TEST_TX_Msk             (_U_(0x3) << MCANX_TEST_TX_Pos)
#define MCANX_TEST_TX(value)          (MCANX_TEST_TX_Msk & ((value) << MCANX_TEST_TX_Pos))
#define   MCANX_TEST_TX_CORE_Val            _U_(0x0)   /**< \brief (MCANX_TEST) TX controlled by CAN core */
#define   MCANX_TEST_TX_SAMPLE_Val          _U_(0x1)   /**< \brief (MCANX_TEST) TX monitoring sample point */
#define   MCANX_TEST_TX_DOMINANT_Val        _U_(0x2)   /**< \brief (MCANX_TEST) Dominant (0) level at pin MCANX_TX */
#define   MCANX_TEST_TX_RECESSIVE_Val       _U_(0x3)   /**< \brief (MCANX_TEST) Recessive (1) level at pin MCANX_TX */
#define MCANX_TEST_TX_CORE            (MCANX_TEST_TX_CORE_Val          << MCANX_TEST_TX_Pos)
#define MCANX_TEST_TX_SAMPLE          (MCANX_TEST_TX_SAMPLE_Val        << MCANX_TEST_TX_Pos)
#define MCANX_TEST_TX_DOMINANT        (MCANX_TEST_TX_DOMINANT_Val      << MCANX_TEST_TX_Pos)
#define MCANX_TEST_TX_RECESSIVE       (MCANX_TEST_TX_RECESSIVE_Val     << MCANX_TEST_TX_Pos)
#define MCANX_TEST_RX_Pos             7            /**< \brief (MCANX_TEST) Receive Pin */
#define MCANX_TEST_RX                 (_U_(0x1) << MCANX_TEST_RX_Pos)
#define MCANX_TEST_MASK               _U_(0x000000F0) /**< \brief (MCANX_TEST) MASK Register */

/* -------- MCANX_RWD : (CAN Offset: 0x14) (R/W 32) RAM Watchdog -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t WDC:8;            /*!< bit:  0.. 7  Watchdog Configuration             */
    uint32_t WDV:8;            /*!< bit:  8..15  Watchdog Value                     */
    uint32_t :16;              /*!< bit: 16..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RWD_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RWD_OFFSET              0x14         /**< \brief (MCANX_RWD offset) RAM Watchdog */
#define MCANX_RWD_RESETVALUE          _U_(0x00000000) /**< \brief (MCANX_RWD reset_value) RAM Watchdog */

#define MCANX_RWD_WDC_Pos             0            /**< \brief (MCANX_RWD) Watchdog Configuration */
#define MCANX_RWD_WDC_Msk             (_U_(0xFF) << MCANX_RWD_WDC_Pos)
#define MCANX_RWD_WDC(value)          (MCANX_RWD_WDC_Msk & ((value) << MCANX_RWD_WDC_Pos))
#define MCANX_RWD_WDV_Pos             8            /**< \brief (MCANX_RWD) Watchdog Value */
#define MCANX_RWD_WDV_Msk             (_U_(0xFF) << MCANX_RWD_WDV_Pos)
#define MCANX_RWD_WDV(value)          (MCANX_RWD_WDV_Msk & ((value) << MCANX_RWD_WDV_Pos))
#define MCANX_RWD_MASK                _U_(0x0000FFFF) /**< \brief (MCANX_RWD) MASK Register */

/* -------- MCANX_CCCR : (CAN Offset: 0x18) (R/W 32) CC Control -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t INIT:1;           /*!< bit:      0  Initialization                     */
    uint32_t CCE:1;            /*!< bit:      1  Configuration Change Enable        */
    uint32_t ASM:1;            /*!< bit:      2  ASM Restricted Operation Mode      */
    uint32_t CSA:1;            /*!< bit:      3  Clock Stop Acknowledge             */
    uint32_t CSR:1;            /*!< bit:      4  Clock Stop Request                 */
    uint32_t MON:1;            /*!< bit:      5  Bus Monitoring Mode                */
    uint32_t DAR:1;            /*!< bit:      6  Disable Automatic Retransmission   */
    uint32_t TEST:1;           /*!< bit:      7  Test Mode Enable                   */
    uint32_t FDOE:1;           /*!< bit:      8  FD Operation Enable                */
    uint32_t BRSE:1;           /*!< bit:      9  Bit Rate Switch Enable             */
    uint32_t :2;               /*!< bit: 10..11  Reserved                           */
    uint32_t PXHD:1;           /*!< bit:     12  Protocol Exception Handling Disable */
    uint32_t EFBI:1;           /*!< bit:     13  Edge Filtering during Bus Integration */
    uint32_t TXP:1;            /*!< bit:     14  Transmit Pause                     */
    uint32_t NISO:1;           /*!< bit:     15  Non ISO Operation                  */
    uint32_t :16;              /*!< bit: 16..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_CCCR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_CCCR_OFFSET             0x18         /**< \brief (MCANX_CCCR offset) CC Control */
#define MCANX_CCCR_RESETVALUE         _U_(0x00000001) /**< \brief (MCANX_CCCR reset_value) CC Control */

#define MCANX_CCCR_INIT_Pos           0            /**< \brief (MCANX_CCCR) Initialization */
#define MCANX_CCCR_INIT               (_U_(0x1) << MCANX_CCCR_INIT_Pos)
#define MCANX_CCCR_CCE_Pos            1            /**< \brief (MCANX_CCCR) Configuration Change Enable */
#define MCANX_CCCR_CCE                (_U_(0x1) << MCANX_CCCR_CCE_Pos)
#define MCANX_CCCR_ASM_Pos            2            /**< \brief (MCANX_CCCR) ASM Restricted Operation Mode */
#define MCANX_CCCR_ASM                (_U_(0x1) << MCANX_CCCR_ASM_Pos)
#define MCANX_CCCR_CSA_Pos            3            /**< \brief (MCANX_CCCR) Clock Stop Acknowledge */
#define MCANX_CCCR_CSA                (_U_(0x1) << MCANX_CCCR_CSA_Pos)
#define MCANX_CCCR_CSR_Pos            4            /**< \brief (MCANX_CCCR) Clock Stop Request */
#define MCANX_CCCR_CSR                (_U_(0x1) << MCANX_CCCR_CSR_Pos)
#define MCANX_CCCR_MON_Pos            5            /**< \brief (MCANX_CCCR) Bus Monitoring Mode */
#define MCANX_CCCR_MON                (_U_(0x1) << MCANX_CCCR_MON_Pos)
#define MCANX_CCCR_DAR_Pos            6            /**< \brief (MCANX_CCCR) Disable Automatic Retransmission */
#define MCANX_CCCR_DAR                (_U_(0x1) << MCANX_CCCR_DAR_Pos)
#define MCANX_CCCR_TEST_Pos           7            /**< \brief (MCANX_CCCR) Test Mode Enable */
#define MCANX_CCCR_TEST               (_U_(0x1) << MCANX_CCCR_TEST_Pos)
#define MCANX_CCCR_FDOE_Pos           8            /**< \brief (MCANX_CCCR) FD Operation Enable */
#define MCANX_CCCR_FDOE               (_U_(0x1) << MCANX_CCCR_FDOE_Pos)
#define MCANX_CCCR_BRSE_Pos           9            /**< \brief (MCANX_CCCR) Bit Rate Switch Enable */
#define MCANX_CCCR_BRSE               (_U_(0x1) << MCANX_CCCR_BRSE_Pos)
#define MCANX_CCCR_PXHD_Pos           12           /**< \brief (MCANX_CCCR) Protocol Exception Handling Disable */
#define MCANX_CCCR_PXHD               (_U_(0x1) << MCANX_CCCR_PXHD_Pos)
#define MCANX_CCCR_EFBI_Pos           13           /**< \brief (MCANX_CCCR) Edge Filtering during Bus Integration */
#define MCANX_CCCR_EFBI               (_U_(0x1) << MCANX_CCCR_EFBI_Pos)
#define MCANX_CCCR_TXP_Pos            14           /**< \brief (MCANX_CCCR) Transmit Pause */
#define MCANX_CCCR_TXP                (_U_(0x1) << MCANX_CCCR_TXP_Pos)
#define MCANX_CCCR_NISO_Pos           15           /**< \brief (MCANX_CCCR) Non ISO Operation */
#define MCANX_CCCR_NISO               (_U_(0x1) << MCANX_CCCR_NISO_Pos)
#define MCANX_CCCR_MASK               _U_(0x0000F3FF) /**< \brief (MCANX_CCCR) MASK Register */

/* -------- MCANX_NBTP : (CAN Offset: 0x1C) (R/W 32) Nominal Bit Timing and Prescaler -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t NTSEG2:7;         /*!< bit:  0.. 6  Nominal Time segment after sample point */
    uint32_t :1;               /*!< bit:      7  Reserved                           */
    uint32_t NTSEG1:8;         /*!< bit:  8..15  Nominal Time segment before sample point */
    uint32_t NBRP:9;           /*!< bit: 16..24  Nominal Baud Rate Prescaler        */
    uint32_t NSJW:7;           /*!< bit: 25..31  Nominal (Re)Synchronization Jump Width */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_NBTP_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_NBTP_OFFSET             0x1C         /**< \brief (MCANX_NBTP offset) Nominal Bit Timing and Prescaler */
#define MCANX_NBTP_RESETVALUE         _U_(0x06000A03) /**< \brief (MCANX_NBTP reset_value) Nominal Bit Timing and Prescaler */

#define MCANX_NBTP_NTSEG2_Pos         0            /**< \brief (MCANX_NBTP) Nominal Time segment after sample point */
#define MCANX_NBTP_NTSEG2_Msk         (_U_(0x7F) << MCANX_NBTP_NTSEG2_Pos)
#define MCANX_NBTP_NTSEG2(value)      (MCANX_NBTP_NTSEG2_Msk & ((value) << MCANX_NBTP_NTSEG2_Pos))
#define MCANX_NBTP_NTSEG1_Pos         8            /**< \brief (MCANX_NBTP) Nominal Time segment before sample point */
#define MCANX_NBTP_NTSEG1_Msk         (_U_(0xFF) << MCANX_NBTP_NTSEG1_Pos)
#define MCANX_NBTP_NTSEG1(value)      (MCANX_NBTP_NTSEG1_Msk & ((value) << MCANX_NBTP_NTSEG1_Pos))
#define MCANX_NBTP_NBRP_Pos           16           /**< \brief (MCANX_NBTP) Nominal Baud Rate Prescaler */
#define MCANX_NBTP_NBRP_Msk           (_U_(0x1FF) << MCANX_NBTP_NBRP_Pos)
#define MCANX_NBTP_NBRP(value)        (MCANX_NBTP_NBRP_Msk & ((value) << MCANX_NBTP_NBRP_Pos))
#define MCANX_NBTP_NSJW_Pos           25           /**< \brief (MCANX_NBTP) Nominal (Re)Synchronization Jump Width */
#define MCANX_NBTP_NSJW_Msk           (_U_(0x7F) << MCANX_NBTP_NSJW_Pos)
#define MCANX_NBTP_NSJW(value)        (MCANX_NBTP_NSJW_Msk & ((value) << MCANX_NBTP_NSJW_Pos))
#define MCANX_NBTP_MASK               _U_(0xFFFFFF7F) /**< \brief (MCANX_NBTP) MASK Register */

/* -------- MCANX_TSCC : (CAN Offset: 0x20) (R/W 32) Timestamp Counter Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TSS:2;            /*!< bit:  0.. 1  Timestamp Select                   */
    uint32_t :14;              /*!< bit:  2..15  Reserved                           */
    uint32_t TCP:4;            /*!< bit: 16..19  Timestamp Counter Prescaler        */
    uint32_t :12;              /*!< bit: 20..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TSCC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TSCC_OFFSET             0x20         /**< \brief (MCANX_TSCC offset) Timestamp Counter Configuration */
#define MCANX_TSCC_RESETVALUE         _U_(0x00000000) /**< \brief (MCANX_TSCC reset_value) Timestamp Counter Configuration */

#define MCANX_TSCC_TSS_Pos            0            /**< \brief (MCANX_TSCC) Timestamp Select */
#define MCANX_TSCC_TSS_Msk            (_U_(0x3) << MCANX_TSCC_TSS_Pos)
#define MCANX_TSCC_TSS(value)         (MCANX_TSCC_TSS_Msk & ((value) << MCANX_TSCC_TSS_Pos))
#define   MCANX_TSCC_TSS_ZERO_Val           _U_(0x0)   /**< \brief (MCANX_TSCC) Timestamp counter value always 0x0000 */
#define   MCANX_TSCC_TSS_INC_Val            _U_(0x1)   /**< \brief (MCANX_TSCC) Timestamp counter value incremented by TCP */
#define   MCANX_TSCC_TSS_EXT_Val            _U_(0x2)   /**< \brief (MCANX_TSCC) External timestamp counter value used */
#define MCANX_TSCC_TSS_ZERO           (MCANX_TSCC_TSS_ZERO_Val         << MCANX_TSCC_TSS_Pos)
#define MCANX_TSCC_TSS_INC            (MCANX_TSCC_TSS_INC_Val          << MCANX_TSCC_TSS_Pos)
#define MCANX_TSCC_TSS_EXT            (MCANX_TSCC_TSS_EXT_Val          << MCANX_TSCC_TSS_Pos)
#define MCANX_TSCC_TCP_Pos            16           /**< \brief (MCANX_TSCC) Timestamp Counter Prescaler */
#define MCANX_TSCC_TCP_Msk            (_U_(0xF) << MCANX_TSCC_TCP_Pos)
#define MCANX_TSCC_TCP(value)         (MCANX_TSCC_TCP_Msk & ((value) << MCANX_TSCC_TCP_Pos))
#define MCANX_TSCC_MASK               _U_(0x000F0003) /**< \brief (MCANX_TSCC) MASK Register */

/* -------- MCANX_TSCV : (CAN Offset: 0x24) (R/  32) Timestamp Counter Value -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TSC:16;           /*!< bit:  0..15  Timestamp Counter                  */
    uint32_t :16;              /*!< bit: 16..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TSCV_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TSCV_OFFSET             0x24         /**< \brief (MCANX_TSCV offset) Timestamp Counter Value */
#define MCANX_TSCV_RESETVALUE         _U_(0x00000000) /**< \brief (MCANX_TSCV reset_value) Timestamp Counter Value */

#define MCANX_TSCV_TSC_Pos            0            /**< \brief (MCANX_TSCV) Timestamp Counter */
#define MCANX_TSCV_TSC_Msk            (_U_(0xFFFF) << MCANX_TSCV_TSC_Pos)
#define MCANX_TSCV_TSC(value)         (MCANX_TSCV_TSC_Msk & ((value) << MCANX_TSCV_TSC_Pos))
#define MCANX_TSCV_MASK               _U_(0x0000FFFF) /**< \brief (MCANX_TSCV) MASK Register */

/* -------- MCANX_TOCC : (CAN Offset: 0x28) (R/W 32) Timeout Counter Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t ETOC:1;           /*!< bit:      0  Enable Timeout Counter             */
    uint32_t TOS:2;            /*!< bit:  1.. 2  Timeout Select                     */
    uint32_t :13;              /*!< bit:  3..15  Reserved                           */
    uint32_t TOP:16;           /*!< bit: 16..31  Timeout Period                     */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TOCC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TOCC_OFFSET             0x28         /**< \brief (MCANX_TOCC offset) Timeout Counter Configuration */
#define MCANX_TOCC_RESETVALUE         _U_(0xFFFF0000) /**< \brief (MCANX_TOCC reset_value) Timeout Counter Configuration */

#define MCANX_TOCC_ETOC_Pos           0            /**< \brief (MCANX_TOCC) Enable Timeout Counter */
#define MCANX_TOCC_ETOC               (_U_(0x1) << MCANX_TOCC_ETOC_Pos)
#define MCANX_TOCC_TOS_Pos            1            /**< \brief (MCANX_TOCC) Timeout Select */
#define MCANX_TOCC_TOS_Msk            (_U_(0x3) << MCANX_TOCC_TOS_Pos)
#define MCANX_TOCC_TOS(value)         (MCANX_TOCC_TOS_Msk & ((value) << MCANX_TOCC_TOS_Pos))
#define   MCANX_TOCC_TOS_CONT_Val           _U_(0x0)   /**< \brief (MCANX_TOCC) Continuout operation */
#define   MCANX_TOCC_TOS_TXEF_Val           _U_(0x1)   /**< \brief (MCANX_TOCC) Timeout controlled by TX Event FIFO */
#define   MCANX_TOCC_TOS_RXF0_Val           _U_(0x2)   /**< \brief (MCANX_TOCC) Timeout controlled by Rx FIFO 0 */
#define   MCANX_TOCC_TOS_RXF1_Val           _U_(0x3)   /**< \brief (MCANX_TOCC) Timeout controlled by Rx FIFO 1 */
#define MCANX_TOCC_TOS_CONT           (MCANX_TOCC_TOS_CONT_Val         << MCANX_TOCC_TOS_Pos)
#define MCANX_TOCC_TOS_TXEF           (MCANX_TOCC_TOS_TXEF_Val         << MCANX_TOCC_TOS_Pos)
#define MCANX_TOCC_TOS_RXF0           (MCANX_TOCC_TOS_RXF0_Val         << MCANX_TOCC_TOS_Pos)
#define MCANX_TOCC_TOS_RXF1           (MCANX_TOCC_TOS_RXF1_Val         << MCANX_TOCC_TOS_Pos)
#define MCANX_TOCC_TOP_Pos            16           /**< \brief (MCANX_TOCC) Timeout Period */
#define MCANX_TOCC_TOP_Msk            (_U_(0xFFFF) << MCANX_TOCC_TOP_Pos)
#define MCANX_TOCC_TOP(value)         (MCANX_TOCC_TOP_Msk & ((value) << MCANX_TOCC_TOP_Pos))
#define MCANX_TOCC_MASK               _U_(0xFFFF0007) /**< \brief (MCANX_TOCC) MASK Register */

/* -------- MCANX_TOCV : (CAN Offset: 0x2C) (R/W 32) Timeout Counter Value -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TOC:16;           /*!< bit:  0..15  Timeout Counter                    */
    uint32_t :16;              /*!< bit: 16..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TOCV_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TOCV_OFFSET             0x2C         /**< \brief (MCANX_TOCV offset) Timeout Counter Value */
#define MCANX_TOCV_RESETVALUE         _U_(0x0000FFFF) /**< \brief (MCANX_TOCV reset_value) Timeout Counter Value */

#define MCANX_TOCV_TOC_Pos            0            /**< \brief (MCANX_TOCV) Timeout Counter */
#define MCANX_TOCV_TOC_Msk            (_U_(0xFFFF) << MCANX_TOCV_TOC_Pos)
#define MCANX_TOCV_TOC(value)         (MCANX_TOCV_TOC_Msk & ((value) << MCANX_TOCV_TOC_Pos))
#define MCANX_TOCV_MASK               _U_(0x0000FFFF) /**< \brief (MCANX_TOCV) MASK Register */

/* -------- MCANX_ECR : (CAN Offset: 0x40) (R/  32) Error Counter -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TEC:8;            /*!< bit:  0.. 7  Transmit Error Counter             */
    uint32_t REC:7;            /*!< bit:  8..14  Receive Error Counter              */
    uint32_t RP:1;             /*!< bit:     15  Receive Error Passive              */
    uint32_t CEL:8;            /*!< bit: 16..23  CAN Error Logging                  */
    uint32_t :8;               /*!< bit: 24..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_ECR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_ECR_OFFSET              0x40         /**< \brief (MCANX_ECR offset) Error Counter */
#define MCANX_ECR_RESETVALUE          _U_(0x00000000) /**< \brief (MCANX_ECR reset_value) Error Counter */

#define MCANX_ECR_TEC_Pos             0            /**< \brief (MCANX_ECR) Transmit Error Counter */
#define MCANX_ECR_TEC_Msk             (_U_(0xFF) << MCANX_ECR_TEC_Pos)
#define MCANX_ECR_TEC(value)          (MCANX_ECR_TEC_Msk & ((value) << MCANX_ECR_TEC_Pos))
#define MCANX_ECR_REC_Pos             8            /**< \brief (MCANX_ECR) Receive Error Counter */
#define MCANX_ECR_REC_Msk             (_U_(0x7F) << MCANX_ECR_REC_Pos)
#define MCANX_ECR_REC(value)          (MCANX_ECR_REC_Msk & ((value) << MCANX_ECR_REC_Pos))
#define MCANX_ECR_RP_Pos              15           /**< \brief (MCANX_ECR) Receive Error Passive */
#define MCANX_ECR_RP                  (_U_(0x1) << MCANX_ECR_RP_Pos)
#define MCANX_ECR_CEL_Pos             16           /**< \brief (MCANX_ECR) CAN Error Logging */
#define MCANX_ECR_CEL_Msk             (_U_(0xFF) << MCANX_ECR_CEL_Pos)
#define MCANX_ECR_CEL(value)          (MCANX_ECR_CEL_Msk & ((value) << MCANX_ECR_CEL_Pos))
#define MCANX_ECR_MASK                _U_(0x00FFFFFF) /**< \brief (MCANX_ECR) MASK Register */

/* -------- MCANX_PSR : (CAN Offset: 0x44) (R/  32) Protocol Status -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t LEC:3;            /*!< bit:  0.. 2  Last Error Code                    */
    uint32_t ACT:2;            /*!< bit:  3.. 4  Activity                           */
    uint32_t EP:1;             /*!< bit:      5  Error Passive                      */
    uint32_t EW:1;             /*!< bit:      6  Warning Status                     */
    uint32_t BO:1;             /*!< bit:      7  Bus_Off Status                     */
    uint32_t DLEC:3;           /*!< bit:  8..10  Data Phase Last Error Code         */
    uint32_t RESI:1;           /*!< bit:     11  ESI flag of last received CAN FD Message */
    uint32_t RBRS:1;           /*!< bit:     12  BRS flag of last received CAN FD Message */
    uint32_t RFDF:1;           /*!< bit:     13  Received a CAN FD Message          */
    uint32_t PXE:1;            /*!< bit:     14  Protocol Exception Event           */
    uint32_t :1;               /*!< bit:     15  Reserved                           */
    uint32_t TDCV:7;           /*!< bit: 16..22  Transmitter Delay Compensation Value */
    uint32_t :9;               /*!< bit: 23..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_PSR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_PSR_OFFSET              0x44         /**< \brief (MCANX_PSR offset) Protocol Status */
#define MCANX_PSR_RESETVALUE          _U_(0x00000707) /**< \brief (MCANX_PSR reset_value) Protocol Status */

#define MCANX_PSR_LEC_Pos             0            /**< \brief (MCANX_PSR) Last Error Code */
#define MCANX_PSR_LEC_Msk             (_U_(0x7) << MCANX_PSR_LEC_Pos)
#define MCANX_PSR_LEC(value)          (MCANX_PSR_LEC_Msk & ((value) << MCANX_PSR_LEC_Pos))
#define   MCANX_PSR_LEC_NONE_Val            _U_(0x0)   /**< \brief (MCANX_PSR) No Error */
#define   MCANX_PSR_LEC_STUFF_Val           _U_(0x1)   /**< \brief (MCANX_PSR) Stuff Error */
#define   MCANX_PSR_LEC_FORM_Val            _U_(0x2)   /**< \brief (MCANX_PSR) Form Error */
#define   MCANX_PSR_LEC_ACK_Val             _U_(0x3)   /**< \brief (MCANX_PSR) Ack Error */
#define   MCANX_PSR_LEC_BIT1_Val            _U_(0x4)   /**< \brief (MCANX_PSR) Bit1 Error */
#define   MCANX_PSR_LEC_BIT0_Val            _U_(0x5)   /**< \brief (MCANX_PSR) Bit0 Error */
#define   MCANX_PSR_LEC_CRC_Val             _U_(0x6)   /**< \brief (MCANX_PSR) CRC Error */
#define   MCANX_PSR_LEC_NC_Val              _U_(0x7)   /**< \brief (MCANX_PSR) No Change */
#define MCANX_PSR_LEC_NONE            (MCANX_PSR_LEC_NONE_Val          << MCANX_PSR_LEC_Pos)
#define MCANX_PSR_LEC_STUFF           (MCANX_PSR_LEC_STUFF_Val         << MCANX_PSR_LEC_Pos)
#define MCANX_PSR_LEC_FORM            (MCANX_PSR_LEC_FORM_Val          << MCANX_PSR_LEC_Pos)
#define MCANX_PSR_LEC_ACK             (MCANX_PSR_LEC_ACK_Val           << MCANX_PSR_LEC_Pos)
#define MCANX_PSR_LEC_BIT1            (MCANX_PSR_LEC_BIT1_Val          << MCANX_PSR_LEC_Pos)
#define MCANX_PSR_LEC_BIT0            (MCANX_PSR_LEC_BIT0_Val          << MCANX_PSR_LEC_Pos)
#define MCANX_PSR_LEC_CRC             (MCANX_PSR_LEC_CRC_Val           << MCANX_PSR_LEC_Pos)
#define MCANX_PSR_LEC_NC              (MCANX_PSR_LEC_NC_Val            << MCANX_PSR_LEC_Pos)
#define MCANX_PSR_ACT_Pos             3            /**< \brief (MCANX_PSR) Activity */
#define MCANX_PSR_ACT_Msk             (_U_(0x3) << MCANX_PSR_ACT_Pos)
#define MCANX_PSR_ACT(value)          (MCANX_PSR_ACT_Msk & ((value) << MCANX_PSR_ACT_Pos))
#define   MCANX_PSR_ACT_SYNC_Val            _U_(0x0)   /**< \brief (MCANX_PSR) Node is synchronizing on CAN communication */
#define   MCANX_PSR_ACT_IDLE_Val            _U_(0x1)   /**< \brief (MCANX_PSR) Node is neither receiver nor transmitter */
#define   MCANX_PSR_ACT_RX_Val              _U_(0x2)   /**< \brief (MCANX_PSR) Node is operating as receiver */
#define   MCANX_PSR_ACT_TX_Val              _U_(0x3)   /**< \brief (MCANX_PSR) Node is operating as transmitter */
#define MCANX_PSR_ACT_SYNC            (MCANX_PSR_ACT_SYNC_Val          << MCANX_PSR_ACT_Pos)
#define MCANX_PSR_ACT_IDLE            (MCANX_PSR_ACT_IDLE_Val          << MCANX_PSR_ACT_Pos)
#define MCANX_PSR_ACT_RX              (MCANX_PSR_ACT_RX_Val            << MCANX_PSR_ACT_Pos)
#define MCANX_PSR_ACT_TX              (MCANX_PSR_ACT_TX_Val            << MCANX_PSR_ACT_Pos)
#define MCANX_PSR_EP_Pos              5            /**< \brief (MCANX_PSR) Error Passive */
#define MCANX_PSR_EP                  (_U_(0x1) << MCANX_PSR_EP_Pos)
#define MCANX_PSR_EW_Pos              6            /**< \brief (MCANX_PSR) Warning Status */
#define MCANX_PSR_EW                  (_U_(0x1) << MCANX_PSR_EW_Pos)
#define MCANX_PSR_BO_Pos              7            /**< \brief (MCANX_PSR) Bus_Off Status */
#define MCANX_PSR_BO                  (_U_(0x1) << MCANX_PSR_BO_Pos)
#define MCANX_PSR_DLEC_Pos            8            /**< \brief (MCANX_PSR) Data Phase Last Error Code */
#define MCANX_PSR_DLEC_Msk            (_U_(0x7) << MCANX_PSR_DLEC_Pos)
#define MCANX_PSR_DLEC(value)         (MCANX_PSR_DLEC_Msk & ((value) << MCANX_PSR_DLEC_Pos))
#define   MCANX_PSR_DLEC_NONE_Val           _U_(0x0)   /**< \brief (MCANX_PSR) No Error */
#define   MCANX_PSR_DLEC_STUFF_Val          _U_(0x1)   /**< \brief (MCANX_PSR) Stuff Error */
#define   MCANX_PSR_DLEC_FORM_Val           _U_(0x2)   /**< \brief (MCANX_PSR) Form Error */
#define   MCANX_PSR_DLEC_ACK_Val            _U_(0x3)   /**< \brief (MCANX_PSR) Ack Error */
#define   MCANX_PSR_DLEC_BIT1_Val           _U_(0x4)   /**< \brief (MCANX_PSR) Bit1 Error */
#define   MCANX_PSR_DLEC_BIT0_Val           _U_(0x5)   /**< \brief (MCANX_PSR) Bit0 Error */
#define   MCANX_PSR_DLEC_CRC_Val            _U_(0x6)   /**< \brief (MCANX_PSR) CRC Error */
#define   MCANX_PSR_DLEC_NC_Val             _U_(0x7)   /**< \brief (MCANX_PSR) No Change */
#define MCANX_PSR_DLEC_NONE           (MCANX_PSR_DLEC_NONE_Val         << MCANX_PSR_DLEC_Pos)
#define MCANX_PSR_DLEC_STUFF          (MCANX_PSR_DLEC_STUFF_Val        << MCANX_PSR_DLEC_Pos)
#define MCANX_PSR_DLEC_FORM           (MCANX_PSR_DLEC_FORM_Val         << MCANX_PSR_DLEC_Pos)
#define MCANX_PSR_DLEC_ACK            (MCANX_PSR_DLEC_ACK_Val          << MCANX_PSR_DLEC_Pos)
#define MCANX_PSR_DLEC_BIT1           (MCANX_PSR_DLEC_BIT1_Val         << MCANX_PSR_DLEC_Pos)
#define MCANX_PSR_DLEC_BIT0           (MCANX_PSR_DLEC_BIT0_Val         << MCANX_PSR_DLEC_Pos)
#define MCANX_PSR_DLEC_CRC            (MCANX_PSR_DLEC_CRC_Val          << MCANX_PSR_DLEC_Pos)
#define MCANX_PSR_DLEC_NC             (MCANX_PSR_DLEC_NC_Val           << MCANX_PSR_DLEC_Pos)
#define MCANX_PSR_RESI_Pos            11           /**< \brief (MCANX_PSR) ESI flag of last received CAN FD Message */
#define MCANX_PSR_RESI                (_U_(0x1) << MCANX_PSR_RESI_Pos)
#define MCANX_PSR_RBRS_Pos            12           /**< \brief (MCANX_PSR) BRS flag of last received CAN FD Message */
#define MCANX_PSR_RBRS                (_U_(0x1) << MCANX_PSR_RBRS_Pos)
#define MCANX_PSR_RFDF_Pos            13           /**< \brief (MCANX_PSR) Received a CAN FD Message */
#define MCANX_PSR_RFDF                (_U_(0x1) << MCANX_PSR_RFDF_Pos)
#define MCANX_PSR_PXE_Pos             14           /**< \brief (MCANX_PSR) Protocol Exception Event */
#define MCANX_PSR_PXE                 (_U_(0x1) << MCANX_PSR_PXE_Pos)
#define MCANX_PSR_TDCV_Pos            16           /**< \brief (MCANX_PSR) Transmitter Delay Compensation Value */
#define MCANX_PSR_TDCV_Msk            (_U_(0x7F) << MCANX_PSR_TDCV_Pos)
#define MCANX_PSR_TDCV(value)         (MCANX_PSR_TDCV_Msk & ((value) << MCANX_PSR_TDCV_Pos))
#define MCANX_PSR_MASK                _U_(0x007F7FFF) /**< \brief (MCANX_PSR) MASK Register */

/* -------- MCANX_TDCR : (CAN Offset: 0x48) (R/W 32) Extended ID Filter Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TDCF:7;           /*!< bit:  0.. 6  Transmitter Delay Compensation Filter Length */
    uint32_t :1;               /*!< bit:      7  Reserved                           */
    uint32_t TDCO:7;           /*!< bit:  8..14  Transmitter Delay Compensation Offset */
    uint32_t :17;              /*!< bit: 15..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TDCR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TDCR_OFFSET             0x48         /**< \brief (MCANX_TDCR offset) Extended ID Filter Configuration */
#define MCANX_TDCR_RESETVALUE         _U_(0x00000000) /**< \brief (MCANX_TDCR reset_value) Extended ID Filter Configuration */

#define MCANX_TDCR_TDCF_Pos           0            /**< \brief (MCANX_TDCR) Transmitter Delay Compensation Filter Length */
#define MCANX_TDCR_TDCF_Msk           (_U_(0x7F) << MCANX_TDCR_TDCF_Pos)
#define MCANX_TDCR_TDCF(value)        (MCANX_TDCR_TDCF_Msk & ((value) << MCANX_TDCR_TDCF_Pos))
#define MCANX_TDCR_TDCO_Pos           8            /**< \brief (MCANX_TDCR) Transmitter Delay Compensation Offset */
#define MCANX_TDCR_TDCO_Msk           (_U_(0x7F) << MCANX_TDCR_TDCO_Pos)
#define MCANX_TDCR_TDCO(value)        (MCANX_TDCR_TDCO_Msk & ((value) << MCANX_TDCR_TDCO_Pos))
#define MCANX_TDCR_MASK               _U_(0x00007F7F) /**< \brief (MCANX_TDCR) MASK Register */

/* -------- MCANX_IR : (CAN Offset: 0x50) (R/W 32) Interrupt -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t RF0N:1;           /*!< bit:      0  Rx FIFO 0 New Message              */
    uint32_t RF0W:1;           /*!< bit:      1  Rx FIFO 0 Watermark Reached        */
    uint32_t RF0F:1;           /*!< bit:      2  Rx FIFO 0 Full                     */
    uint32_t RF0L:1;           /*!< bit:      3  Rx FIFO 0 Message Lost             */
    uint32_t RF1N:1;           /*!< bit:      4  Rx FIFO 1 New Message              */
    uint32_t RF1W:1;           /*!< bit:      5  Rx FIFO 1 Watermark Reached        */
    uint32_t RF1F:1;           /*!< bit:      6  Rx FIFO 1 FIFO Full                */
    uint32_t RF1L:1;           /*!< bit:      7  Rx FIFO 1 Message Lost             */
    uint32_t HPM:1;            /*!< bit:      8  High Priority Message              */
    uint32_t TC:1;             /*!< bit:      9  Timestamp Completed                */
    uint32_t TCF:1;            /*!< bit:     10  Transmission Cancellation Finished */
    uint32_t TFE:1;            /*!< bit:     11  Tx FIFO Empty                      */
    uint32_t TEFN:1;           /*!< bit:     12  Tx Event FIFO New Entry            */
    uint32_t TEFW:1;           /*!< bit:     13  Tx Event FIFO Watermark Reached    */
    uint32_t TEFF:1;           /*!< bit:     14  Tx Event FIFO Full                 */
    uint32_t TEFL:1;           /*!< bit:     15  Tx Event FIFO Element Lost         */
    uint32_t TSW:1;            /*!< bit:     16  Timestamp Wraparound               */
    uint32_t MRAF:1;           /*!< bit:     17  Message RAM Access Failure         */
    uint32_t TOO:1;            /*!< bit:     18  Timeout Occurred                   */
    uint32_t DRX:1;            /*!< bit:     19  Message stored to Dedicated Rx Buffer */
    uint32_t BEC:1;            /*!< bit:     20  Bit Error Corrected                */
    uint32_t BEU:1;            /*!< bit:     21  Bit Error Uncorrected              */
    uint32_t ELO:1;            /*!< bit:     22  Error Logging Overflow             */
    uint32_t EP:1;             /*!< bit:     23  Error Passive                      */
    uint32_t EW:1;             /*!< bit:     24  Warning Status                     */
    uint32_t BO:1;             /*!< bit:     25  Bus_Off Status                     */
    uint32_t WDI:1;            /*!< bit:     26  Watchdog Interrupt                 */
    uint32_t PEA:1;            /*!< bit:     27  Protocol Error in Arbitration Phase */
    uint32_t PED:1;            /*!< bit:     28  Protocol Error in Data Phase       */
    uint32_t ARA:1;            /*!< bit:     29  Access to Reserved Address         */
    uint32_t :2;               /*!< bit: 30..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_IR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_IR_OFFSET               0x50         /**< \brief (MCANX_IR offset) Interrupt */
#define MCANX_IR_RESETVALUE           _U_(0x00000000) /**< \brief (MCANX_IR reset_value) Interrupt */

#define MCANX_IR_RF0N_Pos             0            /**< \brief (MCANX_IR) Rx FIFO 0 New Message */
#define MCANX_IR_RF0N                 (_U_(0x1) << MCANX_IR_RF0N_Pos)
#define MCANX_IR_RF0W_Pos             1            /**< \brief (MCANX_IR) Rx FIFO 0 Watermark Reached */
#define MCANX_IR_RF0W                 (_U_(0x1) << MCANX_IR_RF0W_Pos)
#define MCANX_IR_RF0F_Pos             2            /**< \brief (MCANX_IR) Rx FIFO 0 Full */
#define MCANX_IR_RF0F                 (_U_(0x1) << MCANX_IR_RF0F_Pos)
#define MCANX_IR_RF0L_Pos             3            /**< \brief (MCANX_IR) Rx FIFO 0 Message Lost */
#define MCANX_IR_RF0L                 (_U_(0x1) << MCANX_IR_RF0L_Pos)
#define MCANX_IR_RF1N_Pos             4            /**< \brief (MCANX_IR) Rx FIFO 1 New Message */
#define MCANX_IR_RF1N                 (_U_(0x1) << MCANX_IR_RF1N_Pos)
#define MCANX_IR_RF1W_Pos             5            /**< \brief (MCANX_IR) Rx FIFO 1 Watermark Reached */
#define MCANX_IR_RF1W                 (_U_(0x1) << MCANX_IR_RF1W_Pos)
#define MCANX_IR_RF1F_Pos             6            /**< \brief (MCANX_IR) Rx FIFO 1 FIFO Full */
#define MCANX_IR_RF1F                 (_U_(0x1) << MCANX_IR_RF1F_Pos)
#define MCANX_IR_RF1L_Pos             7            /**< \brief (MCANX_IR) Rx FIFO 1 Message Lost */
#define MCANX_IR_RF1L                 (_U_(0x1) << MCANX_IR_RF1L_Pos)
#define MCANX_IR_HPM_Pos              8            /**< \brief (MCANX_IR) High Priority Message */
#define MCANX_IR_HPM                  (_U_(0x1) << MCANX_IR_HPM_Pos)
#define MCANX_IR_TC_Pos               9            /**< \brief (MCANX_IR) Timestamp Completed */
#define MCANX_IR_TC                   (_U_(0x1) << MCANX_IR_TC_Pos)
#define MCANX_IR_TCF_Pos              10           /**< \brief (MCANX_IR) Transmission Cancellation Finished */
#define MCANX_IR_TCF                  (_U_(0x1) << MCANX_IR_TCF_Pos)
#define MCANX_IR_TFE_Pos              11           /**< \brief (MCANX_IR) Tx FIFO Empty */
#define MCANX_IR_TFE                  (_U_(0x1) << MCANX_IR_TFE_Pos)
#define MCANX_IR_TEFN_Pos             12           /**< \brief (MCANX_IR) Tx Event FIFO New Entry */
#define MCANX_IR_TEFN                 (_U_(0x1) << MCANX_IR_TEFN_Pos)
#define MCANX_IR_TEFW_Pos             13           /**< \brief (MCANX_IR) Tx Event FIFO Watermark Reached */
#define MCANX_IR_TEFW                 (_U_(0x1) << MCANX_IR_TEFW_Pos)
#define MCANX_IR_TEFF_Pos             14           /**< \brief (MCANX_IR) Tx Event FIFO Full */
#define MCANX_IR_TEFF                 (_U_(0x1) << MCANX_IR_TEFF_Pos)
#define MCANX_IR_TEFL_Pos             15           /**< \brief (MCANX_IR) Tx Event FIFO Element Lost */
#define MCANX_IR_TEFL                 (_U_(0x1) << MCANX_IR_TEFL_Pos)
#define MCANX_IR_TSW_Pos              16           /**< \brief (MCANX_IR) Timestamp Wraparound */
#define MCANX_IR_TSW                  (_U_(0x1) << MCANX_IR_TSW_Pos)
#define MCANX_IR_MRAF_Pos             17           /**< \brief (MCANX_IR) Message RAM Access Failure */
#define MCANX_IR_MRAF                 (_U_(0x1) << MCANX_IR_MRAF_Pos)
#define MCANX_IR_TOO_Pos              18           /**< \brief (MCANX_IR) Timeout Occurred */
#define MCANX_IR_TOO                  (_U_(0x1) << MCANX_IR_TOO_Pos)
#define MCANX_IR_DRX_Pos              19           /**< \brief (MCANX_IR) Message stored to Dedicated Rx Buffer */
#define MCANX_IR_DRX                  (_U_(0x1) << MCANX_IR_DRX_Pos)
#define MCANX_IR_BEC_Pos              20           /**< \brief (MCANX_IR) Bit Error Corrected */
#define MCANX_IR_BEC                  (_U_(0x1) << MCANX_IR_BEC_Pos)
#define MCANX_IR_BEU_Pos              21           /**< \brief (MCANX_IR) Bit Error Uncorrected */
#define MCANX_IR_BEU                  (_U_(0x1) << MCANX_IR_BEU_Pos)
#define MCANX_IR_ELO_Pos              22           /**< \brief (MCANX_IR) Error Logging Overflow */
#define MCANX_IR_ELO                  (_U_(0x1) << MCANX_IR_ELO_Pos)
#define MCANX_IR_EP_Pos               23           /**< \brief (MCANX_IR) Error Passive */
#define MCANX_IR_EP                   (_U_(0x1) << MCANX_IR_EP_Pos)
#define MCANX_IR_EW_Pos               24           /**< \brief (MCANX_IR) Warning Status */
#define MCANX_IR_EW                   (_U_(0x1) << MCANX_IR_EW_Pos)
#define MCANX_IR_BO_Pos               25           /**< \brief (MCANX_IR) Bus_Off Status */
#define MCANX_IR_BO                   (_U_(0x1) << MCANX_IR_BO_Pos)
#define MCANX_IR_WDI_Pos              26           /**< \brief (MCANX_IR) Watchdog Interrupt */
#define MCANX_IR_WDI                  (_U_(0x1) << MCANX_IR_WDI_Pos)
#define MCANX_IR_PEA_Pos              27           /**< \brief (MCANX_IR) Protocol Error in Arbitration Phase */
#define MCANX_IR_PEA                  (_U_(0x1) << MCANX_IR_PEA_Pos)
#define MCANX_IR_PED_Pos              28           /**< \brief (MCANX_IR) Protocol Error in Data Phase */
#define MCANX_IR_PED                  (_U_(0x1) << MCANX_IR_PED_Pos)
#define MCANX_IR_ARA_Pos              29           /**< \brief (MCANX_IR) Access to Reserved Address */
#define MCANX_IR_ARA                  (_U_(0x1) << MCANX_IR_ARA_Pos)
#define MCANX_IR_MASK                 _U_(0x3FFFFFFF) /**< \brief (MCANX_IR) MASK Register */

/* -------- MCANX_IE : (CAN Offset: 0x54) (R/W 32) Interrupt Enable -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t RF0NE:1;          /*!< bit:      0  Rx FIFO 0 New Message Interrupt Enable */
    uint32_t RF0WE:1;          /*!< bit:      1  Rx FIFO 0 Watermark Reached Interrupt Enable */
    uint32_t RF0FE:1;          /*!< bit:      2  Rx FIFO 0 Full Interrupt Enable    */
    uint32_t RF0LE:1;          /*!< bit:      3  Rx FIFO 0 Message Lost Interrupt Enable */
    uint32_t RF1NE:1;          /*!< bit:      4  Rx FIFO 1 New Message Interrupt Enable */
    uint32_t RF1WE:1;          /*!< bit:      5  Rx FIFO 1 Watermark Reached Interrupt Enable */
    uint32_t RF1FE:1;          /*!< bit:      6  Rx FIFO 1 FIFO Full Interrupt Enable */
    uint32_t RF1LE:1;          /*!< bit:      7  Rx FIFO 1 Message Lost Interrupt Enable */
    uint32_t HPME:1;           /*!< bit:      8  High Priority Message Interrupt Enable */
    uint32_t TCE:1;            /*!< bit:      9  Timestamp Completed Interrupt Enable */
    uint32_t TCFE:1;           /*!< bit:     10  Transmission Cancellation Finished Interrupt Enable */
    uint32_t TFEE:1;           /*!< bit:     11  Tx FIFO Empty Interrupt Enable     */
    uint32_t TEFNE:1;          /*!< bit:     12  Tx Event FIFO New Entry Interrupt Enable */
    uint32_t TEFWE:1;          /*!< bit:     13  Tx Event FIFO Watermark Reached Interrupt Enable */
    uint32_t TEFFE:1;          /*!< bit:     14  Tx Event FIFO Full Interrupt Enable */
    uint32_t TEFLE:1;          /*!< bit:     15  Tx Event FIFO Element Lost Interrupt Enable */
    uint32_t TSWE:1;           /*!< bit:     16  Timestamp Wraparound Interrupt Enable */
    uint32_t MRAFE:1;          /*!< bit:     17  Message RAM Access Failure Interrupt Enable */
    uint32_t TOOE:1;           /*!< bit:     18  Timeout Occurred Interrupt Enable  */
    uint32_t DRXE:1;           /*!< bit:     19  Message stored to Dedicated Rx Buffer Interrupt Enable */
    uint32_t BECE:1;           /*!< bit:     20  Bit Error Corrected Interrupt Enable */
    uint32_t BEUE:1;           /*!< bit:     21  Bit Error Uncorrected Interrupt Enable */
    uint32_t ELOE:1;           /*!< bit:     22  Error Logging Overflow Interrupt Enable */
    uint32_t EPE:1;            /*!< bit:     23  Error Passive Interrupt Enable     */
    uint32_t EWE:1;            /*!< bit:     24  Warning Status Interrupt Enable    */
    uint32_t BOE:1;            /*!< bit:     25  Bus_Off Status Interrupt Enable    */
    uint32_t WDIE:1;           /*!< bit:     26  Watchdog Interrupt Interrupt Enable */
    uint32_t PEAE:1;           /*!< bit:     27  Protocol Error in Arbitration Phase Enable */
    uint32_t PEDE:1;           /*!< bit:     28  Protocol Error in Data Phase Enable */
    uint32_t ARAE:1;           /*!< bit:     29  Access to Reserved Address Enable  */
    uint32_t :2;               /*!< bit: 30..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_IE_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_IE_OFFSET               0x54         /**< \brief (MCANX_IE offset) Interrupt Enable */
#define MCANX_IE_RESETVALUE           _U_(0x00000000) /**< \brief (MCANX_IE reset_value) Interrupt Enable */

#define MCANX_IE_RF0NE_Pos            0            /**< \brief (MCANX_IE) Rx FIFO 0 New Message Interrupt Enable */
#define MCANX_IE_RF0NE                (_U_(0x1) << MCANX_IE_RF0NE_Pos)
#define MCANX_IE_RF0WE_Pos            1            /**< \brief (MCANX_IE) Rx FIFO 0 Watermark Reached Interrupt Enable */
#define MCANX_IE_RF0WE                (_U_(0x1) << MCANX_IE_RF0WE_Pos)
#define MCANX_IE_RF0FE_Pos            2            /**< \brief (MCANX_IE) Rx FIFO 0 Full Interrupt Enable */
#define MCANX_IE_RF0FE                (_U_(0x1) << MCANX_IE_RF0FE_Pos)
#define MCANX_IE_RF0LE_Pos            3            /**< \brief (MCANX_IE) Rx FIFO 0 Message Lost Interrupt Enable */
#define MCANX_IE_RF0LE                (_U_(0x1) << MCANX_IE_RF0LE_Pos)
#define MCANX_IE_RF1NE_Pos            4            /**< \brief (MCANX_IE) Rx FIFO 1 New Message Interrupt Enable */
#define MCANX_IE_RF1NE                (_U_(0x1) << MCANX_IE_RF1NE_Pos)
#define MCANX_IE_RF1WE_Pos            5            /**< \brief (MCANX_IE) Rx FIFO 1 Watermark Reached Interrupt Enable */
#define MCANX_IE_RF1WE                (_U_(0x1) << MCANX_IE_RF1WE_Pos)
#define MCANX_IE_RF1FE_Pos            6            /**< \brief (MCANX_IE) Rx FIFO 1 FIFO Full Interrupt Enable */
#define MCANX_IE_RF1FE                (_U_(0x1) << MCANX_IE_RF1FE_Pos)
#define MCANX_IE_RF1LE_Pos            7            /**< \brief (MCANX_IE) Rx FIFO 1 Message Lost Interrupt Enable */
#define MCANX_IE_RF1LE                (_U_(0x1) << MCANX_IE_RF1LE_Pos)
#define MCANX_IE_HPME_Pos             8            /**< \brief (MCANX_IE) High Priority Message Interrupt Enable */
#define MCANX_IE_HPME                 (_U_(0x1) << MCANX_IE_HPME_Pos)
#define MCANX_IE_TCE_Pos              9            /**< \brief (MCANX_IE) Timestamp Completed Interrupt Enable */
#define MCANX_IE_TCE                  (_U_(0x1) << MCANX_IE_TCE_Pos)
#define MCANX_IE_TCFE_Pos             10           /**< \brief (MCANX_IE) Transmission Cancellation Finished Interrupt Enable */
#define MCANX_IE_TCFE                 (_U_(0x1) << MCANX_IE_TCFE_Pos)
#define MCANX_IE_TFEE_Pos             11           /**< \brief (MCANX_IE) Tx FIFO Empty Interrupt Enable */
#define MCANX_IE_TFEE                 (_U_(0x1) << MCANX_IE_TFEE_Pos)
#define MCANX_IE_TEFNE_Pos            12           /**< \brief (MCANX_IE) Tx Event FIFO New Entry Interrupt Enable */
#define MCANX_IE_TEFNE                (_U_(0x1) << MCANX_IE_TEFNE_Pos)
#define MCANX_IE_TEFWE_Pos            13           /**< \brief (MCANX_IE) Tx Event FIFO Watermark Reached Interrupt Enable */
#define MCANX_IE_TEFWE                (_U_(0x1) << MCANX_IE_TEFWE_Pos)
#define MCANX_IE_TEFFE_Pos            14           /**< \brief (MCANX_IE) Tx Event FIFO Full Interrupt Enable */
#define MCANX_IE_TEFFE                (_U_(0x1) << MCANX_IE_TEFFE_Pos)
#define MCANX_IE_TEFLE_Pos            15           /**< \brief (MCANX_IE) Tx Event FIFO Element Lost Interrupt Enable */
#define MCANX_IE_TEFLE                (_U_(0x1) << MCANX_IE_TEFLE_Pos)
#define MCANX_IE_TSWE_Pos             16           /**< \brief (MCANX_IE) Timestamp Wraparound Interrupt Enable */
#define MCANX_IE_TSWE                 (_U_(0x1) << MCANX_IE_TSWE_Pos)
#define MCANX_IE_MRAFE_Pos            17           /**< \brief (MCANX_IE) Message RAM Access Failure Interrupt Enable */
#define MCANX_IE_MRAFE                (_U_(0x1) << MCANX_IE_MRAFE_Pos)
#define MCANX_IE_TOOE_Pos             18           /**< \brief (MCANX_IE) Timeout Occurred Interrupt Enable */
#define MCANX_IE_TOOE                 (_U_(0x1) << MCANX_IE_TOOE_Pos)
#define MCANX_IE_DRXE_Pos             19           /**< \brief (MCANX_IE) Message stored to Dedicated Rx Buffer Interrupt Enable */
#define MCANX_IE_DRXE                 (_U_(0x1) << MCANX_IE_DRXE_Pos)
#define MCANX_IE_BECE_Pos             20           /**< \brief (MCANX_IE) Bit Error Corrected Interrupt Enable */
#define MCANX_IE_BECE                 (_U_(0x1) << MCANX_IE_BECE_Pos)
#define MCANX_IE_BEUE_Pos             21           /**< \brief (MCANX_IE) Bit Error Uncorrected Interrupt Enable */
#define MCANX_IE_BEUE                 (_U_(0x1) << MCANX_IE_BEUE_Pos)
#define MCANX_IE_ELOE_Pos             22           /**< \brief (MCANX_IE) Error Logging Overflow Interrupt Enable */
#define MCANX_IE_ELOE                 (_U_(0x1) << MCANX_IE_ELOE_Pos)
#define MCANX_IE_EPE_Pos              23           /**< \brief (MCANX_IE) Error Passive Interrupt Enable */
#define MCANX_IE_EPE                  (_U_(0x1) << MCANX_IE_EPE_Pos)
#define MCANX_IE_EWE_Pos              24           /**< \brief (MCANX_IE) Warning Status Interrupt Enable */
#define MCANX_IE_EWE                  (_U_(0x1) << MCANX_IE_EWE_Pos)
#define MCANX_IE_BOE_Pos              25           /**< \brief (MCANX_IE) Bus_Off Status Interrupt Enable */
#define MCANX_IE_BOE                  (_U_(0x1) << MCANX_IE_BOE_Pos)
#define MCANX_IE_WDIE_Pos             26           /**< \brief (MCANX_IE) Watchdog Interrupt Interrupt Enable */
#define MCANX_IE_WDIE                 (_U_(0x1) << MCANX_IE_WDIE_Pos)
#define MCANX_IE_PEAE_Pos             27           /**< \brief (MCANX_IE) Protocol Error in Arbitration Phase Enable */
#define MCANX_IE_PEAE                 (_U_(0x1) << MCANX_IE_PEAE_Pos)
#define MCANX_IE_PEDE_Pos             28           /**< \brief (MCANX_IE) Protocol Error in Data Phase Enable */
#define MCANX_IE_PEDE                 (_U_(0x1) << MCANX_IE_PEDE_Pos)
#define MCANX_IE_ARAE_Pos             29           /**< \brief (MCANX_IE) Access to Reserved Address Enable */
#define MCANX_IE_ARAE                 (_U_(0x1) << MCANX_IE_ARAE_Pos)
#define MCANX_IE_MASK                 _U_(0x3FFFFFFF) /**< \brief (MCANX_IE) MASK Register */

/* -------- MCANX_ILS : (CAN Offset: 0x58) (R/W 32) Interrupt Line Select -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t RF0NL:1;          /*!< bit:      0  Rx FIFO 0 New Message Interrupt Line */
    uint32_t RF0WL:1;          /*!< bit:      1  Rx FIFO 0 Watermark Reached Interrupt Line */
    uint32_t RF0FL:1;          /*!< bit:      2  Rx FIFO 0 Full Interrupt Line      */
    uint32_t RF0LL:1;          /*!< bit:      3  Rx FIFO 0 Message Lost Interrupt Line */
    uint32_t RF1NL:1;          /*!< bit:      4  Rx FIFO 1 New Message Interrupt Line */
    uint32_t RF1WL:1;          /*!< bit:      5  Rx FIFO 1 Watermark Reached Interrupt Line */
    uint32_t RF1FL:1;          /*!< bit:      6  Rx FIFO 1 FIFO Full Interrupt Line */
    uint32_t RF1LL:1;          /*!< bit:      7  Rx FIFO 1 Message Lost Interrupt Line */
    uint32_t HPML:1;           /*!< bit:      8  High Priority Message Interrupt Line */
    uint32_t TCL:1;            /*!< bit:      9  Timestamp Completed Interrupt Line */
    uint32_t TCFL:1;           /*!< bit:     10  Transmission Cancellation Finished Interrupt Line */
    uint32_t TFEL:1;           /*!< bit:     11  Tx FIFO Empty Interrupt Line       */
    uint32_t TEFNL:1;          /*!< bit:     12  Tx Event FIFO New Entry Interrupt Line */
    uint32_t TEFWL:1;          /*!< bit:     13  Tx Event FIFO Watermark Reached Interrupt Line */
    uint32_t TEFFL:1;          /*!< bit:     14  Tx Event FIFO Full Interrupt Line  */
    uint32_t TEFLL:1;          /*!< bit:     15  Tx Event FIFO Element Lost Interrupt Line */
    uint32_t TSWL:1;           /*!< bit:     16  Timestamp Wraparound Interrupt Line */
    uint32_t MRAFL:1;          /*!< bit:     17  Message RAM Access Failure Interrupt Line */
    uint32_t TOOL:1;           /*!< bit:     18  Timeout Occurred Interrupt Line    */
    uint32_t DRXL:1;           /*!< bit:     19  Message stored to Dedicated Rx Buffer Interrupt Line */
    uint32_t BECL:1;           /*!< bit:     20  Bit Error Corrected Interrupt Line */
    uint32_t BEUL:1;           /*!< bit:     21  Bit Error Uncorrected Interrupt Line */
    uint32_t ELOL:1;           /*!< bit:     22  Error Logging Overflow Interrupt Line */
    uint32_t EPL:1;            /*!< bit:     23  Error Passive Interrupt Line       */
    uint32_t EWL:1;            /*!< bit:     24  Warning Status Interrupt Line      */
    uint32_t BOL:1;            /*!< bit:     25  Bus_Off Status Interrupt Line      */
    uint32_t WDIL:1;           /*!< bit:     26  Watchdog Interrupt Interrupt Line  */
    uint32_t PEAL:1;           /*!< bit:     27  Protocol Error in Arbitration Phase Line */
    uint32_t PEDL:1;           /*!< bit:     28  Protocol Error in Data Phase Line  */
    uint32_t ARAL:1;           /*!< bit:     29  Access to Reserved Address Line    */
    uint32_t :2;               /*!< bit: 30..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_ILS_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_ILS_OFFSET              0x58         /**< \brief (MCANX_ILS offset) Interrupt Line Select */
#define MCANX_ILS_RESETVALUE          _U_(0x00000000) /**< \brief (MCANX_ILS reset_value) Interrupt Line Select */

#define MCANX_ILS_RF0NL_Pos           0            /**< \brief (MCANX_ILS) Rx FIFO 0 New Message Interrupt Line */
#define MCANX_ILS_RF0NL               (_U_(0x1) << MCANX_ILS_RF0NL_Pos)
#define MCANX_ILS_RF0WL_Pos           1            /**< \brief (MCANX_ILS) Rx FIFO 0 Watermark Reached Interrupt Line */
#define MCANX_ILS_RF0WL               (_U_(0x1) << MCANX_ILS_RF0WL_Pos)
#define MCANX_ILS_RF0FL_Pos           2            /**< \brief (MCANX_ILS) Rx FIFO 0 Full Interrupt Line */
#define MCANX_ILS_RF0FL               (_U_(0x1) << MCANX_ILS_RF0FL_Pos)
#define MCANX_ILS_RF0LL_Pos           3            /**< \brief (MCANX_ILS) Rx FIFO 0 Message Lost Interrupt Line */
#define MCANX_ILS_RF0LL               (_U_(0x1) << MCANX_ILS_RF0LL_Pos)
#define MCANX_ILS_RF1NL_Pos           4            /**< \brief (MCANX_ILS) Rx FIFO 1 New Message Interrupt Line */
#define MCANX_ILS_RF1NL               (_U_(0x1) << MCANX_ILS_RF1NL_Pos)
#define MCANX_ILS_RF1WL_Pos           5            /**< \brief (MCANX_ILS) Rx FIFO 1 Watermark Reached Interrupt Line */
#define MCANX_ILS_RF1WL               (_U_(0x1) << MCANX_ILS_RF1WL_Pos)
#define MCANX_ILS_RF1FL_Pos           6            /**< \brief (MCANX_ILS) Rx FIFO 1 FIFO Full Interrupt Line */
#define MCANX_ILS_RF1FL               (_U_(0x1) << MCANX_ILS_RF1FL_Pos)
#define MCANX_ILS_RF1LL_Pos           7            /**< \brief (MCANX_ILS) Rx FIFO 1 Message Lost Interrupt Line */
#define MCANX_ILS_RF1LL               (_U_(0x1) << MCANX_ILS_RF1LL_Pos)
#define MCANX_ILS_HPML_Pos            8            /**< \brief (MCANX_ILS) High Priority Message Interrupt Line */
#define MCANX_ILS_HPML                (_U_(0x1) << MCANX_ILS_HPML_Pos)
#define MCANX_ILS_TCL_Pos             9            /**< \brief (MCANX_ILS) Timestamp Completed Interrupt Line */
#define MCANX_ILS_TCL                 (_U_(0x1) << MCANX_ILS_TCL_Pos)
#define MCANX_ILS_TCFL_Pos            10           /**< \brief (MCANX_ILS) Transmission Cancellation Finished Interrupt Line */
#define MCANX_ILS_TCFL                (_U_(0x1) << MCANX_ILS_TCFL_Pos)
#define MCANX_ILS_TFEL_Pos            11           /**< \brief (MCANX_ILS) Tx FIFO Empty Interrupt Line */
#define MCANX_ILS_TFEL                (_U_(0x1) << MCANX_ILS_TFEL_Pos)
#define MCANX_ILS_TEFNL_Pos           12           /**< \brief (MCANX_ILS) Tx Event FIFO New Entry Interrupt Line */
#define MCANX_ILS_TEFNL               (_U_(0x1) << MCANX_ILS_TEFNL_Pos)
#define MCANX_ILS_TEFWL_Pos           13           /**< \brief (MCANX_ILS) Tx Event FIFO Watermark Reached Interrupt Line */
#define MCANX_ILS_TEFWL               (_U_(0x1) << MCANX_ILS_TEFWL_Pos)
#define MCANX_ILS_TEFFL_Pos           14           /**< \brief (MCANX_ILS) Tx Event FIFO Full Interrupt Line */
#define MCANX_ILS_TEFFL               (_U_(0x1) << MCANX_ILS_TEFFL_Pos)
#define MCANX_ILS_TEFLL_Pos           15           /**< \brief (MCANX_ILS) Tx Event FIFO Element Lost Interrupt Line */
#define MCANX_ILS_TEFLL               (_U_(0x1) << MCANX_ILS_TEFLL_Pos)
#define MCANX_ILS_TSWL_Pos            16           /**< \brief (MCANX_ILS) Timestamp Wraparound Interrupt Line */
#define MCANX_ILS_TSWL                (_U_(0x1) << MCANX_ILS_TSWL_Pos)
#define MCANX_ILS_MRAFL_Pos           17           /**< \brief (MCANX_ILS) Message RAM Access Failure Interrupt Line */
#define MCANX_ILS_MRAFL               (_U_(0x1) << MCANX_ILS_MRAFL_Pos)
#define MCANX_ILS_TOOL_Pos            18           /**< \brief (MCANX_ILS) Timeout Occurred Interrupt Line */
#define MCANX_ILS_TOOL                (_U_(0x1) << MCANX_ILS_TOOL_Pos)
#define MCANX_ILS_DRXL_Pos            19           /**< \brief (MCANX_ILS) Message stored to Dedicated Rx Buffer Interrupt Line */
#define MCANX_ILS_DRXL                (_U_(0x1) << MCANX_ILS_DRXL_Pos)
#define MCANX_ILS_BECL_Pos            20           /**< \brief (MCANX_ILS) Bit Error Corrected Interrupt Line */
#define MCANX_ILS_BECL                (_U_(0x1) << MCANX_ILS_BECL_Pos)
#define MCANX_ILS_BEUL_Pos            21           /**< \brief (MCANX_ILS) Bit Error Uncorrected Interrupt Line */
#define MCANX_ILS_BEUL                (_U_(0x1) << MCANX_ILS_BEUL_Pos)
#define MCANX_ILS_ELOL_Pos            22           /**< \brief (MCANX_ILS) Error Logging Overflow Interrupt Line */
#define MCANX_ILS_ELOL                (_U_(0x1) << MCANX_ILS_ELOL_Pos)
#define MCANX_ILS_EPL_Pos             23           /**< \brief (MCANX_ILS) Error Passive Interrupt Line */
#define MCANX_ILS_EPL                 (_U_(0x1) << MCANX_ILS_EPL_Pos)
#define MCANX_ILS_EWL_Pos             24           /**< \brief (MCANX_ILS) Warning Status Interrupt Line */
#define MCANX_ILS_EWL                 (_U_(0x1) << MCANX_ILS_EWL_Pos)
#define MCANX_ILS_BOL_Pos             25           /**< \brief (MCANX_ILS) Bus_Off Status Interrupt Line */
#define MCANX_ILS_BOL                 (_U_(0x1) << MCANX_ILS_BOL_Pos)
#define MCANX_ILS_WDIL_Pos            26           /**< \brief (MCANX_ILS) Watchdog Interrupt Interrupt Line */
#define MCANX_ILS_WDIL                (_U_(0x1) << MCANX_ILS_WDIL_Pos)
#define MCANX_ILS_PEAL_Pos            27           /**< \brief (MCANX_ILS) Protocol Error in Arbitration Phase Line */
#define MCANX_ILS_PEAL                (_U_(0x1) << MCANX_ILS_PEAL_Pos)
#define MCANX_ILS_PEDL_Pos            28           /**< \brief (MCANX_ILS) Protocol Error in Data Phase Line */
#define MCANX_ILS_PEDL                (_U_(0x1) << MCANX_ILS_PEDL_Pos)
#define MCANX_ILS_ARAL_Pos            29           /**< \brief (MCANX_ILS) Access to Reserved Address Line */
#define MCANX_ILS_ARAL                (_U_(0x1) << MCANX_ILS_ARAL_Pos)
#define MCANX_ILS_MASK                _U_(0x3FFFFFFF) /**< \brief (MCANX_ILS) MASK Register */

/* -------- MCANX_ILE : (CAN Offset: 0x5C) (R/W 32) Interrupt Line Enable -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t EINT0:1;          /*!< bit:      0  Enable Interrupt Line 0            */
    uint32_t EINT1:1;          /*!< bit:      1  Enable Interrupt Line 1            */
    uint32_t :30;              /*!< bit:  2..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_ILE_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_ILE_OFFSET              0x5C         /**< \brief (MCANX_ILE offset) Interrupt Line Enable */
#define MCANX_ILE_RESETVALUE          _U_(0x00000000) /**< \brief (MCANX_ILE reset_value) Interrupt Line Enable */

#define MCANX_ILE_EINT0_Pos           0            /**< \brief (MCANX_ILE) Enable Interrupt Line 0 */
#define MCANX_ILE_EINT0               (_U_(0x1) << MCANX_ILE_EINT0_Pos)
#define MCANX_ILE_EINT1_Pos           1            /**< \brief (MCANX_ILE) Enable Interrupt Line 1 */
#define MCANX_ILE_EINT1               (_U_(0x1) << MCANX_ILE_EINT1_Pos)
#define MCANX_ILE_MASK                _U_(0x00000003) /**< \brief (MCANX_ILE) MASK Register */

/* -------- MCANX_GFC : (CAN Offset: 0x80) (R/W 32) Global Filter Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t RRFE:1;           /*!< bit:      0  Reject Remote Frames Extended      */
    uint32_t RRFS:1;           /*!< bit:      1  Reject Remote Frames Standard      */
    uint32_t ANFE:2;           /*!< bit:  2.. 3  Accept Non-matching Frames Extended */
    uint32_t ANFS:2;           /*!< bit:  4.. 5  Accept Non-matching Frames Standard */
    uint32_t :26;              /*!< bit:  6..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_GFC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_GFC_OFFSET              0x80         /**< \brief (MCANX_GFC offset) Global Filter Configuration */
#define MCANX_GFC_RESETVALUE          _U_(0x00000000) /**< \brief (MCANX_GFC reset_value) Global Filter Configuration */

#define MCANX_GFC_RRFE_Pos            0            /**< \brief (MCANX_GFC) Reject Remote Frames Extended */
#define MCANX_GFC_RRFE                (_U_(0x1) << MCANX_GFC_RRFE_Pos)
#define MCANX_GFC_RRFS_Pos            1            /**< \brief (MCANX_GFC) Reject Remote Frames Standard */
#define MCANX_GFC_RRFS                (_U_(0x1) << MCANX_GFC_RRFS_Pos)
#define MCANX_GFC_ANFE_Pos            2            /**< \brief (MCANX_GFC) Accept Non-matching Frames Extended */
#define MCANX_GFC_ANFE_Msk            (_U_(0x3) << MCANX_GFC_ANFE_Pos)
#define MCANX_GFC_ANFE(value)         (MCANX_GFC_ANFE_Msk & ((value) << MCANX_GFC_ANFE_Pos))
#define   MCANX_GFC_ANFE_RXF0_Val           _U_(0x0)   /**< \brief (MCANX_GFC) Accept in Rx FIFO 0 */
#define   MCANX_GFC_ANFE_RXF1_Val           _U_(0x1)   /**< \brief (MCANX_GFC) Accept in Rx FIFO 1 */
#define   MCANX_GFC_ANFE_REJECT_Val         _U_(0x2)   /**< \brief (MCANX_GFC) Reject */
#define MCANX_GFC_ANFE_RXF0           (MCANX_GFC_ANFE_RXF0_Val         << MCANX_GFC_ANFE_Pos)
#define MCANX_GFC_ANFE_RXF1           (MCANX_GFC_ANFE_RXF1_Val         << MCANX_GFC_ANFE_Pos)
#define MCANX_GFC_ANFE_REJECT         (MCANX_GFC_ANFE_REJECT_Val       << MCANX_GFC_ANFE_Pos)
#define MCANX_GFC_ANFS_Pos            4            /**< \brief (MCANX_GFC) Accept Non-matching Frames Standard */
#define MCANX_GFC_ANFS_Msk            (_U_(0x3) << MCANX_GFC_ANFS_Pos)
#define MCANX_GFC_ANFS(value)         (MCANX_GFC_ANFS_Msk & ((value) << MCANX_GFC_ANFS_Pos))
#define   MCANX_GFC_ANFS_RXF0_Val           _U_(0x0)   /**< \brief (MCANX_GFC) Accept in Rx FIFO 0 */
#define   MCANX_GFC_ANFS_RXF1_Val           _U_(0x1)   /**< \brief (MCANX_GFC) Accept in Rx FIFO 1 */
#define   MCANX_GFC_ANFS_REJECT_Val         _U_(0x2)   /**< \brief (MCANX_GFC) Reject */
#define MCANX_GFC_ANFS_RXF0           (MCANX_GFC_ANFS_RXF0_Val         << MCANX_GFC_ANFS_Pos)
#define MCANX_GFC_ANFS_RXF1           (MCANX_GFC_ANFS_RXF1_Val         << MCANX_GFC_ANFS_Pos)
#define MCANX_GFC_ANFS_REJECT         (MCANX_GFC_ANFS_REJECT_Val       << MCANX_GFC_ANFS_Pos)
#define MCANX_GFC_MASK                _U_(0x0000003F) /**< \brief (MCANX_GFC) MASK Register */

/* -------- MCANX_SIDFC : (CAN Offset: 0x84) (R/W 32) Standard ID Filter Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t FLSSA:16;         /*!< bit:  0..15  Filter List Standard Start Address */
    uint32_t LSS:8;            /*!< bit: 16..23  List Size Standard                 */
    uint32_t :8;               /*!< bit: 24..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_SIDFC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_SIDFC_OFFSET            0x84         /**< \brief (MCANX_SIDFC offset) Standard ID Filter Configuration */
#define MCANX_SIDFC_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_SIDFC reset_value) Standard ID Filter Configuration */

#define MCANX_SIDFC_FLSSA_Pos         0            /**< \brief (MCANX_SIDFC) Filter List Standard Start Address */
#define MCANX_SIDFC_FLSSA_Msk         (_U_(0xFFFF) << MCANX_SIDFC_FLSSA_Pos)
#define MCANX_SIDFC_FLSSA(value)      (MCANX_SIDFC_FLSSA_Msk & ((value) << MCANX_SIDFC_FLSSA_Pos))
#define MCANX_SIDFC_LSS_Pos           16           /**< \brief (MCANX_SIDFC) List Size Standard */
#define MCANX_SIDFC_LSS_Msk           (_U_(0xFF) << MCANX_SIDFC_LSS_Pos)
#define MCANX_SIDFC_LSS(value)        (MCANX_SIDFC_LSS_Msk & ((value) << MCANX_SIDFC_LSS_Pos))
#define MCANX_SIDFC_MASK              _U_(0x00FFFFFF) /**< \brief (MCANX_SIDFC) MASK Register */

/* -------- MCANX_XIDFC : (CAN Offset: 0x88) (R/W 32) Extended ID Filter Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t FLESA:16;         /*!< bit:  0..15  Filter List Extended Start Address */
    uint32_t LSE:7;            /*!< bit: 16..22  List Size Extended                 */
    uint32_t :9;               /*!< bit: 23..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_XIDFC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_XIDFC_OFFSET            0x88         /**< \brief (MCANX_XIDFC offset) Extended ID Filter Configuration */
#define MCANX_XIDFC_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_XIDFC reset_value) Extended ID Filter Configuration */

#define MCANX_XIDFC_FLESA_Pos         0            /**< \brief (MCANX_XIDFC) Filter List Extended Start Address */
#define MCANX_XIDFC_FLESA_Msk         (_U_(0xFFFF) << MCANX_XIDFC_FLESA_Pos)
#define MCANX_XIDFC_FLESA(value)      (MCANX_XIDFC_FLESA_Msk & ((value) << MCANX_XIDFC_FLESA_Pos))
#define MCANX_XIDFC_LSE_Pos           16           /**< \brief (MCANX_XIDFC) List Size Extended */
#define MCANX_XIDFC_LSE_Msk           (_U_(0x7F) << MCANX_XIDFC_LSE_Pos)
#define MCANX_XIDFC_LSE(value)        (MCANX_XIDFC_LSE_Msk & ((value) << MCANX_XIDFC_LSE_Pos))
#define MCANX_XIDFC_MASK              _U_(0x007FFFFF) /**< \brief (MCANX_XIDFC) MASK Register */

/* -------- MCANX_XIDAM : (CAN Offset: 0x90) (R/W 32) Extended ID AND Mask -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t EIDM:29;          /*!< bit:  0..28  Extended ID Mask                   */
    uint32_t :3;               /*!< bit: 29..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_XIDAM_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_XIDAM_OFFSET            0x90         /**< \brief (MCANX_XIDAM offset) Extended ID AND Mask */
#define MCANX_XIDAM_RESETVALUE        _U_(0x1FFFFFFF) /**< \brief (MCANX_XIDAM reset_value) Extended ID AND Mask */

#define MCANX_XIDAM_EIDM_Pos          0            /**< \brief (MCANX_XIDAM) Extended ID Mask */
#define MCANX_XIDAM_EIDM_Msk          (_U_(0x1FFFFFFF) << MCANX_XIDAM_EIDM_Pos)
#define MCANX_XIDAM_EIDM(value)       (MCANX_XIDAM_EIDM_Msk & ((value) << MCANX_XIDAM_EIDM_Pos))
#define MCANX_XIDAM_MASK              _U_(0x1FFFFFFF) /**< \brief (MCANX_XIDAM) MASK Register */

/* -------- MCANX_HPMS : (CAN Offset: 0x94) (R/  32) High Priority Message Status -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t BIDX:6;           /*!< bit:  0.. 5  Buffer Index                       */
    uint32_t MSI:2;            /*!< bit:  6.. 7  Message Storage Indicator          */
    uint32_t FIDX:7;           /*!< bit:  8..14  Filter Index                       */
    uint32_t FLST:1;           /*!< bit:     15  Filter List                        */
    uint32_t :16;              /*!< bit: 16..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_HPMS_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_HPMS_OFFSET             0x94         /**< \brief (MCANX_HPMS offset) High Priority Message Status */
#define MCANX_HPMS_RESETVALUE         _U_(0x00000000) /**< \brief (MCANX_HPMS reset_value) High Priority Message Status */

#define MCANX_HPMS_BIDX_Pos           0            /**< \brief (MCANX_HPMS) Buffer Index */
#define MCANX_HPMS_BIDX_Msk           (_U_(0x3F) << MCANX_HPMS_BIDX_Pos)
#define MCANX_HPMS_BIDX(value)        (MCANX_HPMS_BIDX_Msk & ((value) << MCANX_HPMS_BIDX_Pos))
#define MCANX_HPMS_MSI_Pos            6            /**< \brief (MCANX_HPMS) Message Storage Indicator */
#define MCANX_HPMS_MSI_Msk            (_U_(0x3) << MCANX_HPMS_MSI_Pos)
#define MCANX_HPMS_MSI(value)         (MCANX_HPMS_MSI_Msk & ((value) << MCANX_HPMS_MSI_Pos))
#define   MCANX_HPMS_MSI_NONE_Val           _U_(0x0)   /**< \brief (MCANX_HPMS) No FIFO selected */
#define   MCANX_HPMS_MSI_LOST_Val           _U_(0x1)   /**< \brief (MCANX_HPMS) FIFO message lost */
#define   MCANX_HPMS_MSI_FIFO0_Val          _U_(0x2)   /**< \brief (MCANX_HPMS) Message stored in FIFO 0 */
#define   MCANX_HPMS_MSI_FIFO1_Val          _U_(0x3)   /**< \brief (MCANX_HPMS) Message stored in FIFO 1 */
#define MCANX_HPMS_MSI_NONE           (MCANX_HPMS_MSI_NONE_Val         << MCANX_HPMS_MSI_Pos)
#define MCANX_HPMS_MSI_LOST           (MCANX_HPMS_MSI_LOST_Val         << MCANX_HPMS_MSI_Pos)
#define MCANX_HPMS_MSI_FIFO0          (MCANX_HPMS_MSI_FIFO0_Val        << MCANX_HPMS_MSI_Pos)
#define MCANX_HPMS_MSI_FIFO1          (MCANX_HPMS_MSI_FIFO1_Val        << MCANX_HPMS_MSI_Pos)
#define MCANX_HPMS_FIDX_Pos           8            /**< \brief (MCANX_HPMS) Filter Index */
#define MCANX_HPMS_FIDX_Msk           (_U_(0x7F) << MCANX_HPMS_FIDX_Pos)
#define MCANX_HPMS_FIDX(value)        (MCANX_HPMS_FIDX_Msk & ((value) << MCANX_HPMS_FIDX_Pos))
#define MCANX_HPMS_FLST_Pos           15           /**< \brief (MCANX_HPMS) Filter List */
#define MCANX_HPMS_FLST               (_U_(0x1) << MCANX_HPMS_FLST_Pos)
#define MCANX_HPMS_MASK               _U_(0x0000FFFF) /**< \brief (MCANX_HPMS) MASK Register */

/* -------- MCANX_NDAT1 : (CAN Offset: 0x98) (R/W 32) New Data 1 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t ND0:1;            /*!< bit:      0  New Data 0                         */
    uint32_t ND1:1;            /*!< bit:      1  New Data 1                         */
    uint32_t ND2:1;            /*!< bit:      2  New Data 2                         */
    uint32_t ND3:1;            /*!< bit:      3  New Data 3                         */
    uint32_t ND4:1;            /*!< bit:      4  New Data 4                         */
    uint32_t ND5:1;            /*!< bit:      5  New Data 5                         */
    uint32_t ND6:1;            /*!< bit:      6  New Data 6                         */
    uint32_t ND7:1;            /*!< bit:      7  New Data 7                         */
    uint32_t ND8:1;            /*!< bit:      8  New Data 8                         */
    uint32_t ND9:1;            /*!< bit:      9  New Data 9                         */
    uint32_t ND10:1;           /*!< bit:     10  New Data 10                        */
    uint32_t ND11:1;           /*!< bit:     11  New Data 11                        */
    uint32_t ND12:1;           /*!< bit:     12  New Data 12                        */
    uint32_t ND13:1;           /*!< bit:     13  New Data 13                        */
    uint32_t ND14:1;           /*!< bit:     14  New Data 14                        */
    uint32_t ND15:1;           /*!< bit:     15  New Data 15                        */
    uint32_t ND16:1;           /*!< bit:     16  New Data 16                        */
    uint32_t ND17:1;           /*!< bit:     17  New Data 17                        */
    uint32_t ND18:1;           /*!< bit:     18  New Data 18                        */
    uint32_t ND19:1;           /*!< bit:     19  New Data 19                        */
    uint32_t ND20:1;           /*!< bit:     20  New Data 20                        */
    uint32_t ND21:1;           /*!< bit:     21  New Data 21                        */
    uint32_t ND22:1;           /*!< bit:     22  New Data 22                        */
    uint32_t ND23:1;           /*!< bit:     23  New Data 23                        */
    uint32_t ND24:1;           /*!< bit:     24  New Data 24                        */
    uint32_t ND25:1;           /*!< bit:     25  New Data 25                        */
    uint32_t ND26:1;           /*!< bit:     26  New Data 26                        */
    uint32_t ND27:1;           /*!< bit:     27  New Data 27                        */
    uint32_t ND28:1;           /*!< bit:     28  New Data 28                        */
    uint32_t ND29:1;           /*!< bit:     29  New Data 29                        */
    uint32_t ND30:1;           /*!< bit:     30  New Data 30                        */
    uint32_t ND31:1;           /*!< bit:     31  New Data 31                        */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_NDAT1_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_NDAT1_OFFSET            0x98         /**< \brief (MCANX_NDAT1 offset) New Data 1 */
#define MCANX_NDAT1_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_NDAT1 reset_value) New Data 1 */

#define MCANX_NDAT1_ND0_Pos           0            /**< \brief (MCANX_NDAT1) New Data 0 */
#define MCANX_NDAT1_ND0               (_U_(0x1) << MCANX_NDAT1_ND0_Pos)
#define MCANX_NDAT1_ND1_Pos           1            /**< \brief (MCANX_NDAT1) New Data 1 */
#define MCANX_NDAT1_ND1               (_U_(0x1) << MCANX_NDAT1_ND1_Pos)
#define MCANX_NDAT1_ND2_Pos           2            /**< \brief (MCANX_NDAT1) New Data 2 */
#define MCANX_NDAT1_ND2               (_U_(0x1) << MCANX_NDAT1_ND2_Pos)
#define MCANX_NDAT1_ND3_Pos           3            /**< \brief (MCANX_NDAT1) New Data 3 */
#define MCANX_NDAT1_ND3               (_U_(0x1) << MCANX_NDAT1_ND3_Pos)
#define MCANX_NDAT1_ND4_Pos           4            /**< \brief (MCANX_NDAT1) New Data 4 */
#define MCANX_NDAT1_ND4               (_U_(0x1) << MCANX_NDAT1_ND4_Pos)
#define MCANX_NDAT1_ND5_Pos           5            /**< \brief (MCANX_NDAT1) New Data 5 */
#define MCANX_NDAT1_ND5               (_U_(0x1) << MCANX_NDAT1_ND5_Pos)
#define MCANX_NDAT1_ND6_Pos           6            /**< \brief (MCANX_NDAT1) New Data 6 */
#define MCANX_NDAT1_ND6               (_U_(0x1) << MCANX_NDAT1_ND6_Pos)
#define MCANX_NDAT1_ND7_Pos           7            /**< \brief (MCANX_NDAT1) New Data 7 */
#define MCANX_NDAT1_ND7               (_U_(0x1) << MCANX_NDAT1_ND7_Pos)
#define MCANX_NDAT1_ND8_Pos           8            /**< \brief (MCANX_NDAT1) New Data 8 */
#define MCANX_NDAT1_ND8               (_U_(0x1) << MCANX_NDAT1_ND8_Pos)
#define MCANX_NDAT1_ND9_Pos           9            /**< \brief (MCANX_NDAT1) New Data 9 */
#define MCANX_NDAT1_ND9               (_U_(0x1) << MCANX_NDAT1_ND9_Pos)
#define MCANX_NDAT1_ND10_Pos          10           /**< \brief (MCANX_NDAT1) New Data 10 */
#define MCANX_NDAT1_ND10              (_U_(0x1) << MCANX_NDAT1_ND10_Pos)
#define MCANX_NDAT1_ND11_Pos          11           /**< \brief (MCANX_NDAT1) New Data 11 */
#define MCANX_NDAT1_ND11              (_U_(0x1) << MCANX_NDAT1_ND11_Pos)
#define MCANX_NDAT1_ND12_Pos          12           /**< \brief (MCANX_NDAT1) New Data 12 */
#define MCANX_NDAT1_ND12              (_U_(0x1) << MCANX_NDAT1_ND12_Pos)
#define MCANX_NDAT1_ND13_Pos          13           /**< \brief (MCANX_NDAT1) New Data 13 */
#define MCANX_NDAT1_ND13              (_U_(0x1) << MCANX_NDAT1_ND13_Pos)
#define MCANX_NDAT1_ND14_Pos          14           /**< \brief (MCANX_NDAT1) New Data 14 */
#define MCANX_NDAT1_ND14              (_U_(0x1) << MCANX_NDAT1_ND14_Pos)
#define MCANX_NDAT1_ND15_Pos          15           /**< \brief (MCANX_NDAT1) New Data 15 */
#define MCANX_NDAT1_ND15              (_U_(0x1) << MCANX_NDAT1_ND15_Pos)
#define MCANX_NDAT1_ND16_Pos          16           /**< \brief (MCANX_NDAT1) New Data 16 */
#define MCANX_NDAT1_ND16              (_U_(0x1) << MCANX_NDAT1_ND16_Pos)
#define MCANX_NDAT1_ND17_Pos          17           /**< \brief (MCANX_NDAT1) New Data 17 */
#define MCANX_NDAT1_ND17              (_U_(0x1) << MCANX_NDAT1_ND17_Pos)
#define MCANX_NDAT1_ND18_Pos          18           /**< \brief (MCANX_NDAT1) New Data 18 */
#define MCANX_NDAT1_ND18              (_U_(0x1) << MCANX_NDAT1_ND18_Pos)
#define MCANX_NDAT1_ND19_Pos          19           /**< \brief (MCANX_NDAT1) New Data 19 */
#define MCANX_NDAT1_ND19              (_U_(0x1) << MCANX_NDAT1_ND19_Pos)
#define MCANX_NDAT1_ND20_Pos          20           /**< \brief (MCANX_NDAT1) New Data 20 */
#define MCANX_NDAT1_ND20              (_U_(0x1) << MCANX_NDAT1_ND20_Pos)
#define MCANX_NDAT1_ND21_Pos          21           /**< \brief (MCANX_NDAT1) New Data 21 */
#define MCANX_NDAT1_ND21              (_U_(0x1) << MCANX_NDAT1_ND21_Pos)
#define MCANX_NDAT1_ND22_Pos          22           /**< \brief (MCANX_NDAT1) New Data 22 */
#define MCANX_NDAT1_ND22              (_U_(0x1) << MCANX_NDAT1_ND22_Pos)
#define MCANX_NDAT1_ND23_Pos          23           /**< \brief (MCANX_NDAT1) New Data 23 */
#define MCANX_NDAT1_ND23              (_U_(0x1) << MCANX_NDAT1_ND23_Pos)
#define MCANX_NDAT1_ND24_Pos          24           /**< \brief (MCANX_NDAT1) New Data 24 */
#define MCANX_NDAT1_ND24              (_U_(0x1) << MCANX_NDAT1_ND24_Pos)
#define MCANX_NDAT1_ND25_Pos          25           /**< \brief (MCANX_NDAT1) New Data 25 */
#define MCANX_NDAT1_ND25              (_U_(0x1) << MCANX_NDAT1_ND25_Pos)
#define MCANX_NDAT1_ND26_Pos          26           /**< \brief (MCANX_NDAT1) New Data 26 */
#define MCANX_NDAT1_ND26              (_U_(0x1) << MCANX_NDAT1_ND26_Pos)
#define MCANX_NDAT1_ND27_Pos          27           /**< \brief (MCANX_NDAT1) New Data 27 */
#define MCANX_NDAT1_ND27              (_U_(0x1) << MCANX_NDAT1_ND27_Pos)
#define MCANX_NDAT1_ND28_Pos          28           /**< \brief (MCANX_NDAT1) New Data 28 */
#define MCANX_NDAT1_ND28              (_U_(0x1) << MCANX_NDAT1_ND28_Pos)
#define MCANX_NDAT1_ND29_Pos          29           /**< \brief (MCANX_NDAT1) New Data 29 */
#define MCANX_NDAT1_ND29              (_U_(0x1) << MCANX_NDAT1_ND29_Pos)
#define MCANX_NDAT1_ND30_Pos          30           /**< \brief (MCANX_NDAT1) New Data 30 */
#define MCANX_NDAT1_ND30              (_U_(0x1) << MCANX_NDAT1_ND30_Pos)
#define MCANX_NDAT1_ND31_Pos          31           /**< \brief (MCANX_NDAT1) New Data 31 */
#define MCANX_NDAT1_ND31              (_U_(0x1) << MCANX_NDAT1_ND31_Pos)
#define MCANX_NDAT1_MASK              _U_(0xFFFFFFFF) /**< \brief (MCANX_NDAT1) MASK Register */

/* -------- MCANX_NDAT2 : (CAN Offset: 0x9C) (R/W 32) New Data 2 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t ND32:1;           /*!< bit:      0  New Data 32                        */
    uint32_t ND33:1;           /*!< bit:      1  New Data 33                        */
    uint32_t ND34:1;           /*!< bit:      2  New Data 34                        */
    uint32_t ND35:1;           /*!< bit:      3  New Data 35                        */
    uint32_t ND36:1;           /*!< bit:      4  New Data 36                        */
    uint32_t ND37:1;           /*!< bit:      5  New Data 37                        */
    uint32_t ND38:1;           /*!< bit:      6  New Data 38                        */
    uint32_t ND39:1;           /*!< bit:      7  New Data 39                        */
    uint32_t ND40:1;           /*!< bit:      8  New Data 40                        */
    uint32_t ND41:1;           /*!< bit:      9  New Data 41                        */
    uint32_t ND42:1;           /*!< bit:     10  New Data 42                        */
    uint32_t ND43:1;           /*!< bit:     11  New Data 43                        */
    uint32_t ND44:1;           /*!< bit:     12  New Data 44                        */
    uint32_t ND45:1;           /*!< bit:     13  New Data 45                        */
    uint32_t ND46:1;           /*!< bit:     14  New Data 46                        */
    uint32_t ND47:1;           /*!< bit:     15  New Data 47                        */
    uint32_t ND48:1;           /*!< bit:     16  New Data 48                        */
    uint32_t ND49:1;           /*!< bit:     17  New Data 49                        */
    uint32_t ND50:1;           /*!< bit:     18  New Data 50                        */
    uint32_t ND51:1;           /*!< bit:     19  New Data 51                        */
    uint32_t ND52:1;           /*!< bit:     20  New Data 52                        */
    uint32_t ND53:1;           /*!< bit:     21  New Data 53                        */
    uint32_t ND54:1;           /*!< bit:     22  New Data 54                        */
    uint32_t ND55:1;           /*!< bit:     23  New Data 55                        */
    uint32_t ND56:1;           /*!< bit:     24  New Data 56                        */
    uint32_t ND57:1;           /*!< bit:     25  New Data 57                        */
    uint32_t ND58:1;           /*!< bit:     26  New Data 58                        */
    uint32_t ND59:1;           /*!< bit:     27  New Data 59                        */
    uint32_t ND60:1;           /*!< bit:     28  New Data 60                        */
    uint32_t ND61:1;           /*!< bit:     29  New Data 61                        */
    uint32_t ND62:1;           /*!< bit:     30  New Data 62                        */
    uint32_t ND63:1;           /*!< bit:     31  New Data 63                        */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_NDAT2_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_NDAT2_OFFSET            0x9C         /**< \brief (MCANX_NDAT2 offset) New Data 2 */
#define MCANX_NDAT2_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_NDAT2 reset_value) New Data 2 */

#define MCANX_NDAT2_ND32_Pos          0            /**< \brief (MCANX_NDAT2) New Data 32 */
#define MCANX_NDAT2_ND32              (_U_(0x1) << MCANX_NDAT2_ND32_Pos)
#define MCANX_NDAT2_ND33_Pos          1            /**< \brief (MCANX_NDAT2) New Data 33 */
#define MCANX_NDAT2_ND33              (_U_(0x1) << MCANX_NDAT2_ND33_Pos)
#define MCANX_NDAT2_ND34_Pos          2            /**< \brief (MCANX_NDAT2) New Data 34 */
#define MCANX_NDAT2_ND34              (_U_(0x1) << MCANX_NDAT2_ND34_Pos)
#define MCANX_NDAT2_ND35_Pos          3            /**< \brief (MCANX_NDAT2) New Data 35 */
#define MCANX_NDAT2_ND35              (_U_(0x1) << MCANX_NDAT2_ND35_Pos)
#define MCANX_NDAT2_ND36_Pos          4            /**< \brief (MCANX_NDAT2) New Data 36 */
#define MCANX_NDAT2_ND36              (_U_(0x1) << MCANX_NDAT2_ND36_Pos)
#define MCANX_NDAT2_ND37_Pos          5            /**< \brief (MCANX_NDAT2) New Data 37 */
#define MCANX_NDAT2_ND37              (_U_(0x1) << MCANX_NDAT2_ND37_Pos)
#define MCANX_NDAT2_ND38_Pos          6            /**< \brief (MCANX_NDAT2) New Data 38 */
#define MCANX_NDAT2_ND38              (_U_(0x1) << MCANX_NDAT2_ND38_Pos)
#define MCANX_NDAT2_ND39_Pos          7            /**< \brief (MCANX_NDAT2) New Data 39 */
#define MCANX_NDAT2_ND39              (_U_(0x1) << MCANX_NDAT2_ND39_Pos)
#define MCANX_NDAT2_ND40_Pos          8            /**< \brief (MCANX_NDAT2) New Data 40 */
#define MCANX_NDAT2_ND40              (_U_(0x1) << MCANX_NDAT2_ND40_Pos)
#define MCANX_NDAT2_ND41_Pos          9            /**< \brief (MCANX_NDAT2) New Data 41 */
#define MCANX_NDAT2_ND41              (_U_(0x1) << MCANX_NDAT2_ND41_Pos)
#define MCANX_NDAT2_ND42_Pos          10           /**< \brief (MCANX_NDAT2) New Data 42 */
#define MCANX_NDAT2_ND42              (_U_(0x1) << MCANX_NDAT2_ND42_Pos)
#define MCANX_NDAT2_ND43_Pos          11           /**< \brief (MCANX_NDAT2) New Data 43 */
#define MCANX_NDAT2_ND43              (_U_(0x1) << MCANX_NDAT2_ND43_Pos)
#define MCANX_NDAT2_ND44_Pos          12           /**< \brief (MCANX_NDAT2) New Data 44 */
#define MCANX_NDAT2_ND44              (_U_(0x1) << MCANX_NDAT2_ND44_Pos)
#define MCANX_NDAT2_ND45_Pos          13           /**< \brief (MCANX_NDAT2) New Data 45 */
#define MCANX_NDAT2_ND45              (_U_(0x1) << MCANX_NDAT2_ND45_Pos)
#define MCANX_NDAT2_ND46_Pos          14           /**< \brief (MCANX_NDAT2) New Data 46 */
#define MCANX_NDAT2_ND46              (_U_(0x1) << MCANX_NDAT2_ND46_Pos)
#define MCANX_NDAT2_ND47_Pos          15           /**< \brief (MCANX_NDAT2) New Data 47 */
#define MCANX_NDAT2_ND47              (_U_(0x1) << MCANX_NDAT2_ND47_Pos)
#define MCANX_NDAT2_ND48_Pos          16           /**< \brief (MCANX_NDAT2) New Data 48 */
#define MCANX_NDAT2_ND48              (_U_(0x1) << MCANX_NDAT2_ND48_Pos)
#define MCANX_NDAT2_ND49_Pos          17           /**< \brief (MCANX_NDAT2) New Data 49 */
#define MCANX_NDAT2_ND49              (_U_(0x1) << MCANX_NDAT2_ND49_Pos)
#define MCANX_NDAT2_ND50_Pos          18           /**< \brief (MCANX_NDAT2) New Data 50 */
#define MCANX_NDAT2_ND50              (_U_(0x1) << MCANX_NDAT2_ND50_Pos)
#define MCANX_NDAT2_ND51_Pos          19           /**< \brief (MCANX_NDAT2) New Data 51 */
#define MCANX_NDAT2_ND51              (_U_(0x1) << MCANX_NDAT2_ND51_Pos)
#define MCANX_NDAT2_ND52_Pos          20           /**< \brief (MCANX_NDAT2) New Data 52 */
#define MCANX_NDAT2_ND52              (_U_(0x1) << MCANX_NDAT2_ND52_Pos)
#define MCANX_NDAT2_ND53_Pos          21           /**< \brief (MCANX_NDAT2) New Data 53 */
#define MCANX_NDAT2_ND53              (_U_(0x1) << MCANX_NDAT2_ND53_Pos)
#define MCANX_NDAT2_ND54_Pos          22           /**< \brief (MCANX_NDAT2) New Data 54 */
#define MCANX_NDAT2_ND54              (_U_(0x1) << MCANX_NDAT2_ND54_Pos)
#define MCANX_NDAT2_ND55_Pos          23           /**< \brief (MCANX_NDAT2) New Data 55 */
#define MCANX_NDAT2_ND55              (_U_(0x1) << MCANX_NDAT2_ND55_Pos)
#define MCANX_NDAT2_ND56_Pos          24           /**< \brief (MCANX_NDAT2) New Data 56 */
#define MCANX_NDAT2_ND56              (_U_(0x1) << MCANX_NDAT2_ND56_Pos)
#define MCANX_NDAT2_ND57_Pos          25           /**< \brief (MCANX_NDAT2) New Data 57 */
#define MCANX_NDAT2_ND57              (_U_(0x1) << MCANX_NDAT2_ND57_Pos)
#define MCANX_NDAT2_ND58_Pos          26           /**< \brief (MCANX_NDAT2) New Data 58 */
#define MCANX_NDAT2_ND58              (_U_(0x1) << MCANX_NDAT2_ND58_Pos)
#define MCANX_NDAT2_ND59_Pos          27           /**< \brief (MCANX_NDAT2) New Data 59 */
#define MCANX_NDAT2_ND59              (_U_(0x1) << MCANX_NDAT2_ND59_Pos)
#define MCANX_NDAT2_ND60_Pos          28           /**< \brief (MCANX_NDAT2) New Data 60 */
#define MCANX_NDAT2_ND60              (_U_(0x1) << MCANX_NDAT2_ND60_Pos)
#define MCANX_NDAT2_ND61_Pos          29           /**< \brief (MCANX_NDAT2) New Data 61 */
#define MCANX_NDAT2_ND61              (_U_(0x1) << MCANX_NDAT2_ND61_Pos)
#define MCANX_NDAT2_ND62_Pos          30           /**< \brief (MCANX_NDAT2) New Data 62 */
#define MCANX_NDAT2_ND62              (_U_(0x1) << MCANX_NDAT2_ND62_Pos)
#define MCANX_NDAT2_ND63_Pos          31           /**< \brief (MCANX_NDAT2) New Data 63 */
#define MCANX_NDAT2_ND63              (_U_(0x1) << MCANX_NDAT2_ND63_Pos)
#define MCANX_NDAT2_MASK              _U_(0xFFFFFFFF) /**< \brief (MCANX_NDAT2) MASK Register */

/* -------- MCANX_RXF0C : (CAN Offset: 0xA0) (R/W 32) Rx FIFO 0 Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t F0SA:16;          /*!< bit:  0..15  Rx FIFO 0 Start Address            */
    uint32_t F0S:7;            /*!< bit: 16..22  Rx FIFO 0 Size                     */
    uint32_t :1;               /*!< bit:     23  Reserved                           */
    uint32_t F0WM:7;           /*!< bit: 24..30  Rx FIFO 0 Watermark                */
    uint32_t F0OM:1;           /*!< bit:     31  FIFO 0 Operation Mode              */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF0C_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF0C_OFFSET            0xA0         /**< \brief (MCANX_RXF0C offset) Rx FIFO 0 Configuration */
#define MCANX_RXF0C_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_RXF0C reset_value) Rx FIFO 0 Configuration */

#define MCANX_RXF0C_F0SA_Pos          0            /**< \brief (MCANX_RXF0C) Rx FIFO 0 Start Address */
#define MCANX_RXF0C_F0SA_Msk          (_U_(0xFFFF) << MCANX_RXF0C_F0SA_Pos)
#define MCANX_RXF0C_F0SA(value)       (MCANX_RXF0C_F0SA_Msk & ((value) << MCANX_RXF0C_F0SA_Pos))
#define MCANX_RXF0C_F0S_Pos           16           /**< \brief (MCANX_RXF0C) Rx FIFO 0 Size */
#define MCANX_RXF0C_F0S_Msk           (_U_(0x7F) << MCANX_RXF0C_F0S_Pos)
#define MCANX_RXF0C_F0S(value)        (MCANX_RXF0C_F0S_Msk & ((value) << MCANX_RXF0C_F0S_Pos))
#define MCANX_RXF0C_F0WM_Pos          24           /**< \brief (MCANX_RXF0C) Rx FIFO 0 Watermark */
#define MCANX_RXF0C_F0WM_Msk          (_U_(0x7F) << MCANX_RXF0C_F0WM_Pos)
#define MCANX_RXF0C_F0WM(value)       (MCANX_RXF0C_F0WM_Msk & ((value) << MCANX_RXF0C_F0WM_Pos))
#define MCANX_RXF0C_F0OM_Pos          31           /**< \brief (MCANX_RXF0C) FIFO 0 Operation Mode */
#define MCANX_RXF0C_F0OM              (_U_(0x1) << MCANX_RXF0C_F0OM_Pos)
#define MCANX_RXF0C_MASK              _U_(0xFF7FFFFF) /**< \brief (MCANX_RXF0C) MASK Register */

/* -------- MCANX_RXF0S : (CAN Offset: 0xA4) (R/  32) Rx FIFO 0 Status -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t F0FL:7;           /*!< bit:  0.. 6  Rx FIFO 0 Fill Level               */
    uint32_t :1;               /*!< bit:      7  Reserved                           */
    uint32_t F0GI:6;           /*!< bit:  8..13  Rx FIFO 0 Get Index                */
    uint32_t :2;               /*!< bit: 14..15  Reserved                           */
    uint32_t F0PI:6;           /*!< bit: 16..21  Rx FIFO 0 Put Index                */
    uint32_t :2;               /*!< bit: 22..23  Reserved                           */
    uint32_t F0F:1;            /*!< bit:     24  Rx FIFO 0 Full                     */
    uint32_t RF0L:1;           /*!< bit:     25  Rx FIFO 0 Message Lost             */
    uint32_t :6;               /*!< bit: 26..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF0S_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF0S_OFFSET            0xA4         /**< \brief (MCANX_RXF0S offset) Rx FIFO 0 Status */
#define MCANX_RXF0S_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_RXF0S reset_value) Rx FIFO 0 Status */

#define MCANX_RXF0S_F0FL_Pos          0            /**< \brief (MCANX_RXF0S) Rx FIFO 0 Fill Level */
#define MCANX_RXF0S_F0FL_Msk          (_U_(0x7F) << MCANX_RXF0S_F0FL_Pos)
#define MCANX_RXF0S_F0FL(value)       (MCANX_RXF0S_F0FL_Msk & ((value) << MCANX_RXF0S_F0FL_Pos))
#define MCANX_RXF0S_F0GI_Pos          8            /**< \brief (MCANX_RXF0S) Rx FIFO 0 Get Index */
#define MCANX_RXF0S_F0GI_Msk          (_U_(0x3F) << MCANX_RXF0S_F0GI_Pos)
#define MCANX_RXF0S_F0GI(value)       (MCANX_RXF0S_F0GI_Msk & ((value) << MCANX_RXF0S_F0GI_Pos))
#define MCANX_RXF0S_F0PI_Pos          16           /**< \brief (MCANX_RXF0S) Rx FIFO 0 Put Index */
#define MCANX_RXF0S_F0PI_Msk          (_U_(0x3F) << MCANX_RXF0S_F0PI_Pos)
#define MCANX_RXF0S_F0PI(value)       (MCANX_RXF0S_F0PI_Msk & ((value) << MCANX_RXF0S_F0PI_Pos))
#define MCANX_RXF0S_F0F_Pos           24           /**< \brief (MCANX_RXF0S) Rx FIFO 0 Full */
#define MCANX_RXF0S_F0F               (_U_(0x1) << MCANX_RXF0S_F0F_Pos)
#define MCANX_RXF0S_RF0L_Pos          25           /**< \brief (MCANX_RXF0S) Rx FIFO 0 Message Lost */
#define MCANX_RXF0S_RF0L              (_U_(0x1) << MCANX_RXF0S_RF0L_Pos)
#define MCANX_RXF0S_MASK              _U_(0x033F3F7F) /**< \brief (MCANX_RXF0S) MASK Register */

/* -------- MCANX_RXF0A : (CAN Offset: 0xA8) (R/W 32) Rx FIFO 0 Acknowledge -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t F0AI:6;           /*!< bit:  0.. 5  Rx FIFO 0 Acknowledge Index        */
    uint32_t :26;              /*!< bit:  6..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF0A_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF0A_OFFSET            0xA8         /**< \brief (MCANX_RXF0A offset) Rx FIFO 0 Acknowledge */
#define MCANX_RXF0A_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_RXF0A reset_value) Rx FIFO 0 Acknowledge */

#define MCANX_RXF0A_F0AI_Pos          0            /**< \brief (MCANX_RXF0A) Rx FIFO 0 Acknowledge Index */
#define MCANX_RXF0A_F0AI_Msk          (_U_(0x3F) << MCANX_RXF0A_F0AI_Pos)
#define MCANX_RXF0A_F0AI(value)       (MCANX_RXF0A_F0AI_Msk & ((value) << MCANX_RXF0A_F0AI_Pos))
#define MCANX_RXF0A_MASK              _U_(0x0000003F) /**< \brief (MCANX_RXF0A) MASK Register */

/* -------- MCANX_RXBC : (CAN Offset: 0xAC) (R/W 32) Rx Buffer Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t RBSA:16;          /*!< bit:  0..15  Rx Buffer Start Address            */
    uint32_t :16;              /*!< bit: 16..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXBC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXBC_OFFSET             0xAC         /**< \brief (MCANX_RXBC offset) Rx Buffer Configuration */
#define MCANX_RXBC_RESETVALUE         _U_(0x00000000) /**< \brief (MCANX_RXBC reset_value) Rx Buffer Configuration */

#define MCANX_RXBC_RBSA_Pos           0            /**< \brief (MCANX_RXBC) Rx Buffer Start Address */
#define MCANX_RXBC_RBSA_Msk           (_U_(0xFFFF) << MCANX_RXBC_RBSA_Pos)
#define MCANX_RXBC_RBSA(value)        (MCANX_RXBC_RBSA_Msk & ((value) << MCANX_RXBC_RBSA_Pos))
#define MCANX_RXBC_MASK               _U_(0x0000FFFF) /**< \brief (MCANX_RXBC) MASK Register */

/* -------- MCANX_RXF1C : (CAN Offset: 0xB0) (R/W 32) Rx FIFO 1 Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t F1SA:16;          /*!< bit:  0..15  Rx FIFO 1 Start Address            */
    uint32_t F1S:7;            /*!< bit: 16..22  Rx FIFO 1 Size                     */
    uint32_t :1;               /*!< bit:     23  Reserved                           */
    uint32_t F1WM:7;           /*!< bit: 24..30  Rx FIFO 1 Watermark                */
    uint32_t F1OM:1;           /*!< bit:     31  FIFO 1 Operation Mode              */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF1C_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF1C_OFFSET            0xB0         /**< \brief (MCANX_RXF1C offset) Rx FIFO 1 Configuration */
#define MCANX_RXF1C_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_RXF1C reset_value) Rx FIFO 1 Configuration */

#define MCANX_RXF1C_F1SA_Pos          0            /**< \brief (MCANX_RXF1C) Rx FIFO 1 Start Address */
#define MCANX_RXF1C_F1SA_Msk          (_U_(0xFFFF) << MCANX_RXF1C_F1SA_Pos)
#define MCANX_RXF1C_F1SA(value)       (MCANX_RXF1C_F1SA_Msk & ((value) << MCANX_RXF1C_F1SA_Pos))
#define MCANX_RXF1C_F1S_Pos           16           /**< \brief (MCANX_RXF1C) Rx FIFO 1 Size */
#define MCANX_RXF1C_F1S_Msk           (_U_(0x7F) << MCANX_RXF1C_F1S_Pos)
#define MCANX_RXF1C_F1S(value)        (MCANX_RXF1C_F1S_Msk & ((value) << MCANX_RXF1C_F1S_Pos))
#define MCANX_RXF1C_F1WM_Pos          24           /**< \brief (MCANX_RXF1C) Rx FIFO 1 Watermark */
#define MCANX_RXF1C_F1WM_Msk          (_U_(0x7F) << MCANX_RXF1C_F1WM_Pos)
#define MCANX_RXF1C_F1WM(value)       (MCANX_RXF1C_F1WM_Msk & ((value) << MCANX_RXF1C_F1WM_Pos))
#define MCANX_RXF1C_F1OM_Pos          31           /**< \brief (MCANX_RXF1C) FIFO 1 Operation Mode */
#define MCANX_RXF1C_F1OM              (_U_(0x1) << MCANX_RXF1C_F1OM_Pos)
#define MCANX_RXF1C_MASK              _U_(0xFF7FFFFF) /**< \brief (MCANX_RXF1C) MASK Register */

/* -------- MCANX_RXF1S : (CAN Offset: 0xB4) (R/  32) Rx FIFO 1 Status -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t F1FL:7;           /*!< bit:  0.. 6  Rx FIFO 1 Fill Level               */
    uint32_t :1;               /*!< bit:      7  Reserved                           */
    uint32_t F1GI:6;           /*!< bit:  8..13  Rx FIFO 1 Get Index                */
    uint32_t :2;               /*!< bit: 14..15  Reserved                           */
    uint32_t F1PI:6;           /*!< bit: 16..21  Rx FIFO 1 Put Index                */
    uint32_t :2;               /*!< bit: 22..23  Reserved                           */
    uint32_t F1F:1;            /*!< bit:     24  Rx FIFO 1 Full                     */
    uint32_t RF1L:1;           /*!< bit:     25  Rx FIFO 1 Message Lost             */
    uint32_t :4;               /*!< bit: 26..29  Reserved                           */
    uint32_t DMS:2;            /*!< bit: 30..31  Debug Message Status               */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF1S_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF1S_OFFSET            0xB4         /**< \brief (MCANX_RXF1S offset) Rx FIFO 1 Status */
#define MCANX_RXF1S_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_RXF1S reset_value) Rx FIFO 1 Status */

#define MCANX_RXF1S_F1FL_Pos          0            /**< \brief (MCANX_RXF1S) Rx FIFO 1 Fill Level */
#define MCANX_RXF1S_F1FL_Msk          (_U_(0x7F) << MCANX_RXF1S_F1FL_Pos)
#define MCANX_RXF1S_F1FL(value)       (MCANX_RXF1S_F1FL_Msk & ((value) << MCANX_RXF1S_F1FL_Pos))
#define MCANX_RXF1S_F1GI_Pos          8            /**< \brief (MCANX_RXF1S) Rx FIFO 1 Get Index */
#define MCANX_RXF1S_F1GI_Msk          (_U_(0x3F) << MCANX_RXF1S_F1GI_Pos)
#define MCANX_RXF1S_F1GI(value)       (MCANX_RXF1S_F1GI_Msk & ((value) << MCANX_RXF1S_F1GI_Pos))
#define MCANX_RXF1S_F1PI_Pos          16           /**< \brief (MCANX_RXF1S) Rx FIFO 1 Put Index */
#define MCANX_RXF1S_F1PI_Msk          (_U_(0x3F) << MCANX_RXF1S_F1PI_Pos)
#define MCANX_RXF1S_F1PI(value)       (MCANX_RXF1S_F1PI_Msk & ((value) << MCANX_RXF1S_F1PI_Pos))
#define MCANX_RXF1S_F1F_Pos           24           /**< \brief (MCANX_RXF1S) Rx FIFO 1 Full */
#define MCANX_RXF1S_F1F               (_U_(0x1) << MCANX_RXF1S_F1F_Pos)
#define MCANX_RXF1S_RF1L_Pos          25           /**< \brief (MCANX_RXF1S) Rx FIFO 1 Message Lost */
#define MCANX_RXF1S_RF1L              (_U_(0x1) << MCANX_RXF1S_RF1L_Pos)
#define MCANX_RXF1S_DMS_Pos           30           /**< \brief (MCANX_RXF1S) Debug Message Status */
#define MCANX_RXF1S_DMS_Msk           (_U_(0x3) << MCANX_RXF1S_DMS_Pos)
#define MCANX_RXF1S_DMS(value)        (MCANX_RXF1S_DMS_Msk & ((value) << MCANX_RXF1S_DMS_Pos))
#define   MCANX_RXF1S_DMS_IDLE_Val          _U_(0x0)   /**< \brief (MCANX_RXF1S) Idle state */
#define   MCANX_RXF1S_DMS_DBGA_Val          _U_(0x1)   /**< \brief (MCANX_RXF1S) Debug message A received */
#define   MCANX_RXF1S_DMS_DBGB_Val          _U_(0x2)   /**< \brief (MCANX_RXF1S) Debug message A/B received */
#define   MCANX_RXF1S_DMS_DBGC_Val          _U_(0x3)   /**< \brief (MCANX_RXF1S) Debug message A/B/C received, DMA request set */
#define MCANX_RXF1S_DMS_IDLE          (MCANX_RXF1S_DMS_IDLE_Val        << MCANX_RXF1S_DMS_Pos)
#define MCANX_RXF1S_DMS_DBGA          (MCANX_RXF1S_DMS_DBGA_Val        << MCANX_RXF1S_DMS_Pos)
#define MCANX_RXF1S_DMS_DBGB          (MCANX_RXF1S_DMS_DBGB_Val        << MCANX_RXF1S_DMS_Pos)
#define MCANX_RXF1S_DMS_DBGC          (MCANX_RXF1S_DMS_DBGC_Val        << MCANX_RXF1S_DMS_Pos)
#define MCANX_RXF1S_MASK              _U_(0xC33F3F7F) /**< \brief (MCANX_RXF1S) MASK Register */

/* -------- MCANX_RXF1A : (CAN Offset: 0xB8) (R/W 32) Rx FIFO 1 Acknowledge -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t F1AI:6;           /*!< bit:  0.. 5  Rx FIFO 1 Acknowledge Index        */
    uint32_t :26;              /*!< bit:  6..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF1A_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF1A_OFFSET            0xB8         /**< \brief (MCANX_RXF1A offset) Rx FIFO 1 Acknowledge */
#define MCANX_RXF1A_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_RXF1A reset_value) Rx FIFO 1 Acknowledge */

#define MCANX_RXF1A_F1AI_Pos          0            /**< \brief (MCANX_RXF1A) Rx FIFO 1 Acknowledge Index */
#define MCANX_RXF1A_F1AI_Msk          (_U_(0x3F) << MCANX_RXF1A_F1AI_Pos)
#define MCANX_RXF1A_F1AI(value)       (MCANX_RXF1A_F1AI_Msk & ((value) << MCANX_RXF1A_F1AI_Pos))
#define MCANX_RXF1A_MASK              _U_(0x0000003F) /**< \brief (MCANX_RXF1A) MASK Register */

/* -------- MCANX_RXESC : (CAN Offset: 0xBC) (R/W 32) Rx Buffer / FIFO Element Size Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t F0DS:3;           /*!< bit:  0.. 2  Rx FIFO 0 Data Field Size          */
    uint32_t :1;               /*!< bit:      3  Reserved                           */
    uint32_t F1DS:3;           /*!< bit:  4.. 6  Rx FIFO 1 Data Field Size          */
    uint32_t :1;               /*!< bit:      7  Reserved                           */
    uint32_t RBDS:3;           /*!< bit:  8..10  Rx Buffer Data Field Size          */
    uint32_t :21;              /*!< bit: 11..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXESC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXESC_OFFSET            0xBC         /**< \brief (MCANX_RXESC offset) Rx Buffer / FIFO Element Size Configuration */
#define MCANX_RXESC_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_RXESC reset_value) Rx Buffer / FIFO Element Size Configuration */

#define MCANX_RXESC_F0DS_Pos          0            /**< \brief (MCANX_RXESC) Rx FIFO 0 Data Field Size */
#define MCANX_RXESC_F0DS_Msk          (_U_(0x7) << MCANX_RXESC_F0DS_Pos)
#define MCANX_RXESC_F0DS(value)       (MCANX_RXESC_F0DS_Msk & ((value) << MCANX_RXESC_F0DS_Pos))
#define   MCANX_RXESC_F0DS_DATA8_Val        _U_(0x0)   /**< \brief (MCANX_RXESC) 8 byte data field */
#define   MCANX_RXESC_F0DS_DATA12_Val       _U_(0x1)   /**< \brief (MCANX_RXESC) 12 byte data field */
#define   MCANX_RXESC_F0DS_DATA16_Val       _U_(0x2)   /**< \brief (MCANX_RXESC) 16 byte data field */
#define   MCANX_RXESC_F0DS_DATA20_Val       _U_(0x3)   /**< \brief (MCANX_RXESC) 20 byte data field */
#define   MCANX_RXESC_F0DS_DATA24_Val       _U_(0x4)   /**< \brief (MCANX_RXESC) 24 byte data field */
#define   MCANX_RXESC_F0DS_DATA32_Val       _U_(0x5)   /**< \brief (MCANX_RXESC) 32 byte data field */
#define   MCANX_RXESC_F0DS_DATA48_Val       _U_(0x6)   /**< \brief (MCANX_RXESC) 48 byte data field */
#define   MCANX_RXESC_F0DS_DATA64_Val       _U_(0x7)   /**< \brief (MCANX_RXESC) 64 byte data field */
#define MCANX_RXESC_F0DS_DATA8        (MCANX_RXESC_F0DS_DATA8_Val      << MCANX_RXESC_F0DS_Pos)
#define MCANX_RXESC_F0DS_DATA12       (MCANX_RXESC_F0DS_DATA12_Val     << MCANX_RXESC_F0DS_Pos)
#define MCANX_RXESC_F0DS_DATA16       (MCANX_RXESC_F0DS_DATA16_Val     << MCANX_RXESC_F0DS_Pos)
#define MCANX_RXESC_F0DS_DATA20       (MCANX_RXESC_F0DS_DATA20_Val     << MCANX_RXESC_F0DS_Pos)
#define MCANX_RXESC_F0DS_DATA24       (MCANX_RXESC_F0DS_DATA24_Val     << MCANX_RXESC_F0DS_Pos)
#define MCANX_RXESC_F0DS_DATA32       (MCANX_RXESC_F0DS_DATA32_Val     << MCANX_RXESC_F0DS_Pos)
#define MCANX_RXESC_F0DS_DATA48       (MCANX_RXESC_F0DS_DATA48_Val     << MCANX_RXESC_F0DS_Pos)
#define MCANX_RXESC_F0DS_DATA64       (MCANX_RXESC_F0DS_DATA64_Val     << MCANX_RXESC_F0DS_Pos)
#define MCANX_RXESC_F1DS_Pos          4            /**< \brief (MCANX_RXESC) Rx FIFO 1 Data Field Size */
#define MCANX_RXESC_F1DS_Msk          (_U_(0x7) << MCANX_RXESC_F1DS_Pos)
#define MCANX_RXESC_F1DS(value)       (MCANX_RXESC_F1DS_Msk & ((value) << MCANX_RXESC_F1DS_Pos))
#define   MCANX_RXESC_F1DS_DATA8_Val        _U_(0x0)   /**< \brief (MCANX_RXESC) 8 byte data field */
#define   MCANX_RXESC_F1DS_DATA12_Val       _U_(0x1)   /**< \brief (MCANX_RXESC) 12 byte data field */
#define   MCANX_RXESC_F1DS_DATA16_Val       _U_(0x2)   /**< \brief (MCANX_RXESC) 16 byte data field */
#define   MCANX_RXESC_F1DS_DATA20_Val       _U_(0x3)   /**< \brief (MCANX_RXESC) 20 byte data field */
#define   MCANX_RXESC_F1DS_DATA24_Val       _U_(0x4)   /**< \brief (MCANX_RXESC) 24 byte data field */
#define   MCANX_RXESC_F1DS_DATA32_Val       _U_(0x5)   /**< \brief (MCANX_RXESC) 32 byte data field */
#define   MCANX_RXESC_F1DS_DATA48_Val       _U_(0x6)   /**< \brief (MCANX_RXESC) 48 byte data field */
#define   MCANX_RXESC_F1DS_DATA64_Val       _U_(0x7)   /**< \brief (MCANX_RXESC) 64 byte data field */
#define MCANX_RXESC_F1DS_DATA8        (MCANX_RXESC_F1DS_DATA8_Val      << MCANX_RXESC_F1DS_Pos)
#define MCANX_RXESC_F1DS_DATA12       (MCANX_RXESC_F1DS_DATA12_Val     << MCANX_RXESC_F1DS_Pos)
#define MCANX_RXESC_F1DS_DATA16       (MCANX_RXESC_F1DS_DATA16_Val     << MCANX_RXESC_F1DS_Pos)
#define MCANX_RXESC_F1DS_DATA20       (MCANX_RXESC_F1DS_DATA20_Val     << MCANX_RXESC_F1DS_Pos)
#define MCANX_RXESC_F1DS_DATA24       (MCANX_RXESC_F1DS_DATA24_Val     << MCANX_RXESC_F1DS_Pos)
#define MCANX_RXESC_F1DS_DATA32       (MCANX_RXESC_F1DS_DATA32_Val     << MCANX_RXESC_F1DS_Pos)
#define MCANX_RXESC_F1DS_DATA48       (MCANX_RXESC_F1DS_DATA48_Val     << MCANX_RXESC_F1DS_Pos)
#define MCANX_RXESC_F1DS_DATA64       (MCANX_RXESC_F1DS_DATA64_Val     << MCANX_RXESC_F1DS_Pos)
#define MCANX_RXESC_RBDS_Pos          8            /**< \brief (MCANX_RXESC) Rx Buffer Data Field Size */
#define MCANX_RXESC_RBDS_Msk          (_U_(0x7) << MCANX_RXESC_RBDS_Pos)
#define MCANX_RXESC_RBDS(value)       (MCANX_RXESC_RBDS_Msk & ((value) << MCANX_RXESC_RBDS_Pos))
#define   MCANX_RXESC_RBDS_DATA8_Val        _U_(0x0)   /**< \brief (MCANX_RXESC) 8 byte data field */
#define   MCANX_RXESC_RBDS_DATA12_Val       _U_(0x1)   /**< \brief (MCANX_RXESC) 12 byte data field */
#define   MCANX_RXESC_RBDS_DATA16_Val       _U_(0x2)   /**< \brief (MCANX_RXESC) 16 byte data field */
#define   MCANX_RXESC_RBDS_DATA20_Val       _U_(0x3)   /**< \brief (MCANX_RXESC) 20 byte data field */
#define   MCANX_RXESC_RBDS_DATA24_Val       _U_(0x4)   /**< \brief (MCANX_RXESC) 24 byte data field */
#define   MCANX_RXESC_RBDS_DATA32_Val       _U_(0x5)   /**< \brief (MCANX_RXESC) 32 byte data field */
#define   MCANX_RXESC_RBDS_DATA48_Val       _U_(0x6)   /**< \brief (MCANX_RXESC) 48 byte data field */
#define   MCANX_RXESC_RBDS_DATA64_Val       _U_(0x7)   /**< \brief (MCANX_RXESC) 64 byte data field */
#define MCANX_RXESC_RBDS_DATA8        (MCANX_RXESC_RBDS_DATA8_Val      << MCANX_RXESC_RBDS_Pos)
#define MCANX_RXESC_RBDS_DATA12       (MCANX_RXESC_RBDS_DATA12_Val     << MCANX_RXESC_RBDS_Pos)
#define MCANX_RXESC_RBDS_DATA16       (MCANX_RXESC_RBDS_DATA16_Val     << MCANX_RXESC_RBDS_Pos)
#define MCANX_RXESC_RBDS_DATA20       (MCANX_RXESC_RBDS_DATA20_Val     << MCANX_RXESC_RBDS_Pos)
#define MCANX_RXESC_RBDS_DATA24       (MCANX_RXESC_RBDS_DATA24_Val     << MCANX_RXESC_RBDS_Pos)
#define MCANX_RXESC_RBDS_DATA32       (MCANX_RXESC_RBDS_DATA32_Val     << MCANX_RXESC_RBDS_Pos)
#define MCANX_RXESC_RBDS_DATA48       (MCANX_RXESC_RBDS_DATA48_Val     << MCANX_RXESC_RBDS_Pos)
#define MCANX_RXESC_RBDS_DATA64       (MCANX_RXESC_RBDS_DATA64_Val     << MCANX_RXESC_RBDS_Pos)
#define MCANX_RXESC_MASK              _U_(0x00000777) /**< \brief (MCANX_RXESC) MASK Register */

/* -------- MCANX_TXBC : (CAN Offset: 0xC0) (R/W 32) Tx Buffer Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TBSA:16;          /*!< bit:  0..15  Tx Buffers Start Address           */
    uint32_t NDTB:6;           /*!< bit: 16..21  Number of Dedicated Transmit Buffers */
    uint32_t :2;               /*!< bit: 22..23  Reserved                           */
    uint32_t TFQS:6;           /*!< bit: 24..29  Transmit FIFO/Queue Size           */
    uint32_t TFQM:1;           /*!< bit:     30  Tx FIFO/Queue Mode                 */
    uint32_t :1;               /*!< bit:     31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXBC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXBC_OFFSET             0xC0         /**< \brief (MCANX_TXBC offset) Tx Buffer Configuration */
#define MCANX_TXBC_RESETVALUE         _U_(0x00000000) /**< \brief (MCANX_TXBC reset_value) Tx Buffer Configuration */

#define MCANX_TXBC_TBSA_Pos           0            /**< \brief (MCANX_TXBC) Tx Buffers Start Address */
#define MCANX_TXBC_TBSA_Msk           (_U_(0xFFFF) << MCANX_TXBC_TBSA_Pos)
#define MCANX_TXBC_TBSA(value)        (MCANX_TXBC_TBSA_Msk & ((value) << MCANX_TXBC_TBSA_Pos))
#define MCANX_TXBC_NDTB_Pos           16           /**< \brief (MCANX_TXBC) Number of Dedicated Transmit Buffers */
#define MCANX_TXBC_NDTB_Msk           (_U_(0x3F) << MCANX_TXBC_NDTB_Pos)
#define MCANX_TXBC_NDTB(value)        (MCANX_TXBC_NDTB_Msk & ((value) << MCANX_TXBC_NDTB_Pos))
#define MCANX_TXBC_TFQS_Pos           24           /**< \brief (MCANX_TXBC) Transmit FIFO/Queue Size */
#define MCANX_TXBC_TFQS_Msk           (_U_(0x3F) << MCANX_TXBC_TFQS_Pos)
#define MCANX_TXBC_TFQS(value)        (MCANX_TXBC_TFQS_Msk & ((value) << MCANX_TXBC_TFQS_Pos))
#define MCANX_TXBC_TFQM_Pos           30           /**< \brief (MCANX_TXBC) Tx FIFO/Queue Mode */
#define MCANX_TXBC_TFQM               (_U_(0x1) << MCANX_TXBC_TFQM_Pos)
#define MCANX_TXBC_MASK               _U_(0x7F3FFFFF) /**< \brief (MCANX_TXBC) MASK Register */

/* -------- MCANX_TXFQS : (CAN Offset: 0xC4) (R/  32) Tx FIFO / Queue Status -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TFFL:6;           /*!< bit:  0.. 5  Tx FIFO Free Level                 */
    uint32_t :2;               /*!< bit:  6.. 7  Reserved                           */
    uint32_t TFGI:5;           /*!< bit:  8..12  Tx FIFO Get Index                  */
    uint32_t :3;               /*!< bit: 13..15  Reserved                           */
    uint32_t TFQPI:5;          /*!< bit: 16..20  Tx FIFO/Queue Put Index            */
    uint32_t TFQF:1;           /*!< bit:     21  Tx FIFO/Queue Full                 */
    uint32_t :10;              /*!< bit: 22..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXFQS_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXFQS_OFFSET            0xC4         /**< \brief (MCANX_TXFQS offset) Tx FIFO / Queue Status */
#define MCANX_TXFQS_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_TXFQS reset_value) Tx FIFO / Queue Status */

#define MCANX_TXFQS_TFFL_Pos          0            /**< \brief (MCANX_TXFQS) Tx FIFO Free Level */
#define MCANX_TXFQS_TFFL_Msk          (_U_(0x3F) << MCANX_TXFQS_TFFL_Pos)
#define MCANX_TXFQS_TFFL(value)       (MCANX_TXFQS_TFFL_Msk & ((value) << MCANX_TXFQS_TFFL_Pos))
#define MCANX_TXFQS_TFGI_Pos          8            /**< \brief (MCANX_TXFQS) Tx FIFO Get Index */
#define MCANX_TXFQS_TFGI_Msk          (_U_(0x1F) << MCANX_TXFQS_TFGI_Pos)
#define MCANX_TXFQS_TFGI(value)       (MCANX_TXFQS_TFGI_Msk & ((value) << MCANX_TXFQS_TFGI_Pos))
#define MCANX_TXFQS_TFQPI_Pos         16           /**< \brief (MCANX_TXFQS) Tx FIFO/Queue Put Index */
#define MCANX_TXFQS_TFQPI_Msk         (_U_(0x1F) << MCANX_TXFQS_TFQPI_Pos)
#define MCANX_TXFQS_TFQPI(value)      (MCANX_TXFQS_TFQPI_Msk & ((value) << MCANX_TXFQS_TFQPI_Pos))
#define MCANX_TXFQS_TFQF_Pos          21           /**< \brief (MCANX_TXFQS) Tx FIFO/Queue Full */
#define MCANX_TXFQS_TFQF              (_U_(0x1) << MCANX_TXFQS_TFQF_Pos)
#define MCANX_TXFQS_MASK              _U_(0x003F1F3F) /**< \brief (MCANX_TXFQS) MASK Register */

/* -------- MCANX_TXESC : (CAN Offset: 0xC8) (R/W 32) Tx Buffer Element Size Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TBDS:3;           /*!< bit:  0.. 2  Tx Buffer Data Field Size          */
    uint32_t :29;              /*!< bit:  3..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXESC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXESC_OFFSET            0xC8         /**< \brief (MCANX_TXESC offset) Tx Buffer Element Size Configuration */
#define MCANX_TXESC_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_TXESC reset_value) Tx Buffer Element Size Configuration */

#define MCANX_TXESC_TBDS_Pos          0            /**< \brief (MCANX_TXESC) Tx Buffer Data Field Size */
#define MCANX_TXESC_TBDS_Msk          (_U_(0x7) << MCANX_TXESC_TBDS_Pos)
#define MCANX_TXESC_TBDS(value)       (MCANX_TXESC_TBDS_Msk & ((value) << MCANX_TXESC_TBDS_Pos))
#define   MCANX_TXESC_TBDS_DATA8_Val        _U_(0x0)   /**< \brief (MCANX_TXESC) 8 byte data field */
#define   MCANX_TXESC_TBDS_DATA12_Val       _U_(0x1)   /**< \brief (MCANX_TXESC) 12 byte data field */
#define   MCANX_TXESC_TBDS_DATA16_Val       _U_(0x2)   /**< \brief (MCANX_TXESC) 16 byte data field */
#define   MCANX_TXESC_TBDS_DATA20_Val       _U_(0x3)   /**< \brief (MCANX_TXESC) 20 byte data field */
#define   MCANX_TXESC_TBDS_DATA24_Val       _U_(0x4)   /**< \brief (MCANX_TXESC) 24 byte data field */
#define   MCANX_TXESC_TBDS_DATA32_Val       _U_(0x5)   /**< \brief (MCANX_TXESC) 32 byte data field */
#define   MCANX_TXESC_TBDS_DATA48_Val       _U_(0x6)   /**< \brief (MCANX_TXESC) 48 byte data field */
#define   MCANX_TXESC_TBDS_DATA64_Val       _U_(0x7)   /**< \brief (MCANX_TXESC) 64 byte data field */
#define MCANX_TXESC_TBDS_DATA8        (MCANX_TXESC_TBDS_DATA8_Val      << MCANX_TXESC_TBDS_Pos)
#define MCANX_TXESC_TBDS_DATA12       (MCANX_TXESC_TBDS_DATA12_Val     << MCANX_TXESC_TBDS_Pos)
#define MCANX_TXESC_TBDS_DATA16       (MCANX_TXESC_TBDS_DATA16_Val     << MCANX_TXESC_TBDS_Pos)
#define MCANX_TXESC_TBDS_DATA20       (MCANX_TXESC_TBDS_DATA20_Val     << MCANX_TXESC_TBDS_Pos)
#define MCANX_TXESC_TBDS_DATA24       (MCANX_TXESC_TBDS_DATA24_Val     << MCANX_TXESC_TBDS_Pos)
#define MCANX_TXESC_TBDS_DATA32       (MCANX_TXESC_TBDS_DATA32_Val     << MCANX_TXESC_TBDS_Pos)
#define MCANX_TXESC_TBDS_DATA48       (MCANX_TXESC_TBDS_DATA48_Val     << MCANX_TXESC_TBDS_Pos)
#define MCANX_TXESC_TBDS_DATA64       (MCANX_TXESC_TBDS_DATA64_Val     << MCANX_TXESC_TBDS_Pos)
#define MCANX_TXESC_MASK              _U_(0x00000007) /**< \brief (MCANX_TXESC) MASK Register */

/* -------- MCANX_TXBRP : (CAN Offset: 0xCC) (R/  32) Tx Buffer Request Pending -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TRP0:1;           /*!< bit:      0  Transmission Request Pending 0     */
    uint32_t TRP1:1;           /*!< bit:      1  Transmission Request Pending 1     */
    uint32_t TRP2:1;           /*!< bit:      2  Transmission Request Pending 2     */
    uint32_t TRP3:1;           /*!< bit:      3  Transmission Request Pending 3     */
    uint32_t TRP4:1;           /*!< bit:      4  Transmission Request Pending 4     */
    uint32_t TRP5:1;           /*!< bit:      5  Transmission Request Pending 5     */
    uint32_t TRP6:1;           /*!< bit:      6  Transmission Request Pending 6     */
    uint32_t TRP7:1;           /*!< bit:      7  Transmission Request Pending 7     */
    uint32_t TRP8:1;           /*!< bit:      8  Transmission Request Pending 8     */
    uint32_t TRP9:1;           /*!< bit:      9  Transmission Request Pending 9     */
    uint32_t TRP10:1;          /*!< bit:     10  Transmission Request Pending 10    */
    uint32_t TRP11:1;          /*!< bit:     11  Transmission Request Pending 11    */
    uint32_t TRP12:1;          /*!< bit:     12  Transmission Request Pending 12    */
    uint32_t TRP13:1;          /*!< bit:     13  Transmission Request Pending 13    */
    uint32_t TRP14:1;          /*!< bit:     14  Transmission Request Pending 14    */
    uint32_t TRP15:1;          /*!< bit:     15  Transmission Request Pending 15    */
    uint32_t TRP16:1;          /*!< bit:     16  Transmission Request Pending 16    */
    uint32_t TRP17:1;          /*!< bit:     17  Transmission Request Pending 17    */
    uint32_t TRP18:1;          /*!< bit:     18  Transmission Request Pending 18    */
    uint32_t TRP19:1;          /*!< bit:     19  Transmission Request Pending 19    */
    uint32_t TRP20:1;          /*!< bit:     20  Transmission Request Pending 20    */
    uint32_t TRP21:1;          /*!< bit:     21  Transmission Request Pending 21    */
    uint32_t TRP22:1;          /*!< bit:     22  Transmission Request Pending 22    */
    uint32_t TRP23:1;          /*!< bit:     23  Transmission Request Pending 23    */
    uint32_t TRP24:1;          /*!< bit:     24  Transmission Request Pending 24    */
    uint32_t TRP25:1;          /*!< bit:     25  Transmission Request Pending 25    */
    uint32_t TRP26:1;          /*!< bit:     26  Transmission Request Pending 26    */
    uint32_t TRP27:1;          /*!< bit:     27  Transmission Request Pending 27    */
    uint32_t TRP28:1;          /*!< bit:     28  Transmission Request Pending 28    */
    uint32_t TRP29:1;          /*!< bit:     29  Transmission Request Pending 29    */
    uint32_t TRP30:1;          /*!< bit:     30  Transmission Request Pending 30    */
    uint32_t TRP31:1;          /*!< bit:     31  Transmission Request Pending 31    */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXBRP_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXBRP_OFFSET            0xCC         /**< \brief (MCANX_TXBRP offset) Tx Buffer Request Pending */
#define MCANX_TXBRP_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_TXBRP reset_value) Tx Buffer Request Pending */

#define MCANX_TXBRP_TRP0_Pos          0            /**< \brief (MCANX_TXBRP) Transmission Request Pending 0 */
#define MCANX_TXBRP_TRP0              (_U_(0x1) << MCANX_TXBRP_TRP0_Pos)
#define MCANX_TXBRP_TRP1_Pos          1            /**< \brief (MCANX_TXBRP) Transmission Request Pending 1 */
#define MCANX_TXBRP_TRP1              (_U_(0x1) << MCANX_TXBRP_TRP1_Pos)
#define MCANX_TXBRP_TRP2_Pos          2            /**< \brief (MCANX_TXBRP) Transmission Request Pending 2 */
#define MCANX_TXBRP_TRP2              (_U_(0x1) << MCANX_TXBRP_TRP2_Pos)
#define MCANX_TXBRP_TRP3_Pos          3            /**< \brief (MCANX_TXBRP) Transmission Request Pending 3 */
#define MCANX_TXBRP_TRP3              (_U_(0x1) << MCANX_TXBRP_TRP3_Pos)
#define MCANX_TXBRP_TRP4_Pos          4            /**< \brief (MCANX_TXBRP) Transmission Request Pending 4 */
#define MCANX_TXBRP_TRP4              (_U_(0x1) << MCANX_TXBRP_TRP4_Pos)
#define MCANX_TXBRP_TRP5_Pos          5            /**< \brief (MCANX_TXBRP) Transmission Request Pending 5 */
#define MCANX_TXBRP_TRP5              (_U_(0x1) << MCANX_TXBRP_TRP5_Pos)
#define MCANX_TXBRP_TRP6_Pos          6            /**< \brief (MCANX_TXBRP) Transmission Request Pending 6 */
#define MCANX_TXBRP_TRP6              (_U_(0x1) << MCANX_TXBRP_TRP6_Pos)
#define MCANX_TXBRP_TRP7_Pos          7            /**< \brief (MCANX_TXBRP) Transmission Request Pending 7 */
#define MCANX_TXBRP_TRP7              (_U_(0x1) << MCANX_TXBRP_TRP7_Pos)
#define MCANX_TXBRP_TRP8_Pos          8            /**< \brief (MCANX_TXBRP) Transmission Request Pending 8 */
#define MCANX_TXBRP_TRP8              (_U_(0x1) << MCANX_TXBRP_TRP8_Pos)
#define MCANX_TXBRP_TRP9_Pos          9            /**< \brief (MCANX_TXBRP) Transmission Request Pending 9 */
#define MCANX_TXBRP_TRP9              (_U_(0x1) << MCANX_TXBRP_TRP9_Pos)
#define MCANX_TXBRP_TRP10_Pos         10           /**< \brief (MCANX_TXBRP) Transmission Request Pending 10 */
#define MCANX_TXBRP_TRP10             (_U_(0x1) << MCANX_TXBRP_TRP10_Pos)
#define MCANX_TXBRP_TRP11_Pos         11           /**< \brief (MCANX_TXBRP) Transmission Request Pending 11 */
#define MCANX_TXBRP_TRP11             (_U_(0x1) << MCANX_TXBRP_TRP11_Pos)
#define MCANX_TXBRP_TRP12_Pos         12           /**< \brief (MCANX_TXBRP) Transmission Request Pending 12 */
#define MCANX_TXBRP_TRP12             (_U_(0x1) << MCANX_TXBRP_TRP12_Pos)
#define MCANX_TXBRP_TRP13_Pos         13           /**< \brief (MCANX_TXBRP) Transmission Request Pending 13 */
#define MCANX_TXBRP_TRP13             (_U_(0x1) << MCANX_TXBRP_TRP13_Pos)
#define MCANX_TXBRP_TRP14_Pos         14           /**< \brief (MCANX_TXBRP) Transmission Request Pending 14 */
#define MCANX_TXBRP_TRP14             (_U_(0x1) << MCANX_TXBRP_TRP14_Pos)
#define MCANX_TXBRP_TRP15_Pos         15           /**< \brief (MCANX_TXBRP) Transmission Request Pending 15 */
#define MCANX_TXBRP_TRP15             (_U_(0x1) << MCANX_TXBRP_TRP15_Pos)
#define MCANX_TXBRP_TRP16_Pos         16           /**< \brief (MCANX_TXBRP) Transmission Request Pending 16 */
#define MCANX_TXBRP_TRP16             (_U_(0x1) << MCANX_TXBRP_TRP16_Pos)
#define MCANX_TXBRP_TRP17_Pos         17           /**< \brief (MCANX_TXBRP) Transmission Request Pending 17 */
#define MCANX_TXBRP_TRP17             (_U_(0x1) << MCANX_TXBRP_TRP17_Pos)
#define MCANX_TXBRP_TRP18_Pos         18           /**< \brief (MCANX_TXBRP) Transmission Request Pending 18 */
#define MCANX_TXBRP_TRP18             (_U_(0x1) << MCANX_TXBRP_TRP18_Pos)
#define MCANX_TXBRP_TRP19_Pos         19           /**< \brief (MCANX_TXBRP) Transmission Request Pending 19 */
#define MCANX_TXBRP_TRP19             (_U_(0x1) << MCANX_TXBRP_TRP19_Pos)
#define MCANX_TXBRP_TRP20_Pos         20           /**< \brief (MCANX_TXBRP) Transmission Request Pending 20 */
#define MCANX_TXBRP_TRP20             (_U_(0x1) << MCANX_TXBRP_TRP20_Pos)
#define MCANX_TXBRP_TRP21_Pos         21           /**< \brief (MCANX_TXBRP) Transmission Request Pending 21 */
#define MCANX_TXBRP_TRP21             (_U_(0x1) << MCANX_TXBRP_TRP21_Pos)
#define MCANX_TXBRP_TRP22_Pos         22           /**< \brief (MCANX_TXBRP) Transmission Request Pending 22 */
#define MCANX_TXBRP_TRP22             (_U_(0x1) << MCANX_TXBRP_TRP22_Pos)
#define MCANX_TXBRP_TRP23_Pos         23           /**< \brief (MCANX_TXBRP) Transmission Request Pending 23 */
#define MCANX_TXBRP_TRP23             (_U_(0x1) << MCANX_TXBRP_TRP23_Pos)
#define MCANX_TXBRP_TRP24_Pos         24           /**< \brief (MCANX_TXBRP) Transmission Request Pending 24 */
#define MCANX_TXBRP_TRP24             (_U_(0x1) << MCANX_TXBRP_TRP24_Pos)
#define MCANX_TXBRP_TRP25_Pos         25           /**< \brief (MCANX_TXBRP) Transmission Request Pending 25 */
#define MCANX_TXBRP_TRP25             (_U_(0x1) << MCANX_TXBRP_TRP25_Pos)
#define MCANX_TXBRP_TRP26_Pos         26           /**< \brief (MCANX_TXBRP) Transmission Request Pending 26 */
#define MCANX_TXBRP_TRP26             (_U_(0x1) << MCANX_TXBRP_TRP26_Pos)
#define MCANX_TXBRP_TRP27_Pos         27           /**< \brief (MCANX_TXBRP) Transmission Request Pending 27 */
#define MCANX_TXBRP_TRP27             (_U_(0x1) << MCANX_TXBRP_TRP27_Pos)
#define MCANX_TXBRP_TRP28_Pos         28           /**< \brief (MCANX_TXBRP) Transmission Request Pending 28 */
#define MCANX_TXBRP_TRP28             (_U_(0x1) << MCANX_TXBRP_TRP28_Pos)
#define MCANX_TXBRP_TRP29_Pos         29           /**< \brief (MCANX_TXBRP) Transmission Request Pending 29 */
#define MCANX_TXBRP_TRP29             (_U_(0x1) << MCANX_TXBRP_TRP29_Pos)
#define MCANX_TXBRP_TRP30_Pos         30           /**< \brief (MCANX_TXBRP) Transmission Request Pending 30 */
#define MCANX_TXBRP_TRP30             (_U_(0x1) << MCANX_TXBRP_TRP30_Pos)
#define MCANX_TXBRP_TRP31_Pos         31           /**< \brief (MCANX_TXBRP) Transmission Request Pending 31 */
#define MCANX_TXBRP_TRP31             (_U_(0x1) << MCANX_TXBRP_TRP31_Pos)
#define MCANX_TXBRP_MASK              _U_(0xFFFFFFFF) /**< \brief (MCANX_TXBRP) MASK Register */

/* -------- MCANX_TXBAR : (CAN Offset: 0xD0) (R/W 32) Tx Buffer Add Request -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t AR0:1;            /*!< bit:      0  Add Request 0                      */
    uint32_t AR1:1;            /*!< bit:      1  Add Request 1                      */
    uint32_t AR2:1;            /*!< bit:      2  Add Request 2                      */
    uint32_t AR3:1;            /*!< bit:      3  Add Request 3                      */
    uint32_t AR4:1;            /*!< bit:      4  Add Request 4                      */
    uint32_t AR5:1;            /*!< bit:      5  Add Request 5                      */
    uint32_t AR6:1;            /*!< bit:      6  Add Request 6                      */
    uint32_t AR7:1;            /*!< bit:      7  Add Request 7                      */
    uint32_t AR8:1;            /*!< bit:      8  Add Request 8                      */
    uint32_t AR9:1;            /*!< bit:      9  Add Request 9                      */
    uint32_t AR10:1;           /*!< bit:     10  Add Request 10                     */
    uint32_t AR11:1;           /*!< bit:     11  Add Request 11                     */
    uint32_t AR12:1;           /*!< bit:     12  Add Request 12                     */
    uint32_t AR13:1;           /*!< bit:     13  Add Request 13                     */
    uint32_t AR14:1;           /*!< bit:     14  Add Request 14                     */
    uint32_t AR15:1;           /*!< bit:     15  Add Request 15                     */
    uint32_t AR16:1;           /*!< bit:     16  Add Request 16                     */
    uint32_t AR17:1;           /*!< bit:     17  Add Request 17                     */
    uint32_t AR18:1;           /*!< bit:     18  Add Request 18                     */
    uint32_t AR19:1;           /*!< bit:     19  Add Request 19                     */
    uint32_t AR20:1;           /*!< bit:     20  Add Request 20                     */
    uint32_t AR21:1;           /*!< bit:     21  Add Request 21                     */
    uint32_t AR22:1;           /*!< bit:     22  Add Request 22                     */
    uint32_t AR23:1;           /*!< bit:     23  Add Request 23                     */
    uint32_t AR24:1;           /*!< bit:     24  Add Request 24                     */
    uint32_t AR25:1;           /*!< bit:     25  Add Request 25                     */
    uint32_t AR26:1;           /*!< bit:     26  Add Request 26                     */
    uint32_t AR27:1;           /*!< bit:     27  Add Request 27                     */
    uint32_t AR28:1;           /*!< bit:     28  Add Request 28                     */
    uint32_t AR29:1;           /*!< bit:     29  Add Request 29                     */
    uint32_t AR30:1;           /*!< bit:     30  Add Request 30                     */
    uint32_t AR31:1;           /*!< bit:     31  Add Request 31                     */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXBAR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXBAR_OFFSET            0xD0         /**< \brief (MCANX_TXBAR offset) Tx Buffer Add Request */
#define MCANX_TXBAR_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_TXBAR reset_value) Tx Buffer Add Request */

#define MCANX_TXBAR_AR0_Pos           0            /**< \brief (MCANX_TXBAR) Add Request 0 */
#define MCANX_TXBAR_AR0               (_U_(0x1) << MCANX_TXBAR_AR0_Pos)
#define MCANX_TXBAR_AR1_Pos           1            /**< \brief (MCANX_TXBAR) Add Request 1 */
#define MCANX_TXBAR_AR1               (_U_(0x1) << MCANX_TXBAR_AR1_Pos)
#define MCANX_TXBAR_AR2_Pos           2            /**< \brief (MCANX_TXBAR) Add Request 2 */
#define MCANX_TXBAR_AR2               (_U_(0x1) << MCANX_TXBAR_AR2_Pos)
#define MCANX_TXBAR_AR3_Pos           3            /**< \brief (MCANX_TXBAR) Add Request 3 */
#define MCANX_TXBAR_AR3               (_U_(0x1) << MCANX_TXBAR_AR3_Pos)
#define MCANX_TXBAR_AR4_Pos           4            /**< \brief (MCANX_TXBAR) Add Request 4 */
#define MCANX_TXBAR_AR4               (_U_(0x1) << MCANX_TXBAR_AR4_Pos)
#define MCANX_TXBAR_AR5_Pos           5            /**< \brief (MCANX_TXBAR) Add Request 5 */
#define MCANX_TXBAR_AR5               (_U_(0x1) << MCANX_TXBAR_AR5_Pos)
#define MCANX_TXBAR_AR6_Pos           6            /**< \brief (MCANX_TXBAR) Add Request 6 */
#define MCANX_TXBAR_AR6               (_U_(0x1) << MCANX_TXBAR_AR6_Pos)
#define MCANX_TXBAR_AR7_Pos           7            /**< \brief (MCANX_TXBAR) Add Request 7 */
#define MCANX_TXBAR_AR7               (_U_(0x1) << MCANX_TXBAR_AR7_Pos)
#define MCANX_TXBAR_AR8_Pos           8            /**< \brief (MCANX_TXBAR) Add Request 8 */
#define MCANX_TXBAR_AR8               (_U_(0x1) << MCANX_TXBAR_AR8_Pos)
#define MCANX_TXBAR_AR9_Pos           9            /**< \brief (MCANX_TXBAR) Add Request 9 */
#define MCANX_TXBAR_AR9               (_U_(0x1) << MCANX_TXBAR_AR9_Pos)
#define MCANX_TXBAR_AR10_Pos          10           /**< \brief (MCANX_TXBAR) Add Request 10 */
#define MCANX_TXBAR_AR10              (_U_(0x1) << MCANX_TXBAR_AR10_Pos)
#define MCANX_TXBAR_AR11_Pos          11           /**< \brief (MCANX_TXBAR) Add Request 11 */
#define MCANX_TXBAR_AR11              (_U_(0x1) << MCANX_TXBAR_AR11_Pos)
#define MCANX_TXBAR_AR12_Pos          12           /**< \brief (MCANX_TXBAR) Add Request 12 */
#define MCANX_TXBAR_AR12              (_U_(0x1) << MCANX_TXBAR_AR12_Pos)
#define MCANX_TXBAR_AR13_Pos          13           /**< \brief (MCANX_TXBAR) Add Request 13 */
#define MCANX_TXBAR_AR13              (_U_(0x1) << MCANX_TXBAR_AR13_Pos)
#define MCANX_TXBAR_AR14_Pos          14           /**< \brief (MCANX_TXBAR) Add Request 14 */
#define MCANX_TXBAR_AR14              (_U_(0x1) << MCANX_TXBAR_AR14_Pos)
#define MCANX_TXBAR_AR15_Pos          15           /**< \brief (MCANX_TXBAR) Add Request 15 */
#define MCANX_TXBAR_AR15              (_U_(0x1) << MCANX_TXBAR_AR15_Pos)
#define MCANX_TXBAR_AR16_Pos          16           /**< \brief (MCANX_TXBAR) Add Request 16 */
#define MCANX_TXBAR_AR16              (_U_(0x1) << MCANX_TXBAR_AR16_Pos)
#define MCANX_TXBAR_AR17_Pos          17           /**< \brief (MCANX_TXBAR) Add Request 17 */
#define MCANX_TXBAR_AR17              (_U_(0x1) << MCANX_TXBAR_AR17_Pos)
#define MCANX_TXBAR_AR18_Pos          18           /**< \brief (MCANX_TXBAR) Add Request 18 */
#define MCANX_TXBAR_AR18              (_U_(0x1) << MCANX_TXBAR_AR18_Pos)
#define MCANX_TXBAR_AR19_Pos          19           /**< \brief (MCANX_TXBAR) Add Request 19 */
#define MCANX_TXBAR_AR19              (_U_(0x1) << MCANX_TXBAR_AR19_Pos)
#define MCANX_TXBAR_AR20_Pos          20           /**< \brief (MCANX_TXBAR) Add Request 20 */
#define MCANX_TXBAR_AR20              (_U_(0x1) << MCANX_TXBAR_AR20_Pos)
#define MCANX_TXBAR_AR21_Pos          21           /**< \brief (MCANX_TXBAR) Add Request 21 */
#define MCANX_TXBAR_AR21              (_U_(0x1) << MCANX_TXBAR_AR21_Pos)
#define MCANX_TXBAR_AR22_Pos          22           /**< \brief (MCANX_TXBAR) Add Request 22 */
#define MCANX_TXBAR_AR22              (_U_(0x1) << MCANX_TXBAR_AR22_Pos)
#define MCANX_TXBAR_AR23_Pos          23           /**< \brief (MCANX_TXBAR) Add Request 23 */
#define MCANX_TXBAR_AR23              (_U_(0x1) << MCANX_TXBAR_AR23_Pos)
#define MCANX_TXBAR_AR24_Pos          24           /**< \brief (MCANX_TXBAR) Add Request 24 */
#define MCANX_TXBAR_AR24              (_U_(0x1) << MCANX_TXBAR_AR24_Pos)
#define MCANX_TXBAR_AR25_Pos          25           /**< \brief (MCANX_TXBAR) Add Request 25 */
#define MCANX_TXBAR_AR25              (_U_(0x1) << MCANX_TXBAR_AR25_Pos)
#define MCANX_TXBAR_AR26_Pos          26           /**< \brief (MCANX_TXBAR) Add Request 26 */
#define MCANX_TXBAR_AR26              (_U_(0x1) << MCANX_TXBAR_AR26_Pos)
#define MCANX_TXBAR_AR27_Pos          27           /**< \brief (MCANX_TXBAR) Add Request 27 */
#define MCANX_TXBAR_AR27              (_U_(0x1) << MCANX_TXBAR_AR27_Pos)
#define MCANX_TXBAR_AR28_Pos          28           /**< \brief (MCANX_TXBAR) Add Request 28 */
#define MCANX_TXBAR_AR28              (_U_(0x1) << MCANX_TXBAR_AR28_Pos)
#define MCANX_TXBAR_AR29_Pos          29           /**< \brief (MCANX_TXBAR) Add Request 29 */
#define MCANX_TXBAR_AR29              (_U_(0x1) << MCANX_TXBAR_AR29_Pos)
#define MCANX_TXBAR_AR30_Pos          30           /**< \brief (MCANX_TXBAR) Add Request 30 */
#define MCANX_TXBAR_AR30              (_U_(0x1) << MCANX_TXBAR_AR30_Pos)
#define MCANX_TXBAR_AR31_Pos          31           /**< \brief (MCANX_TXBAR) Add Request 31 */
#define MCANX_TXBAR_AR31              (_U_(0x1) << MCANX_TXBAR_AR31_Pos)
#define MCANX_TXBAR_MASK              _U_(0xFFFFFFFF) /**< \brief (MCANX_TXBAR) MASK Register */

/* -------- MCANX_TXBCR : (CAN Offset: 0xD4) (R/W 32) Tx Buffer Cancellation Request -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t CR0:1;            /*!< bit:      0  Cancellation Request 0             */
    uint32_t CR1:1;            /*!< bit:      1  Cancellation Request 1             */
    uint32_t CR2:1;            /*!< bit:      2  Cancellation Request 2             */
    uint32_t CR3:1;            /*!< bit:      3  Cancellation Request 3             */
    uint32_t CR4:1;            /*!< bit:      4  Cancellation Request 4             */
    uint32_t CR5:1;            /*!< bit:      5  Cancellation Request 5             */
    uint32_t CR6:1;            /*!< bit:      6  Cancellation Request 6             */
    uint32_t CR7:1;            /*!< bit:      7  Cancellation Request 7             */
    uint32_t CR8:1;            /*!< bit:      8  Cancellation Request 8             */
    uint32_t CR9:1;            /*!< bit:      9  Cancellation Request 9             */
    uint32_t CR10:1;           /*!< bit:     10  Cancellation Request 10            */
    uint32_t CR11:1;           /*!< bit:     11  Cancellation Request 11            */
    uint32_t CR12:1;           /*!< bit:     12  Cancellation Request 12            */
    uint32_t CR13:1;           /*!< bit:     13  Cancellation Request 13            */
    uint32_t CR14:1;           /*!< bit:     14  Cancellation Request 14            */
    uint32_t CR15:1;           /*!< bit:     15  Cancellation Request 15            */
    uint32_t CR16:1;           /*!< bit:     16  Cancellation Request 16            */
    uint32_t CR17:1;           /*!< bit:     17  Cancellation Request 17            */
    uint32_t CR18:1;           /*!< bit:     18  Cancellation Request 18            */
    uint32_t CR19:1;           /*!< bit:     19  Cancellation Request 19            */
    uint32_t CR20:1;           /*!< bit:     20  Cancellation Request 20            */
    uint32_t CR21:1;           /*!< bit:     21  Cancellation Request 21            */
    uint32_t CR22:1;           /*!< bit:     22  Cancellation Request 22            */
    uint32_t CR23:1;           /*!< bit:     23  Cancellation Request 23            */
    uint32_t CR24:1;           /*!< bit:     24  Cancellation Request 24            */
    uint32_t CR25:1;           /*!< bit:     25  Cancellation Request 25            */
    uint32_t CR26:1;           /*!< bit:     26  Cancellation Request 26            */
    uint32_t CR27:1;           /*!< bit:     27  Cancellation Request 27            */
    uint32_t CR28:1;           /*!< bit:     28  Cancellation Request 28            */
    uint32_t CR29:1;           /*!< bit:     29  Cancellation Request 29            */
    uint32_t CR30:1;           /*!< bit:     30  Cancellation Request 30            */
    uint32_t CR31:1;           /*!< bit:     31  Cancellation Request 31            */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXBCR_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXBCR_OFFSET            0xD4         /**< \brief (MCANX_TXBCR offset) Tx Buffer Cancellation Request */
#define MCANX_TXBCR_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_TXBCR reset_value) Tx Buffer Cancellation Request */

#define MCANX_TXBCR_CR0_Pos           0            /**< \brief (MCANX_TXBCR) Cancellation Request 0 */
#define MCANX_TXBCR_CR0               (_U_(0x1) << MCANX_TXBCR_CR0_Pos)
#define MCANX_TXBCR_CR1_Pos           1            /**< \brief (MCANX_TXBCR) Cancellation Request 1 */
#define MCANX_TXBCR_CR1               (_U_(0x1) << MCANX_TXBCR_CR1_Pos)
#define MCANX_TXBCR_CR2_Pos           2            /**< \brief (MCANX_TXBCR) Cancellation Request 2 */
#define MCANX_TXBCR_CR2               (_U_(0x1) << MCANX_TXBCR_CR2_Pos)
#define MCANX_TXBCR_CR3_Pos           3            /**< \brief (MCANX_TXBCR) Cancellation Request 3 */
#define MCANX_TXBCR_CR3               (_U_(0x1) << MCANX_TXBCR_CR3_Pos)
#define MCANX_TXBCR_CR4_Pos           4            /**< \brief (MCANX_TXBCR) Cancellation Request 4 */
#define MCANX_TXBCR_CR4               (_U_(0x1) << MCANX_TXBCR_CR4_Pos)
#define MCANX_TXBCR_CR5_Pos           5            /**< \brief (MCANX_TXBCR) Cancellation Request 5 */
#define MCANX_TXBCR_CR5               (_U_(0x1) << MCANX_TXBCR_CR5_Pos)
#define MCANX_TXBCR_CR6_Pos           6            /**< \brief (MCANX_TXBCR) Cancellation Request 6 */
#define MCANX_TXBCR_CR6               (_U_(0x1) << MCANX_TXBCR_CR6_Pos)
#define MCANX_TXBCR_CR7_Pos           7            /**< \brief (MCANX_TXBCR) Cancellation Request 7 */
#define MCANX_TXBCR_CR7               (_U_(0x1) << MCANX_TXBCR_CR7_Pos)
#define MCANX_TXBCR_CR8_Pos           8            /**< \brief (MCANX_TXBCR) Cancellation Request 8 */
#define MCANX_TXBCR_CR8               (_U_(0x1) << MCANX_TXBCR_CR8_Pos)
#define MCANX_TXBCR_CR9_Pos           9            /**< \brief (MCANX_TXBCR) Cancellation Request 9 */
#define MCANX_TXBCR_CR9               (_U_(0x1) << MCANX_TXBCR_CR9_Pos)
#define MCANX_TXBCR_CR10_Pos          10           /**< \brief (MCANX_TXBCR) Cancellation Request 10 */
#define MCANX_TXBCR_CR10              (_U_(0x1) << MCANX_TXBCR_CR10_Pos)
#define MCANX_TXBCR_CR11_Pos          11           /**< \brief (MCANX_TXBCR) Cancellation Request 11 */
#define MCANX_TXBCR_CR11              (_U_(0x1) << MCANX_TXBCR_CR11_Pos)
#define MCANX_TXBCR_CR12_Pos          12           /**< \brief (MCANX_TXBCR) Cancellation Request 12 */
#define MCANX_TXBCR_CR12              (_U_(0x1) << MCANX_TXBCR_CR12_Pos)
#define MCANX_TXBCR_CR13_Pos          13           /**< \brief (MCANX_TXBCR) Cancellation Request 13 */
#define MCANX_TXBCR_CR13              (_U_(0x1) << MCANX_TXBCR_CR13_Pos)
#define MCANX_TXBCR_CR14_Pos          14           /**< \brief (MCANX_TXBCR) Cancellation Request 14 */
#define MCANX_TXBCR_CR14              (_U_(0x1) << MCANX_TXBCR_CR14_Pos)
#define MCANX_TXBCR_CR15_Pos          15           /**< \brief (MCANX_TXBCR) Cancellation Request 15 */
#define MCANX_TXBCR_CR15              (_U_(0x1) << MCANX_TXBCR_CR15_Pos)
#define MCANX_TXBCR_CR16_Pos          16           /**< \brief (MCANX_TXBCR) Cancellation Request 16 */
#define MCANX_TXBCR_CR16              (_U_(0x1) << MCANX_TXBCR_CR16_Pos)
#define MCANX_TXBCR_CR17_Pos          17           /**< \brief (MCANX_TXBCR) Cancellation Request 17 */
#define MCANX_TXBCR_CR17              (_U_(0x1) << MCANX_TXBCR_CR17_Pos)
#define MCANX_TXBCR_CR18_Pos          18           /**< \brief (MCANX_TXBCR) Cancellation Request 18 */
#define MCANX_TXBCR_CR18              (_U_(0x1) << MCANX_TXBCR_CR18_Pos)
#define MCANX_TXBCR_CR19_Pos          19           /**< \brief (MCANX_TXBCR) Cancellation Request 19 */
#define MCANX_TXBCR_CR19              (_U_(0x1) << MCANX_TXBCR_CR19_Pos)
#define MCANX_TXBCR_CR20_Pos          20           /**< \brief (MCANX_TXBCR) Cancellation Request 20 */
#define MCANX_TXBCR_CR20              (_U_(0x1) << MCANX_TXBCR_CR20_Pos)
#define MCANX_TXBCR_CR21_Pos          21           /**< \brief (MCANX_TXBCR) Cancellation Request 21 */
#define MCANX_TXBCR_CR21              (_U_(0x1) << MCANX_TXBCR_CR21_Pos)
#define MCANX_TXBCR_CR22_Pos          22           /**< \brief (MCANX_TXBCR) Cancellation Request 22 */
#define MCANX_TXBCR_CR22              (_U_(0x1) << MCANX_TXBCR_CR22_Pos)
#define MCANX_TXBCR_CR23_Pos          23           /**< \brief (MCANX_TXBCR) Cancellation Request 23 */
#define MCANX_TXBCR_CR23              (_U_(0x1) << MCANX_TXBCR_CR23_Pos)
#define MCANX_TXBCR_CR24_Pos          24           /**< \brief (MCANX_TXBCR) Cancellation Request 24 */
#define MCANX_TXBCR_CR24              (_U_(0x1) << MCANX_TXBCR_CR24_Pos)
#define MCANX_TXBCR_CR25_Pos          25           /**< \brief (MCANX_TXBCR) Cancellation Request 25 */
#define MCANX_TXBCR_CR25              (_U_(0x1) << MCANX_TXBCR_CR25_Pos)
#define MCANX_TXBCR_CR26_Pos          26           /**< \brief (MCANX_TXBCR) Cancellation Request 26 */
#define MCANX_TXBCR_CR26              (_U_(0x1) << MCANX_TXBCR_CR26_Pos)
#define MCANX_TXBCR_CR27_Pos          27           /**< \brief (MCANX_TXBCR) Cancellation Request 27 */
#define MCANX_TXBCR_CR27              (_U_(0x1) << MCANX_TXBCR_CR27_Pos)
#define MCANX_TXBCR_CR28_Pos          28           /**< \brief (MCANX_TXBCR) Cancellation Request 28 */
#define MCANX_TXBCR_CR28              (_U_(0x1) << MCANX_TXBCR_CR28_Pos)
#define MCANX_TXBCR_CR29_Pos          29           /**< \brief (MCANX_TXBCR) Cancellation Request 29 */
#define MCANX_TXBCR_CR29              (_U_(0x1) << MCANX_TXBCR_CR29_Pos)
#define MCANX_TXBCR_CR30_Pos          30           /**< \brief (MCANX_TXBCR) Cancellation Request 30 */
#define MCANX_TXBCR_CR30              (_U_(0x1) << MCANX_TXBCR_CR30_Pos)
#define MCANX_TXBCR_CR31_Pos          31           /**< \brief (MCANX_TXBCR) Cancellation Request 31 */
#define MCANX_TXBCR_CR31              (_U_(0x1) << MCANX_TXBCR_CR31_Pos)
#define MCANX_TXBCR_MASK              _U_(0xFFFFFFFF) /**< \brief (MCANX_TXBCR) MASK Register */

/* -------- MCANX_TXBTO : (CAN Offset: 0xD8) (R/  32) Tx Buffer Transmission Occurred -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TO0:1;            /*!< bit:      0  Transmission Occurred 0            */
    uint32_t TO1:1;            /*!< bit:      1  Transmission Occurred 1            */
    uint32_t TO2:1;            /*!< bit:      2  Transmission Occurred 2            */
    uint32_t TO3:1;            /*!< bit:      3  Transmission Occurred 3            */
    uint32_t TO4:1;            /*!< bit:      4  Transmission Occurred 4            */
    uint32_t TO5:1;            /*!< bit:      5  Transmission Occurred 5            */
    uint32_t TO6:1;            /*!< bit:      6  Transmission Occurred 6            */
    uint32_t TO7:1;            /*!< bit:      7  Transmission Occurred 7            */
    uint32_t TO8:1;            /*!< bit:      8  Transmission Occurred 8            */
    uint32_t TO9:1;            /*!< bit:      9  Transmission Occurred 9            */
    uint32_t TO10:1;           /*!< bit:     10  Transmission Occurred 10           */
    uint32_t TO11:1;           /*!< bit:     11  Transmission Occurred 11           */
    uint32_t TO12:1;           /*!< bit:     12  Transmission Occurred 12           */
    uint32_t TO13:1;           /*!< bit:     13  Transmission Occurred 13           */
    uint32_t TO14:1;           /*!< bit:     14  Transmission Occurred 14           */
    uint32_t TO15:1;           /*!< bit:     15  Transmission Occurred 15           */
    uint32_t TO16:1;           /*!< bit:     16  Transmission Occurred 16           */
    uint32_t TO17:1;           /*!< bit:     17  Transmission Occurred 17           */
    uint32_t TO18:1;           /*!< bit:     18  Transmission Occurred 18           */
    uint32_t TO19:1;           /*!< bit:     19  Transmission Occurred 19           */
    uint32_t TO20:1;           /*!< bit:     20  Transmission Occurred 20           */
    uint32_t TO21:1;           /*!< bit:     21  Transmission Occurred 21           */
    uint32_t TO22:1;           /*!< bit:     22  Transmission Occurred 22           */
    uint32_t TO23:1;           /*!< bit:     23  Transmission Occurred 23           */
    uint32_t TO24:1;           /*!< bit:     24  Transmission Occurred 24           */
    uint32_t TO25:1;           /*!< bit:     25  Transmission Occurred 25           */
    uint32_t TO26:1;           /*!< bit:     26  Transmission Occurred 26           */
    uint32_t TO27:1;           /*!< bit:     27  Transmission Occurred 27           */
    uint32_t TO28:1;           /*!< bit:     28  Transmission Occurred 28           */
    uint32_t TO29:1;           /*!< bit:     29  Transmission Occurred 29           */
    uint32_t TO30:1;           /*!< bit:     30  Transmission Occurred 30           */
    uint32_t TO31:1;           /*!< bit:     31  Transmission Occurred 31           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXBTO_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXBTO_OFFSET            0xD8         /**< \brief (MCANX_TXBTO offset) Tx Buffer Transmission Occurred */
#define MCANX_TXBTO_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_TXBTO reset_value) Tx Buffer Transmission Occurred */

#define MCANX_TXBTO_TO0_Pos           0            /**< \brief (MCANX_TXBTO) Transmission Occurred 0 */
#define MCANX_TXBTO_TO0               (_U_(0x1) << MCANX_TXBTO_TO0_Pos)
#define MCANX_TXBTO_TO1_Pos           1            /**< \brief (MCANX_TXBTO) Transmission Occurred 1 */
#define MCANX_TXBTO_TO1               (_U_(0x1) << MCANX_TXBTO_TO1_Pos)
#define MCANX_TXBTO_TO2_Pos           2            /**< \brief (MCANX_TXBTO) Transmission Occurred 2 */
#define MCANX_TXBTO_TO2               (_U_(0x1) << MCANX_TXBTO_TO2_Pos)
#define MCANX_TXBTO_TO3_Pos           3            /**< \brief (MCANX_TXBTO) Transmission Occurred 3 */
#define MCANX_TXBTO_TO3               (_U_(0x1) << MCANX_TXBTO_TO3_Pos)
#define MCANX_TXBTO_TO4_Pos           4            /**< \brief (MCANX_TXBTO) Transmission Occurred 4 */
#define MCANX_TXBTO_TO4               (_U_(0x1) << MCANX_TXBTO_TO4_Pos)
#define MCANX_TXBTO_TO5_Pos           5            /**< \brief (MCANX_TXBTO) Transmission Occurred 5 */
#define MCANX_TXBTO_TO5               (_U_(0x1) << MCANX_TXBTO_TO5_Pos)
#define MCANX_TXBTO_TO6_Pos           6            /**< \brief (MCANX_TXBTO) Transmission Occurred 6 */
#define MCANX_TXBTO_TO6               (_U_(0x1) << MCANX_TXBTO_TO6_Pos)
#define MCANX_TXBTO_TO7_Pos           7            /**< \brief (MCANX_TXBTO) Transmission Occurred 7 */
#define MCANX_TXBTO_TO7               (_U_(0x1) << MCANX_TXBTO_TO7_Pos)
#define MCANX_TXBTO_TO8_Pos           8            /**< \brief (MCANX_TXBTO) Transmission Occurred 8 */
#define MCANX_TXBTO_TO8               (_U_(0x1) << MCANX_TXBTO_TO8_Pos)
#define MCANX_TXBTO_TO9_Pos           9            /**< \brief (MCANX_TXBTO) Transmission Occurred 9 */
#define MCANX_TXBTO_TO9               (_U_(0x1) << MCANX_TXBTO_TO9_Pos)
#define MCANX_TXBTO_TO10_Pos          10           /**< \brief (MCANX_TXBTO) Transmission Occurred 10 */
#define MCANX_TXBTO_TO10              (_U_(0x1) << MCANX_TXBTO_TO10_Pos)
#define MCANX_TXBTO_TO11_Pos          11           /**< \brief (MCANX_TXBTO) Transmission Occurred 11 */
#define MCANX_TXBTO_TO11              (_U_(0x1) << MCANX_TXBTO_TO11_Pos)
#define MCANX_TXBTO_TO12_Pos          12           /**< \brief (MCANX_TXBTO) Transmission Occurred 12 */
#define MCANX_TXBTO_TO12              (_U_(0x1) << MCANX_TXBTO_TO12_Pos)
#define MCANX_TXBTO_TO13_Pos          13           /**< \brief (MCANX_TXBTO) Transmission Occurred 13 */
#define MCANX_TXBTO_TO13              (_U_(0x1) << MCANX_TXBTO_TO13_Pos)
#define MCANX_TXBTO_TO14_Pos          14           /**< \brief (MCANX_TXBTO) Transmission Occurred 14 */
#define MCANX_TXBTO_TO14              (_U_(0x1) << MCANX_TXBTO_TO14_Pos)
#define MCANX_TXBTO_TO15_Pos          15           /**< \brief (MCANX_TXBTO) Transmission Occurred 15 */
#define MCANX_TXBTO_TO15              (_U_(0x1) << MCANX_TXBTO_TO15_Pos)
#define MCANX_TXBTO_TO16_Pos          16           /**< \brief (MCANX_TXBTO) Transmission Occurred 16 */
#define MCANX_TXBTO_TO16              (_U_(0x1) << MCANX_TXBTO_TO16_Pos)
#define MCANX_TXBTO_TO17_Pos          17           /**< \brief (MCANX_TXBTO) Transmission Occurred 17 */
#define MCANX_TXBTO_TO17              (_U_(0x1) << MCANX_TXBTO_TO17_Pos)
#define MCANX_TXBTO_TO18_Pos          18           /**< \brief (MCANX_TXBTO) Transmission Occurred 18 */
#define MCANX_TXBTO_TO18              (_U_(0x1) << MCANX_TXBTO_TO18_Pos)
#define MCANX_TXBTO_TO19_Pos          19           /**< \brief (MCANX_TXBTO) Transmission Occurred 19 */
#define MCANX_TXBTO_TO19              (_U_(0x1) << MCANX_TXBTO_TO19_Pos)
#define MCANX_TXBTO_TO20_Pos          20           /**< \brief (MCANX_TXBTO) Transmission Occurred 20 */
#define MCANX_TXBTO_TO20              (_U_(0x1) << MCANX_TXBTO_TO20_Pos)
#define MCANX_TXBTO_TO21_Pos          21           /**< \brief (MCANX_TXBTO) Transmission Occurred 21 */
#define MCANX_TXBTO_TO21              (_U_(0x1) << MCANX_TXBTO_TO21_Pos)
#define MCANX_TXBTO_TO22_Pos          22           /**< \brief (MCANX_TXBTO) Transmission Occurred 22 */
#define MCANX_TXBTO_TO22              (_U_(0x1) << MCANX_TXBTO_TO22_Pos)
#define MCANX_TXBTO_TO23_Pos          23           /**< \brief (MCANX_TXBTO) Transmission Occurred 23 */
#define MCANX_TXBTO_TO23              (_U_(0x1) << MCANX_TXBTO_TO23_Pos)
#define MCANX_TXBTO_TO24_Pos          24           /**< \brief (MCANX_TXBTO) Transmission Occurred 24 */
#define MCANX_TXBTO_TO24              (_U_(0x1) << MCANX_TXBTO_TO24_Pos)
#define MCANX_TXBTO_TO25_Pos          25           /**< \brief (MCANX_TXBTO) Transmission Occurred 25 */
#define MCANX_TXBTO_TO25              (_U_(0x1) << MCANX_TXBTO_TO25_Pos)
#define MCANX_TXBTO_TO26_Pos          26           /**< \brief (MCANX_TXBTO) Transmission Occurred 26 */
#define MCANX_TXBTO_TO26              (_U_(0x1) << MCANX_TXBTO_TO26_Pos)
#define MCANX_TXBTO_TO27_Pos          27           /**< \brief (MCANX_TXBTO) Transmission Occurred 27 */
#define MCANX_TXBTO_TO27              (_U_(0x1) << MCANX_TXBTO_TO27_Pos)
#define MCANX_TXBTO_TO28_Pos          28           /**< \brief (MCANX_TXBTO) Transmission Occurred 28 */
#define MCANX_TXBTO_TO28              (_U_(0x1) << MCANX_TXBTO_TO28_Pos)
#define MCANX_TXBTO_TO29_Pos          29           /**< \brief (MCANX_TXBTO) Transmission Occurred 29 */
#define MCANX_TXBTO_TO29              (_U_(0x1) << MCANX_TXBTO_TO29_Pos)
#define MCANX_TXBTO_TO30_Pos          30           /**< \brief (MCANX_TXBTO) Transmission Occurred 30 */
#define MCANX_TXBTO_TO30              (_U_(0x1) << MCANX_TXBTO_TO30_Pos)
#define MCANX_TXBTO_TO31_Pos          31           /**< \brief (MCANX_TXBTO) Transmission Occurred 31 */
#define MCANX_TXBTO_TO31              (_U_(0x1) << MCANX_TXBTO_TO31_Pos)
#define MCANX_TXBTO_MASK              _U_(0xFFFFFFFF) /**< \brief (MCANX_TXBTO) MASK Register */

/* -------- MCANX_TXBCF : (CAN Offset: 0xDC) (R/  32) Tx Buffer Cancellation Finished -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t CF0:1;            /*!< bit:      0  Tx Buffer Cancellation Finished 0  */
    uint32_t CF1:1;            /*!< bit:      1  Tx Buffer Cancellation Finished 1  */
    uint32_t CF2:1;            /*!< bit:      2  Tx Buffer Cancellation Finished 2  */
    uint32_t CF3:1;            /*!< bit:      3  Tx Buffer Cancellation Finished 3  */
    uint32_t CF4:1;            /*!< bit:      4  Tx Buffer Cancellation Finished 4  */
    uint32_t CF5:1;            /*!< bit:      5  Tx Buffer Cancellation Finished 5  */
    uint32_t CF6:1;            /*!< bit:      6  Tx Buffer Cancellation Finished 6  */
    uint32_t CF7:1;            /*!< bit:      7  Tx Buffer Cancellation Finished 7  */
    uint32_t CF8:1;            /*!< bit:      8  Tx Buffer Cancellation Finished 8  */
    uint32_t CF9:1;            /*!< bit:      9  Tx Buffer Cancellation Finished 9  */
    uint32_t CF10:1;           /*!< bit:     10  Tx Buffer Cancellation Finished 10 */
    uint32_t CF11:1;           /*!< bit:     11  Tx Buffer Cancellation Finished 11 */
    uint32_t CF12:1;           /*!< bit:     12  Tx Buffer Cancellation Finished 12 */
    uint32_t CF13:1;           /*!< bit:     13  Tx Buffer Cancellation Finished 13 */
    uint32_t CF14:1;           /*!< bit:     14  Tx Buffer Cancellation Finished 14 */
    uint32_t CF15:1;           /*!< bit:     15  Tx Buffer Cancellation Finished 15 */
    uint32_t CF16:1;           /*!< bit:     16  Tx Buffer Cancellation Finished 16 */
    uint32_t CF17:1;           /*!< bit:     17  Tx Buffer Cancellation Finished 17 */
    uint32_t CF18:1;           /*!< bit:     18  Tx Buffer Cancellation Finished 18 */
    uint32_t CF19:1;           /*!< bit:     19  Tx Buffer Cancellation Finished 19 */
    uint32_t CF20:1;           /*!< bit:     20  Tx Buffer Cancellation Finished 20 */
    uint32_t CF21:1;           /*!< bit:     21  Tx Buffer Cancellation Finished 21 */
    uint32_t CF22:1;           /*!< bit:     22  Tx Buffer Cancellation Finished 22 */
    uint32_t CF23:1;           /*!< bit:     23  Tx Buffer Cancellation Finished 23 */
    uint32_t CF24:1;           /*!< bit:     24  Tx Buffer Cancellation Finished 24 */
    uint32_t CF25:1;           /*!< bit:     25  Tx Buffer Cancellation Finished 25 */
    uint32_t CF26:1;           /*!< bit:     26  Tx Buffer Cancellation Finished 26 */
    uint32_t CF27:1;           /*!< bit:     27  Tx Buffer Cancellation Finished 27 */
    uint32_t CF28:1;           /*!< bit:     28  Tx Buffer Cancellation Finished 28 */
    uint32_t CF29:1;           /*!< bit:     29  Tx Buffer Cancellation Finished 29 */
    uint32_t CF30:1;           /*!< bit:     30  Tx Buffer Cancellation Finished 30 */
    uint32_t CF31:1;           /*!< bit:     31  Tx Buffer Cancellation Finished 31 */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXBCF_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXBCF_OFFSET            0xDC         /**< \brief (MCANX_TXBCF offset) Tx Buffer Cancellation Finished */
#define MCANX_TXBCF_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_TXBCF reset_value) Tx Buffer Cancellation Finished */

#define MCANX_TXBCF_CF0_Pos           0            /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 0 */
#define MCANX_TXBCF_CF0               (_U_(0x1) << MCANX_TXBCF_CF0_Pos)
#define MCANX_TXBCF_CF1_Pos           1            /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 1 */
#define MCANX_TXBCF_CF1               (_U_(0x1) << MCANX_TXBCF_CF1_Pos)
#define MCANX_TXBCF_CF2_Pos           2            /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 2 */
#define MCANX_TXBCF_CF2               (_U_(0x1) << MCANX_TXBCF_CF2_Pos)
#define MCANX_TXBCF_CF3_Pos           3            /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 3 */
#define MCANX_TXBCF_CF3               (_U_(0x1) << MCANX_TXBCF_CF3_Pos)
#define MCANX_TXBCF_CF4_Pos           4            /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 4 */
#define MCANX_TXBCF_CF4               (_U_(0x1) << MCANX_TXBCF_CF4_Pos)
#define MCANX_TXBCF_CF5_Pos           5            /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 5 */
#define MCANX_TXBCF_CF5               (_U_(0x1) << MCANX_TXBCF_CF5_Pos)
#define MCANX_TXBCF_CF6_Pos           6            /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 6 */
#define MCANX_TXBCF_CF6               (_U_(0x1) << MCANX_TXBCF_CF6_Pos)
#define MCANX_TXBCF_CF7_Pos           7            /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 7 */
#define MCANX_TXBCF_CF7               (_U_(0x1) << MCANX_TXBCF_CF7_Pos)
#define MCANX_TXBCF_CF8_Pos           8            /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 8 */
#define MCANX_TXBCF_CF8               (_U_(0x1) << MCANX_TXBCF_CF8_Pos)
#define MCANX_TXBCF_CF9_Pos           9            /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 9 */
#define MCANX_TXBCF_CF9               (_U_(0x1) << MCANX_TXBCF_CF9_Pos)
#define MCANX_TXBCF_CF10_Pos          10           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 10 */
#define MCANX_TXBCF_CF10              (_U_(0x1) << MCANX_TXBCF_CF10_Pos)
#define MCANX_TXBCF_CF11_Pos          11           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 11 */
#define MCANX_TXBCF_CF11              (_U_(0x1) << MCANX_TXBCF_CF11_Pos)
#define MCANX_TXBCF_CF12_Pos          12           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 12 */
#define MCANX_TXBCF_CF12              (_U_(0x1) << MCANX_TXBCF_CF12_Pos)
#define MCANX_TXBCF_CF13_Pos          13           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 13 */
#define MCANX_TXBCF_CF13              (_U_(0x1) << MCANX_TXBCF_CF13_Pos)
#define MCANX_TXBCF_CF14_Pos          14           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 14 */
#define MCANX_TXBCF_CF14              (_U_(0x1) << MCANX_TXBCF_CF14_Pos)
#define MCANX_TXBCF_CF15_Pos          15           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 15 */
#define MCANX_TXBCF_CF15              (_U_(0x1) << MCANX_TXBCF_CF15_Pos)
#define MCANX_TXBCF_CF16_Pos          16           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 16 */
#define MCANX_TXBCF_CF16              (_U_(0x1) << MCANX_TXBCF_CF16_Pos)
#define MCANX_TXBCF_CF17_Pos          17           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 17 */
#define MCANX_TXBCF_CF17              (_U_(0x1) << MCANX_TXBCF_CF17_Pos)
#define MCANX_TXBCF_CF18_Pos          18           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 18 */
#define MCANX_TXBCF_CF18              (_U_(0x1) << MCANX_TXBCF_CF18_Pos)
#define MCANX_TXBCF_CF19_Pos          19           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 19 */
#define MCANX_TXBCF_CF19              (_U_(0x1) << MCANX_TXBCF_CF19_Pos)
#define MCANX_TXBCF_CF20_Pos          20           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 20 */
#define MCANX_TXBCF_CF20              (_U_(0x1) << MCANX_TXBCF_CF20_Pos)
#define MCANX_TXBCF_CF21_Pos          21           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 21 */
#define MCANX_TXBCF_CF21              (_U_(0x1) << MCANX_TXBCF_CF21_Pos)
#define MCANX_TXBCF_CF22_Pos          22           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 22 */
#define MCANX_TXBCF_CF22              (_U_(0x1) << MCANX_TXBCF_CF22_Pos)
#define MCANX_TXBCF_CF23_Pos          23           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 23 */
#define MCANX_TXBCF_CF23              (_U_(0x1) << MCANX_TXBCF_CF23_Pos)
#define MCANX_TXBCF_CF24_Pos          24           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 24 */
#define MCANX_TXBCF_CF24              (_U_(0x1) << MCANX_TXBCF_CF24_Pos)
#define MCANX_TXBCF_CF25_Pos          25           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 25 */
#define MCANX_TXBCF_CF25              (_U_(0x1) << MCANX_TXBCF_CF25_Pos)
#define MCANX_TXBCF_CF26_Pos          26           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 26 */
#define MCANX_TXBCF_CF26              (_U_(0x1) << MCANX_TXBCF_CF26_Pos)
#define MCANX_TXBCF_CF27_Pos          27           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 27 */
#define MCANX_TXBCF_CF27              (_U_(0x1) << MCANX_TXBCF_CF27_Pos)
#define MCANX_TXBCF_CF28_Pos          28           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 28 */
#define MCANX_TXBCF_CF28              (_U_(0x1) << MCANX_TXBCF_CF28_Pos)
#define MCANX_TXBCF_CF29_Pos          29           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 29 */
#define MCANX_TXBCF_CF29              (_U_(0x1) << MCANX_TXBCF_CF29_Pos)
#define MCANX_TXBCF_CF30_Pos          30           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 30 */
#define MCANX_TXBCF_CF30              (_U_(0x1) << MCANX_TXBCF_CF30_Pos)
#define MCANX_TXBCF_CF31_Pos          31           /**< \brief (MCANX_TXBCF) Tx Buffer Cancellation Finished 31 */
#define MCANX_TXBCF_CF31              (_U_(0x1) << MCANX_TXBCF_CF31_Pos)
#define MCANX_TXBCF_MASK              _U_(0xFFFFFFFF) /**< \brief (MCANX_TXBCF) MASK Register */

/* -------- MCANX_TXBTIE : (CAN Offset: 0xE0) (R/W 32) Tx Buffer Transmission Interrupt Enable -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TIE0:1;           /*!< bit:      0  Transmission Interrupt Enable 0    */
    uint32_t TIE1:1;           /*!< bit:      1  Transmission Interrupt Enable 1    */
    uint32_t TIE2:1;           /*!< bit:      2  Transmission Interrupt Enable 2    */
    uint32_t TIE3:1;           /*!< bit:      3  Transmission Interrupt Enable 3    */
    uint32_t TIE4:1;           /*!< bit:      4  Transmission Interrupt Enable 4    */
    uint32_t TIE5:1;           /*!< bit:      5  Transmission Interrupt Enable 5    */
    uint32_t TIE6:1;           /*!< bit:      6  Transmission Interrupt Enable 6    */
    uint32_t TIE7:1;           /*!< bit:      7  Transmission Interrupt Enable 7    */
    uint32_t TIE8:1;           /*!< bit:      8  Transmission Interrupt Enable 8    */
    uint32_t TIE9:1;           /*!< bit:      9  Transmission Interrupt Enable 9    */
    uint32_t TIE10:1;          /*!< bit:     10  Transmission Interrupt Enable 10   */
    uint32_t TIE11:1;          /*!< bit:     11  Transmission Interrupt Enable 11   */
    uint32_t TIE12:1;          /*!< bit:     12  Transmission Interrupt Enable 12   */
    uint32_t TIE13:1;          /*!< bit:     13  Transmission Interrupt Enable 13   */
    uint32_t TIE14:1;          /*!< bit:     14  Transmission Interrupt Enable 14   */
    uint32_t TIE15:1;          /*!< bit:     15  Transmission Interrupt Enable 15   */
    uint32_t TIE16:1;          /*!< bit:     16  Transmission Interrupt Enable 16   */
    uint32_t TIE17:1;          /*!< bit:     17  Transmission Interrupt Enable 17   */
    uint32_t TIE18:1;          /*!< bit:     18  Transmission Interrupt Enable 18   */
    uint32_t TIE19:1;          /*!< bit:     19  Transmission Interrupt Enable 19   */
    uint32_t TIE20:1;          /*!< bit:     20  Transmission Interrupt Enable 20   */
    uint32_t TIE21:1;          /*!< bit:     21  Transmission Interrupt Enable 21   */
    uint32_t TIE22:1;          /*!< bit:     22  Transmission Interrupt Enable 22   */
    uint32_t TIE23:1;          /*!< bit:     23  Transmission Interrupt Enable 23   */
    uint32_t TIE24:1;          /*!< bit:     24  Transmission Interrupt Enable 24   */
    uint32_t TIE25:1;          /*!< bit:     25  Transmission Interrupt Enable 25   */
    uint32_t TIE26:1;          /*!< bit:     26  Transmission Interrupt Enable 26   */
    uint32_t TIE27:1;          /*!< bit:     27  Transmission Interrupt Enable 27   */
    uint32_t TIE28:1;          /*!< bit:     28  Transmission Interrupt Enable 28   */
    uint32_t TIE29:1;          /*!< bit:     29  Transmission Interrupt Enable 29   */
    uint32_t TIE30:1;          /*!< bit:     30  Transmission Interrupt Enable 30   */
    uint32_t TIE31:1;          /*!< bit:     31  Transmission Interrupt Enable 31   */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXBTIE_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXBTIE_OFFSET           0xE0         /**< \brief (MCANX_TXBTIE offset) Tx Buffer Transmission Interrupt Enable */
#define MCANX_TXBTIE_RESETVALUE       _U_(0x00000000) /**< \brief (MCANX_TXBTIE reset_value) Tx Buffer Transmission Interrupt Enable */

#define MCANX_TXBTIE_TIE0_Pos         0            /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 0 */
#define MCANX_TXBTIE_TIE0             (_U_(0x1) << MCANX_TXBTIE_TIE0_Pos)
#define MCANX_TXBTIE_TIE1_Pos         1            /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 1 */
#define MCANX_TXBTIE_TIE1             (_U_(0x1) << MCANX_TXBTIE_TIE1_Pos)
#define MCANX_TXBTIE_TIE2_Pos         2            /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 2 */
#define MCANX_TXBTIE_TIE2             (_U_(0x1) << MCANX_TXBTIE_TIE2_Pos)
#define MCANX_TXBTIE_TIE3_Pos         3            /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 3 */
#define MCANX_TXBTIE_TIE3             (_U_(0x1) << MCANX_TXBTIE_TIE3_Pos)
#define MCANX_TXBTIE_TIE4_Pos         4            /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 4 */
#define MCANX_TXBTIE_TIE4             (_U_(0x1) << MCANX_TXBTIE_TIE4_Pos)
#define MCANX_TXBTIE_TIE5_Pos         5            /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 5 */
#define MCANX_TXBTIE_TIE5             (_U_(0x1) << MCANX_TXBTIE_TIE5_Pos)
#define MCANX_TXBTIE_TIE6_Pos         6            /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 6 */
#define MCANX_TXBTIE_TIE6             (_U_(0x1) << MCANX_TXBTIE_TIE6_Pos)
#define MCANX_TXBTIE_TIE7_Pos         7            /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 7 */
#define MCANX_TXBTIE_TIE7             (_U_(0x1) << MCANX_TXBTIE_TIE7_Pos)
#define MCANX_TXBTIE_TIE8_Pos         8            /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 8 */
#define MCANX_TXBTIE_TIE8             (_U_(0x1) << MCANX_TXBTIE_TIE8_Pos)
#define MCANX_TXBTIE_TIE9_Pos         9            /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 9 */
#define MCANX_TXBTIE_TIE9             (_U_(0x1) << MCANX_TXBTIE_TIE9_Pos)
#define MCANX_TXBTIE_TIE10_Pos        10           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 10 */
#define MCANX_TXBTIE_TIE10            (_U_(0x1) << MCANX_TXBTIE_TIE10_Pos)
#define MCANX_TXBTIE_TIE11_Pos        11           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 11 */
#define MCANX_TXBTIE_TIE11            (_U_(0x1) << MCANX_TXBTIE_TIE11_Pos)
#define MCANX_TXBTIE_TIE12_Pos        12           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 12 */
#define MCANX_TXBTIE_TIE12            (_U_(0x1) << MCANX_TXBTIE_TIE12_Pos)
#define MCANX_TXBTIE_TIE13_Pos        13           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 13 */
#define MCANX_TXBTIE_TIE13            (_U_(0x1) << MCANX_TXBTIE_TIE13_Pos)
#define MCANX_TXBTIE_TIE14_Pos        14           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 14 */
#define MCANX_TXBTIE_TIE14            (_U_(0x1) << MCANX_TXBTIE_TIE14_Pos)
#define MCANX_TXBTIE_TIE15_Pos        15           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 15 */
#define MCANX_TXBTIE_TIE15            (_U_(0x1) << MCANX_TXBTIE_TIE15_Pos)
#define MCANX_TXBTIE_TIE16_Pos        16           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 16 */
#define MCANX_TXBTIE_TIE16            (_U_(0x1) << MCANX_TXBTIE_TIE16_Pos)
#define MCANX_TXBTIE_TIE17_Pos        17           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 17 */
#define MCANX_TXBTIE_TIE17            (_U_(0x1) << MCANX_TXBTIE_TIE17_Pos)
#define MCANX_TXBTIE_TIE18_Pos        18           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 18 */
#define MCANX_TXBTIE_TIE18            (_U_(0x1) << MCANX_TXBTIE_TIE18_Pos)
#define MCANX_TXBTIE_TIE19_Pos        19           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 19 */
#define MCANX_TXBTIE_TIE19            (_U_(0x1) << MCANX_TXBTIE_TIE19_Pos)
#define MCANX_TXBTIE_TIE20_Pos        20           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 20 */
#define MCANX_TXBTIE_TIE20            (_U_(0x1) << MCANX_TXBTIE_TIE20_Pos)
#define MCANX_TXBTIE_TIE21_Pos        21           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 21 */
#define MCANX_TXBTIE_TIE21            (_U_(0x1) << MCANX_TXBTIE_TIE21_Pos)
#define MCANX_TXBTIE_TIE22_Pos        22           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 22 */
#define MCANX_TXBTIE_TIE22            (_U_(0x1) << MCANX_TXBTIE_TIE22_Pos)
#define MCANX_TXBTIE_TIE23_Pos        23           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 23 */
#define MCANX_TXBTIE_TIE23            (_U_(0x1) << MCANX_TXBTIE_TIE23_Pos)
#define MCANX_TXBTIE_TIE24_Pos        24           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 24 */
#define MCANX_TXBTIE_TIE24            (_U_(0x1) << MCANX_TXBTIE_TIE24_Pos)
#define MCANX_TXBTIE_TIE25_Pos        25           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 25 */
#define MCANX_TXBTIE_TIE25            (_U_(0x1) << MCANX_TXBTIE_TIE25_Pos)
#define MCANX_TXBTIE_TIE26_Pos        26           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 26 */
#define MCANX_TXBTIE_TIE26            (_U_(0x1) << MCANX_TXBTIE_TIE26_Pos)
#define MCANX_TXBTIE_TIE27_Pos        27           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 27 */
#define MCANX_TXBTIE_TIE27            (_U_(0x1) << MCANX_TXBTIE_TIE27_Pos)
#define MCANX_TXBTIE_TIE28_Pos        28           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 28 */
#define MCANX_TXBTIE_TIE28            (_U_(0x1) << MCANX_TXBTIE_TIE28_Pos)
#define MCANX_TXBTIE_TIE29_Pos        29           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 29 */
#define MCANX_TXBTIE_TIE29            (_U_(0x1) << MCANX_TXBTIE_TIE29_Pos)
#define MCANX_TXBTIE_TIE30_Pos        30           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 30 */
#define MCANX_TXBTIE_TIE30            (_U_(0x1) << MCANX_TXBTIE_TIE30_Pos)
#define MCANX_TXBTIE_TIE31_Pos        31           /**< \brief (MCANX_TXBTIE) Transmission Interrupt Enable 31 */
#define MCANX_TXBTIE_TIE31            (_U_(0x1) << MCANX_TXBTIE_TIE31_Pos)
#define MCANX_TXBTIE_MASK             _U_(0xFFFFFFFF) /**< \brief (MCANX_TXBTIE) MASK Register */

/* -------- MCANX_TXBCIE : (CAN Offset: 0xE4) (R/W 32) Tx Buffer Cancellation Finished Interrupt Enable -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t CFIE0:1;          /*!< bit:      0  Cancellation Finished Interrupt Enable 0 */
    uint32_t CFIE1:1;          /*!< bit:      1  Cancellation Finished Interrupt Enable 1 */
    uint32_t CFIE2:1;          /*!< bit:      2  Cancellation Finished Interrupt Enable 2 */
    uint32_t CFIE3:1;          /*!< bit:      3  Cancellation Finished Interrupt Enable 3 */
    uint32_t CFIE4:1;          /*!< bit:      4  Cancellation Finished Interrupt Enable 4 */
    uint32_t CFIE5:1;          /*!< bit:      5  Cancellation Finished Interrupt Enable 5 */
    uint32_t CFIE6:1;          /*!< bit:      6  Cancellation Finished Interrupt Enable 6 */
    uint32_t CFIE7:1;          /*!< bit:      7  Cancellation Finished Interrupt Enable 7 */
    uint32_t CFIE8:1;          /*!< bit:      8  Cancellation Finished Interrupt Enable 8 */
    uint32_t CFIE9:1;          /*!< bit:      9  Cancellation Finished Interrupt Enable 9 */
    uint32_t CFIE10:1;         /*!< bit:     10  Cancellation Finished Interrupt Enable 10 */
    uint32_t CFIE11:1;         /*!< bit:     11  Cancellation Finished Interrupt Enable 11 */
    uint32_t CFIE12:1;         /*!< bit:     12  Cancellation Finished Interrupt Enable 12 */
    uint32_t CFIE13:1;         /*!< bit:     13  Cancellation Finished Interrupt Enable 13 */
    uint32_t CFIE14:1;         /*!< bit:     14  Cancellation Finished Interrupt Enable 14 */
    uint32_t CFIE15:1;         /*!< bit:     15  Cancellation Finished Interrupt Enable 15 */
    uint32_t CFIE16:1;         /*!< bit:     16  Cancellation Finished Interrupt Enable 16 */
    uint32_t CFIE17:1;         /*!< bit:     17  Cancellation Finished Interrupt Enable 17 */
    uint32_t CFIE18:1;         /*!< bit:     18  Cancellation Finished Interrupt Enable 18 */
    uint32_t CFIE19:1;         /*!< bit:     19  Cancellation Finished Interrupt Enable 19 */
    uint32_t CFIE20:1;         /*!< bit:     20  Cancellation Finished Interrupt Enable 20 */
    uint32_t CFIE21:1;         /*!< bit:     21  Cancellation Finished Interrupt Enable 21 */
    uint32_t CFIE22:1;         /*!< bit:     22  Cancellation Finished Interrupt Enable 22 */
    uint32_t CFIE23:1;         /*!< bit:     23  Cancellation Finished Interrupt Enable 23 */
    uint32_t CFIE24:1;         /*!< bit:     24  Cancellation Finished Interrupt Enable 24 */
    uint32_t CFIE25:1;         /*!< bit:     25  Cancellation Finished Interrupt Enable 25 */
    uint32_t CFIE26:1;         /*!< bit:     26  Cancellation Finished Interrupt Enable 26 */
    uint32_t CFIE27:1;         /*!< bit:     27  Cancellation Finished Interrupt Enable 27 */
    uint32_t CFIE28:1;         /*!< bit:     28  Cancellation Finished Interrupt Enable 28 */
    uint32_t CFIE29:1;         /*!< bit:     29  Cancellation Finished Interrupt Enable 29 */
    uint32_t CFIE30:1;         /*!< bit:     30  Cancellation Finished Interrupt Enable 30 */
    uint32_t CFIE31:1;         /*!< bit:     31  Cancellation Finished Interrupt Enable 31 */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXBCIE_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXBCIE_OFFSET           0xE4         /**< \brief (MCANX_TXBCIE offset) Tx Buffer Cancellation Finished Interrupt Enable */
#define MCANX_TXBCIE_RESETVALUE       _U_(0x00000000) /**< \brief (MCANX_TXBCIE reset_value) Tx Buffer Cancellation Finished Interrupt Enable */

#define MCANX_TXBCIE_CFIE0_Pos        0            /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 0 */
#define MCANX_TXBCIE_CFIE0            (_U_(0x1) << MCANX_TXBCIE_CFIE0_Pos)
#define MCANX_TXBCIE_CFIE1_Pos        1            /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 1 */
#define MCANX_TXBCIE_CFIE1            (_U_(0x1) << MCANX_TXBCIE_CFIE1_Pos)
#define MCANX_TXBCIE_CFIE2_Pos        2            /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 2 */
#define MCANX_TXBCIE_CFIE2            (_U_(0x1) << MCANX_TXBCIE_CFIE2_Pos)
#define MCANX_TXBCIE_CFIE3_Pos        3            /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 3 */
#define MCANX_TXBCIE_CFIE3            (_U_(0x1) << MCANX_TXBCIE_CFIE3_Pos)
#define MCANX_TXBCIE_CFIE4_Pos        4            /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 4 */
#define MCANX_TXBCIE_CFIE4            (_U_(0x1) << MCANX_TXBCIE_CFIE4_Pos)
#define MCANX_TXBCIE_CFIE5_Pos        5            /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 5 */
#define MCANX_TXBCIE_CFIE5            (_U_(0x1) << MCANX_TXBCIE_CFIE5_Pos)
#define MCANX_TXBCIE_CFIE6_Pos        6            /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 6 */
#define MCANX_TXBCIE_CFIE6            (_U_(0x1) << MCANX_TXBCIE_CFIE6_Pos)
#define MCANX_TXBCIE_CFIE7_Pos        7            /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 7 */
#define MCANX_TXBCIE_CFIE7            (_U_(0x1) << MCANX_TXBCIE_CFIE7_Pos)
#define MCANX_TXBCIE_CFIE8_Pos        8            /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 8 */
#define MCANX_TXBCIE_CFIE8            (_U_(0x1) << MCANX_TXBCIE_CFIE8_Pos)
#define MCANX_TXBCIE_CFIE9_Pos        9            /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 9 */
#define MCANX_TXBCIE_CFIE9            (_U_(0x1) << MCANX_TXBCIE_CFIE9_Pos)
#define MCANX_TXBCIE_CFIE10_Pos       10           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 10 */
#define MCANX_TXBCIE_CFIE10           (_U_(0x1) << MCANX_TXBCIE_CFIE10_Pos)
#define MCANX_TXBCIE_CFIE11_Pos       11           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 11 */
#define MCANX_TXBCIE_CFIE11           (_U_(0x1) << MCANX_TXBCIE_CFIE11_Pos)
#define MCANX_TXBCIE_CFIE12_Pos       12           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 12 */
#define MCANX_TXBCIE_CFIE12           (_U_(0x1) << MCANX_TXBCIE_CFIE12_Pos)
#define MCANX_TXBCIE_CFIE13_Pos       13           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 13 */
#define MCANX_TXBCIE_CFIE13           (_U_(0x1) << MCANX_TXBCIE_CFIE13_Pos)
#define MCANX_TXBCIE_CFIE14_Pos       14           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 14 */
#define MCANX_TXBCIE_CFIE14           (_U_(0x1) << MCANX_TXBCIE_CFIE14_Pos)
#define MCANX_TXBCIE_CFIE15_Pos       15           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 15 */
#define MCANX_TXBCIE_CFIE15           (_U_(0x1) << MCANX_TXBCIE_CFIE15_Pos)
#define MCANX_TXBCIE_CFIE16_Pos       16           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 16 */
#define MCANX_TXBCIE_CFIE16           (_U_(0x1) << MCANX_TXBCIE_CFIE16_Pos)
#define MCANX_TXBCIE_CFIE17_Pos       17           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 17 */
#define MCANX_TXBCIE_CFIE17           (_U_(0x1) << MCANX_TXBCIE_CFIE17_Pos)
#define MCANX_TXBCIE_CFIE18_Pos       18           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 18 */
#define MCANX_TXBCIE_CFIE18           (_U_(0x1) << MCANX_TXBCIE_CFIE18_Pos)
#define MCANX_TXBCIE_CFIE19_Pos       19           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 19 */
#define MCANX_TXBCIE_CFIE19           (_U_(0x1) << MCANX_TXBCIE_CFIE19_Pos)
#define MCANX_TXBCIE_CFIE20_Pos       20           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 20 */
#define MCANX_TXBCIE_CFIE20           (_U_(0x1) << MCANX_TXBCIE_CFIE20_Pos)
#define MCANX_TXBCIE_CFIE21_Pos       21           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 21 */
#define MCANX_TXBCIE_CFIE21           (_U_(0x1) << MCANX_TXBCIE_CFIE21_Pos)
#define MCANX_TXBCIE_CFIE22_Pos       22           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 22 */
#define MCANX_TXBCIE_CFIE22           (_U_(0x1) << MCANX_TXBCIE_CFIE22_Pos)
#define MCANX_TXBCIE_CFIE23_Pos       23           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 23 */
#define MCANX_TXBCIE_CFIE23           (_U_(0x1) << MCANX_TXBCIE_CFIE23_Pos)
#define MCANX_TXBCIE_CFIE24_Pos       24           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 24 */
#define MCANX_TXBCIE_CFIE24           (_U_(0x1) << MCANX_TXBCIE_CFIE24_Pos)
#define MCANX_TXBCIE_CFIE25_Pos       25           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 25 */
#define MCANX_TXBCIE_CFIE25           (_U_(0x1) << MCANX_TXBCIE_CFIE25_Pos)
#define MCANX_TXBCIE_CFIE26_Pos       26           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 26 */
#define MCANX_TXBCIE_CFIE26           (_U_(0x1) << MCANX_TXBCIE_CFIE26_Pos)
#define MCANX_TXBCIE_CFIE27_Pos       27           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 27 */
#define MCANX_TXBCIE_CFIE27           (_U_(0x1) << MCANX_TXBCIE_CFIE27_Pos)
#define MCANX_TXBCIE_CFIE28_Pos       28           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 28 */
#define MCANX_TXBCIE_CFIE28           (_U_(0x1) << MCANX_TXBCIE_CFIE28_Pos)
#define MCANX_TXBCIE_CFIE29_Pos       29           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 29 */
#define MCANX_TXBCIE_CFIE29           (_U_(0x1) << MCANX_TXBCIE_CFIE29_Pos)
#define MCANX_TXBCIE_CFIE30_Pos       30           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 30 */
#define MCANX_TXBCIE_CFIE30           (_U_(0x1) << MCANX_TXBCIE_CFIE30_Pos)
#define MCANX_TXBCIE_CFIE31_Pos       31           /**< \brief (MCANX_TXBCIE) Cancellation Finished Interrupt Enable 31 */
#define MCANX_TXBCIE_CFIE31           (_U_(0x1) << MCANX_TXBCIE_CFIE31_Pos)
#define MCANX_TXBCIE_MASK             _U_(0xFFFFFFFF) /**< \brief (MCANX_TXBCIE) MASK Register */

/* -------- MCANX_TXEFC : (CAN Offset: 0xF0) (R/W 32) Tx Event FIFO Configuration -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t EFSA:16;          /*!< bit:  0..15  Event FIFO Start Address           */
    uint32_t EFS:6;            /*!< bit: 16..21  Event FIFO Size                    */
    uint32_t :2;               /*!< bit: 22..23  Reserved                           */
    uint32_t EFWM:6;           /*!< bit: 24..29  Event FIFO Watermark               */
    uint32_t :2;               /*!< bit: 30..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXEFC_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXEFC_OFFSET            0xF0         /**< \brief (MCANX_TXEFC offset) Tx Event FIFO Configuration */
#define MCANX_TXEFC_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_TXEFC reset_value) Tx Event FIFO Configuration */

#define MCANX_TXEFC_EFSA_Pos          0            /**< \brief (MCANX_TXEFC) Event FIFO Start Address */
#define MCANX_TXEFC_EFSA_Msk          (_U_(0xFFFF) << MCANX_TXEFC_EFSA_Pos)
#define MCANX_TXEFC_EFSA(value)       (MCANX_TXEFC_EFSA_Msk & ((value) << MCANX_TXEFC_EFSA_Pos))
#define MCANX_TXEFC_EFS_Pos           16           /**< \brief (MCANX_TXEFC) Event FIFO Size */
#define MCANX_TXEFC_EFS_Msk           (_U_(0x3F) << MCANX_TXEFC_EFS_Pos)
#define MCANX_TXEFC_EFS(value)        (MCANX_TXEFC_EFS_Msk & ((value) << MCANX_TXEFC_EFS_Pos))
#define MCANX_TXEFC_EFWM_Pos          24           /**< \brief (MCANX_TXEFC) Event FIFO Watermark */
#define MCANX_TXEFC_EFWM_Msk          (_U_(0x3F) << MCANX_TXEFC_EFWM_Pos)
#define MCANX_TXEFC_EFWM(value)       (MCANX_TXEFC_EFWM_Msk & ((value) << MCANX_TXEFC_EFWM_Pos))
#define MCANX_TXEFC_MASK              _U_(0x3F3FFFFF) /**< \brief (MCANX_TXEFC) MASK Register */

/* -------- MCANX_TXEFS : (CAN Offset: 0xF4) (R/  32) Tx Event FIFO Status -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t EFFL:6;           /*!< bit:  0.. 5  Event FIFO Fill Level              */
    uint32_t :2;               /*!< bit:  6.. 7  Reserved                           */
    uint32_t EFGI:5;           /*!< bit:  8..12  Event FIFO Get Index               */
    uint32_t :3;               /*!< bit: 13..15  Reserved                           */
    uint32_t EFPI:5;           /*!< bit: 16..20  Event FIFO Put Index               */
    uint32_t :3;               /*!< bit: 21..23  Reserved                           */
    uint32_t EFF:1;            /*!< bit:     24  Event FIFO Full                    */
    uint32_t TEFL:1;           /*!< bit:     25  Tx Event FIFO Element Lost         */
    uint32_t :6;               /*!< bit: 26..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXEFS_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXEFS_OFFSET            0xF4         /**< \brief (MCANX_TXEFS offset) Tx Event FIFO Status */
#define MCANX_TXEFS_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_TXEFS reset_value) Tx Event FIFO Status */

#define MCANX_TXEFS_EFFL_Pos          0            /**< \brief (MCANX_TXEFS) Event FIFO Fill Level */
#define MCANX_TXEFS_EFFL_Msk          (_U_(0x3F) << MCANX_TXEFS_EFFL_Pos)
#define MCANX_TXEFS_EFFL(value)       (MCANX_TXEFS_EFFL_Msk & ((value) << MCANX_TXEFS_EFFL_Pos))
#define MCANX_TXEFS_EFGI_Pos          8            /**< \brief (MCANX_TXEFS) Event FIFO Get Index */
#define MCANX_TXEFS_EFGI_Msk          (_U_(0x1F) << MCANX_TXEFS_EFGI_Pos)
#define MCANX_TXEFS_EFGI(value)       (MCANX_TXEFS_EFGI_Msk & ((value) << MCANX_TXEFS_EFGI_Pos))
#define MCANX_TXEFS_EFPI_Pos          16           /**< \brief (MCANX_TXEFS) Event FIFO Put Index */
#define MCANX_TXEFS_EFPI_Msk          (_U_(0x1F) << MCANX_TXEFS_EFPI_Pos)
#define MCANX_TXEFS_EFPI(value)       (MCANX_TXEFS_EFPI_Msk & ((value) << MCANX_TXEFS_EFPI_Pos))
#define MCANX_TXEFS_EFF_Pos           24           /**< \brief (MCANX_TXEFS) Event FIFO Full */
#define MCANX_TXEFS_EFF               (_U_(0x1) << MCANX_TXEFS_EFF_Pos)
#define MCANX_TXEFS_TEFL_Pos          25           /**< \brief (MCANX_TXEFS) Tx Event FIFO Element Lost */
#define MCANX_TXEFS_TEFL              (_U_(0x1) << MCANX_TXEFS_TEFL_Pos)
#define MCANX_TXEFS_MASK              _U_(0x031F1F3F) /**< \brief (MCANX_TXEFS) MASK Register */

/* -------- MCANX_TXEFA : (CAN Offset: 0xF8) (R/W 32) Tx Event FIFO Acknowledge -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t EFAI:5;           /*!< bit:  0.. 4  Event FIFO Acknowledge Index       */
    uint32_t :27;              /*!< bit:  5..31  Reserved                           */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXEFA_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXEFA_OFFSET            0xF8         /**< \brief (MCANX_TXEFA offset) Tx Event FIFO Acknowledge */
#define MCANX_TXEFA_RESETVALUE        _U_(0x00000000) /**< \brief (MCANX_TXEFA reset_value) Tx Event FIFO Acknowledge */

#define MCANX_TXEFA_EFAI_Pos          0            /**< \brief (MCANX_TXEFA) Event FIFO Acknowledge Index */
#define MCANX_TXEFA_EFAI_Msk          (_U_(0x1F) << MCANX_TXEFA_EFAI_Pos)
#define MCANX_TXEFA_EFAI(value)       (MCANX_TXEFA_EFAI_Msk & ((value) << MCANX_TXEFA_EFAI_Pos))
#define MCANX_TXEFA_MASK              _U_(0x0000001F) /**< \brief (MCANX_TXEFA) MASK Register */

/* -------- MCANX_RXBE_0 : (CAN Offset: 0x00) (R/W 32) Rx Buffer Element 0 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t ID:29;            /*!< bit:  0..28  Identifier                         */
    uint32_t RTR:1;            /*!< bit:     29  Remote Transmission Request        */
    uint32_t XTD:1;            /*!< bit:     30  Extended Identifier                */
    uint32_t ESI:1;            /*!< bit:     31  Error State Indicator              */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXBE_0_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXBE_0_OFFSET           0x00         /**< \brief (MCANX_RXBE_0 offset) Rx Buffer Element 0 */
#define MCANX_RXBE_0_RESETVALUE       _U_(0x00000000) /**< \brief (MCANX_RXBE_0 reset_value) Rx Buffer Element 0 */

#define MCANX_RXBE_0_ID_Pos           0            /**< \brief (MCANX_RXBE_0) Identifier */
#define MCANX_RXBE_0_ID_Msk           (_U_(0x1FFFFFFF) << MCANX_RXBE_0_ID_Pos)
#define MCANX_RXBE_0_ID(value)        (MCANX_RXBE_0_ID_Msk & ((value) << MCANX_RXBE_0_ID_Pos))
#define MCANX_RXBE_0_RTR_Pos          29           /**< \brief (MCANX_RXBE_0) Remote Transmission Request */
#define MCANX_RXBE_0_RTR              (_U_(0x1) << MCANX_RXBE_0_RTR_Pos)
#define MCANX_RXBE_0_XTD_Pos          30           /**< \brief (MCANX_RXBE_0) Extended Identifier */
#define MCANX_RXBE_0_XTD              (_U_(0x1) << MCANX_RXBE_0_XTD_Pos)
#define MCANX_RXBE_0_ESI_Pos          31           /**< \brief (MCANX_RXBE_0) Error State Indicator */
#define MCANX_RXBE_0_ESI              (_U_(0x1) << MCANX_RXBE_0_ESI_Pos)
#define MCANX_RXBE_0_MASK             _U_(0xFFFFFFFF) /**< \brief (MCANX_RXBE_0) MASK Register */

/* -------- MCANX_RXBE_1 : (CAN Offset: 0x04) (R/W 32) Rx Buffer Element 1 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t RXTS:16;          /*!< bit:  0..15  Rx Timestamp                       */
    uint32_t DLC:4;            /*!< bit: 16..19  Data Length Code                   */
    uint32_t BRS:1;            /*!< bit:     20  Bit Rate Search                    */
    uint32_t FDF:1;            /*!< bit:     21  FD Format                          */
    uint32_t :2;               /*!< bit: 22..23  Reserved                           */
    uint32_t FIDX:7;           /*!< bit: 24..30  Filter Index                       */
    uint32_t ANMF:1;           /*!< bit:     31  Accepted Non-matching Frame        */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXBE_1_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXBE_1_OFFSET           0x04         /**< \brief (MCANX_RXBE_1 offset) Rx Buffer Element 1 */
#define MCANX_RXBE_1_RESETVALUE       _U_(0x00000000) /**< \brief (MCANX_RXBE_1 reset_value) Rx Buffer Element 1 */

#define MCANX_RXBE_1_RXTS_Pos         0            /**< \brief (MCANX_RXBE_1) Rx Timestamp */
#define MCANX_RXBE_1_RXTS_Msk         (_U_(0xFFFF) << MCANX_RXBE_1_RXTS_Pos)
#define MCANX_RXBE_1_RXTS(value)      (MCANX_RXBE_1_RXTS_Msk & ((value) << MCANX_RXBE_1_RXTS_Pos))
#define MCANX_RXBE_1_DLC_Pos          16           /**< \brief (MCANX_RXBE_1) Data Length Code */
#define MCANX_RXBE_1_DLC_Msk          (_U_(0xF) << MCANX_RXBE_1_DLC_Pos)
#define MCANX_RXBE_1_DLC(value)       (MCANX_RXBE_1_DLC_Msk & ((value) << MCANX_RXBE_1_DLC_Pos))
#define MCANX_RXBE_1_BRS_Pos          20           /**< \brief (MCANX_RXBE_1) Bit Rate Search */
#define MCANX_RXBE_1_BRS              (_U_(0x1) << MCANX_RXBE_1_BRS_Pos)
#define MCANX_RXBE_1_FDF_Pos          21           /**< \brief (MCANX_RXBE_1) FD Format */
#define MCANX_RXBE_1_FDF              (_U_(0x1) << MCANX_RXBE_1_FDF_Pos)
#define MCANX_RXBE_1_FIDX_Pos         24           /**< \brief (MCANX_RXBE_1) Filter Index */
#define MCANX_RXBE_1_FIDX_Msk         (_U_(0x7F) << MCANX_RXBE_1_FIDX_Pos)
#define MCANX_RXBE_1_FIDX(value)      (MCANX_RXBE_1_FIDX_Msk & ((value) << MCANX_RXBE_1_FIDX_Pos))
#define MCANX_RXBE_1_ANMF_Pos         31           /**< \brief (MCANX_RXBE_1) Accepted Non-matching Frame */
#define MCANX_RXBE_1_ANMF             (_U_(0x1) << MCANX_RXBE_1_ANMF_Pos)
#define MCANX_RXBE_1_MASK             _U_(0xFF3FFFFF) /**< \brief (MCANX_RXBE_1) MASK Register */

/* -------- MCANX_RXBE_DATA : (CAN Offset: 0x08) (R/W 32) Rx Buffer Element Data -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t DB0:8;            /*!< bit:  0.. 7  Data Byte 0                        */
    uint32_t DB1:8;            /*!< bit:  8..15  Data Byte 1                        */
    uint32_t DB2:8;            /*!< bit: 16..23  Data Byte 2                        */
    uint32_t DB3:8;            /*!< bit: 24..31  Data Byte 3                        */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXBE_DATA_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXBE_DATA_OFFSET        0x08         /**< \brief (MCANX_RXBE_DATA offset) Rx Buffer Element Data */
#define MCANX_RXBE_DATA_RESETVALUE    _U_(0x00000000) /**< \brief (MCANX_RXBE_DATA reset_value) Rx Buffer Element Data */

#define MCANX_RXBE_DATA_DB0_Pos       0            /**< \brief (MCANX_RXBE_DATA) Data Byte 0 */
#define MCANX_RXBE_DATA_DB0_Msk       (_U_(0xFF) << MCANX_RXBE_DATA_DB0_Pos)
#define MCANX_RXBE_DATA_DB0(value)    (MCANX_RXBE_DATA_DB0_Msk & ((value) << MCANX_RXBE_DATA_DB0_Pos))
#define MCANX_RXBE_DATA_DB1_Pos       8            /**< \brief (MCANX_RXBE_DATA) Data Byte 1 */
#define MCANX_RXBE_DATA_DB1_Msk       (_U_(0xFF) << MCANX_RXBE_DATA_DB1_Pos)
#define MCANX_RXBE_DATA_DB1(value)    (MCANX_RXBE_DATA_DB1_Msk & ((value) << MCANX_RXBE_DATA_DB1_Pos))
#define MCANX_RXBE_DATA_DB2_Pos       16           /**< \brief (MCANX_RXBE_DATA) Data Byte 2 */
#define MCANX_RXBE_DATA_DB2_Msk       (_U_(0xFF) << MCANX_RXBE_DATA_DB2_Pos)
#define MCANX_RXBE_DATA_DB2(value)    (MCANX_RXBE_DATA_DB2_Msk & ((value) << MCANX_RXBE_DATA_DB2_Pos))
#define MCANX_RXBE_DATA_DB3_Pos       24           /**< \brief (MCANX_RXBE_DATA) Data Byte 3 */
#define MCANX_RXBE_DATA_DB3_Msk       (_U_(0xFF) << MCANX_RXBE_DATA_DB3_Pos)
#define MCANX_RXBE_DATA_DB3(value)    (MCANX_RXBE_DATA_DB3_Msk & ((value) << MCANX_RXBE_DATA_DB3_Pos))
#define MCANX_RXBE_DATA_MASK          _U_(0xFFFFFFFF) /**< \brief (MCANX_RXBE_DATA) MASK Register */

/* -------- MCANX_RXF0E_0 : (CAN Offset: 0x00) (R/W 32) Rx FIFO 0 Element 0 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t ID:29;            /*!< bit:  0..28  Identifier                         */
    uint32_t RTR:1;            /*!< bit:     29  Remote Transmission Request        */
    uint32_t XTD:1;            /*!< bit:     30  Extended Identifier                */
    uint32_t ESI:1;            /*!< bit:     31  Error State Indicator              */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF0E_0_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF0E_0_OFFSET          0x00         /**< \brief (MCANX_RXF0E_0 offset) Rx FIFO 0 Element 0 */
#define MCANX_RXF0E_0_RESETVALUE      _U_(0x00000000) /**< \brief (MCANX_RXF0E_0 reset_value) Rx FIFO 0 Element 0 */

#define MCANX_RXF0E_0_ID_Pos          0            /**< \brief (MCANX_RXF0E_0) Identifier */
#define MCANX_RXF0E_0_ID_Msk          (_U_(0x1FFFFFFF) << MCANX_RXF0E_0_ID_Pos)
#define MCANX_RXF0E_0_ID(value)       (MCANX_RXF0E_0_ID_Msk & ((value) << MCANX_RXF0E_0_ID_Pos))
#define MCANX_RXF0E_0_RTR_Pos         29           /**< \brief (MCANX_RXF0E_0) Remote Transmission Request */
#define MCANX_RXF0E_0_RTR             (_U_(0x1) << MCANX_RXF0E_0_RTR_Pos)
#define MCANX_RXF0E_0_XTD_Pos         30           /**< \brief (MCANX_RXF0E_0) Extended Identifier */
#define MCANX_RXF0E_0_XTD             (_U_(0x1) << MCANX_RXF0E_0_XTD_Pos)
#define MCANX_RXF0E_0_ESI_Pos         31           /**< \brief (MCANX_RXF0E_0) Error State Indicator */
#define MCANX_RXF0E_0_ESI             (_U_(0x1) << MCANX_RXF0E_0_ESI_Pos)
#define MCANX_RXF0E_0_MASK            _U_(0xFFFFFFFF) /**< \brief (MCANX_RXF0E_0) MASK Register */

/* -------- MCANX_RXF0E_1 : (CAN Offset: 0x04) (R/W 32) Rx FIFO 0 Element 1 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t RXTS:16;          /*!< bit:  0..15  Rx Timestamp                       */
    uint32_t DLC:4;            /*!< bit: 16..19  Data Length Code                   */
    uint32_t BRS:1;            /*!< bit:     20  Bit Rate Search                    */
    uint32_t FDF:1;            /*!< bit:     21  FD Format                          */
    uint32_t :2;               /*!< bit: 22..23  Reserved                           */
    uint32_t FIDX:7;           /*!< bit: 24..30  Filter Index                       */
    uint32_t ANMF:1;           /*!< bit:     31  Accepted Non-matching Frame        */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF0E_1_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF0E_1_OFFSET          0x04         /**< \brief (MCANX_RXF0E_1 offset) Rx FIFO 0 Element 1 */
#define MCANX_RXF0E_1_RESETVALUE      _U_(0x00000000) /**< \brief (MCANX_RXF0E_1 reset_value) Rx FIFO 0 Element 1 */

#define MCANX_RXF0E_1_RXTS_Pos        0            /**< \brief (MCANX_RXF0E_1) Rx Timestamp */
#define MCANX_RXF0E_1_RXTS_Msk        (_U_(0xFFFF) << MCANX_RXF0E_1_RXTS_Pos)
#define MCANX_RXF0E_1_RXTS(value)     (MCANX_RXF0E_1_RXTS_Msk & ((value) << MCANX_RXF0E_1_RXTS_Pos))
#define MCANX_RXF0E_1_DLC_Pos         16           /**< \brief (MCANX_RXF0E_1) Data Length Code */
#define MCANX_RXF0E_1_DLC_Msk         (_U_(0xF) << MCANX_RXF0E_1_DLC_Pos)
#define MCANX_RXF0E_1_DLC(value)      (MCANX_RXF0E_1_DLC_Msk & ((value) << MCANX_RXF0E_1_DLC_Pos))
#define MCANX_RXF0E_1_BRS_Pos         20           /**< \brief (MCANX_RXF0E_1) Bit Rate Search */
#define MCANX_RXF0E_1_BRS             (_U_(0x1) << MCANX_RXF0E_1_BRS_Pos)
#define MCANX_RXF0E_1_FDF_Pos         21           /**< \brief (MCANX_RXF0E_1) FD Format */
#define MCANX_RXF0E_1_FDF             (_U_(0x1) << MCANX_RXF0E_1_FDF_Pos)
#define MCANX_RXF0E_1_FIDX_Pos        24           /**< \brief (MCANX_RXF0E_1) Filter Index */
#define MCANX_RXF0E_1_FIDX_Msk        (_U_(0x7F) << MCANX_RXF0E_1_FIDX_Pos)
#define MCANX_RXF0E_1_FIDX(value)     (MCANX_RXF0E_1_FIDX_Msk & ((value) << MCANX_RXF0E_1_FIDX_Pos))
#define MCANX_RXF0E_1_ANMF_Pos        31           /**< \brief (MCANX_RXF0E_1) Accepted Non-matching Frame */
#define MCANX_RXF0E_1_ANMF            (_U_(0x1) << MCANX_RXF0E_1_ANMF_Pos)
#define MCANX_RXF0E_1_MASK            _U_(0xFF3FFFFF) /**< \brief (MCANX_RXF0E_1) MASK Register */

/* -------- MCANX_RXF0E_DATA : (CAN Offset: 0x08) (R/W 32) Rx FIFO 0 Element Data -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t DB0:8;            /*!< bit:  0.. 7  Data Byte 0                        */
    uint32_t DB1:8;            /*!< bit:  8..15  Data Byte 1                        */
    uint32_t DB2:8;            /*!< bit: 16..23  Data Byte 2                        */
    uint32_t DB3:8;            /*!< bit: 24..31  Data Byte 3                        */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF0E_DATA_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF0E_DATA_OFFSET       0x08         /**< \brief (MCANX_RXF0E_DATA offset) Rx FIFO 0 Element Data */
#define MCANX_RXF0E_DATA_RESETVALUE   _U_(0x00000000) /**< \brief (MCANX_RXF0E_DATA reset_value) Rx FIFO 0 Element Data */

#define MCANX_RXF0E_DATA_DB0_Pos      0            /**< \brief (MCANX_RXF0E_DATA) Data Byte 0 */
#define MCANX_RXF0E_DATA_DB0_Msk      (_U_(0xFF) << MCANX_RXF0E_DATA_DB0_Pos)
#define MCANX_RXF0E_DATA_DB0(value)   (MCANX_RXF0E_DATA_DB0_Msk & ((value) << MCANX_RXF0E_DATA_DB0_Pos))
#define MCANX_RXF0E_DATA_DB1_Pos      8            /**< \brief (MCANX_RXF0E_DATA) Data Byte 1 */
#define MCANX_RXF0E_DATA_DB1_Msk      (_U_(0xFF) << MCANX_RXF0E_DATA_DB1_Pos)
#define MCANX_RXF0E_DATA_DB1(value)   (MCANX_RXF0E_DATA_DB1_Msk & ((value) << MCANX_RXF0E_DATA_DB1_Pos))
#define MCANX_RXF0E_DATA_DB2_Pos      16           /**< \brief (MCANX_RXF0E_DATA) Data Byte 2 */
#define MCANX_RXF0E_DATA_DB2_Msk      (_U_(0xFF) << MCANX_RXF0E_DATA_DB2_Pos)
#define MCANX_RXF0E_DATA_DB2(value)   (MCANX_RXF0E_DATA_DB2_Msk & ((value) << MCANX_RXF0E_DATA_DB2_Pos))
#define MCANX_RXF0E_DATA_DB3_Pos      24           /**< \brief (MCANX_RXF0E_DATA) Data Byte 3 */
#define MCANX_RXF0E_DATA_DB3_Msk      (_U_(0xFF) << MCANX_RXF0E_DATA_DB3_Pos)
#define MCANX_RXF0E_DATA_DB3(value)   (MCANX_RXF0E_DATA_DB3_Msk & ((value) << MCANX_RXF0E_DATA_DB3_Pos))
#define MCANX_RXF0E_DATA_MASK         _U_(0xFFFFFFFF) /**< \brief (MCANX_RXF0E_DATA) MASK Register */

/* -------- MCANX_RXF1E_0 : (CAN Offset: 0x00) (R/W 32) Rx FIFO 1 Element 0 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t ID:29;            /*!< bit:  0..28  Identifier                         */
    uint32_t RTR:1;            /*!< bit:     29  Remote Transmission Request        */
    uint32_t XTD:1;            /*!< bit:     30  Extended Identifier                */
    uint32_t ESI:1;            /*!< bit:     31  Error State Indicator              */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF1E_0_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF1E_0_OFFSET          0x00         /**< \brief (MCANX_RXF1E_0 offset) Rx FIFO 1 Element 0 */
#define MCANX_RXF1E_0_RESETVALUE      _U_(0x00000000) /**< \brief (MCANX_RXF1E_0 reset_value) Rx FIFO 1 Element 0 */

#define MCANX_RXF1E_0_ID_Pos          0            /**< \brief (MCANX_RXF1E_0) Identifier */
#define MCANX_RXF1E_0_ID_Msk          (_U_(0x1FFFFFFF) << MCANX_RXF1E_0_ID_Pos)
#define MCANX_RXF1E_0_ID(value)       (MCANX_RXF1E_0_ID_Msk & ((value) << MCANX_RXF1E_0_ID_Pos))
#define MCANX_RXF1E_0_RTR_Pos         29           /**< \brief (MCANX_RXF1E_0) Remote Transmission Request */
#define MCANX_RXF1E_0_RTR             (_U_(0x1) << MCANX_RXF1E_0_RTR_Pos)
#define MCANX_RXF1E_0_XTD_Pos         30           /**< \brief (MCANX_RXF1E_0) Extended Identifier */
#define MCANX_RXF1E_0_XTD             (_U_(0x1) << MCANX_RXF1E_0_XTD_Pos)
#define MCANX_RXF1E_0_ESI_Pos         31           /**< \brief (MCANX_RXF1E_0) Error State Indicator */
#define MCANX_RXF1E_0_ESI             (_U_(0x1) << MCANX_RXF1E_0_ESI_Pos)
#define MCANX_RXF1E_0_MASK            _U_(0xFFFFFFFF) /**< \brief (MCANX_RXF1E_0) MASK Register */

/* -------- MCANX_RXF1E_1 : (CAN Offset: 0x04) (R/W 32) Rx FIFO 1 Element 1 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t RXTS:16;          /*!< bit:  0..15  Rx Timestamp                       */
    uint32_t DLC:4;            /*!< bit: 16..19  Data Length Code                   */
    uint32_t BRS:1;            /*!< bit:     20  Bit Rate Search                    */
    uint32_t FDF:1;            /*!< bit:     21  FD Format                          */
    uint32_t :2;               /*!< bit: 22..23  Reserved                           */
    uint32_t FIDX:7;           /*!< bit: 24..30  Filter Index                       */
    uint32_t ANMF:1;           /*!< bit:     31  Accepted Non-matching Frame        */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF1E_1_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF1E_1_OFFSET          0x04         /**< \brief (MCANX_RXF1E_1 offset) Rx FIFO 1 Element 1 */
#define MCANX_RXF1E_1_RESETVALUE      _U_(0x00000000) /**< \brief (MCANX_RXF1E_1 reset_value) Rx FIFO 1 Element 1 */

#define MCANX_RXF1E_1_RXTS_Pos        0            /**< \brief (MCANX_RXF1E_1) Rx Timestamp */
#define MCANX_RXF1E_1_RXTS_Msk        (_U_(0xFFFF) << MCANX_RXF1E_1_RXTS_Pos)
#define MCANX_RXF1E_1_RXTS(value)     (MCANX_RXF1E_1_RXTS_Msk & ((value) << MCANX_RXF1E_1_RXTS_Pos))
#define MCANX_RXF1E_1_DLC_Pos         16           /**< \brief (MCANX_RXF1E_1) Data Length Code */
#define MCANX_RXF1E_1_DLC_Msk         (_U_(0xF) << MCANX_RXF1E_1_DLC_Pos)
#define MCANX_RXF1E_1_DLC(value)      (MCANX_RXF1E_1_DLC_Msk & ((value) << MCANX_RXF1E_1_DLC_Pos))
#define MCANX_RXF1E_1_BRS_Pos         20           /**< \brief (MCANX_RXF1E_1) Bit Rate Search */
#define MCANX_RXF1E_1_BRS             (_U_(0x1) << MCANX_RXF1E_1_BRS_Pos)
#define MCANX_RXF1E_1_FDF_Pos         21           /**< \brief (MCANX_RXF1E_1) FD Format */
#define MCANX_RXF1E_1_FDF             (_U_(0x1) << MCANX_RXF1E_1_FDF_Pos)
#define MCANX_RXF1E_1_FIDX_Pos        24           /**< \brief (MCANX_RXF1E_1) Filter Index */
#define MCANX_RXF1E_1_FIDX_Msk        (_U_(0x7F) << MCANX_RXF1E_1_FIDX_Pos)
#define MCANX_RXF1E_1_FIDX(value)     (MCANX_RXF1E_1_FIDX_Msk & ((value) << MCANX_RXF1E_1_FIDX_Pos))
#define MCANX_RXF1E_1_ANMF_Pos        31           /**< \brief (MCANX_RXF1E_1) Accepted Non-matching Frame */
#define MCANX_RXF1E_1_ANMF            (_U_(0x1) << MCANX_RXF1E_1_ANMF_Pos)
#define MCANX_RXF1E_1_MASK            _U_(0xFF3FFFFF) /**< \brief (MCANX_RXF1E_1) MASK Register */

/* -------- MCANX_RXF1E_DATA : (CAN Offset: 0x08) (R/W 32) Rx FIFO 1 Element Data -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t DB0:8;            /*!< bit:  0.. 7  Data Byte 0                        */
    uint32_t DB1:8;            /*!< bit:  8..15  Data Byte 1                        */
    uint32_t DB2:8;            /*!< bit: 16..23  Data Byte 2                        */
    uint32_t DB3:8;            /*!< bit: 24..31  Data Byte 3                        */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_RXF1E_DATA_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_RXF1E_DATA_OFFSET       0x08         /**< \brief (MCANX_RXF1E_DATA offset) Rx FIFO 1 Element Data */
#define MCANX_RXF1E_DATA_RESETVALUE   _U_(0x00000000) /**< \brief (MCANX_RXF1E_DATA reset_value) Rx FIFO 1 Element Data */

#define MCANX_RXF1E_DATA_DB0_Pos      0            /**< \brief (MCANX_RXF1E_DATA) Data Byte 0 */
#define MCANX_RXF1E_DATA_DB0_Msk      (_U_(0xFF) << MCANX_RXF1E_DATA_DB0_Pos)
#define MCANX_RXF1E_DATA_DB0(value)   (MCANX_RXF1E_DATA_DB0_Msk & ((value) << MCANX_RXF1E_DATA_DB0_Pos))
#define MCANX_RXF1E_DATA_DB1_Pos      8            /**< \brief (MCANX_RXF1E_DATA) Data Byte 1 */
#define MCANX_RXF1E_DATA_DB1_Msk      (_U_(0xFF) << MCANX_RXF1E_DATA_DB1_Pos)
#define MCANX_RXF1E_DATA_DB1(value)   (MCANX_RXF1E_DATA_DB1_Msk & ((value) << MCANX_RXF1E_DATA_DB1_Pos))
#define MCANX_RXF1E_DATA_DB2_Pos      16           /**< \brief (MCANX_RXF1E_DATA) Data Byte 2 */
#define MCANX_RXF1E_DATA_DB2_Msk      (_U_(0xFF) << MCANX_RXF1E_DATA_DB2_Pos)
#define MCANX_RXF1E_DATA_DB2(value)   (MCANX_RXF1E_DATA_DB2_Msk & ((value) << MCANX_RXF1E_DATA_DB2_Pos))
#define MCANX_RXF1E_DATA_DB3_Pos      24           /**< \brief (MCANX_RXF1E_DATA) Data Byte 3 */
#define MCANX_RXF1E_DATA_DB3_Msk      (_U_(0xFF) << MCANX_RXF1E_DATA_DB3_Pos)
#define MCANX_RXF1E_DATA_DB3(value)   (MCANX_RXF1E_DATA_DB3_Msk & ((value) << MCANX_RXF1E_DATA_DB3_Pos))
#define MCANX_RXF1E_DATA_MASK         _U_(0xFFFFFFFF) /**< \brief (MCANX_RXF1E_DATA) MASK Register */

/* -------- MCANX_SIDFE_0 : (CAN Offset: 0x00) (R/W 32) Standard Message ID Filter Element -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t SFID2:11;         /*!< bit:  0..10  Standard Filter ID 2               */
    uint32_t :5;               /*!< bit: 11..15  Reserved                           */
    uint32_t SFID1:11;         /*!< bit: 16..26  Standard Filter ID 1               */
    uint32_t SFEC:3;           /*!< bit: 27..29  Standard Filter Element Configuration */
    uint32_t SFT:2;            /*!< bit: 30..31  Standard Filter Type               */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_SIDFE_0_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_SIDFE_0_OFFSET          0x00         /**< \brief (MCANX_SIDFE_0 offset) Standard Message ID Filter Element */
#define MCANX_SIDFE_0_RESETVALUE      _U_(0x00000000) /**< \brief (MCANX_SIDFE_0 reset_value) Standard Message ID Filter Element */

#define MCANX_SIDFE_0_SFID2_Pos       0            /**< \brief (MCANX_SIDFE_0) Standard Filter ID 2 */
#define MCANX_SIDFE_0_SFID2_Msk       (_U_(0x7FF) << MCANX_SIDFE_0_SFID2_Pos)
#define MCANX_SIDFE_0_SFID2(value)    (MCANX_SIDFE_0_SFID2_Msk & ((value) << MCANX_SIDFE_0_SFID2_Pos))
#define MCANX_SIDFE_0_SFID1_Pos       16           /**< \brief (MCANX_SIDFE_0) Standard Filter ID 1 */
#define MCANX_SIDFE_0_SFID1_Msk       (_U_(0x7FF) << MCANX_SIDFE_0_SFID1_Pos)
#define MCANX_SIDFE_0_SFID1(value)    (MCANX_SIDFE_0_SFID1_Msk & ((value) << MCANX_SIDFE_0_SFID1_Pos))
#define MCANX_SIDFE_0_SFEC_Pos        27           /**< \brief (MCANX_SIDFE_0) Standard Filter Element Configuration */
#define MCANX_SIDFE_0_SFEC_Msk        (_U_(0x7) << MCANX_SIDFE_0_SFEC_Pos)
#define MCANX_SIDFE_0_SFEC(value)     (MCANX_SIDFE_0_SFEC_Msk & ((value) << MCANX_SIDFE_0_SFEC_Pos))
#define   MCANX_SIDFE_0_SFEC_DISABLE_Val    _U_(0x0)   /**< \brief (MCANX_SIDFE_0) Disable filter element */
#define   MCANX_SIDFE_0_SFEC_STF0M_Val      _U_(0x1)   /**< \brief (MCANX_SIDFE_0) Store in Rx FIFO 0 if filter match */
#define   MCANX_SIDFE_0_SFEC_STF1M_Val      _U_(0x2)   /**< \brief (MCANX_SIDFE_0) Store in Rx FIFO 1 if filter match */
#define   MCANX_SIDFE_0_SFEC_REJECT_Val     _U_(0x3)   /**< \brief (MCANX_SIDFE_0) Reject ID if filter match */
#define   MCANX_SIDFE_0_SFEC_PRIORITY_Val   _U_(0x4)   /**< \brief (MCANX_SIDFE_0) Set priority if filter match */
#define   MCANX_SIDFE_0_SFEC_PRIF0M_Val     _U_(0x5)   /**< \brief (MCANX_SIDFE_0) Set priority and store in FIFO 0 if filter match */
#define   MCANX_SIDFE_0_SFEC_PRIF1M_Val     _U_(0x6)   /**< \brief (MCANX_SIDFE_0) Set priority and store in FIFO 1 if filter match */
#define   MCANX_SIDFE_0_SFEC_STRXBUF_Val    _U_(0x7)   /**< \brief (MCANX_SIDFE_0) Store into Rx Buffer */
#define MCANX_SIDFE_0_SFEC_DISABLE    (MCANX_SIDFE_0_SFEC_DISABLE_Val  << MCANX_SIDFE_0_SFEC_Pos)
#define MCANX_SIDFE_0_SFEC_STF0M      (MCANX_SIDFE_0_SFEC_STF0M_Val    << MCANX_SIDFE_0_SFEC_Pos)
#define MCANX_SIDFE_0_SFEC_STF1M      (MCANX_SIDFE_0_SFEC_STF1M_Val    << MCANX_SIDFE_0_SFEC_Pos)
#define MCANX_SIDFE_0_SFEC_REJECT     (MCANX_SIDFE_0_SFEC_REJECT_Val   << MCANX_SIDFE_0_SFEC_Pos)
#define MCANX_SIDFE_0_SFEC_PRIORITY   (MCANX_SIDFE_0_SFEC_PRIORITY_Val << MCANX_SIDFE_0_SFEC_Pos)
#define MCANX_SIDFE_0_SFEC_PRIF0M     (MCANX_SIDFE_0_SFEC_PRIF0M_Val   << MCANX_SIDFE_0_SFEC_Pos)
#define MCANX_SIDFE_0_SFEC_PRIF1M     (MCANX_SIDFE_0_SFEC_PRIF1M_Val   << MCANX_SIDFE_0_SFEC_Pos)
#define MCANX_SIDFE_0_SFEC_STRXBUF    (MCANX_SIDFE_0_SFEC_STRXBUF_Val  << MCANX_SIDFE_0_SFEC_Pos)
#define MCANX_SIDFE_0_SFT_Pos         30           /**< \brief (MCANX_SIDFE_0) Standard Filter Type */
#define MCANX_SIDFE_0_SFT_Msk         (_U_(0x3) << MCANX_SIDFE_0_SFT_Pos)
#define MCANX_SIDFE_0_SFT(value)      (MCANX_SIDFE_0_SFT_Msk & ((value) << MCANX_SIDFE_0_SFT_Pos))
#define   MCANX_SIDFE_0_SFT_RANGE_Val       _U_(0x0)   /**< \brief (MCANX_SIDFE_0) Range filter from SFID1 to SFID2 */
#define   MCANX_SIDFE_0_SFT_DUAL_Val        _U_(0x1)   /**< \brief (MCANX_SIDFE_0) Dual ID filter for SFID1 or SFID2 */
#define   MCANX_SIDFE_0_SFT_CLASSIC_Val     _U_(0x2)   /**< \brief (MCANX_SIDFE_0) Classic filter */
#define MCANX_SIDFE_0_SFT_RANGE       (MCANX_SIDFE_0_SFT_RANGE_Val     << MCANX_SIDFE_0_SFT_Pos)
#define MCANX_SIDFE_0_SFT_DUAL        (MCANX_SIDFE_0_SFT_DUAL_Val      << MCANX_SIDFE_0_SFT_Pos)
#define MCANX_SIDFE_0_SFT_CLASSIC     (MCANX_SIDFE_0_SFT_CLASSIC_Val   << MCANX_SIDFE_0_SFT_Pos)
#define MCANX_SIDFE_0_MASK            _U_(0xFFFF07FF) /**< \brief (MCANX_SIDFE_0) MASK Register */

/* -------- MCANX_TXBE_0 : (CAN Offset: 0x00) (R/W 32) Tx Buffer Element 0 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t ID:29;            /*!< bit:  0..28  Identifier                         */
    uint32_t RTR:1;            /*!< bit:     29  Remote Transmission Request        */
    uint32_t XTD:1;            /*!< bit:     30  Extended Identifier                */
    uint32_t ESI:1;            /*!< bit:     31  Error State Indicator              */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXBE_0_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXBE_0_OFFSET           0x00         /**< \brief (MCANX_TXBE_0 offset) Tx Buffer Element 0 */
#define MCANX_TXBE_0_RESETVALUE       _U_(0x00000000) /**< \brief (MCANX_TXBE_0 reset_value) Tx Buffer Element 0 */

#define MCANX_TXBE_0_ID_Pos           0            /**< \brief (MCANX_TXBE_0) Identifier */
#define MCANX_TXBE_0_ID_Msk           (_U_(0x1FFFFFFF) << MCANX_TXBE_0_ID_Pos)
#define MCANX_TXBE_0_ID(value)        (MCANX_TXBE_0_ID_Msk & ((value) << MCANX_TXBE_0_ID_Pos))
#define MCANX_TXBE_0_RTR_Pos          29           /**< \brief (MCANX_TXBE_0) Remote Transmission Request */
#define MCANX_TXBE_0_RTR              (_U_(0x1) << MCANX_TXBE_0_RTR_Pos)
#define MCANX_TXBE_0_XTD_Pos          30           /**< \brief (MCANX_TXBE_0) Extended Identifier */
#define MCANX_TXBE_0_XTD              (_U_(0x1) << MCANX_TXBE_0_XTD_Pos)
#define MCANX_TXBE_0_ESI_Pos          31           /**< \brief (MCANX_TXBE_0) Error State Indicator */
#define MCANX_TXBE_0_ESI              (_U_(0x1) << MCANX_TXBE_0_ESI_Pos)
#define MCANX_TXBE_0_MASK             _U_(0xFFFFFFFF) /**< \brief (MCANX_TXBE_0) MASK Register */

/* -------- MCANX_TXBE_1 : (CAN Offset: 0x04) (R/W 32) Tx Buffer Element 1 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t :16;              /*!< bit:  0..15  Reserved                           */
    uint32_t DLC:4;            /*!< bit: 16..19  Identifier                         */
    uint32_t BRS:1;            /*!< bit:     20  Bit Rate Search                    */
    uint32_t FDF:1;            /*!< bit:     21  FD Format                          */
    uint32_t :1;               /*!< bit:     22  Reserved                           */
    uint32_t EFC:1;            /*!< bit:     23  Event FIFO Control                 */
    uint32_t MM:8;             /*!< bit: 24..31  Message Marker                     */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXBE_1_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXBE_1_OFFSET           0x04         /**< \brief (MCANX_TXBE_1 offset) Tx Buffer Element 1 */
#define MCANX_TXBE_1_RESETVALUE       _U_(0x00000000) /**< \brief (MCANX_TXBE_1 reset_value) Tx Buffer Element 1 */

#define MCANX_TXBE_1_DLC_Pos          16           /**< \brief (MCANX_TXBE_1) Identifier */
#define MCANX_TXBE_1_DLC_Msk          (_U_(0xF) << MCANX_TXBE_1_DLC_Pos)
#define MCANX_TXBE_1_DLC(value)       (MCANX_TXBE_1_DLC_Msk & ((value) << MCANX_TXBE_1_DLC_Pos))
#define MCANX_TXBE_1_BRS_Pos          20           /**< \brief (MCANX_TXBE_1) Bit Rate Search */
#define MCANX_TXBE_1_BRS              (_U_(0x1) << MCANX_TXBE_1_BRS_Pos)
#define MCANX_TXBE_1_FDF_Pos          21           /**< \brief (MCANX_TXBE_1) FD Format */
#define MCANX_TXBE_1_FDF              (_U_(0x1) << MCANX_TXBE_1_FDF_Pos)
#define MCANX_TXBE_1_EFC_Pos          23           /**< \brief (MCANX_TXBE_1) Event FIFO Control */
#define MCANX_TXBE_1_EFC              (_U_(0x1) << MCANX_TXBE_1_EFC_Pos)
#define MCANX_TXBE_1_MM_Pos           24           /**< \brief (MCANX_TXBE_1) Message Marker */
#define MCANX_TXBE_1_MM_Msk           (_U_(0xFF) << MCANX_TXBE_1_MM_Pos)
#define MCANX_TXBE_1_MM(value)        (MCANX_TXBE_1_MM_Msk & ((value) << MCANX_TXBE_1_MM_Pos))
#define MCANX_TXBE_1_MASK             _U_(0xFFBF0000) /**< \brief (MCANX_TXBE_1) MASK Register */

/* -------- MCANX_TXBE_DATA : (CAN Offset: 0x08) (R/W 32) Tx Buffer Element Data -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t DB0:8;            /*!< bit:  0.. 7  Data Byte 0                        */
    uint32_t DB1:8;            /*!< bit:  8..15  Data Byte 1                        */
    uint32_t DB2:8;            /*!< bit: 16..23  Data Byte 2                        */
    uint32_t DB3:8;            /*!< bit: 24..31  Data Byte 3                        */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXBE_DATA_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXBE_DATA_OFFSET        0x08         /**< \brief (MCANX_TXBE_DATA offset) Tx Buffer Element Data */
#define MCANX_TXBE_DATA_RESETVALUE    _U_(0x00000000) /**< \brief (MCANX_TXBE_DATA reset_value) Tx Buffer Element Data */

#define MCANX_TXBE_DATA_DB0_Pos       0            /**< \brief (MCANX_TXBE_DATA) Data Byte 0 */
#define MCANX_TXBE_DATA_DB0_Msk       (_U_(0xFF) << MCANX_TXBE_DATA_DB0_Pos)
#define MCANX_TXBE_DATA_DB0(value)    (MCANX_TXBE_DATA_DB0_Msk & ((value) << MCANX_TXBE_DATA_DB0_Pos))
#define MCANX_TXBE_DATA_DB1_Pos       8            /**< \brief (MCANX_TXBE_DATA) Data Byte 1 */
#define MCANX_TXBE_DATA_DB1_Msk       (_U_(0xFF) << MCANX_TXBE_DATA_DB1_Pos)
#define MCANX_TXBE_DATA_DB1(value)    (MCANX_TXBE_DATA_DB1_Msk & ((value) << MCANX_TXBE_DATA_DB1_Pos))
#define MCANX_TXBE_DATA_DB2_Pos       16           /**< \brief (MCANX_TXBE_DATA) Data Byte 2 */
#define MCANX_TXBE_DATA_DB2_Msk       (_U_(0xFF) << MCANX_TXBE_DATA_DB2_Pos)
#define MCANX_TXBE_DATA_DB2(value)    (MCANX_TXBE_DATA_DB2_Msk & ((value) << MCANX_TXBE_DATA_DB2_Pos))
#define MCANX_TXBE_DATA_DB3_Pos       24           /**< \brief (MCANX_TXBE_DATA) Data Byte 3 */
#define MCANX_TXBE_DATA_DB3_Msk       (_U_(0xFF) << MCANX_TXBE_DATA_DB3_Pos)
#define MCANX_TXBE_DATA_DB3(value)    (MCANX_TXBE_DATA_DB3_Msk & ((value) << MCANX_TXBE_DATA_DB3_Pos))
#define MCANX_TXBE_DATA_MASK          _U_(0xFFFFFFFF) /**< \brief (MCANX_TXBE_DATA) MASK Register */

/* -------- MCANX_TXEFE_0 : (CAN Offset: 0x00) (R/W 32) Tx Event FIFO Element 0 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t ID:29;            /*!< bit:  0..28  Identifier                         */
    uint32_t RTR:1;            /*!< bit:     29  Remote Transmission Request        */
    uint32_t XTD:1;            /*!< bit:     30  Extended Indentifier               */
    uint32_t ESI:1;            /*!< bit:     31  Error State Indicator              */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXEFE_0_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXEFE_0_OFFSET          0x00         /**< \brief (MCANX_TXEFE_0 offset) Tx Event FIFO Element 0 */
#define MCANX_TXEFE_0_RESETVALUE      _U_(0x00000000) /**< \brief (MCANX_TXEFE_0 reset_value) Tx Event FIFO Element 0 */

#define MCANX_TXEFE_0_ID_Pos          0            /**< \brief (MCANX_TXEFE_0) Identifier */
#define MCANX_TXEFE_0_ID_Msk          (_U_(0x1FFFFFFF) << MCANX_TXEFE_0_ID_Pos)
#define MCANX_TXEFE_0_ID(value)       (MCANX_TXEFE_0_ID_Msk & ((value) << MCANX_TXEFE_0_ID_Pos))
#define MCANX_TXEFE_0_RTR_Pos         29           /**< \brief (MCANX_TXEFE_0) Remote Transmission Request */
#define MCANX_TXEFE_0_RTR             (_U_(0x1) << MCANX_TXEFE_0_RTR_Pos)
#define MCANX_TXEFE_0_XTD_Pos         30           /**< \brief (MCANX_TXEFE_0) Extended Indentifier */
#define MCANX_TXEFE_0_XTD             (_U_(0x1) << MCANX_TXEFE_0_XTD_Pos)
#define MCANX_TXEFE_0_ESI_Pos         31           /**< \brief (MCANX_TXEFE_0) Error State Indicator */
#define MCANX_TXEFE_0_ESI             (_U_(0x1) << MCANX_TXEFE_0_ESI_Pos)
#define MCANX_TXEFE_0_MASK            _U_(0xFFFFFFFF) /**< \brief (MCANX_TXEFE_0) MASK Register */

/* -------- MCANX_TXEFE_1 : (CAN Offset: 0x04) (R/W 32) Tx Event FIFO Element 1 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t TXTS:16;          /*!< bit:  0..15  Tx Timestamp                       */
    uint32_t DLC:4;            /*!< bit: 16..19  Data Length Code                   */
    uint32_t BRS:1;            /*!< bit:     20  Bit Rate Search                    */
    uint32_t FDF:1;            /*!< bit:     21  FD Format                          */
    uint32_t ET:2;             /*!< bit: 22..23  Event Type                         */
    uint32_t MM:8;             /*!< bit: 24..31  Message Marker                     */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_TXEFE_1_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_TXEFE_1_OFFSET          0x04         /**< \brief (MCANX_TXEFE_1 offset) Tx Event FIFO Element 1 */
#define MCANX_TXEFE_1_RESETVALUE      _U_(0x00000000) /**< \brief (MCANX_TXEFE_1 reset_value) Tx Event FIFO Element 1 */

#define MCANX_TXEFE_1_TXTS_Pos        0            /**< \brief (MCANX_TXEFE_1) Tx Timestamp */
#define MCANX_TXEFE_1_TXTS_Msk        (_U_(0xFFFF) << MCANX_TXEFE_1_TXTS_Pos)
#define MCANX_TXEFE_1_TXTS(value)     (MCANX_TXEFE_1_TXTS_Msk & ((value) << MCANX_TXEFE_1_TXTS_Pos))
#define MCANX_TXEFE_1_DLC_Pos         16           /**< \brief (MCANX_TXEFE_1) Data Length Code */
#define MCANX_TXEFE_1_DLC_Msk         (_U_(0xF) << MCANX_TXEFE_1_DLC_Pos)
#define MCANX_TXEFE_1_DLC(value)      (MCANX_TXEFE_1_DLC_Msk & ((value) << MCANX_TXEFE_1_DLC_Pos))
#define MCANX_TXEFE_1_BRS_Pos         20           /**< \brief (MCANX_TXEFE_1) Bit Rate Search */
#define MCANX_TXEFE_1_BRS             (_U_(0x1) << MCANX_TXEFE_1_BRS_Pos)
#define MCANX_TXEFE_1_FDF_Pos         21           /**< \brief (MCANX_TXEFE_1) FD Format */
#define MCANX_TXEFE_1_FDF             (_U_(0x1) << MCANX_TXEFE_1_FDF_Pos)
#define MCANX_TXEFE_1_ET_Pos          22           /**< \brief (MCANX_TXEFE_1) Event Type */
#define MCANX_TXEFE_1_ET_Msk          (_U_(0x3) << MCANX_TXEFE_1_ET_Pos)
#define MCANX_TXEFE_1_ET(value)       (MCANX_TXEFE_1_ET_Msk & ((value) << MCANX_TXEFE_1_ET_Pos))
#define   MCANX_TXEFE_1_ET_TXE_Val          _U_(0x1)   /**< \brief (MCANX_TXEFE_1) Tx event */
#define   MCANX_TXEFE_1_ET_TXC_Val          _U_(0x2)   /**< \brief (MCANX_TXEFE_1) Transmission in spite of cancellation */
#define MCANX_TXEFE_1_ET_TXE          (MCANX_TXEFE_1_ET_TXE_Val        << MCANX_TXEFE_1_ET_Pos)
#define MCANX_TXEFE_1_ET_TXC          (MCANX_TXEFE_1_ET_TXC_Val        << MCANX_TXEFE_1_ET_Pos)
#define MCANX_TXEFE_1_MM_Pos          24           /**< \brief (MCANX_TXEFE_1) Message Marker */
#define MCANX_TXEFE_1_MM_Msk          (_U_(0xFF) << MCANX_TXEFE_1_MM_Pos)
#define MCANX_TXEFE_1_MM(value)       (MCANX_TXEFE_1_MM_Msk & ((value) << MCANX_TXEFE_1_MM_Pos))
#define MCANX_TXEFE_1_MASK            _U_(0xFFFFFFFF) /**< \brief (MCANX_TXEFE_1) MASK Register */

/* -------- MCANX_XIDFE_0 : (CAN Offset: 0x00) (R/W 32) Extended Message ID Filter Element 0 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t EFID1:29;         /*!< bit:  0..28  Extended Filter ID 1               */
    uint32_t EFEC:3;           /*!< bit: 29..31  Extended Filter Element Configuration */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_XIDFE_0_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_XIDFE_0_OFFSET          0x00         /**< \brief (MCANX_XIDFE_0 offset) Extended Message ID Filter Element 0 */
#define MCANX_XIDFE_0_RESETVALUE      _U_(0x00000000) /**< \brief (MCANX_XIDFE_0 reset_value) Extended Message ID Filter Element 0 */

#define MCANX_XIDFE_0_EFID1_Pos       0            /**< \brief (MCANX_XIDFE_0) Extended Filter ID 1 */
#define MCANX_XIDFE_0_EFID1_Msk       (_U_(0x1FFFFFFF) << MCANX_XIDFE_0_EFID1_Pos)
#define MCANX_XIDFE_0_EFID1(value)    (MCANX_XIDFE_0_EFID1_Msk & ((value) << MCANX_XIDFE_0_EFID1_Pos))
#define MCANX_XIDFE_0_EFEC_Pos        29           /**< \brief (MCANX_XIDFE_0) Extended Filter Element Configuration */
#define MCANX_XIDFE_0_EFEC_Msk        (_U_(0x7) << MCANX_XIDFE_0_EFEC_Pos)
#define MCANX_XIDFE_0_EFEC(value)     (MCANX_XIDFE_0_EFEC_Msk & ((value) << MCANX_XIDFE_0_EFEC_Pos))
#define   MCANX_XIDFE_0_EFEC_DISABLE_Val    _U_(0x0)   /**< \brief (MCANX_XIDFE_0) Disable filter element */
#define   MCANX_XIDFE_0_EFEC_STF0M_Val      _U_(0x1)   /**< \brief (MCANX_XIDFE_0) Store in Rx FIFO 0 if filter match */
#define   MCANX_XIDFE_0_EFEC_STF1M_Val      _U_(0x2)   /**< \brief (MCANX_XIDFE_0) Store in Rx FIFO 1 if filter match */
#define   MCANX_XIDFE_0_EFEC_REJECT_Val     _U_(0x3)   /**< \brief (MCANX_XIDFE_0) Reject ID if filter match */
#define   MCANX_XIDFE_0_EFEC_PRIORITY_Val   _U_(0x4)   /**< \brief (MCANX_XIDFE_0) Set priority if filter match */
#define   MCANX_XIDFE_0_EFEC_PRIF0M_Val     _U_(0x5)   /**< \brief (MCANX_XIDFE_0) Set priority and store in FIFO 0 if filter match */
#define   MCANX_XIDFE_0_EFEC_PRIF1M_Val     _U_(0x6)   /**< \brief (MCANX_XIDFE_0) Set priority and store in FIFO 1 if filter match */
#define   MCANX_XIDFE_0_EFEC_STRXBUF_Val    _U_(0x7)   /**< \brief (MCANX_XIDFE_0) Store into Rx Buffer */
#define MCANX_XIDFE_0_EFEC_DISABLE    (MCANX_XIDFE_0_EFEC_DISABLE_Val  << MCANX_XIDFE_0_EFEC_Pos)
#define MCANX_XIDFE_0_EFEC_STF0M      (MCANX_XIDFE_0_EFEC_STF0M_Val    << MCANX_XIDFE_0_EFEC_Pos)
#define MCANX_XIDFE_0_EFEC_STF1M      (MCANX_XIDFE_0_EFEC_STF1M_Val    << MCANX_XIDFE_0_EFEC_Pos)
#define MCANX_XIDFE_0_EFEC_REJECT     (MCANX_XIDFE_0_EFEC_REJECT_Val   << MCANX_XIDFE_0_EFEC_Pos)
#define MCANX_XIDFE_0_EFEC_PRIORITY   (MCANX_XIDFE_0_EFEC_PRIORITY_Val << MCANX_XIDFE_0_EFEC_Pos)
#define MCANX_XIDFE_0_EFEC_PRIF0M     (MCANX_XIDFE_0_EFEC_PRIF0M_Val   << MCANX_XIDFE_0_EFEC_Pos)
#define MCANX_XIDFE_0_EFEC_PRIF1M     (MCANX_XIDFE_0_EFEC_PRIF1M_Val   << MCANX_XIDFE_0_EFEC_Pos)
#define MCANX_XIDFE_0_EFEC_STRXBUF    (MCANX_XIDFE_0_EFEC_STRXBUF_Val  << MCANX_XIDFE_0_EFEC_Pos)
#define MCANX_XIDFE_0_MASK            _U_(0xFFFFFFFF) /**< \brief (MCANX_XIDFE_0) MASK Register */

/* -------- MCANX_XIDFE_1 : (CAN Offset: 0x04) (R/W 32) Extended Message ID Filter Element 1 -------- */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef union {
  struct {
    uint32_t EFID2:29;         /*!< bit:  0..28  Extended Filter ID 2               */
    uint32_t :1;               /*!< bit:     29  Reserved                           */
    uint32_t EFT:2;            /*!< bit: 30..31  Extended Filter Type               */
  } bit;                       /*!< Structure used for bit  access                  */
  uint32_t reg;                /*!< Type      used for register access              */
} MCANX_XIDFE_1_Type;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define MCANX_XIDFE_1_OFFSET          0x04         /**< \brief (MCANX_XIDFE_1 offset) Extended Message ID Filter Element 1 */
#define MCANX_XIDFE_1_RESETVALUE      _U_(0x00000000) /**< \brief (MCANX_XIDFE_1 reset_value) Extended Message ID Filter Element 1 */

#define MCANX_XIDFE_1_EFID2_Pos       0            /**< \brief (MCANX_XIDFE_1) Extended Filter ID 2 */
#define MCANX_XIDFE_1_EFID2_Msk       (_U_(0x1FFFFFFF) << MCANX_XIDFE_1_EFID2_Pos)
#define MCANX_XIDFE_1_EFID2(value)    (MCANX_XIDFE_1_EFID2_Msk & ((value) << MCANX_XIDFE_1_EFID2_Pos))
#define MCANX_XIDFE_1_EFT_Pos         30           /**< \brief (MCANX_XIDFE_1) Extended Filter Type */
#define MCANX_XIDFE_1_EFT_Msk         (_U_(0x3) << MCANX_XIDFE_1_EFT_Pos)
#define MCANX_XIDFE_1_EFT(value)      (MCANX_XIDFE_1_EFT_Msk & ((value) << MCANX_XIDFE_1_EFT_Pos))
#define   MCANX_XIDFE_1_EFT_RANGEM_Val      _U_(0x0)   /**< \brief (MCANX_XIDFE_1) Range filter from EFID1 to EFID2 */
#define   MCANX_XIDFE_1_EFT_DUAL_Val        _U_(0x1)   /**< \brief (MCANX_XIDFE_1) Dual ID filter for EFID1 or EFID2 */
#define   MCANX_XIDFE_1_EFT_CLASSIC_Val     _U_(0x2)   /**< \brief (MCANX_XIDFE_1) Classic filter */
#define   MCANX_XIDFE_1_EFT_RANGE_Val       _U_(0x3)   /**< \brief (MCANX_XIDFE_1) Range filter from EFID1 to EFID2 with no XIDAM mask */
#define MCANX_XIDFE_1_EFT_RANGEM      (MCANX_XIDFE_1_EFT_RANGEM_Val    << MCANX_XIDFE_1_EFT_Pos)
#define MCANX_XIDFE_1_EFT_DUAL        (MCANX_XIDFE_1_EFT_DUAL_Val      << MCANX_XIDFE_1_EFT_Pos)
#define MCANX_XIDFE_1_EFT_CLASSIC     (MCANX_XIDFE_1_EFT_CLASSIC_Val   << MCANX_XIDFE_1_EFT_Pos)
#define MCANX_XIDFE_1_EFT_RANGE       (MCANX_XIDFE_1_EFT_RANGE_Val     << MCANX_XIDFE_1_EFT_Pos)
#define MCANX_XIDFE_1_MASK            _U_(0xDFFFFFFF) /**< \brief (MCANX_XIDFE_1) MASK Register */

// /** \brief CAN APB hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct MCanX MCanX;
// typedef struct {
//   __I  MCANX_CREL_Type             CREL;        /**< \brief Offset: 0x00 (R/  32) Core Release */
//   __I  MCANX_ENDN_Type             ENDN;        /**< \brief Offset: 0x04 (R/  32) Endian */
//   __IO MCANX_MRCFG_Type            MRCFG;       /**< \brief Offset: 0x08 (R/W 32) Message RAM Configuration */
//   __IO MCANX_DBTP_Type             DBTP;        /**< \brief Offset: 0x0C (R/W 32) Fast Bit Timing and Prescaler */
//   __IO MCANX_TEST_Type             TEST;        /**< \brief Offset: 0x10 (R/W 32) Test */
//   __IO MCANX_RWD_Type              RWD;         /**< \brief Offset: 0x14 (R/W 32) RAM Watchdog */
//   __IO MCANX_CCCR_Type             CCCR;        /**< \brief Offset: 0x18 (R/W 32) CC Control */
//   __IO MCANX_NBTP_Type             NBTP;        /**< \brief Offset: 0x1C (R/W 32) Nominal Bit Timing and Prescaler */
//   __IO MCANX_TSCC_Type             TSCC;        /**< \brief Offset: 0x20 (R/W 32) Timestamp Counter Configuration */
//   __I  MCANX_TSCV_Type             TSCV;        /**< \brief Offset: 0x24 (R/  32) Timestamp Counter Value */
//   __IO MCANX_TOCC_Type             TOCC;        /**< \brief Offset: 0x28 (R/W 32) Timeout Counter Configuration */
//   __IO MCANX_TOCV_Type             TOCV;        /**< \brief Offset: 0x2C (R/W 32) Timeout Counter Value */
//        RoReg8                    Reserved1[0x10];
//   __I  MCANX_ECR_Type              ECR;         /**< \brief Offset: 0x40 (R/  32) Error Counter */
//   __I  MCANX_PSR_Type              PSR;         /**< \brief Offset: 0x44 (R/  32) Protocol Status */
//   __IO MCANX_TDCR_Type             TDCR;        /**< \brief Offset: 0x48 (R/W 32) Extended ID Filter Configuration */
//        RoReg8                    Reserved2[0x4];
//   __IO MCANX_IR_Type               IR;          /**< \brief Offset: 0x50 (R/W 32) Interrupt */
//   __IO MCANX_IE_Type               IE;          /**< \brief Offset: 0x54 (R/W 32) Interrupt Enable */
//   __IO MCANX_ILS_Type              ILS;         /**< \brief Offset: 0x58 (R/W 32) Interrupt Line Select */
//   __IO MCANX_ILE_Type              ILE;         /**< \brief Offset: 0x5C (R/W 32) Interrupt Line Enable */
//        RoReg8                    Reserved3[0x20];
//   __IO MCANX_GFC_Type              GFC;         /**< \brief Offset: 0x80 (R/W 32) Global Filter Configuration */
//   __IO MCANX_SIDFC_Type            SIDFC;       /**< \brief Offset: 0x84 (R/W 32) Standard ID Filter Configuration */
//   __IO MCANX_XIDFC_Type            XIDFC;       /**< \brief Offset: 0x88 (R/W 32) Extended ID Filter Configuration */
//        RoReg8                    Reserved4[0x4];
//   __IO MCANX_XIDAM_Type            XIDAM;       /**< \brief Offset: 0x90 (R/W 32) Extended ID AND Mask */
//   __I  MCANX_HPMS_Type             HPMS;        /**< \brief Offset: 0x94 (R/  32) High Priority Message Status */
//   __IO MCANX_NDAT1_Type            NDAT1;       /**< \brief Offset: 0x98 (R/W 32) New Data 1 */
//   __IO MCANX_NDAT2_Type            NDAT2;       /**< \brief Offset: 0x9C (R/W 32) New Data 2 */
//   __IO MCANX_RXF0C_Type            RXF0C;       /**< \brief Offset: 0xA0 (R/W 32) Rx FIFO 0 Configuration */
//   __I  MCANX_RXF0S_Type            RXF0S;       /**< \brief Offset: 0xA4 (R/  32) Rx FIFO 0 Status */
//   __IO MCANX_RXF0A_Type            RXF0A;       /**< \brief Offset: 0xA8 (R/W 32) Rx FIFO 0 Acknowledge */
//   __IO MCANX_RXBC_Type             RXBC;        /**< \brief Offset: 0xAC (R/W 32) Rx Buffer Configuration */
//   __IO MCANX_RXF1C_Type            RXF1C;       /**< \brief Offset: 0xB0 (R/W 32) Rx FIFO 1 Configuration */
//   __I  MCANX_RXF1S_Type            RXF1S;       /**< \brief Offset: 0xB4 (R/  32) Rx FIFO 1 Status */
//   __IO MCANX_RXF1A_Type            RXF1A;       /**< \brief Offset: 0xB8 (R/W 32) Rx FIFO 1 Acknowledge */
//   __IO MCANX_RXESC_Type            RXESC;       /**< \brief Offset: 0xBC (R/W 32) Rx Buffer / FIFO Element Size Configuration */
//   __IO MCANX_TXBC_Type             TXBC;        /**< \brief Offset: 0xC0 (R/W 32) Tx Buffer Configuration */
//   __I  MCANX_TXFQS_Type            TXFQS;       /**< \brief Offset: 0xC4 (R/  32) Tx FIFO / Queue Status */
//   __IO MCANX_TXESC_Type            TXESC;       /**< \brief Offset: 0xC8 (R/W 32) Tx Buffer Element Size Configuration */
//   __I  MCANX_TXBRP_Type            TXBRP;       /**< \brief Offset: 0xCC (R/  32) Tx Buffer Request Pending */
//   __IO MCANX_TXBAR_Type            TXBAR;       /**< \brief Offset: 0xD0 (R/W 32) Tx Buffer Add Request */
//   __IO MCANX_TXBCR_Type            TXBCR;       /**< \brief Offset: 0xD4 (R/W 32) Tx Buffer Cancellation Request */
//   __I  MCANX_TXBTO_Type            TXBTO;       /**< \brief Offset: 0xD8 (R/  32) Tx Buffer Transmission Occurred */
//   __I  MCANX_TXBCF_Type            TXBCF;       /**< \brief Offset: 0xDC (R/  32) Tx Buffer Cancellation Finished */
//   __IO MCANX_TXBTIE_Type           TXBTIE;      /**< \brief Offset: 0xE0 (R/W 32) Tx Buffer Transmission Interrupt Enable */
//   __IO MCANX_TXBCIE_Type           TXBCIE;      /**< \brief Offset: 0xE4 (R/W 32) Tx Buffer Cancellation Finished Interrupt Enable */
//        RoReg8                    Reserved5[0x8];
//   __IO MCANX_TXEFC_Type            TXEFC;       /**< \brief Offset: 0xF0 (R/W 32) Tx Event FIFO Configuration */
//   __I  MCANX_TXEFS_Type            TXEFS;       /**< \brief Offset: 0xF4 (R/  32) Tx Event FIFO Status */
//   __IO MCANX_TXEFA_Type            TXEFA;       /**< \brief Offset: 0xF8 (R/W 32) Tx Event FIFO Acknowledge */
// } MCanX;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/** \brief CAN Mram_rxbe hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct {
  __IO MCANX_RXBE_0_Type           RXBE_0;      /**< \brief Offset: 0x00 (R/W 32) Rx Buffer Element 0 */
  __IO MCANX_RXBE_1_Type           RXBE_1;      /**< \brief Offset: 0x04 (R/W 32) Rx Buffer Element 1 */
  __IO MCANX_RXBE_DATA_Type        RXBE_DATA[16]; /**< \brief Offset: 0x08 (R/W 32) Rx Buffer Element Data */
} MCanMramRxbeX
#ifdef __GNUC__
  __attribute__ ((aligned (4)))
#endif
;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/** \brief CAN Mram_rxf0e hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct {
  __IO MCANX_RXF0E_0_Type          RXF0E_0;     /**< \brief Offset: 0x00 (R/W 32) Rx FIFO 0 Element 0 */
  __IO MCANX_RXF0E_1_Type          RXF0E_1;     /**< \brief Offset: 0x04 (R/W 32) Rx FIFO 0 Element 1 */
  __IO MCANX_RXF0E_DATA_Type       RXF0E_DATA[16]; /**< \brief Offset: 0x08 (R/W 32) Rx FIFO 0 Element Data */
} MCanMramRxf0eX
#ifdef __GNUC__
  __attribute__ ((aligned (4)))
#endif
;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/** \brief CAN Mram_rxf1e hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct {
  __IO MCANX_RXF1E_0_Type          RXF1E_0;     /**< \brief Offset: 0x00 (R/W 32) Rx FIFO 1 Element 0 */
  __IO MCANX_RXF1E_1_Type          RXF1E_1;     /**< \brief Offset: 0x04 (R/W 32) Rx FIFO 1 Element 1 */
  __IO MCANX_RXF1E_DATA_Type       RXF1E_DATA[16]; /**< \brief Offset: 0x08 (R/W 32) Rx FIFO 1 Element Data */
} MCanMramRxf1eX
#ifdef __GNUC__
  __attribute__ ((aligned (4)))
#endif
;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/** \brief CAN Mram_sidfe hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct {
  __IO MCANX_SIDFE_0_Type          SIDFE_0;     /**< \brief Offset: 0x00 (R/W 32) Standard Message ID Filter Element */
} MCanMramSidfeX
#ifdef __GNUC__
  __attribute__ ((aligned (4)))
#endif
;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/** \brief CAN Mram_txbe hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct {
  __IO MCANX_TXBE_0_Type           TXBE_0;      /**< \brief Offset: 0x00 (R/W 32) Tx Buffer Element 0 */
  __IO MCANX_TXBE_1_Type           TXBE_1;      /**< \brief Offset: 0x04 (R/W 32) Tx Buffer Element 1 */
  __IO MCANX_TXBE_DATA_Type        TXBE_DATA[16]; /**< \brief Offset: 0x08 (R/W 32) Tx Buffer Element Data */
} MCanMramTxbeX
#ifdef __GNUC__
  __attribute__ ((aligned (4)))
#endif
;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/** \brief CAN Mram_txefe hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct {
  __IO MCANX_TXEFE_0_Type          TXEFE_0;     /**< \brief Offset: 0x00 (R/W 32) Tx Event FIFO Element 0 */
  __IO MCANX_TXEFE_1_Type          TXEFE_1;     /**< \brief Offset: 0x04 (R/W 32) Tx Event FIFO Element 1 */
} MCanMramTxefeX
#ifdef __GNUC__
  __attribute__ ((aligned (4)))
#endif
;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/** \brief CAN Mram_xifde hardware registers */
#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct {
  __IO MCANX_XIDFE_0_Type          XIDFE_0;     /**< \brief Offset: 0x00 (R/W 32) Extended Message ID Filter Element 0 */
  __IO MCANX_XIDFE_1_Type          XIDFE_1;     /**< \brief Offset: 0x04 (R/W 32) Extended Message ID Filter Element 1 */
} MCanMramXifdeX
#ifdef __GNUC__
  __attribute__ ((aligned (4)))
#endif
;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#define SECTION_MCANX_MRAM_RXBE

#define SECTION_MCANX_MRAM_RXF0E

#define SECTION_MCANX_MRAM_RXF1E

#define SECTION_MCANX_MRAM_SIDFE

#define SECTION_MCANX_MRAM_TXBE

#define SECTION_MCANX_MRAM_TXEFE

#define SECTION_MCANX_MRAM_XIFDE

