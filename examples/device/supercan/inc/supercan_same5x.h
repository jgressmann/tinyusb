/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2023 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#define SUPERCAN_MCAN 1
#define MCAN_MESSAGE_RAM_CONFIGURABLE 1
#define MCAN_HW_RX_FIFO_SIZE 64
#define MCAN_HW_TX_FIFO_SIZE 32

#define SKIP_INTEGER_LITERALS

#include <supercan_mcan.h>


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



struct MCanX {
  __I  MCANX_CREL_Type             CREL;        /**< \brief Offset: 0x00 (R/  32) Core Release */
  __I  MCANX_ENDN_Type             ENDN;        /**< \brief Offset: 0x04 (R/  32) Endian */
  __IO MCANX_MRCFG_Type            MRCFG;       /**< \brief Offset: 0x08 (R/W 32) Message RAM Configuration */
  __IO MCANX_DBTP_Type             DBTP;        /**< \brief Offset: 0x0C (R/W 32) Fast Bit Timing and Prescaler */
  __IO MCANX_TEST_Type             TEST;        /**< \brief Offset: 0x10 (R/W 32) Test */
  __IO MCANX_RWD_Type              RWD;         /**< \brief Offset: 0x14 (R/W 32) RAM Watchdog */
  __IO MCANX_CCCR_Type             CCCR;        /**< \brief Offset: 0x18 (R/W 32) CC Control */
  __IO MCANX_NBTP_Type             NBTP;        /**< \brief Offset: 0x1C (R/W 32) Nominal Bit Timing and Prescaler */
  __IO MCANX_TSCC_Type             TSCC;        /**< \brief Offset: 0x20 (R/W 32) Timestamp Counter Configuration */
  __I  MCANX_TSCV_Type             TSCV;        /**< \brief Offset: 0x24 (R/  32) Timestamp Counter Value */
  __IO MCANX_TOCC_Type             TOCC;        /**< \brief Offset: 0x28 (R/W 32) Timeout Counter Configuration */
  __IO MCANX_TOCV_Type             TOCV;        /**< \brief Offset: 0x2C (R/W 32) Timeout Counter Value */
       RoReg8                    Reserved1[0x10];
  __I  MCANX_ECR_Type              ECR;         /**< \brief Offset: 0x40 (R/  32) Error Counter */
  __I  MCANX_PSR_Type              PSR;         /**< \brief Offset: 0x44 (R/  32) Protocol Status */
  __IO MCANX_TDCR_Type             TDCR;        /**< \brief Offset: 0x48 (R/W 32) Extended ID Filter Configuration */
       RoReg8                    Reserved2[0x4];
  __IO MCANX_IR_Type               IR;          /**< \brief Offset: 0x50 (R/W 32) Interrupt */
  __IO MCANX_IE_Type               IE;          /**< \brief Offset: 0x54 (R/W 32) Interrupt Enable */
  __IO MCANX_ILS_Type              ILS;         /**< \brief Offset: 0x58 (R/W 32) Interrupt Line Select */
  __IO MCANX_ILE_Type              ILE;         /**< \brief Offset: 0x5C (R/W 32) Interrupt Line Enable */
       RoReg8                    Reserved3[0x20];
  __IO MCANX_GFC_Type              GFC;         /**< \brief Offset: 0x80 (R/W 32) Global Filter Configuration */
  __IO MCANX_SIDFC_Type            SIDFC;       /**< \brief Offset: 0x84 (R/W 32) Standard ID Filter Configuration */
  __IO MCANX_XIDFC_Type            XIDFC;       /**< \brief Offset: 0x88 (R/W 32) Extended ID Filter Configuration */
       RoReg8                    Reserved4[0x4];
  __IO MCANX_XIDAM_Type            XIDAM;       /**< \brief Offset: 0x90 (R/W 32) Extended ID AND Mask */
  __I  MCANX_HPMS_Type             HPMS;        /**< \brief Offset: 0x94 (R/  32) High Priority Message Status */
  __IO MCANX_NDAT1_Type            NDAT1;       /**< \brief Offset: 0x98 (R/W 32) New Data 1 */
  __IO MCANX_NDAT2_Type            NDAT2;       /**< \brief Offset: 0x9C (R/W 32) New Data 2 */
  __IO MCANX_RXF0C_Type            RXF0C;       /**< \brief Offset: 0xA0 (R/W 32) Rx FIFO 0 Configuration */
  __I  MCANX_RXF0S_Type            RXF0S;       /**< \brief Offset: 0xA4 (R/  32) Rx FIFO 0 Status */
  __IO MCANX_RXF0A_Type            RXF0A;       /**< \brief Offset: 0xA8 (R/W 32) Rx FIFO 0 Acknowledge */
  __IO MCANX_RXBC_Type             RXBC;        /**< \brief Offset: 0xAC (R/W 32) Rx Buffer Configuration */
  __IO MCANX_RXF1C_Type            RXF1C;       /**< \brief Offset: 0xB0 (R/W 32) Rx FIFO 1 Configuration */
  __I  MCANX_RXF1S_Type            RXF1S;       /**< \brief Offset: 0xB4 (R/  32) Rx FIFO 1 Status */
  __IO MCANX_RXF1A_Type            RXF1A;       /**< \brief Offset: 0xB8 (R/W 32) Rx FIFO 1 Acknowledge */
  __IO MCANX_RXESC_Type            RXESC;       /**< \brief Offset: 0xBC (R/W 32) Rx Buffer / FIFO Element Size Configuration */
  __IO MCANX_TXBC_Type             TXBC;        /**< \brief Offset: 0xC0 (R/W 32) Tx Buffer Configuration */
  __I  MCANX_TXFQS_Type            TXFQS;       /**< \brief Offset: 0xC4 (R/  32) Tx FIFO / Queue Status */
  __IO MCANX_TXESC_Type            TXESC;       /**< \brief Offset: 0xC8 (R/W 32) Tx Buffer Element Size Configuration */
  __I  MCANX_TXBRP_Type            TXBRP;       /**< \brief Offset: 0xCC (R/  32) Tx Buffer Request Pending */
  __IO MCANX_TXBAR_Type            TXBAR;       /**< \brief Offset: 0xD0 (R/W 32) Tx Buffer Add Request */
  __IO MCANX_TXBCR_Type            TXBCR;       /**< \brief Offset: 0xD4 (R/W 32) Tx Buffer Cancellation Request */
  __I  MCANX_TXBTO_Type            TXBTO;       /**< \brief Offset: 0xD8 (R/  32) Tx Buffer Transmission Occurred */
  __I  MCANX_TXBCF_Type            TXBCF;       /**< \brief Offset: 0xDC (R/  32) Tx Buffer Cancellation Finished */
  __IO MCANX_TXBTIE_Type           TXBTIE;      /**< \brief Offset: 0xE0 (R/W 32) Tx Buffer Transmission Interrupt Enable */
  __IO MCANX_TXBCIE_Type           TXBCIE;      /**< \brief Offset: 0xE4 (R/W 32) Tx Buffer Cancellation Finished Interrupt Enable */
       RoReg8                    Reserved5[0x8];
  __IO MCANX_TXEFC_Type            TXEFC;       /**< \brief Offset: 0xF0 (R/W 32) Tx Event FIFO Configuration */
  __I  MCANX_TXEFS_Type            TXEFS;       /**< \brief Offset: 0xF4 (R/  32) Tx Event FIFO Status */
  __IO MCANX_TXEFA_Type            TXEFA;       /**< \brief Offset: 0xF8 (R/W 32) Tx Event FIFO Acknowledge */
};


#include <m_can.h>


SC_RAMFUNC static inline void sc_board_can_ts_request(uint8_t index)
{
	uint8_t reg;
	(void)index;

	reg = __atomic_load_n(&TC0->COUNT32.CTRLBSET.reg, __ATOMIC_ACQUIRE);

	while (1) {
		uint8_t cmd = reg & TC_CTRLBSET_CMD_Msk;

		SC_DEBUG_ASSERT(cmd == TC_CTRLBSET_CMD_READSYNC || cmd == TC_CTRLBSET_CMD_NONE);
		if (cmd == TC_CTRLBSET_CMD_READSYNC) {
			break;
		}

		if (likely(__atomic_compare_exchange_n(
			&TC0->COUNT32.CTRLBSET.reg,
			&reg,
			TC_CTRLBSET_CMD_READSYNC,
			false, /* weak? */
			__ATOMIC_RELEASE,
			__ATOMIC_ACQUIRE))) {
				break;
			}

	}
}

SC_RAMFUNC extern uint32_t sc_board_can_ts_wait(uint8_t index);

extern void same5x_init_device_identifier(void);
extern void same5x_can_init(void);

static inline void same5x_enable_cache(void)
{
	// DS60001507E-page 83
	if (!CMCC->SR.bit.CSTS) {
		CMCC->CTRL.bit.CEN = 1;
	}
}

#define same5x_counter_1MHz_request_current_value() do { TC0->COUNT32.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_READSYNC_Val; } while (0)
#define same5x_counter_1MHz_is_current_value_ready() ((__atomic_load_n(&TC0->COUNT32.CTRLBSET.reg, __ATOMIC_ACQUIRE) & TC_CTRLBSET_CMD_Msk) == TC_CTRLBSET_CMD_NONE)
#define same5x_counter_1MHz_read_unsafe() (TC0->COUNT32.COUNT.reg & SC_TS_MAX)



#define same5x_counter_1MHz_wait_for_current_value() \
	({ \
		while (!same5x_counter_1MHz_is_current_value_ready()); \
		uint32_t counter = same5x_counter_1MHz_read_unsafe(); \
		counter; \
	})


#define sc_board_can_ts_request(index) same5x_counter_1MHz_request_current_value()
#define sc_board_can_ts_wait(index) same5x_counter_1MHz_wait_for_current_value()