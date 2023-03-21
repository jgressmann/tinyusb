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