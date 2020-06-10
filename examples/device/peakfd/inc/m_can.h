/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jean Gressmann <jean@0x42.de>
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
 * Code to support Bosch's M_CAN CAN-FD
 */

#include <sam.h>

// #define M_CAN_REG_CREL_OFFSET              (0x00) /**< Core Release */
// #define M_CAN_REG_ENDN_OFFSET              (0x04) /**< Endian */
// #define M_CAN_REG_MRCFG_OFFSET             (0x08) /**< Message RAM Configuration */
// #define M_CAN_REG_DBTP_OFFSET              (0x0C) /**< Fast Bit Timing and Prescaler */
// #define M_CAN_REG_TEST_OFFSET              (0x10) /**< Test */
// #define M_CAN_REG_RWD_OFFSET               (0x14) /**< RAM Watchdog */
// #define M_CAN_REG_CCCR_OFFSET              (0x18) /**< CC Control */
// #define M_CAN_REG_NBTP_OFFSET              (0x1C) /**< Nominal Bit Timing and Prescaler */
// #define M_CAN_REG_TSCC_OFFSET              (0x20) /**< Timestamp Counter Configuration */
// #define M_CAN_REG_TSCV_OFFSET              (0x24) /**< Timestamp Counter Value */
// #define M_CAN_REG_TOCC_OFFSET              (0x28) /**< Timeout Counter Configuration */
// #define M_CAN_REG_TOCV_OFFSET              (0x2C) /**< Timeout Counter Value */
// #define M_CAN_REG_ECR_OFFSET               (0x40) /**< Error Counter */
// #define M_CAN_REG_PSR_OFFSET               (0x44) /**< Protocol Status */
// #define M_CAN_REG_TDCR_OFFSET              (0x48) /**< Extended ID Filter Configuration */
// #define M_CAN_REG_IR_OFFSET                (0x50) /**< Interrupt */
// #define M_CAN_REG_IE_OFFSET                (0x54) /**< Interrupt Enable */
// #define M_CAN_REG_ILS_OFFSET               (0x58) /**< Interrupt Line Select */
// #define M_CAN_REG_ILE_OFFSET               (0x5C) /**< Interrupt Line Enable */
// #define M_CAN_REG_GFC_OFFSET               (0x80) /**< Global Filter Configuration */
// #define M_CAN_REG_SIDFC_OFFSET             (0x84) /**< Standard ID Filter Configuration */
// #define M_CAN_REG_XIDFC_OFFSET             (0x88) /**< Extended ID Filter Configuration */
// #define M_CAN_REG_XIDAM_OFFSET             (0x90) /**< Extended ID AND Mask */
// #define M_CAN_REG_HPMS_OFFSET              (0x94) /**< High Priority Message Status */
// #define M_CAN_REG_NDAT1_OFFSET             (0x98) /**< New Data 1 */
// #define M_CAN_REG_NDAT2_OFFSET             (0x9C) /**< New Data 2 */
// #define M_CAN_REG_RXF0C_OFFSET             (0xA0) /**< Rx FIFO 0 Configuration */
// #define M_CAN_REG_RXF0S_OFFSET             (0xA4) /**< Rx FIFO 0 Status */
// #define M_CAN_REG_RXF0A_OFFSET             (0xA8) /**< Rx FIFO 0 Acknowledge */
// #define M_CAN_REG_RXBC_OFFSET              (0xAC) /**< Rx Buffer Configuration */
// #define M_CAN_REG_RXF1C_OFFSET             (0xB0) /**< Rx FIFO 1 Configuration */
// #define M_CAN_REG_RXF1S_OFFSET             (0xB4) /**< Rx FIFO 1 Status */
// #define M_CAN_REG_RXF1A_OFFSET             (0xB8) /**< Rx FIFO 1 Acknowledge */
// #define M_CAN_REG_RXESC_OFFSET             (0xBC) /**< Rx Buffer / FIFO Element Size Configuration */
// #define M_CAN_REG_TXBC_OFFSET              (0xC0) /**< Tx Buffer Configuration */
// #define M_CAN_REG_TXFQS_OFFSET             (0xC4) /**< Tx FIFO / Queue Status */
// #define M_CAN_REG_TXESC_OFFSET             (0xC8) /**< Tx Buffer Element Size Configuration */
// #define M_CAN_REG_TXBRP_OFFSET             (0xCC) /**< Tx Buffer Request Pending */
// #define M_CAN_REG_TXBAR_OFFSET             (0xD0) /**< Tx Buffer Add Request */
// #define M_CAN_REG_TXBCR_OFFSET             (0xD4) /**< Tx Buffer Cancellation Request */
// #define M_CAN_REG_TXBTO_OFFSET             (0xD8) /**< Tx Buffer Transmission Occurred */
// #define M_CAN_REG_TXBCF_OFFSET             (0xDC) /**< Tx Buffer Cancellation Finished */
// #define M_CAN_REG_TXBTIE_OFFSET            (0xE0) /**< Tx Buffer Transmission Interrupt Enable */
// #define M_CAN_REG_TXBCIE_OFFSET            (0xE4) /**< Tx Buffer Cancellation Finished Interrupt Enable */
// #define M_CAN_REG_TXEFC_OFFSET             (0xF0) /**< Tx Event FIFO Configuration */
// #define M_CAN_REG_TXEFS_OFFSET             (0xF4) /**< Tx Event FIFO Status */
// #define M_CAN_REG_TXEFA_OFFSET             (0xF8) /**< Tx Event FIFO Acknowledge */

#define M_CAN_TS_COUNTER_BITS   15
#define M_CAN_NMBT_TQ_MIN            0x0004
#define M_CAN_NMBT_TQ_MAX            0x0181
#define M_CAN_NMBT_BRP_MIN           0x0001
#define M_CAN_NMBT_BRP_MAX           0x0200
#define M_CAN_NMBT_SJW_MIN           0x0001
#define M_CAN_NMBT_SJW_MAX           0x0080
#define M_CAN_NMBT_TSEG1_MIN         0x0002
#define M_CAN_NMBT_TSEG1_MAX         0x0100
#define M_CAN_NMBT_TSEG2_MIN         0x0002
#define M_CAN_NMBT_TSEG2_MAX         0x0080

#define M_CAN_DTBT_TQ_MIN            0x04
#define M_CAN_DTBT_TQ_MAX            0x31
#define M_CAN_DTBT_BRP_MIN           0x01
#define M_CAN_DTBT_BRP_MAX           0x20
#define M_CAN_DTBT_SJW_MIN           0x01
#define M_CAN_DTBT_SJW_MAX           0x10
#define M_CAN_DTBT_TSEG1_MIN         0x01
#define M_CAN_DTBT_TSEG1_MAX         0x20
#define M_CAN_DTBT_TSEG2_MIN         0x01
#define M_CAN_DTBT_TSEG2_MAX         0x10

static inline void m_can_init_begin(Can *can)
{
    can->CCCR.reg |= CAN_CCCR_INIT; // set CAN-Module to init, reset-default
	while((can->CCCR.reg & CAN_CCCR_INIT) == 0);
}

static inline void m_can_init_end(Can *can)
{
    can->CCCR.reg &= ~CAN_CCCR_INIT; // start CAN-Module
	while(can->CCCR.reg & CAN_CCCR_INIT);
}

static inline void m_can_conf_begin(Can *can)
{
    can->CCCR.reg |= CAN_CCCR_CCE; // set CCE bit to change config
	while((can->CCCR.reg & CAN_CCCR_CCE) == 0);
}

static inline void m_can_conf_end(Can *can)
{
    can->CCCR.reg &= ~CAN_CCCR_CCE; // clear CCE bit
}

static inline uint32_t m_can_compute_nbtp(
	uint8_t sjw,
	uint32_t bitrate_bps,
	uint8_t sample_point_01,
	uint32_t can_clock_hz)
{
	// assert(sjw > 0);
	// assert(bitrate_bps > 0 && bitrate_bps <= 10000000);
	// assert(sample_point_pct > 0 && sample_point_pct < 100);
	// assert(can_clock_hz > 0);

	// The [nominal] CAN bit time may be programed in the range of 4 to 385 time quanta.
	// mcan_users_manual_v330.pdf p13

	// The [data] CAN bit time may be programed in the range of 4 to 49 time quanta.
	// mcan_users_manual_v330.pdf p8



	const uint32_t ticks_per_bit = can_clock_hz / bitrate_bps;
	uint32_t prescaler = ticks_per_bit / 385;
	prescaler += prescaler * 385 != ticks_per_bit;
	const uint32_t tqs = ticks_per_bit / prescaler;
	const uint32_t tseg2 = ((255 - sample_point_01) * tqs) / 255;
	const uint32_t tseg1 = tqs - sjw - tseg2;


	return CAN_NBTP_NSJW(sjw-1)
			| CAN_NBTP_NBRP(prescaler-1)
			| CAN_NBTP_NTSEG1(tseg1-1)
			| CAN_NBTP_NTSEG2(tseg2-1);
}

static inline uint32_t m_can_compute_dbtp(
	uint8_t sjw,
	uint32_t bitrate_bps,
	uint8_t sample_point_01,
	uint32_t can_clock_hz)
{
	// assert(sjw > 0);
	// assert(bitrate_bps > 0 && bitrate_bps <= 10000000);
	// assert(sample_point_pct > 0 && sample_point_pct < 100);
	// assert(can_clock_hz > 0);

	// The [data] CAN bit time may be programed in the range of 4 to 49 time quanta.
	// mcan_users_manual_v330.pdf p8


	////REG_CAN0_DBTP = CAN_DBTP_DBRP(2) | CAN_DBTP_DTSEG1(12) | CAN_DBTP_DTSEG2(5) | CAN_DBTP_DSJW (5); /* 2MBit @ 120 / 3 = 40MHz, 70% */
	const uint32_t ticks_per_bit = can_clock_hz / bitrate_bps;
	uint32_t prescaler = ticks_per_bit / 49;
	prescaler += prescaler * 49 != ticks_per_bit;
	const uint32_t tqs = ticks_per_bit / prescaler;
	const uint32_t tseg2 = ((255 - sample_point_01) * tqs) / 255;
	const uint32_t tseg1 = tqs - sjw - tseg2;


	return CAN_DBTP_DBRP(prescaler-1)
			| CAN_DBTP_DTSEG1(tseg1-1)
			| CAN_DBTP_DTSEG2(tseg2-1)
			| CAN_DBTP_DSJW(sjw-1);
}


static inline bool m_can_rx0_msg_fifo_avail(Can *can)
{
	return (can->RXF0S.reg & CAN_RXF0S_F0FL_Msk) != 0;
}

static inline bool m_can_rx0_msg_fifo_full(Can *can)
{
	return (can->RXF0S.reg & CAN_RXF0S_F0F) != 0;
}

static inline void m_can_rx0_pop(Can *can)
{
	uint8_t index = can->RXF0S.bit.F0GI;
	can->RXF0A.reg = CAN_RXF0A_F0AI(index);
}

static inline void m_can_rx0_clear(Can *can)
{
	while (m_can_rx0_msg_fifo_avail(can)) {
		uint8_t index = can->RXF0S.bit.F0GI;
		TU_LOG2("msg @ %u\n", index);
		m_can_rx0_pop(can);
	}
}

static inline bool m_can_tx_event_fifo_avail(Can *can)
{
	return (can->TXEFS.reg & CAN_TXEFS_EFFL_Msk) != 0;
}

