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


#define M_CAN_TS_COUNTER_BITS   16 // mcan_users_manual_v330.pdf p. 14, 60001507E.pdf is wrong

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
#define M_CAN_TDCR_TDCO_MAX          0x7f

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
#if CFG_TUSB_DEBUG > 1
		uint8_t index = can->RXF0S.bit.F0GI;
		TU_LOG2("msg @ %u\n", index);
#endif
		m_can_rx0_pop(can);
	}
}

static inline bool m_can_tx_event_fifo_avail(Can *can)
{
	return (can->TXEFS.reg & CAN_TXEFS_EFFL_Msk) != 0;
}

