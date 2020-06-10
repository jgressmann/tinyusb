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
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define SC_NAME "SuperCAN"
#define SC_VERSION          1

#define SC_HEADER_LEN           2
#define SC_HEADER_ID_OFFSET     0
#define SC_HEADER_LEN_OFFSET    1

#define SC_MSG_EOF              0x00
#define SC_MSG_HELLO_DEVICE     0x01
#define SC_MSG_HELLO_HOST       0x02
#define SC_MSG_DEVICE_INFO      0x03
#define SC_MSG_RESET            0x04

#define SC_MSG_BITTIMING        0x06
#define SC_MSG_MODE             0x07
#define SC_MSG_BUS              0x08


#define SC_MSG_CAN_STATUS       0x10
#define SC_MSG_CAN_RX           0x11
#define SC_MSG_CAN_TX           0x12
#define SC_MSG_CAN_TXR          0x13



#define SC_BYTE_ORDER_LE        0
#define SC_BYTE_ORDER_BE        1

#define SC_FEATURE_FLAG_CAN_FD       0x01 // device supports CAN-FD
#define SC_FEATURE_FLAG_AUTO_RE      0x02 // device supports automatic retransmission
#define SC_FEATURE_FLAG_EH           0x04 // device supports automatic retransmission

#define SC_CAN_FLAG_EXT         0x01 // extended (29 bit id) frame
#define SC_CAN_FLAG_RTR         0x02 // remote request frame
#define SC_CAN_FLAG_FDF         0x04 // CAN-FD frame
#define SC_CAN_FLAG_BRS         0x08 // CAN-FD bitrate switching (set zero to transmit at arbitration rate)
#define SC_CAN_FLAG_ESI         0x10 // set to 1 to transmit with active error state
#define SC_CAN_FLAG_DRP         0x20 // CAN frame was dropped b/c the tx fifo was full

#define SC_STATUS_FLAG_BUS_OFF       0x01
#define SC_STATUS_FLAG_ERROR_WARNING 0x02
#define SC_STATUS_FLAG_ERROR_PASSIVE 0x04
#define SC_STATUS_FLAG_RX_FULL       0x08 // rx queue is full, can -> usb messages lost
#define SC_STATUS_FLAG_TX_FULL       0x10 // tx queue is full, usb -> can messages lost
#define SC_STATUS_FLAG_TX_DESYNC     0x20 // no USB buffer space to queue TXR message



#define SC_MODE_FLAG_RX         0x00 // enable reception
#define SC_MODE_FLAG_TX         0x01 // enable transmission
#define SC_MODE_FLAG_CAN_FD     0x02 // enable CAN-FD
#define SC_MODE_FLAG_BRS        0x04 // enable CAN-FD bitrate switching
#define SC_MODE_FLAG_AUTO_RE    0x08 // enable automatic retransmission (tx)
#define SC_MODE_FLAG_EH         0x10 // enable protocol exception handling (error frames)

// This is the only message that uses
// non-device byte order.
struct sc_msg_hello {
    uint8_t proto_version;
    uint8_t byte_order;
    uint16_t buffer_size; // always in network byte order
} SC_PACKED;

struct sc_msg_config {
    uint8_t channel;
    uint8_t args[1];
} SC_PACKED;

struct sc_msg_info {
    uint8_t channels;
    uint8_t features;
    uint32_t can_clk_hz;
    uint16_t nmbt_pre_max;
    uint16_t nmbt_pre_min;
    uint16_t nmbt_tq_max;
    uint8_t nmbt_tq_min;
    uint8_t nmbt_seg1_min;
    uint16_t nmbt_seg1_max;
    uint8_t nmbt_sjw_min;
    uint8_t nmbt_sjw_max;
    uint8_t nmbt_seg2_min;
    uint8_t nmbt_seg2_max;
    uint8_t dtbt_pre_max;
    uint8_t dtbt_pre_min;
    uint8_t dtbt_tq_max;
    uint8_t dtbt_tq_min;
    uint8_t dtbt_seg1_min;
    uint8_t dtbt_seg1_max;
    uint8_t dtbt_sjw_min;
    uint8_t dtbt_sjw_max;
    uint8_t dtbt_seg2_min;
    uint8_t dtbt_seg2_max;
} SC_PACKED;

struct sc_msg_status {
    uint8_t channel;
    uint8_t flags;
    uint32_t timestamp;     // CAN bittime
    uint16_t rx_lost;       // messages can->usb lost since last time b/c of full rx fifo
    uint16_t tx_dropped;    // messages usb->can dropped since last time b/c of full tx fifo
} SC_PACKED;

struct sc_msg_bittiming {
    uint8_t channel;
    uint8_t nmbt_sjw;
    uint16_t nmbt_pre;
    uint16_t nmbt_seg1;
    uint8_t nmbt_seg2;
    uint8_t dtbt_pre;
    uint8_t dtbt_sjw;
    uint8_t dtbt_seg1;
    uint8_t dtbt_seg2;
    // uint8_t arbitration_sjw;            // CAN / CAN-FD
    // uint32_t arbitration_bitrate_bps;   // CAN / CAN-FD
    // uint32_t data_bitrate_bps;          // CAN-FD
    // uint8_t data_sjw;                   // CAN-FD
    // uint8_t arbitration_sample_point;   // normlized to [0-1] * 255
    // uint8_t data_sample_point;          // normlized to [0-1] * 255

} SC_PACKED;

struct sc_msg_can_rx {
    uint8_t channel;
    uint8_t dlc;
    uint32_t can_id;
    uint32_t timestamp;
    uint8_t flags;
    uint8_t data[0];
} SC_PACKED;

struct sc_msg_can_tx {
    uint8_t channel;
    uint8_t dlc;
    uint32_t can_id;
    uint16_t track_id;
    uint8_t flags;
    uint8_t data[0];
} SC_PACKED;

struct sc_msg_can_txr {
    uint8_t channel;
    uint8_t flags;
    uint32_t timestamp;
    uint16_t track_id;
} SC_PACKED;

struct sc_msg {
    uint8_t id;
    uint8_t len;
    union {
        struct sc_msg_hello hello;
        struct sc_msg_info info;
        struct sc_msg_bittiming bittiming;
        struct sc_msg_can_rx rx;
        struct sc_msg_can_tx tx;
        struct sc_msg_can_txr txr;
        struct sc_msg_status status;
        struct sc_msg_config config;
    } u;
} SC_PACKED;

enum {
    static_assert_sc_msg_is_not_larger_than_64_bytes = sizeof(int[sizeof(struct sc_msg) <= 64 ? 1 : -1]),
};

#ifdef __cplusplus
} // extern "C"
#endif
