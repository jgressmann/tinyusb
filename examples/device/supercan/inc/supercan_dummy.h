/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once



/* dummy board, simulated CAN */
#define SC_BOARD_USB_BCD_DEVICE (HWREV << 8)
#define SC_BOARD_USB_MANUFACTURER_STRING "<unknown>"
#define SC_BOARD_CAN_COUNT 1
#ifdef BOARD_NAME
	#define SC_BOARD_NAME BOARD_NAME
#else
	#define SC_BOARD_NAME "<unknown>"
#endif
#define SC_BOARD_CAN_CLK_HZ 48000000

enum {
	SC_BOARD_DEBUG_DEFAULT,
	SC_BOARD_LED_COUNT
};

enum {
	SC_BOARD_CAN_TX_FIFO_SIZE = 8,
	SC_BOARD_CAN_RX_FIFO_SIZE = 8,
	CAN_FEAT_PERM = SC_FEATURE_FLAG_TXR,
	CAN_FEAT_CONF = (MSG_BUFFER_SIZE >= 128 ? SC_FEATURE_FLAG_FDF : 0),
};

#define sc_board_led_can_status_set(index, on)
#define sc_board_led_usb_burst()
#define sc_board_led_can_traffic_burst(index)
#define sc_board_can_ts_request(index)
#define sc_board_can_ts_wait(index) (board_millis() * 1000U)
