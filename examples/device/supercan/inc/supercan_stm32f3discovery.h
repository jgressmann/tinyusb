/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2023 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once



#define SC_BOARD_USB_BCD_DEVICE (HWREV << 8)
#define SC_BOARD_USB_MANUFACTURER_STRING "STMicroelectronics"
#define SC_BOARD_CAN_COUNT 1
#define SC_BOARD_NAME "STM32F3DISCOVERY"
#define SC_BOARD_CAN_CLK_HZ 48000000

enum {
	SC_BOARD_DEBUG_DEFAULT,
	LED_USB_TRAFFIC,
	LED_CAN_TRAFFIC,
	LED_CAN_STATUS_GREEN,
	LED_CAN_STATUS_RED,
	SC_BOARD_LED_COUNT
};

enum {
	SC_BOARD_CAN_TX_FIFO_SIZE = 3,
	SC_BOARD_CAN_RX_FIFO_SIZE = 3,
	CAN_FEAT_PERM = SC_FEATURE_FLAG_TXR,
	CAN_FEAT_CONF = SC_FEATURE_FLAG_MON_MODE | SC_FEATURE_FLAG_DAR,
};

#define sc_board_led_usb_burst() led_burst(LED_USB_TRAFFIC, SC_LED_BURST_DURATION_MS)
#define sc_board_led_can_traffic_burst(index) led_burst(LED_CAN_TRAFFIC, SC_LED_BURST_DURATION_MS)
SC_RAMFUNC extern void sc_board_led_can_status_set(uint8_t index, int status);
#define sc_board_can_ts_request(index)
#define sc_board_can_ts_wait(index) (board_millis() * 1000U)
