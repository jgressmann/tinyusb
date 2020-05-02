#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t le16_t;
typedef uint16_t be16_t;

/* Microchip command id */
#define MBCA_CMD_RECEIVE_MESSAGE 0xE3
#define MBCA_CMD_I_AM_ALIVE_FROM_CAN 0xF5
#define MBCA_CMD_I_AM_ALIVE_FROM_USB 0xF7
#define MBCA_CMD_CHANGE_BIT_RATE 0xA1
#define MBCA_CMD_TRANSMIT_MESSAGE_EV 0xA3
#define MBCA_CMD_SETUP_TERMINATION_RESISTANCE 0xA8
#define MBCA_CMD_READ_FW_VERSION 0xA9
#define MBCA_CMD_NOTHING_TO_SEND 0xFF
#define MBCA_CMD_TRANSMIT_MESSAGE_RSP 0xE2

#define MCBA_VER_REQ_USB 1
#define MCBA_VER_REQ_CAN 2

#define MCBA_SIDL_EXID_MASK 0x8
#define MCBA_DLC_MASK 0xf
#define MCBA_DLC_RTR_MASK 0x40

#define MCBA_CAN_STATE_WRN_TH 95
#define MCBA_CAN_STATE_ERR_PSV_TH 127

#define MCBA_TERMINATION_DISABLED CAN_TERMINATION_DISABLED
#define MCBA_TERMINATION_ENABLED 120

//#define MCBA_USB_MSG_ATTRS __packed __attribute__((__aligned__(4)))
#define MCBA_USB_MSG_ATTRS __packed

/* CAN frame */
struct MCBA_USB_MSG_ATTRS mcba_usb_msg_can {
	uint8_t cmd_id;
	be16_t eid;
	be16_t sid;
	uint8_t dlc;
	uint8_t data[8];
	uint8_t timestamp[4];
	uint8_t checksum;
};

/* command frame */
struct MCBA_USB_MSG_ATTRS mcba_usb_msg {
	uint8_t cmd_id;
	uint8_t unused[18];
};

struct MCBA_USB_MSG_ATTRS mcba_usb_msg_ka_usb {
	uint8_t cmd_id;
	uint8_t termination_state;
	uint8_t soft_ver_major;
	uint8_t soft_ver_minor;
	uint8_t unused[15];
};

struct MCBA_USB_MSG_ATTRS mcba_usb_msg_ka_can {
	uint8_t cmd_id;
	uint8_t tx_err_cnt;
	uint8_t rx_err_cnt;
	uint8_t rx_buff_ovfl;
	uint8_t tx_bus_off;
	be16_t can_bitrate;
	le16_t rx_lost;
	uint8_t can_stat;
	uint8_t soft_ver_major;
	uint8_t soft_ver_minor;
	uint8_t debug_mode;
	uint8_t test_complete;
	uint8_t test_result;
	uint8_t unused[4];
};

struct MCBA_USB_MSG_ATTRS mcba_usb_msg_change_bitrate {
	uint8_t cmd_id;
	be16_t bitrate;
	uint8_t unused[16];
};

struct MCBA_USB_MSG_ATTRS mcba_usb_msg_termination {
	uint8_t cmd_id;
	uint8_t termination;
	uint8_t unused[17];
};

struct MCBA_USB_MSG_ATTRS mcba_usb_msg_fw_ver {
	uint8_t cmd_id;
	uint8_t pic;
	uint8_t unused[17];
};



#ifdef __cplusplus
} /* extern C */
#endif
