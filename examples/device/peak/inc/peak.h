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

#include <stdint.h>


struct peak_cmd {
    uint8_t func;
    uint8_t num;
    uint8_t argv[14];
};

#define PEAK_REPLY_PACK_RECORD_COUNTER_OFFSET 1

#define PEAK_USB_F  16000000LU
#define PEAK_USB_CMD_TIMEOUT_MS 1000
#define PEAK_USB_REPLY_PACK_INIT_SIZE   2
#define PEAK_USB_REPLY_PACK_MIN_CAN_MSG_SIZE    3   // sff, dlc=0
#define PEAK_USB_REPLY_PACK_MAX_CAN_MSG_SIZE    13  // eff, dlc=8

#define PEAK_USB_EP_SIZE    64
#define PEAK_USB_ID_VENDOR  0x0c72
#define PEAK_USB_ID_PRODUCT 0x000c
#define PEAK_USB_EP_BULK_OUT_CMD    0x01
#define PEAK_USB_EP_BULK_IN_CMD     0x81
#define PEAK_USB_EP_BULK_OUT_MSG    0x02
#define PEAK_USB_EP_BULK_IN_MSG     0x82

#define PCAN_USB_STATUSLEN_TIMESTAMP	(1 << 7)
#define PCAN_USB_STATUSLEN_INTERNAL	(1 << 6)
#define PCAN_USB_STATUSLEN_EXT_ID	(1 << 5)
#define PCAN_USB_STATUSLEN_RTR		(1 << 4)
#define PCAN_USB_STATUSLEN_DLC		(0xf)

#define PCAN_USB_REC_ERROR		1
#define PCAN_USB_REC_ANALOG		2
#define PCAN_USB_REC_BUSLOAD		3
#define PCAN_USB_REC_TS			4
#define PCAN_USB_REC_BUSEVT		5


/* PCAN-USB error flags */
#define PCAN_USB_ERROR_TXFULL		0x01
#define PCAN_USB_ERROR_RXQOVR		0x02
#define PCAN_USB_ERROR_BUS_LIGHT	0x04
#define PCAN_USB_ERROR_BUS_HEAVY	0x08
#define PCAN_USB_ERROR_BUS_OFF		0x10
#define PCAN_USB_ERROR_RXQEMPTY		0x20
#define PCAN_USB_ERROR_QOVR		0x40
#define PCAN_USB_ERROR_TXQFULL		0x80

