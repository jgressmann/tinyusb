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
// #include <slipstream.h>

#define SC_VERSION 1

// #define SC_SLIPSTREAM_MAX_PAYLOAD    SLIPSTREAM_MAX_PAYLOAD
#define SC_MAX_MESSAGE_SIZE 80

#define SC_MSG_ERROR       0
#define SC_MSG_HELLO       1
#define SC_MSG_ECHO        2

#define SC_BYTE_ORDER_LE   0
#define SC_BYTE_ORDER_BE   1

#define SC_ERROR_NONE                   0
#define SC_ERROR_HANDSHAKE_INCOMPLETE   1
#define SC_ERROR_VERSION_MISMATCH       2
#define SC_ERROR_INPUT_TOO       2

struct sc_msg {
    uint8_t id;
};

struct sc_hello_req {
    uint8_t id;
    uint8_t version;
};

struct sc_hello_res {
    uint8_t id;
    uint8_t byte_order;
};

struct sc_msg_error {
    uint8_t id;
    uint8_t version;
    uint8_t code;
};

struct sc_msg_echo {
    uint8_t id;
    uint8_t data[0];
};

struct sc_msg_canfd {
    uint32_t can_id;
    uint8_t dlc : 4;
    uint8_t data[64];
};

