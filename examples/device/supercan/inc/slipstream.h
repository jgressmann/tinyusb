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

#define SLIPSTREAM_VERSION  1
#define SLIPSTREAM_MAX_PAYLOAD 62

typedef uint16_t st_be16_t;

struct slipstream_frame_header {
    uint16_t version    :  2;
    uint16_t seq        :  3;
    uint16_t ack        :  3;
    uint16_t len        :  6;
    uint16_t reserved   :  2;
};

// frames carry messages, n-m relation
struct slipstream_frame {
    struct slipstream_frame_header header;
    uint8_t data[SLIPSTREAM_MAX_PAYLOAD];
};

// message header
struct slipstream_message {
    uint8_t len;
    uint8_t data[0];
};

enum {
    static_assert_slipstream_msg_header_size_is_2 = sizeof(int[sizeof(struct slipstream_frame_header) == 2 ? 1 : -1]),
    static_assert_slipstream_msg_size_is_64 = sizeof(int[sizeof(struct slipstream_frame) == 64 ? 1 : -1])
};