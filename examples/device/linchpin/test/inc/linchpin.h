/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Jean Gressmann <jean@0x42.de>
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
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#define LP_USB_BUFFER_SIZE 16
#define LP_SIGNAL_BUFFER_SIZE 32

#define CONF_CPU_FREQUENCY 200000000

#define LP_LOG(...) fprintf(stdout, __VA_ARGS__)
#define TEST_ASSERT(x) \
    do { \
        if (!(x)) { \
            LP_LOG("%s failed\n", #x); \
            exit(42); \
        } \
    } while (0)

#define LP_ASSERT TEST_ASSERT
#define LP_ISR_ASSERT TEST_ASSERT
#define LP_DEBUG_ASSERT TEST_ASSERT
#define lp_dump_mem(a, b)

#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#define likely(x) x
#define unlikely(x) x
#define LP_RAMFUNC

#include <linchpin_error.h>


#define RLE_ASSERT TEST_ASSERT
#define RLE_H
#define RLE_INT_TYPE size_t
#include <rle.h>
#undef RLE_H

#define BS_ASSERT TEST_ASSERT
#include <bitstream.h>



#define lp_rx_pin_read test_rx_pin_read
#define lp_tx_pin_set test_tx_pin_set
#define lp_tx_pin_clear test_tx_pin_clear
#define lp_timer_stop test_timer_stop
#define lp_timer_start test_timer_start
#define lp_cdc_is_connected test_cdc_is_connected
#define lp_cdc_rx_available test_cdc_rx_available
#define lp_cdc_tx_available test_cdc_tx_available
#define lp_cdc_rx test_cdc_rx
#define lp_cdc_tx test_cdc_tx
#define lp_cdc_tx_flush test_cdc_tx_flush
#define lp_cdc_rx_clear test_cdc_rx_clear
#define lp_cdc_tx_clear test_cdc_tx_clear

#ifdef __cplusplus
extern "C" {
#endif

extern bool test_rx_pin_read(void);
extern void test_tx_pin_set(void);
extern void test_tx_pin_clear(void);
extern void test_timer_stop(void);
extern void test_timer_start(void);
extern bool test_cdc_is_connected(uint8_t itf);
extern uint32_t test_cdc_rx_available(uint8_t itf);
extern uint32_t test_cdc_tx_available(uint8_t itf);
extern uint32_t test_cdc_rx(uint8_t itf, uint8_t *ptr, uint32_t count);
extern uint32_t test_cdc_tx(uint8_t itf, uint8_t const *ptr, uint32_t count);
extern void test_cdc_tx_flush(uint8_t itf);
extern void test_cdc_rx_clear(uint8_t itf);
extern void test_cdc_tx_clear(uint8_t itf);

#include <linchpin_state.h>


#ifdef __cplusplus
}
#endif
;
