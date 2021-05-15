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

#include <linchpin.h>

#include <stdlib.h>
#include <usnprintf.h>
#include <inttypes.h>

#ifndef SAME54XPLAINEDPRO
    #define SAME54XPLAINEDPRO 0
#endif


LP_RAMFUNC static void lp_next_bit_relaxed(void);
LP_RAMFUNC static void lp_next_bit_strict(void);

lp_timer_callback_t lp_timer_callback;
struct linchpin lp;

static inline void lp_lin_clear_rx_tx(void)
{
    lp.lin.signal_tx_buffer_pi = 0;
    lp.lin.signal_tx_buffer_gi = 0;
    lp.lin.signal_rx_buffer_pi = 0;
    lp.lin.signal_rx_buffer_gi = 0;
}

static inline void lp_lin_reset(void)
{
    lp_timer_stop();

    rlew_dec_init(&lp.lin.dec);
    rlew_enc_init(&lp.lin.enc);
    lp_lin_clear_rx_tx();
    lp.lin.signal_flags = 0;
    lp.lin.mode = LP_LIN_MODE_OFF;

    __atomic_thread_fence(__ATOMIC_RELEASE);
}

void lp_init(void)
{
    memset(&lp, 0, sizeof(lp));
    lp.lin.signal_frequency = 19200*16;
    lp_lin_reset();
    lp_timer_callback = &lp_next_bit_relaxed;
}




LP_RAMFUNC static bool usb_try_rx2(
    uint8_t itf,
    volatile size_t *gi_ptr,
    volatile size_t *pi_ptr,
    void *buf_ptr,
    size_t element_size,
    size_t buf_size)
{
    size_t gi = __atomic_load_n(gi_ptr, __ATOMIC_ACQUIRE);
    size_t pi = *pi_ptr;
    size_t used = pi - gi;

    if (used < buf_size && lp_cdc_rx_available(itf)) {
        size_t count = buf_size - used;
        size_t index = pi % buf_size;

        if (index + count > buf_size) {
            count = buf_size - index;
        }

        uint32_t bytes = lp_cdc_rx_available(itf);
        uint32_t elements = bytes / element_size;
        if (elements < count) {
            count = elements;
        }

        if (count) {
            (void)lp_cdc_rx(itf, ((uint8_t*)buf_ptr) + index * element_size, count * element_size);

            // LP_LOG("read: ");
            // for (uint8_t i = offset; i < lp.cmd.usb_rx_count; ++i) {
            // 	LP_LOG("%c %#x", lp.cmd.usb_rx_buffer[i], lp.cmd.usb_rx_buffer[i]);
            // }
            // LP_LOG("\n");
            __atomic_store_n(pi_ptr, pi + count, __ATOMIC_RELEASE);
            return true;
        }


//        uint32_t r = lp_cdc_rx(itf, ((uint8_t*)buf_ptr) + index * element_size, count * element_size) / element_size;
//        if (r) {

//        }
    }

    return false;
}


LP_RAMFUNC static bool usb_try_tx2(
    uint8_t itf,
    volatile size_t *gi_ptr,
    volatile size_t *pi_ptr,
    void *buf_ptr,
    size_t element_size,
    size_t buf_size)
{
    size_t gi = *gi_ptr;
    size_t pi = __atomic_load_n(pi_ptr, __ATOMIC_ACQUIRE);
    size_t count = pi - gi;

    if (count) {
        size_t index = gi % buf_size;
        if (index + count > buf_size) {
            count = buf_size - index;
        }

        uint32_t bytes = lp_cdc_tx_available(itf);
        uint32_t elements = bytes / element_size;
        if (elements < count) {
            count = elements;
        }

        if (count) {
            (void)lp_cdc_tx(itf, ((uint8_t*)buf_ptr) + index * element_size, count * element_size);
            __atomic_store_n(gi_ptr, gi + count, __ATOMIC_RELEASE);
            return true;
        }
    }

    return false;
}

static inline void cmd_clear_rx_tx(void)
{
    lp.cmd.usb_rx_buffer_gi = 0;
    lp.cmd.usb_rx_buffer_pi = 0;
    lp.cmd.usb_tx_buffer_gi = 0;
    lp.cmd.usb_tx_buffer_pi = 0;
}

static inline void place_error_response(int code)
{
    lp.cmd.usb_tx_buffer_gi = 0;
    lp.cmd.usb_tx_buffer_pi = usnprintf((char*)lp.cmd.usb_tx_buffer, sizeof(lp.cmd.usb_tx_buffer), "%d %s\n", code, lp_strerror(code));
}

static inline void place_unknown_cmd_response(void)
{
    place_error_response(LP_ERROR_UNKNOWN_CMD);
}


static void place_signal_flags(void)
{
    lp.cmd.usb_tx_buffer_gi = 0;
    lp.cmd.usb_tx_buffer_pi = usnprintf((char*)lp.cmd.usb_tx_buffer, sizeof(lp.cmd.usb_tx_buffer), "%d %" PRIu32 "\n", LP_ERROR_NONE, (uint32_t)LP_SIGNAL_BUFFER_SIZE);


    char* ptr = (char*)lp.cmd.usb_tx_buffer;
    size_t left = sizeof(lp.cmd.usb_tx_buffer);
    int chars = usnprintf(ptr, left, "0 ");
    LP_DEBUG_ASSERT(chars > 0);
    LP_DEBUG_ASSERT((size_t)chars <= left);
    ptr += chars;
    left -= chars;



    if (lp.lin.signal_flags & OUTPUT_FLAG_RX_OVERFLOW) {
        chars = usnprintf(ptr, left, "SIGRXOVRFL ");
        LP_DEBUG_ASSERT(chars > 0);
        LP_DEBUG_ASSERT((size_t)chars <= left);
        ptr += chars;
        left -= chars;
    }

    if (lp.lin.signal_flags & OUTPUT_FLAG_TX_STALLED) {
        chars = usnprintf(ptr, left, "SIGTXSTALL ");
        LP_DEBUG_ASSERT(chars > 0);
        LP_DEBUG_ASSERT((size_t)chars <= left);
        ptr += chars;
        left -= chars;
    }

    if (lp.lin.signal_flags & OUTPUT_FLAG_INPUT_BAD) {
        chars = usnprintf(ptr, left, "SIGBAD ");
        LP_DEBUG_ASSERT(chars > 0);
        LP_DEBUG_ASSERT((size_t)chars <= left);
        ptr += chars;
        left -= chars;
    }

    if (lp.lin.signal_flags & OUTPUT_FLAG_ERROR) {
        chars = usnprintf(ptr, left, "SIGERROR ");
        LP_DEBUG_ASSERT(chars > 0);
        LP_DEBUG_ASSERT((size_t)chars <= left);
        ptr += chars;
        left -= chars;
    }

    chars = usnprintf(ptr, left, "\n");
    LP_DEBUG_ASSERT(chars == 1);
    LP_DEBUG_ASSERT((size_t)chars <= left);
    ptr += chars;
    left -= chars;

    lp.cmd.usb_tx_buffer_pi = sizeof(lp.cmd.usb_tx_buffer) - left;
}


static void process_cmd(uint8_t *ptr, uint8_t count)
{
    if (count == 0) {
        return;
    }

    bool is_write = *ptr == '!';
    bool is_read = *ptr == '?';

    if (is_write) {
        if (count > 1)  {
            switch (ptr[1]) {
            case 'f':
            case 'F': {
                if (count > 3)  {
                    char* start = (char*)&ptr[3];
                    char* end = NULL;
                    uint32_t f = strtoul(start, &end, 10);
                    int error = LP_ERROR_NONE;
                    if (end == NULL || end == start) {
//                        LP_LOG("'%s' not integer\n", ptr);
                        error = LP_ERROR_INVALID_PARAM;
                    } else {
                        if (f) {
                            if (f > CONF_CPU_FREQUENCY / 16) {
//                                LP_LOG("%" PRIu32 " > %" PRIu32 " / 16\n", f, CONF_CPU_FREQUENCY);
                                error = LP_ERROR_OUT_OF_RANGE;
                            } else {
//                                LP_LOG("set signal frequency to %" PRIu32 " [Hz]\n", f);
                                lp.lin.signal_frequency = f;
                            }
                        } else {
                            error = LP_ERROR_OUT_OF_RANGE;
                        }
                    }
                    place_error_response(error);
                } else {
                    place_error_response(LP_ERROR_MALFORMED);
                }
            } break;
            case 'l':
            case 'L': {
                if (count >= 4)  {
                    lp_lin_reset();

                    switch (ptr[3]) {
                    case 'o':
                    case 'O':
                        LP_LOG("LIN mode off\n");
                        place_error_response(LP_ERROR_NONE);
                        break;
                    case 'r':
                    case 'R':
                        LP_LOG("LIN mode relaxed\n");
                        lp_timer_callback = &lp_next_bit_relaxed;
                        lp.lin.mode = LP_LIN_MODE_RELAXED;
                        __atomic_thread_fence(__ATOMIC_RELEASE);
                        place_error_response(LP_ERROR_NONE);
                        lp_timer_start();
                        LP_LOG("LIN timer started\n");
                        break;
                    case 's':
                    case 'S':
                        LP_LOG("LIN mode strict\n");
                        lp_timer_callback = &lp_next_bit_strict;
                        lp.lin.mode = LP_LIN_MODE_STRICT;
                        __atomic_thread_fence(__ATOMIC_RELEASE);
                        place_error_response(LP_ERROR_NONE);
                        lp_timer_start();
                        LP_LOG("LIN timer started\n");
                        break;
                    default:
                        place_error_response(LP_ERROR_MALFORMED);
                        break;
                    }
                } else {
                    place_error_response(LP_ERROR_MALFORMED);
                }
            } break;
            case 'p':
            case 'P': {
                if (count > 3)  {
                    char* start = (char*)&ptr[3];
                    char* end = NULL;
                    int error = LP_ERROR_INVALID_PARAM;
                    uint32_t pin;
                    bool value;
                    pin = (uint32_t)strtoul(start, &end, 10);
                    if (end != NULL && end != start) {
                        start = end;
                        end = NULL;
                        value = strtoul(start, &end, 10) != 0;
                        if (end != NULL && end != start) {
//                                LP_LOG("set pin=%#" PRIx32 " value=%u\n", pin, value);
                            if (lp_pin_set(pin, value)) {
                                error = LP_ERROR_NONE;
                            }
                        }
                    }

                    place_error_response(error);
                } else {
                    place_error_response(LP_ERROR_MALFORMED);
                }
            } break;
            default:
                place_unknown_cmd_response();
                break;
            }
        } else {
            place_unknown_cmd_response();
        }
    } else if (is_read) {
        if (count > 1)  {
            switch (ptr[1]) {
            case 'e':
            case 'E':
                lp.cmd.usb_tx_buffer_gi = 0;
                lp.cmd.usb_tx_buffer_pi = usnprintf((char*)lp.cmd.usb_tx_buffer, sizeof(lp.cmd.usb_tx_buffer), "%d %s\n", LP_ERROR_NONE,
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
                                                    "LE"
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
                                                    "BE"
#else
    #error Unsupported byte order
#endif
                                                    );
                break;
            case 'f':
            case 'F':
                lp.cmd.usb_tx_buffer_gi = 0;
                lp.cmd.usb_tx_buffer_pi = usnprintf((char*)lp.cmd.usb_tx_buffer, sizeof(lp.cmd.usb_tx_buffer), "%d %" PRIu32 "\n", LP_ERROR_NONE, lp.lin.signal_frequency);
                break;
            case 'l':
            case 'L': {
                place_signal_flags();
            } break;
            case 'v':
            case 'V': {
                char buf[LP_CMD_BUFFER_SIZE];
                memset(buf, 0, sizeof(buf));
                lp_version(buf, sizeof(buf)-1);
                lp.cmd.usb_tx_buffer_gi = 0;
                lp.cmd.usb_tx_buffer_pi = usnprintf((char*)lp.cmd.usb_tx_buffer, sizeof(lp.cmd.usb_tx_buffer), "0 %s\n", buf);
            } break;
            default:
                place_unknown_cmd_response();
                break;
            }
        } else {
            place_unknown_cmd_response();
        }
    } else {
        place_unknown_cmd_response();
    }
}



void lp_cdc_cmd_task(void)
{
    const uint8_t index = 0;

    for (bool more = true; more; ) {
        more = false;
        bool connected = lp_cdc_is_connected(index);

        switch (lp.cmd.state) {
        case LP_CMD_DISCONNECTED:
            if (connected) {
                lp.cmd.state = LP_CMD_CONNECTED;
                more = true;
            }
            break;
        case LP_CMD_CONNECTED: {
            if (connected) {

                // if there is data waiting in the tx buffer, don't process commands
                bool usb_tx = usb_try_tx2(
                    index,
                    &lp.cmd.usb_tx_buffer_gi,
                    &lp.cmd.usb_tx_buffer_pi,
                    lp.cmd.usb_tx_buffer,
                    1,
                    ARRAY_SIZE(lp.cmd.usb_tx_buffer));

                more = more || usb_tx;
                if (!usb_tx) {
                    bool read_from_usb = usb_try_rx2(
                        index,
                        &lp.cmd.usb_rx_buffer_gi,
                        &lp.cmd.usb_rx_buffer_pi,
                        lp.cmd.usb_rx_buffer,
                        1,
                        ARRAY_SIZE(lp.cmd.usb_rx_buffer));

                    more = more || read_from_usb;

                    while (lp.cmd.usb_rx_buffer_gi != lp.cmd.usb_rx_buffer_pi) {
                        char c = lp.cmd.usb_rx_buffer[lp.cmd.usb_rx_buffer_gi];

                        switch (c) {
                        case '\n':
                        case '\r': {
                            LP_LOG("process cmd\n");
                            lp.cmd.usb_rx_buffer[lp.cmd.usb_rx_buffer_gi + 1] = 0;
                            process_cmd(lp.cmd.usb_rx_buffer, lp.cmd.usb_rx_buffer_gi);
                            lp.cmd.usb_rx_buffer_gi = 0;
                            lp.cmd.usb_rx_buffer_pi = 0;

                            usb_try_tx2(
                                index,
                                &lp.cmd.usb_tx_buffer_gi,
                                &lp.cmd.usb_tx_buffer_pi,
                                lp.cmd.usb_tx_buffer,
                                1,
                                ARRAY_SIZE(lp.cmd.usb_tx_buffer));
                            lp_cdc_tx_flush(index);
                        } break;
                        default:

                            if (lp.cmd.usb_rx_buffer_pi + 1 == ARRAY_SIZE(lp.cmd.usb_rx_buffer)) {
                                LP_LOG("cmd overflow\n");
                                lp.cmd.usb_rx_buffer_gi = 0;
                                lp.cmd.usb_rx_buffer_pi = 0;
                                place_unknown_cmd_response();
                                usb_try_tx2(
                                    index,
                                    &lp.cmd.usb_tx_buffer_gi,
                                    &lp.cmd.usb_tx_buffer_pi,
                                    lp.cmd.usb_tx_buffer,
                                    1,
                                    ARRAY_SIZE(lp.cmd.usb_tx_buffer));
                                lp_cdc_tx_flush(index);
                            } else {
                                LP_LOG("cmd input char=%c hex=%x\n", c, c);
                                ++lp.cmd.usb_rx_buffer_gi;
                            }
                            break;
                        }

                        more = true;
                    }
                }

            } else {
                lp.cmd.state = LP_CMD_DISCONNECTED;
                cmd_clear_rx_tx();
                lp_cdc_rx_clear(index);
                lp_cdc_tx_clear(index);
                more = true;
            }
        } break;
        default:
            LP_LOG("unhandled LP_CMD_ state %d\n", lp.cmd.state);
            break;
        }
    }

    lp_delay_ms(1);
}



LP_RAMFUNC void lp_cdc_lin_task(void)
{
    const uint8_t index = 1;

    for (bool more = true; more; ) {
        more = false;

        bool connected = lp_cdc_is_connected(index);

        switch (lp.lin.state) {
        case LP_LIN_DISCONNECTED:
            if (connected) {
                lp.lin.state = LP_LIN_CONNECTED;
                more = true;
            }
            break;
        case LP_LIN_CONNECTED: {
            if (connected) {
                bool read_from_usb = usb_try_rx2(
                    index,
                    &lp.lin.signal_tx_buffer_gi,
                    &lp.lin.signal_tx_buffer_pi,
                    lp.lin.signal_tx_buffer,
                    4,
                    ARRAY_SIZE(lp.lin.signal_tx_buffer));

                more = more || read_from_usb;

                bool wrote_to_usb = usb_try_tx2(
                    index,
                    &lp.lin.signal_rx_buffer_gi,
                    &lp.lin.signal_rx_buffer_pi,
                    lp.lin.signal_rx_buffer,
                    4,
                    ARRAY_SIZE(lp.lin.signal_rx_buffer));

                more = more || wrote_to_usb;
            } else {
                lp_cdc_rx_clear(index);
                lp_cdc_tx_clear(index);
                more = true;
            }
        } break;
        default:
            LP_LOG("unhandled LP_LIN_ state %d\n", lp.lin.state);
            break;
        }
    }

    lp_delay_ms(1);
}


LP_RAMFUNC static int rlew_store(void* ctx, uint32_t value)
{
    (void)ctx;

    LP_LOG("rlew store\n");

    size_t gi = __atomic_load_n(&lp.lin.signal_rx_buffer_gi, __ATOMIC_ACQUIRE);
    size_t pi = lp.lin.signal_rx_buffer_pi;
    size_t count = pi - gi;
    LP_ISR_ASSERT(count <= ARRAY_SIZE(lp.lin.signal_rx_buffer));

    if (likely(count < ARRAY_SIZE(lp.lin.signal_rx_buffer))) {
        lp.lin.signal_rx_buffer[pi % ARRAY_SIZE(lp.lin.signal_rx_buffer)] = value;
        __atomic_store_n(&lp.lin.signal_rx_buffer_pi, pi + 1, __ATOMIC_RELEASE);
        return 0;
    }

    return 1;
}

LP_RAMFUNC static int rlew_load(void* ctx, uint32_t *value)
{
    (void)ctx;



    size_t gi = lp.lin.signal_tx_buffer_gi;
    size_t pi = __atomic_load_n(&lp.lin.signal_tx_buffer_pi, __ATOMIC_ACQUIRE);

    if (likely(pi != gi)) {
        // LP_LOG("rlew load\n");
        *value = lp.lin.signal_tx_buffer[gi % ARRAY_SIZE(lp.lin.signal_tx_buffer)];
        __atomic_store_n(&lp.lin.signal_tx_buffer_gi, gi + 1, __ATOMIC_RELEASE);
        return 0;
    }

    return 1;
}

// LP_RAMFUNC void lp_rle_task(void)
// {
//     for (;;) {
//         // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
// 			// vTaskNotifyGiveFromISR(can->usb_task_handle, &xHigherPriorityTaskWoken);

//         (void)ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

//     }
// }


LP_RAMFUNC static void lp_signal_store_error(unsigned lin_enc_flags)
{
    lp_timer_stop();

    uint8_t flags = OUTPUT_FLAG_OUTPUT_DONE;

    if (lin_enc_flags & (RLEW_FLAG_ENC_OVERFLOW)) {
        flags |= OUTPUT_FLAG_RX_OVERFLOW;
    } else {
        flags |= OUTPUT_FLAG_ERROR;
    }

    __atomic_or_fetch(&lp.lin.signal_flags, flags, __ATOMIC_ACQ_REL);
}

LP_RAMFUNC static void lp_signal_load_error(unsigned lin_dec_flags)
{
    lp_timer_stop();

    if (lin_dec_flags & RLEW_FLAG_DEC_EOS) {
        rlew_enc_finish(&lp.lin.enc, NULL, &rlew_store);
        if (unlikely(lp.lin.enc.flags)) {
            lp_signal_store_error(lp.lin.enc.flags);
        } else {
            __atomic_or_fetch(&lp.lin.signal_flags, OUTPUT_FLAG_OUTPUT_DONE, __ATOMIC_ACQ_REL);
        }
    } else if (lin_dec_flags & RLEW_FLAG_DEC_UNDERFLOW) {
        __atomic_or_fetch(&lp.lin.signal_flags, OUTPUT_FLAG_OUTPUT_DONE | OUTPUT_FLAG_TX_STALLED, __ATOMIC_ACQ_REL);
    } else {
        __atomic_or_fetch(&lp.lin.signal_flags, OUTPUT_FLAG_OUTPUT_DONE | OUTPUT_FLAG_ERROR, __ATOMIC_ACQ_REL);
    }
}



LP_RAMFUNC static void lp_next_bit_strict(void)
{
    int output_bit = 0;
    unsigned input_bit = lp_rx_pin_read();

    output_bit = rlew_dec_bit(&lp.lin.dec, NULL, &rlew_load);
    if (output_bit < 0) {
        lp_signal_load_error(lp.lin.dec.flags);
    } else {
        if (output_bit) {
            lp_tx_pin_set();
        } else {
            lp_tx_pin_clear();
        }

        rlew_enc_bit(&lp.lin.enc, NULL, &rlew_store, input_bit);
        if (unlikely(lp.lin.enc.flags)) {
            lp_signal_store_error(lp.lin.enc.flags);
        }
    }
}

static uint32_t lp_next_bit_relaxed_sum;
// static uint32_t lp_next_bit_relaxed_count;



LP_RAMFUNC static void lp_next_bit_relaxed(void)
{
#if SAME54XPLAINEDPRO
    CMCC->MCTRL.bit.SWRST = 1;
    // uint32_t start = CMCC->MSR.reg;
#endif

    int output_bit = 0;
    unsigned input_bit = lp_rx_pin_read();

    rlew_enc_bit(&lp.lin.enc, NULL, &rlew_store, input_bit);
    lp.lin.enc.flags = 0;

    output_bit = rlew_dec_bit(&lp.lin.dec, NULL, &rlew_load);
    if (output_bit < 0) {
        lp.lin.dec.flags = 0;
        lp_tx_pin_set();
    } else {
        if (output_bit) {
            lp_tx_pin_set();
        } else {
            lp_tx_pin_clear();
        }
    }


#if SAME54XPLAINEDPRO
    // uint32_t end = CMCC->MSR.reg;
    // lp_next_bit_relaxed_sum += end - start;
    lp_next_bit_relaxed_sum += CMCC->MSR.reg;

    if (!(lp.lin.enc.count & 0xffff)) {
        unsigned x = lp_next_bit_relaxed_sum >> 16;
        lp_next_bit_relaxed_sum = 0;
        LP_LOG("lin rx 16K %lu ticks\n", x);

        // board_uart_write("lin rx 16K\n", -1);
    }
#endif
}


// // @120 MHz 1.625 us, cache off
// // @200 MHz 1.250 us, cache off
// // @200 MHz 550-580 ns, cache on
// // @200 MHz 100 ns, cache on (pin toggle)
// LP_RAMFUNC void lp_signal_next_bit(void)
// {
//     // PORT->Group[2].OUTTGL.reg = 0b10000;
//     // goto out;
//     timer_callback();


//     goto out; // prevent compiler warning



// out:
//     ;
// }

