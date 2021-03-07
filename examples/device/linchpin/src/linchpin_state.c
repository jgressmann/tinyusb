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

static bool usb_try_rx(void)
{
    if (lp_cdc_rx_available() && lp.usb_rx_count < ARRAY_SIZE(lp.usb_rx_buffer)) {
        uint8_t offset = lp.usb_rx_count;
        lp.usb_rx_count += (uint8_t)lp_cdc_rx(&lp.usb_rx_buffer[offset], ARRAY_SIZE(lp.usb_rx_buffer) - offset);
        // LP_LOG("read: ");
        // for (uint8_t i = offset; i < lp.usb_rx_count; ++i) {
        // 	LP_LOG("%c %#x", lp.usb_rx_buffer[i], lp.usb_rx_buffer[i]);
        // }
        // LP_LOG("\n");
        return true;
    }

    return false;
}

static bool usb_try_tx(void)
{
    size_t count = lp.usb_tx_buffer_pi - lp.usb_tx_buffer_gi;
    if (count) {
        size_t index = lp.usb_tx_buffer_gi % ARRAY_SIZE(lp.usb_tx_buffer);
        if (index + count > ARRAY_SIZE(lp.usb_tx_buffer)) {
            count = ARRAY_SIZE(lp.usb_tx_buffer) - index;
        }


        uint32_t w = lp_cdc_tx(&lp.usb_tx_buffer[index], count);
        if (w) {
            LP_ASSERT(w <= count);
            lp.usb_tx_buffer_gi += w;

            return true;
        }
    }

    return false;
}

static inline void place_error_response(int code)
{
    lp.usb_tx_buffer_gi = 0;
    lp.usb_tx_buffer_pi = usnprintf((char*)lp.usb_tx_buffer, sizeof(lp.usb_tx_buffer), "%d %s\n", code, lp_strerror(code));
}

static inline void place_unknown_cmd_response(void)
{
    place_error_response(LP_ERROR_UNKNOWN_CMD);
}


static void run_abort(void)
{

}

static void run_init(void)
{
    lp.run_state = LP_RUN_REQUESTED;
    lp.signal_tx_buffer_pi = 0;
    lp.signal_tx_buffer_gi = 0;
    lp.signal_rx_buffer_pi = 0;
    lp.signal_rx_buffer_gi = 0;
    lp.signal_priv_input_bit = 0;
    lp.signal_priv_input_count = 0;
    lp.signal_flags = 0;
    lp.signal_priv_output_count = 0;
    lp.signal_priv_output_value = 0;
    base64_init(&lp.usb_rx_base64_state);
    base64_init(&lp.usb_tx_base64_state);

    __atomic_thread_fence(__ATOMIC_RELEASE);
}


static void process_cmd(void)
{
    uint8_t *ptr = lp.cmd_buffer;
    uint8_t count = lp.cmd_count;

    if (count > 0) {
        bool is_write = *ptr == '!';
        bool is_read = *ptr == '?';

        if (is_write) {
            if (count > 1)  {
                switch (ptr[1]) {
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
                                    lp.signal_frequency = f;
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
                                //LP_LOG("set pin=%#" PRIx32 " value=%u\n", pin, value);
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
                case 'R': {
                    lp.state = LP_RUNNING;
                    run_init();
                    place_error_response(LP_ERROR_NONE);
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
                case 'F':
                    lp.usb_tx_buffer_gi = 0;
                    lp.usb_tx_buffer_pi = usnprintf((char*)lp.usb_tx_buffer, sizeof(lp.usb_tx_buffer), "%d %lu\n", LP_ERROR_NONE, lp.signal_frequency);
                    break;
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
}

static bool try_base64_encode_input_data(void)
{
    bool more = false;

    for (;;) {
        uint32_t usb_tx_available = ARRAY_SIZE(lp.usb_tx_buffer) - (lp.usb_tx_buffer_pi - lp.usb_tx_buffer_gi);

        if (usb_tx_available < 2) {
            break;
        }

        uint8_t pi = __atomic_load_n(&lp.signal_rx_buffer_pi, __ATOMIC_ACQUIRE);
        uint8_t gi = lp.signal_rx_buffer_gi;
        uint32_t input_available = pi - gi;
        if (!input_available) {
            break;
        }

        more = true;

        size_t index = gi % ARRAY_SIZE(lp.signal_rx_buffer);
        uint8_t byte = lp.signal_rx_buffer[index];
        base64_encode_shift(
            byte,
            &lp.usb_tx_base64_state,
            &lp.usb_tx_buffer_gi,
            &lp.usb_tx_buffer_pi,
            lp.usb_tx_buffer,
            ARRAY_SIZE(lp.usb_tx_buffer)
        );

        __atomic_store_n(&lp.signal_rx_buffer_gi, gi + 1, __ATOMIC_RELEASE);
    }


    return more;
}


static bool run_task(void)
{
    bool more = false;
    uint8_t i = 0;

    switch (lp.run_state) {
    case LP_RUN_REQUESTED: {
        bool read_from_usb = usb_try_rx();
        more = more || read_from_usb;

        if (lp.usb_rx_count) {
            more = true;
            for (; i < lp.usb_rx_count; ++i) {
                uint8_t c = lp.usb_rx_buffer[i];
                switch (c) {
                case ' ':
                case '\t':
                case '\n':
                case '\r':
                    break;
                default:
                    if (base64_is_base64_char(c)) {
//                        LP_LOG("start\n");
//                        lp_tx_pin_set();
                        lp_timer_start();
                        lp.run_state = LP_RUN_STARTED;
                        goto started;
                    } else {
                        place_error_response(LP_ERROR_MALFORMED);
                        run_abort();
                    }
                }
            }

            lp.usb_rx_count = 0;
        }
    } break;
    case LP_RUN_STARTED: {
        bool read_from_usb = usb_try_rx();
        more = more || read_from_usb;

        if (lp.usb_rx_count) {
            more = true;
started:
            for (; i < lp.usb_rx_count; ++i) {
                uint8_t c = lp.usb_rx_buffer[i];
                switch (c) {
                case '\n':
                case '\r':
                    __atomic_or_fetch(&lp.signal_flags, OUTPUT_FLAG_INPUT_DONE, __ATOMIC_ACQ_REL);
                    lp.run_state = LP_RUN_STOPPING;
                    i = lp.usb_rx_count;
                    break;
                default:
                    base64_decode_shift(
                        c,
                        &lp.usb_rx_base64_state,
                        &lp.signal_tx_buffer_gi,
                        &lp.signal_tx_buffer_pi,
                        lp.signal_tx_buffer,
                        ARRAY_SIZE(lp.signal_tx_buffer)
                    );

                    if (lp.usb_rx_base64_state.flags & BASE64_FLAG_DONE) {
                        __atomic_or_fetch(&lp.signal_flags, OUTPUT_FLAG_INPUT_DONE, __ATOMIC_ACQ_REL);
                        lp.run_state = LP_RUN_STOPPING;
                        i = lp.usb_rx_count;
                    }
                    break;
                }
            }

            lp.usb_rx_count = 0;
        }

        bool encoded = try_base64_encode_input_data();
        more = more || encoded;

        bool wrote_to_usb = usb_try_tx();
        more = more || wrote_to_usb;

        if ((lp.signal_flags & ~(OUTPUT_FLAG_OUTPUT_DONE | OUTPUT_FLAG_INPUT_DONE)) ||
             (lp.usb_rx_base64_state.flags & ~BASE64_FLAG_DONE)) {
            lp_timer_stop();
            __atomic_or_fetch(&lp.signal_flags, OUTPUT_FLAG_OUTPUT_DONE, __ATOMIC_ACQ_REL);
            lp.run_state = LP_RUN_STOPPING;
            more = true;
        }
    } break;
    case LP_RUN_STOPPING: {
        bool encoded = try_base64_encode_input_data();
        more = more || encoded;

        bool wrote_to_usb = usb_try_tx();
        more = more || wrote_to_usb;

        uint8_t flags = __atomic_load_n(&lp.signal_flags, __ATOMIC_ACQUIRE);

        if (flags & OUTPUT_FLAG_OUTPUT_DONE) {
            if (!encoded) {
                // flush base64 encoder
                base64_encode_finalize(
                    &lp.usb_tx_base64_state,
                    &lp.usb_tx_buffer_gi,
                    &lp.usb_tx_buffer_pi,
                            lp.usb_tx_buffer,
                            ARRAY_SIZE(lp.usb_tx_buffer));

                // finish with newline
                if ((size_t)(lp.usb_tx_buffer_pi - lp.usb_tx_buffer_gi) < ARRAY_SIZE(lp.usb_tx_buffer)) {
                    lp.usb_tx_buffer[lp.usb_tx_buffer_pi % ARRAY_SIZE(lp.usb_tx_buffer)] = '\n';
                    ++lp.usb_tx_buffer_pi;
                }

                lp.run_state = LP_RUN_STOPPED;
                more = true;
            }
        }
    } break;
    case LP_RUN_STOPPED: {
        bool wrote_to_usb = usb_try_tx();
        more = more || wrote_to_usb;

        if (lp.usb_tx_buffer_gi == lp.usb_tx_buffer_pi) {
            lp_cdc_tx_flush();

            int error = LP_ERROR_NONE;
            if ((lp.signal_flags & (OUTPUT_FLAG_RX_OVERFLOW | OUTPUT_FLAG_TX_STALLED)) ||
                (lp.usb_rx_base64_state.flags & (BASE64_FLAG_OVERFLOW | BASE64_FLAG_INVALID_CHAR)) ||
                (lp.usb_tx_base64_state.flags & (BASE64_FLAG_OVERFLOW | BASE64_FLAG_INVALID_CHAR))) {
                error = LP_ERROR_FAILED;
            }

            char* ptr = (char*)lp.usb_tx_buffer;
            size_t left = sizeof(lp.usb_tx_buffer);
            int chars = usnprintf(ptr, left, "%d ", error);
            LP_DEBUG_ASSERT(chars > 0);
            LP_DEBUG_ASSERT((size_t)chars <= left);
            ptr += chars;
            left -= chars;

            if (lp.signal_flags & OUTPUT_FLAG_RX_OVERFLOW) {
                chars = usnprintf(ptr, left, "SIGRXOVRFL ");
                LP_DEBUG_ASSERT(chars > 0);
                LP_DEBUG_ASSERT((size_t)chars <= left);
                ptr += chars;
                left -= chars;
            }

            if (lp.signal_flags & OUTPUT_FLAG_TX_STALLED) {
                chars = usnprintf(ptr, left, "SIGTXSTALL ");
                LP_DEBUG_ASSERT(chars > 0);
                LP_DEBUG_ASSERT((size_t)chars <= left);
                ptr += chars;
                left -= chars;
            }

            if (lp.usb_rx_base64_state.flags & BASE64_FLAG_OVERFLOW) {
                chars = usnprintf(ptr, left, "USBRXB64OVRFL ");
                LP_DEBUG_ASSERT(chars > 0);
                LP_DEBUG_ASSERT((size_t)chars <= left);
                ptr += chars;
                left -= chars;
            }

            if (lp.usb_rx_base64_state.flags & BASE64_FLAG_INVALID_CHAR) {
                chars = usnprintf(ptr, left, "USBRXB64INV ");
                LP_DEBUG_ASSERT(chars > 0);
                LP_DEBUG_ASSERT((size_t)chars <= left);
                ptr += chars;
                left -= chars;
            }

            if (lp.usb_tx_base64_state.flags & BASE64_FLAG_OVERFLOW) {
                chars = usnprintf(ptr, left, "USBTXB64OVRFL ");
                LP_DEBUG_ASSERT(chars > 0);
                LP_DEBUG_ASSERT((size_t)chars <= left);
                ptr += chars;
                left -= chars;
            }

            if (lp.usb_tx_base64_state.flags & BASE64_FLAG_INVALID_CHAR) {
                chars = usnprintf(ptr, left, "USBTXB64INV ");
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

            lp.state = LP_CONNECTED;
            lp.usb_tx_buffer_gi = 0;
            lp.usb_tx_buffer_pi = (sizeof(lp.usb_tx_buffer) - left);
            more = true;
        }
    } break;
    default:
        LP_LOG("unhandled LP_RUN_ state %d\n", lp.run_state);
        break;
    }

    return more;
}


struct linchpin lp;

void lp_init(void)
{
    memset(&lp, 0, sizeof(lp));
    lp.signal_frequency = 1250000;
    lp.state = LP_DISCONNECTED;
}

void lp_cdc_task(void)
{
    for (bool more = true; more; ) {
        more = false;
        bool connected = lp_cdc_is_connected();

        switch (lp.state) {
        case LP_DISCONNECTED:
            if (connected) {
                lp.state = LP_CONNECTED;
                more = true;
            }
            break;
        case LP_CONNECTED: {
            bool read_from_usb = usb_try_rx();
            more = more || read_from_usb;

            if (lp.usb_rx_count) {
                char* p = (char*)lp.usb_rx_buffer;
                for (uint8_t i = 0; i < lp.usb_rx_count; ++i) {
                    char c = p[i];
                    switch (c) {
                    case '\n':
                    case '\r':
                        // while (i < lp.usb_rx_count && (p[i] == '\r' || p[i] == '\r'))) {
                        // 	++i;
                        // }
                        lp.cmd_buffer[lp.cmd_count] = 0;
                        process_cmd();
                        lp.cmd_count = 0;
                        break;
                    default:
                        if (lp.cmd_count + 1 == ARRAY_SIZE(lp.cmd_buffer)) {
                            lp.cmd_count = 0;
                            place_unknown_cmd_response();
                            lp_cdc_tx_flush();
                        } else {
                            lp.cmd_buffer[lp.cmd_count++] = c;
                        }
                        break;
                    }
                }


                lp.usb_rx_count = 0;

                more = true;
            }

            bool wrote_to_usb = usb_try_tx();
            more = more || wrote_to_usb;
        } break;
        case LP_RUNNING: {
            if (connected) {
                more = run_task();
            } else {
                run_abort();
                lp.state = LP_DISCONNECTED;
                more = true;
            }
        } break;
        default:
            LP_LOG("unhandled LP_ state %d\n", lp.state);
            break;
        }
    }

    lp_delay_ms(1);
}

LP_RAMFUNC static inline void store_current_rle_bit(void)
{
    uint8_t pi = lp.signal_rx_buffer_pi;
    uint8_t gi = __atomic_load_n(&lp.signal_rx_buffer_gi, __ATOMIC_ACQUIRE);
    size_t used = pi - gi;

    if (unlikely(used == ARRAY_SIZE(lp.signal_rx_buffer))) {
        lp_timer_stop();
        __atomic_or_fetch(&lp.signal_flags, OUTPUT_FLAG_RX_OVERFLOW | OUTPUT_FLAG_OUTPUT_DONE, __ATOMIC_ACQ_REL);
    } else {
        // LP_LOG("IN pin=%u count=%u\n", lp.input.bit.value, lp.input.bit.count);
        union rle_bit rle;
        size_t index = pi % ARRAY_SIZE(lp.signal_rx_buffer);

        rle.bit.value = lp.signal_priv_input_bit;
        rle.bit.count = lp.signal_priv_input_count;
        lp.signal_rx_buffer[index] = rle.mux;
        __atomic_store_n(&lp.signal_rx_buffer_pi, pi + 1, __ATOMIC_RELEASE);
    }
}

LP_RAMFUNC static void load_next(void)
{
    uint8_t pi, gi;
    uint8_t flags_now, flags_target;
    bool flush = false;

fetch:
    pi = __atomic_load_n(&lp.signal_tx_buffer_pi, __ATOMIC_ACQUIRE);
    gi = lp.signal_tx_buffer_gi;

    if (unlikely(pi == gi)) {
        lp_timer_stop();
        do {
            flags_now = __atomic_load_n(&lp.signal_flags, __ATOMIC_ACQUIRE);
            if (flags_now & OUTPUT_FLAG_OUTPUT_DONE) {
                flush = false;
                break;
            }

            if (flags_now & OUTPUT_FLAG_INPUT_DONE) {
                flush = true;
                flags_target = flags_now | OUTPUT_FLAG_OUTPUT_DONE;
            } else {
                flush = false;
                flags_target = flags_now | OUTPUT_FLAG_OUTPUT_DONE | OUTPUT_FLAG_TX_STALLED;
            }
        } while (!__atomic_compare_exchange_n(
                     &lp.signal_flags,
                     &flags_now,
                     flags_target,
                     false,
                     __ATOMIC_RELEASE,
                     __ATOMIC_RELAXED
                     ));

        if (flush && lp.signal_priv_input_count) {
            store_current_rle_bit();
        }
    } else {
        size_t index = gi % ARRAY_SIZE(lp.signal_tx_buffer);
        union rle_bit r;
        r.mux = lp.signal_tx_buffer[index];
        __atomic_store_n(&lp.signal_tx_buffer_gi, gi + 1, __ATOMIC_RELEASE);

        lp.signal_priv_output_value = r.bit.value;
        lp.signal_priv_output_count = r.bit.count;

        if (unlikely(!lp.signal_priv_output_count)) {
            goto fetch;
        }

        // LP_LOG("OUT pin=%u count=%u\n", r.value, r.bit.count);
    }
}

LP_RAMFUNC static void save_next(bool value)
{
    if (likely(lp.signal_priv_input_count)) {
        bool store = true;

        if (value == lp.signal_priv_input_bit) {
            if (likely(lp.signal_priv_input_count != RLE_BIT_MAX_COUNT)) {
                ++lp.signal_priv_input_count;
                store = false;
            }
        }

        if (store) {
            store_current_rle_bit();

            lp.signal_priv_input_count = 1;
            lp.signal_priv_input_bit = value;
        }
    } else {
        lp.signal_priv_input_count = 1;
        lp.signal_priv_input_bit = value;
    }
}


// @120 MHz 1.625 us, cache off
// @200 MHz 1.250 us, cache off
// @200 MHz 550-580 ns, cache on
// @200 MHz 100 ns, cache on (pin toggle)
LP_RAMFUNC void lp_signal_next_bit(void)
{
    // PORT->Group[2].OUTTGL.reg = 0b10000;
    // goto out;
    bool value = lp_rx_pin_read();

    if (likely(lp.signal_priv_output_count)) {
        --lp.signal_priv_output_count;
        if (lp.signal_priv_output_value) {
            lp_tx_pin_set();
        } else {
            lp_tx_pin_clear();
        }

        save_next(value);

        if (unlikely(!lp.signal_priv_output_count)) {
            load_next();
        }
    } else {
        load_next();
    }

    goto out; // avoid warning



out:
    ;
}
