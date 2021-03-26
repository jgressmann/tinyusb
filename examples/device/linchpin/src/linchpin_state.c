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
#include <fastlz.h>


//static inline bool usb_try_rx_clear(void)
//{
//    if (lp_cdc_rx_available()) {
//        lp_cdc_rx_clear();
//        return true;
//    }

//    return false;
//}

static bool usb_try_rx(void)
{
    size_t used = lp.usb_rx_buffer_pi - lp.usb_rx_buffer_gi;

    if (used < ARRAY_SIZE(lp.usb_rx_buffer) && lp_cdc_rx_available()) {
        size_t count = ARRAY_SIZE(lp.usb_rx_buffer) - used;
        size_t index = lp.usb_rx_buffer_pi % ARRAY_SIZE(lp.usb_rx_buffer);

        if (index + count > ARRAY_SIZE(lp.usb_rx_buffer)) {
            count = ARRAY_SIZE(lp.usb_rx_buffer) - index;
        }

        lp.usb_rx_buffer_pi += lp_cdc_rx(&lp.usb_rx_buffer[index], count);
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

static inline void usb_clear_rx_tx(void)
{
    lp.usb_rx_buffer_gi = 0;
    lp.usb_rx_buffer_pi = 0;
    lp.usb_tx_buffer_gi = 0;
    lp.usb_tx_buffer_pi = 0;
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
//    lp.usb_rx_buffer_gi = 0;
//    lp.usb_rx_buffer_pi = 0;
//    lp.usb_tx_buffer_gi = 0;
//    lp.usb_tx_buffer_pi = 0;
    lp.usb_tx_compressed_gi = 0;
    lp.usb_tx_compressed_pi = 0;
    lp.usb_rx_compressed_offset = 0;

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
                case 'R': {
                    LP_LOG("run start\n");
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
                    lp.usb_tx_buffer_pi = usnprintf((char*)lp.usb_tx_buffer, sizeof(lp.usb_tx_buffer), "%d %" PRIu32 "\n", LP_ERROR_NONE, lp.signal_frequency);
                    break;
                case 'S':
                    lp.usb_tx_buffer_gi = 0;
                    lp.usb_tx_buffer_pi = usnprintf((char*)lp.usb_tx_buffer, sizeof(lp.usb_tx_buffer), "%d %" PRIu32 "\n", LP_ERROR_NONE, (uint32_t)LP_SIGNAL_BUFFER_SIZE);
                    break;
                case 'V': {
                    char buf[LP_USB_BUFFER_SIZE];
                    memset(buf, 0, sizeof(buf));
                    lp_version(buf, sizeof(buf)-1);
                    lp.usb_tx_buffer_gi = 0;
                    lp.usb_tx_buffer_pi = usnprintf((char*)lp.usb_tx_buffer, sizeof(lp.usb_tx_buffer), "0 %s\n", buf);
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
}

static bool run_try_base64_encode_input_data(void)
{
    bool more = false;

    for (;;) {
        size_t usb_tx_available = ARRAY_SIZE(lp.usb_tx_buffer) - (lp.usb_tx_buffer_pi - lp.usb_tx_buffer_gi);

        if (usb_tx_available < 2) {
            break;
        }

        if (lp.usb_tx_compressed_gi == lp.usb_tx_compressed_pi) {
            break;
        }

        more = true;

        size_t index = lp.usb_tx_compressed_gi++ % ARRAY_SIZE(lp.usb_tx_compressed_buffer);
        uint8_t byte = lp.usb_tx_compressed_buffer[index];
        base64_encode_shift(
            byte,
            &lp.usb_tx_base64_state,
            &lp.usb_tx_buffer_gi,
            &lp.usb_tx_buffer_pi,
            lp.usb_tx_buffer,
            ARRAY_SIZE(lp.usb_tx_buffer)
        );
    }


    return more;
}

static inline void run_abort_error(int code)
{
    LP_LOG("run abort code=%d\n", code);
    lp_timer_stop();

    usb_clear_rx_tx();
    lp_cdc_rx_clear();

    lp.usb_tx_buffer_pi = usnprintf((char*)lp.usb_tx_buffer, sizeof(lp.usb_tx_buffer), "\n%d %s\n", code, lp_strerror(code));


    lp.signal_rx_buffer_gi = __atomic_load_n(&lp.signal_rx_buffer_pi, __ATOMIC_ACQUIRE);
    lp.usb_tx_compressed_gi = lp.usb_tx_compressed_pi;
    lp.state = LP_CONNECTED;
}

static bool run_try_decompress(void)
{
//    size_t gi = 0;

//    base64_decode_flush(
//        &lp.usb_rx_base64_state,
//        &gi,
//        &lp.usb_base64_decoded_offset,
//        lp.usb_base64_decoded_buffer,
//        ARRAY_SIZE(lp.usb_base64_decoded_buffer));

//    if ((lp.usb_rx_base64_state.flags & ~BASE64_FLAG_DONE)) {
//        // something broken
//        run_abort_error(LP_ERROR_MALFORMED);
////        return false;
//    } else if (lp.usb_rx_base64_state.flags & BASE64_FLAG_DONE) {
        if (lp.usb_rx_compressed_offset) {
            size_t gi = __atomic_load_n(&lp.signal_tx_buffer_gi, __ATOMIC_ACQUIRE);
            size_t pi = lp.signal_tx_buffer_pi;
            size_t used = pi - gi;
            size_t available = ARRAY_SIZE(lp.signal_tx_buffer) - used;

            if (available >= LP_SIGNAL_BUFFER_SIZE) {
                LP_LOG("try decompress %" PRIu32 " bytes\n", (uint32_t)lp.usb_rx_compressed_offset);
                int bytes_out = fastlz_decompress(
                            lp.usb_rx_compressed_buffer,
                            lp.usb_rx_compressed_offset,
                            lp.fastlz_buffer,
                            ARRAY_SIZE(lp.fastlz_buffer));

                lp.usb_rx_compressed_offset = 0;

                if (bytes_out > 0) {
                    LP_LOG("decompressed to %" PRIu32 " bytes\n", (uint32_t)bytes_out);
                    LP_DEBUG_ASSERT((size_t)bytes_out <= LP_SIGNAL_BUFFER_SIZE);

                    size_t offset = 0;
                    size_t count = bytes_out;
                    size_t index = pi % ARRAY_SIZE(lp.signal_tx_buffer);
                    LP_ASSERT(count <= LP_SIGNAL_BUFFER_SIZE);

                    if (index + count > ARRAY_SIZE(lp.signal_tx_buffer)) {
                        size_t count1 = ARRAY_SIZE(lp.signal_tx_buffer) - index;
                        memcpy(&lp.signal_tx_buffer[index], &lp.fastlz_buffer[offset], count1);
                        index = 0;
                        count -= count1;
                        offset += count1;
                    }

                    memcpy(&lp.signal_tx_buffer[index], &lp.fastlz_buffer[offset], count);

                    // update tx pi
                    __atomic_store_n(&lp.signal_tx_buffer_pi, lp.signal_tx_buffer_pi + bytes_out, __ATOMIC_RELEASE);

                    return true;
                } else {
                    // decompression failed
                    run_abort_error(LP_ERROR_MALFORMED);
                }
            }
        }
//    }

    return false;
}

//static bool run_try_add_base64_char(uint8_t c)
//{
//    if (lp.usb_base64_decoded_offset == ARRAY_SIZE(lp.usb_base64_decoded_buffer)) {
//        // whoopsie, expect at packets that fit in buffer
//        run_abort_error(LP_ERROR_MALFORMED);

//    } else {
//        size_t gi = 0;

//        base64_decode_shift(
//            c,
//            &lp.usb_rx_base64_state,
//            &gi,
//            &lp.usb_base64_decoded_offset,
//            lp.usb_base64_decoded_buffer,
//            ARRAY_SIZE(lp.usb_base64_decoded_buffer));

//        if (lp.usb_rx_base64_state.flags & BASE64_FLAG_DONE) {
//            __atomic_or_fetch(&lp.signal_flags, OUTPUT_FLAG_INPUT_DONE, __ATOMIC_ACQ_REL);
//            lp.run_state = LP_RUN_STOPPING;
////            lp.usb_rx_buffer_gi = lp.usb_rx_buffer_pi;
////            return false;
//        } else {

//            return true;
//        }
//    }

//    return false;
//}

static bool run_try_compress_input_data(bool pad)
{
    if (lp.usb_tx_compressed_gi == lp.usb_tx_compressed_pi
        /*lp.usb_tx_buffer_gi == lp.usb_tx_buffer_pi*/) {

        size_t gi = lp.signal_rx_buffer_gi;
        size_t pi = __atomic_load_n(&lp.signal_rx_buffer_pi, __ATOMIC_ACQUIRE);
        size_t used = pi - gi;

        if (used && (used >= FASTLZ_MIN_INPUT_SIZE || pad)) {
            size_t count = used;
            const size_t LIMIT = LP_SIGNAL_BUFFER_SIZE - (LP_SIGNAL_BUFFER_SIZE / 10 + FASTLZ_MIN_INPUT_SIZE);
            if (count > LIMIT) {
                count = LIMIT;
            }

            size_t index = gi % ARRAY_SIZE(lp.signal_rx_buffer);
            size_t offset = 0;
            size_t rem_count = count;
            if (index + count > ARRAY_SIZE(lp.signal_rx_buffer)) {
                size_t first_count = ARRAY_SIZE(lp.signal_rx_buffer) - index;
                memcpy(&lp.fastlz_buffer[offset], &lp.signal_rx_buffer[index], first_count);
                offset += first_count;
                index = 0;
                rem_count = count - first_count;
            }

            memcpy(&lp.fastlz_buffer[offset], &lp.signal_rx_buffer[index], rem_count);

            __atomic_store_n(&lp.signal_rx_buffer_gi, gi + count, __ATOMIC_RELEASE);

            while (count < FASTLZ_MIN_INPUT_SIZE) {
                lp.fastlz_buffer[count++] = 0;
            }

            int compressed = fastlz_compress_level(
                        1,
                        lp.fastlz_buffer,
                        count,
                        lp.usb_tx_compressed_buffer);

            if (compressed > 0) {
                lp.usb_tx_compressed_gi = 0;
                lp.usb_tx_compressed_pi = compressed;
                return true;
            } else {
                // whoopsie!
            }
        }
    }

    return false;
}

static bool run_task(void)
{
    bool more = false;

    switch (lp.run_state) {
    case LP_RUN_REQUESTED: {
        bool read_from_usb = usb_try_rx();
        more = more || read_from_usb;

        for (bool done = false; !done && lp.usb_rx_buffer_gi != lp.usb_rx_buffer_pi; ) {
            char c = 0;

            more = true;

            c = lp.usb_rx_buffer[lp.usb_rx_buffer_gi % ARRAY_SIZE(lp.usb_rx_buffer)];

            switch (c) {
            case '\n':
            case '\r':
            case '!': {
//                LP_LOG("%c (%#x)\n", c, c);
                ++lp.usb_rx_buffer_gi;
                if (lp.usb_rx_base64_state.flags == BASE64_FLAG_DONE) {
                    if (c != '!') {
                        __atomic_or_fetch(&lp.signal_flags, OUTPUT_FLAG_INPUT_DONE, __ATOMIC_ACQ_REL);
                    }

                    LP_LOG("decompress input of %" PRIu32 " bytes\n", (uint32_t)lp.usb_rx_compressed_offset);
                    LP_LOG("first byte=\n");
                    lp_dump_mem(lp.usb_rx_compressed_buffer, 1);
//                    lp_dump_mem(lp.usb_rx_compressed_buffer, lp.usb_rx_compressed_offset);
                    if (run_try_decompress()) {
                        LP_LOG("decompressed\n");
                        lp_timer_start();
                        lp.run_state = LP_RUN_STARTED;
                        base64_init(&lp.usb_rx_base64_state); // ! only
                    } else {
                        LP_LOG("decomp failed\n");
                        run_abort_error(LP_ERROR_MALFORMED);
                    }
                } else {
                    LP_LOG("base64 flags=%#x\n", lp.usb_rx_base64_state.flags);
                    run_abort_error(LP_ERROR_MALFORMED);
                }
                done = true;
            } break;
//            case '!': {
//                ++lp.usb_rx_buffer_gi;
//                if (lp.usb_rx_base64_state.flags == BASE64_FLAG_DONE) {
//                    __atomic_or_fetch(&lp.signal_flags, OUTPUT_FLAG_INPUT_DONE, __ATOMIC_ACQ_REL);

//                    if (run_try_decompress()) {
//                        lp_timer_start();
//                        lp.run_state = LP_RUN_STARTED;
//                        base64_init(&lp.usb_rx_base64_state);
//                    } else {
//                        run_abort_error(LP_ERROR_MALFORMED);
//                    }
//                } else {
//                    run_abort_error(LP_ERROR_MALFORMED);
//                }

//                done = true;

////                if (run_try_decompress()) {
////                    lp_timer_start();
////                    lp.run_state = LP_RUN_STARTED;
////                } else {
////                    run_abort_error(LP_ERROR_MALFORMED);
////                }
////                done = true;
//            } break;
            default:
                ++lp.usb_rx_buffer_gi;
                if (lp.usb_rx_compressed_offset == ARRAY_SIZE(lp.usb_rx_compressed_buffer)) {
                    // whoopsie, expect at packets that fit in buffer
                    run_abort_error(LP_ERROR_MALFORMED);
                    done = true;
                } else {
                    size_t gi = 0;

                    base64_decode_shift(
                        c,
                        &lp.usb_rx_base64_state,
                        &gi,
                        &lp.usb_rx_compressed_offset,
                        lp.usb_rx_compressed_buffer,
                        ARRAY_SIZE(lp.usb_rx_compressed_buffer));

                    if (lp.usb_rx_base64_state.flags & ~(BASE64_FLAG_DONE | BASE64_FLAG_UNDERFLOW)) {
                        // some decode error
                        run_abort_error(LP_ERROR_MALFORMED);
                        done = true;
                    } else {
//                        if (lp.usb_rx_base64_state.flags == BASE64_FLAG_DONE) {
//                            __atomic_or_fetch(&lp.signal_flags, OUTPUT_FLAG_INPUT_DONE, __ATOMIC_ACQ_REL);

//                            if (run_try_decompress()) {
//                                lp_timer_start();
//                                lp.run_state = LP_RUN_STARTED;
//                            } else {
//                                run_abort_error(LP_ERROR_MALFORMED);
//                            }

//                            done = true;
//                        }
                    }
                }
                break;
            }
        }
    } break;
    case LP_RUN_STARTED: {
        bool read_from_usb = usb_try_rx();
        more = more || read_from_usb;

        for (bool done = false; !done && lp.usb_rx_buffer_gi != lp.usb_rx_buffer_pi; ) {
            char c = 0;

            more = true;

            c = lp.usb_rx_buffer[lp.usb_rx_buffer_gi % ARRAY_SIZE(lp.usb_rx_buffer)];
            switch (c) {
            case '\n':
            case '\r':
            case '!':
                ++lp.usb_rx_buffer_gi;
                if (lp.usb_rx_base64_state.flags == BASE64_FLAG_DONE) {
                    if (c != '!') {
                        __atomic_or_fetch(&lp.signal_flags, OUTPUT_FLAG_INPUT_DONE, __ATOMIC_ACQ_REL);
                    }

                    if (run_try_decompress()) {
                        base64_init(&lp.usb_rx_base64_state); // ! only
                    } else {
                        run_abort_error(LP_ERROR_MALFORMED);
                    }
                } else {
                    run_abort_error(LP_ERROR_MALFORMED);
                }
                done = true;
                break;
            default: {
                ++lp.usb_rx_buffer_gi;
                if (lp.usb_rx_compressed_offset == ARRAY_SIZE(lp.usb_rx_compressed_buffer)) {
                    // whoopsie, expect at packets that fit in buffer
                    run_abort_error(LP_ERROR_MALFORMED);
                    done = true;
                } else {
                    size_t gi = 0;

                    base64_decode_shift(
                        c,
                        &lp.usb_rx_base64_state,
                        &gi,
                        &lp.usb_rx_compressed_offset,
                        lp.usb_rx_compressed_buffer,
                        ARRAY_SIZE(lp.usb_rx_compressed_buffer));

                    if (lp.usb_rx_base64_state.flags & ~(BASE64_FLAG_DONE | BASE64_FLAG_UNDERFLOW)) {
                        // some decode error
                        run_abort_error(LP_ERROR_MALFORMED);
                        done = true;
                    }
                }
            } break;
            }
        }

        bool compressed = run_try_compress_input_data(false);
        more = more || compressed;

        bool encoded = run_try_base64_encode_input_data();
        more = more || encoded;

        bool wrote_to_usb = usb_try_tx();
        more = more || wrote_to_usb;

        uint8_t flags = __atomic_load_n(&lp.signal_flags, __ATOMIC_ACQUIRE);
        if (flags & OUTPUT_FLAG_OUTPUT_DONE) {
            lp.run_state = LP_RUN_STOPPING;
            more = true;
        }/*
        if ((flags & ~(OUTPUT_FLAG_OUTPUT_DONE | OUTPUT_FLAG_INPUT_DONE))) {

        if ((flags & ~(OUTPUT_FLAG_OUTPUT_DONE | OUTPUT_FLAG_INPUT_DONE))) {
//            lp_timer_stop();
            // timer will have stopped on its own account
        } else {
            lp_timer_stop();
            __atomic_or_fetch(&lp.signal_flags, OUTPUT_FLAG_OUTPUT_DONE, __ATOMIC_ACQ_REL);
            lp.run_state = LP_RUN_STOPPING;
            more = true;
        }*/
    } break;

//    case LP_RUN_STOPPING: {
//        bool compressed = run_try_compress_input_data(false);
//        more = more || compressed;

//        bool encoded = run_try_base64_encode_input_data();
//        more = more || encoded;

//        bool wrote_to_usb = usb_try_tx();
//        more = more || wrote_to_usb;

//        uint8_t flags = __atomic_load_n(&lp.signal_flags, __ATOMIC_ACQUIRE);

//        if (flags & OUTPUT_FLAG_OUTPUT_DONE) {
//            lp.run_state = LP_RUN_STOPPING2;
//            more = true;
//        }
//    } break;
    case LP_RUN_STOPPING:
//    case LP_RUN_STOPPING2:
    {
        bool compressed = run_try_compress_input_data(true);
        more = more || compressed;

        bool encoded = run_try_base64_encode_input_data();
        more = more || encoded;

        bool wrote_to_usb = usb_try_tx();
        more = more || wrote_to_usb;

        if (!compressed && !encoded) {
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
    } break;
    case LP_RUN_STOPPED: {
        bool wrote_to_usb = usb_try_tx();
        more = more || wrote_to_usb;

        if (lp.usb_tx_buffer_gi == lp.usb_tx_buffer_pi) {
            lp_cdc_tx_flush();

            int error = LP_ERROR_NONE;
//            if ((lp.signal_flags & (OUTPUT_FLAG_RX_OVERFLOW | OUTPUT_FLAG_TX_STALLED)) ||
//                (lp.usb_rx_base64_state.flags & (BASE64_FLAG_OVERFLOW | BASE64_FLAG_INVALID_CHAR)) ||
//                (lp.usb_tx_base64_state.flags & (BASE64_FLAG_OVERFLOW | BASE64_FLAG_INVALID_CHAR))) {
//                error = LP_ERROR_FAILED;
//            }

            if ((lp.signal_flags & (OUTPUT_FLAG_RX_OVERFLOW | OUTPUT_FLAG_TX_STALLED))) {
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

//            if (lp.usb_rx_base64_state.flags & BASE64_FLAG_OVERFLOW) {
//                chars = usnprintf(ptr, left, "USBRXB64OVRFL ");
//                LP_DEBUG_ASSERT(chars > 0);
//                LP_DEBUG_ASSERT((size_t)chars <= left);
//                ptr += chars;
//                left -= chars;
//            }

//            if (lp.usb_rx_base64_state.flags & BASE64_FLAG_INVALID_CHAR) {
//                chars = usnprintf(ptr, left, "USBRXB64INV ");
//                LP_DEBUG_ASSERT(chars > 0);
//                LP_DEBUG_ASSERT((size_t)chars <= left);
//                ptr += chars;
//                left -= chars;
//            }

//            if (lp.usb_tx_base64_state.flags & BASE64_FLAG_OVERFLOW) {
//                chars = usnprintf(ptr, left, "USBTXB64OVRFL ");
//                LP_DEBUG_ASSERT(chars > 0);
//                LP_DEBUG_ASSERT((size_t)chars <= left);
//                ptr += chars;
//                left -= chars;
//            }

//            if (lp.usb_tx_base64_state.flags & BASE64_FLAG_INVALID_CHAR) {
//                chars = usnprintf(ptr, left, "USBTXB64INV ");
//                LP_DEBUG_ASSERT(chars > 0);
//                LP_DEBUG_ASSERT((size_t)chars <= left);
//                ptr += chars;
//                left -= chars;
//            }

            chars = usnprintf(ptr, left, "\n");
            LP_DEBUG_ASSERT(chars == 1);
            LP_DEBUG_ASSERT((size_t)chars <= left);
            ptr += chars;
            left -= chars;

            lp.state = LP_CONNECTED;
            lp.usb_tx_buffer_gi = 0;
            lp.usb_tx_buffer_pi = (sizeof(lp.usb_tx_buffer) - left);

            usb_try_tx();
            lp_cdc_tx_flush();

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

void lp_cdc_cmd_task(void)
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
            if (connected) {

                // if there is data waiting in the tx buffer, don't process commands
                bool usb_tx = usb_try_tx();
                more = more || usb_tx;
                if (!usb_tx) {
                    bool read_from_usb = usb_try_rx();
                    more = more || read_from_usb;

                    while (lp.usb_rx_buffer_gi != lp.usb_rx_buffer_pi) {
                        char c = lp.usb_rx_buffer[lp.usb_rx_buffer_gi % ARRAY_SIZE(lp.usb_rx_buffer)];

                        switch (c) {
                        case '\n':
                        case '\r': {
                            lp.cmd_buffer[lp.cmd_count] = 0;
                            process_cmd();
                            lp.cmd_count = 0;

                            usb_try_tx();
                            lp_cdc_tx_flush();
                        } break;
                        default:
                            if (lp.cmd_count + 1 == ARRAY_SIZE(lp.cmd_buffer)) {
                                lp.cmd_count = 0;
                                place_unknown_cmd_response();
                                usb_try_tx();
                                lp_cdc_tx_flush();
                            } else {
                                lp.cmd_buffer[lp.cmd_count++] = c;
                            }
                            break;
                        }


                        ++lp.usb_rx_buffer_gi;

                        more = true;
                    }
                }

            } else {
                lp.state = LP_DISCONNECTED;
                usb_clear_rx_tx();
                lp_cdc_rx_clear();
                lp_cdc_tx_clear();
                more = true;
            }
        } break;
        case LP_RUNNING: {
            if (connected) {
                more = run_task();
            } else {
                lp_timer_stop();
                lp.state = LP_DISCONNECTED;
                usb_clear_rx_tx();
                lp_cdc_rx_clear();
                lp_cdc_tx_clear();
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
