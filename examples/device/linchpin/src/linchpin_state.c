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
    bs_init(&lp.lin.bs);
    rle_init(&lp.lin.decoder);
    rle_init(&lp.lin.encoder);
    lp_lin_clear_rx_tx();
    lp.lin.signal_flags = 0;
    lp.lin.mode = LP_LIN_MODE_OFF;

    __atomic_thread_fence(__ATOMIC_RELEASE);
}

void lp_init(void)
{
    memset(&lp, 0, sizeof(lp));
    lp.lin.signal_frequency = 1250000;
    lp_lin_reset();
}




LP_RAMFUNC static bool usb_try_rx2(
    uint8_t itf,
    volatile size_t *gi_ptr,
    volatile size_t *pi_ptr,
    uint8_t *buf_ptr,
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

        uint32_t r = lp_cdc_rx(itf, &buf_ptr[index], count);
        if (r) {
            // LP_LOG("read: ");
            // for (uint8_t i = offset; i < lp.cmd.usb_rx_count; ++i) {
            // 	LP_LOG("%c %#x", lp.cmd.usb_rx_buffer[i], lp.cmd.usb_rx_buffer[i]);
            // }
            // LP_LOG("\n");
            __atomic_store_n(pi_ptr, pi + r, __ATOMIC_RELEASE);
            return true;
        }
    }

    return false;
}


LP_RAMFUNC static bool usb_try_tx2(
    uint8_t itf,
    volatile size_t *gi_ptr,
    volatile size_t *pi_ptr,
    uint8_t *buf_ptr,
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

        uint32_t w = lp_cdc_tx(itf, &buf_ptr[index], count);
        if (w) {
            LP_ASSERT(w <= count);
            __atomic_store_n(gi_ptr, gi + w, __ATOMIC_RELEASE);

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
                        place_error_response(LP_ERROR_NONE);
                        break;
                    case 'r':
                    case 'R':
                        __atomic_store_n(&lp.lin.mode, LP_LIN_MODE_RELAXED, __ATOMIC_RELEASE);
                        lp_timer_start();
                        place_error_response(LP_ERROR_NONE);
                        break;
                    case 's':
                    case 'S':
                        __atomic_store_n(&lp.lin.mode, LP_LIN_MODE_STRICT, __ATOMIC_RELEASE);
                        lp_timer_start();
                        place_error_response(LP_ERROR_NONE);
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
            case 'f':
            case 'F':
                lp.cmd.usb_tx_buffer_gi = 0;
                lp.cmd.usb_tx_buffer_pi = usnprintf((char*)lp.cmd.usb_tx_buffer, sizeof(lp.cmd.usb_tx_buffer), "%d %" PRIu32 "\n", LP_ERROR_NONE, lp.lin.signal_frequency);
                break;
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
                    ARRAY_SIZE(lp.cmd.usb_tx_buffer));

                more = more || usb_tx;
                if (!usb_tx) {
                    bool read_from_usb = usb_try_rx2(
                        index,
                        &lp.cmd.usb_rx_buffer_gi,
                        &lp.cmd.usb_rx_buffer_pi,
                        lp.cmd.usb_rx_buffer,
                        ARRAY_SIZE(lp.cmd.usb_rx_buffer));

                    more = more || read_from_usb;

                    while (lp.cmd.usb_rx_buffer_gi != lp.cmd.usb_rx_buffer_pi) {
                        char c = lp.cmd.usb_rx_buffer[lp.cmd.usb_rx_buffer_gi];

                        switch (c) {
                        case '\n':
                        case '\r': {
                            lp.cmd.usb_rx_buffer[lp.cmd.usb_rx_buffer_gi + 1] = 0;
                            process_cmd(lp.cmd.usb_rx_buffer, lp.cmd.usb_rx_buffer_gi);
                            lp.cmd.usb_rx_buffer_gi = 0;
                            lp.cmd.usb_rx_buffer_pi = 0;

                            usb_try_tx2(
                                index,
                                &lp.cmd.usb_tx_buffer_gi,
                                &lp.cmd.usb_tx_buffer_pi,
                                lp.cmd.usb_tx_buffer,
                                ARRAY_SIZE(lp.cmd.usb_tx_buffer));
                            lp_cdc_tx_flush(index);
                        } break;
                        default:
                            if (lp.cmd.usb_rx_buffer_pi + 1 == ARRAY_SIZE(lp.cmd.usb_rx_buffer)) {
                                lp.cmd.usb_rx_buffer_gi = 0;
                                lp.cmd.usb_rx_buffer_pi = 0;
                                place_unknown_cmd_response();
                                usb_try_tx2(
                                    index,
                                    &lp.cmd.usb_tx_buffer_gi,
                                    &lp.cmd.usb_tx_buffer_pi,
                                    lp.cmd.usb_tx_buffer,
                                    ARRAY_SIZE(lp.cmd.usb_tx_buffer));
                                lp_cdc_tx_flush(index);
                            } else {
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
                    ARRAY_SIZE(lp.lin.signal_tx_buffer));

                more = more || read_from_usb;

                bool wrote_to_usb = usb_try_tx2(
                    index,
                    &lp.lin.signal_rx_buffer_gi,
                    &lp.lin.signal_rx_buffer_pi,
                    lp.lin.signal_rx_buffer,
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

LP_RAMFUNC static int bs_read_byte(void* ctx, uint8_t* byte)
{
    (void)ctx;

    size_t gi = lp.lin.signal_tx_buffer_gi;
    size_t pi = __atomic_load_n(&lp.lin.signal_tx_buffer_pi, __ATOMIC_ACQUIRE);

    if (pi == gi) {
        return 1;
    }

    *byte = lp.lin.signal_tx_buffer[gi % ARRAY_SIZE(lp.lin.signal_tx_buffer)];

    __atomic_store_n(&lp.lin.signal_tx_buffer_gi, gi + 1, __ATOMIC_RELEASE);

    return 0;
}

LP_RAMFUNC static int rle_read_bits(void* ctx, uint8_t* ptr, unsigned count)
{
    unsigned out_bits_left = 8;

    (void)ctx;


    for (unsigned i = 0; i < count; ++i) {
        int bit;
        int e = bs_read(&lp.lin.bs, NULL, &bs_read_byte, &bit);
        if (unlikely(e)) {
            return 0;
        }

        *ptr <<= 1;
        *ptr |= bit & 1;

        --out_bits_left;

        if (out_bits_left == 0) {
            if (i + 1 < count) {
                out_bits_left = 8;
                ++ptr;
            }
        }
    }

    *ptr <<= out_bits_left;

    return count;

}

LP_RAMFUNC static int bs_write_byte(void* ctx, uint8_t byte)
{
    size_t gi = __atomic_load_n(&lp.lin.signal_rx_buffer_gi, __ATOMIC_ACQUIRE);
    size_t pi = lp.lin.signal_rx_buffer_pi;
    size_t used = pi - gi;

    (void)ctx;

    if (unlikely(used == ARRAY_SIZE(lp.lin.signal_rx_buffer))) {
        return 1;
    }

    lp.lin.signal_rx_buffer[pi % ARRAY_SIZE(lp.lin.signal_rx_buffer)] = byte;
    __atomic_store_n(&lp.lin.signal_rx_buffer_pi, pi + 1, __ATOMIC_RELEASE);

    return 0;
}

LP_RAMFUNC static int rle_write_bits(void* ctx, uint8_t const* ptr, unsigned count)
{
    (void)ctx;

    for (unsigned i = 0; i < count; ++i) {
        unsigned bo = 7 - (i & 7);
        int bit = ((*ptr) & (1u << bo)) == bo;
        int e = bs_write(&lp.lin.bs, NULL, &bs_write_byte, bit);
        if (e) {
            return 0;
        }

        if (i && !(i & 7)) {
            ++ptr;
        }
    }

    return count;
}



LP_RAMFUNC static void lp_signal_store_error(unsigned lin_enc_flags)
{
    lp_timer_stop();

    uint8_t flags = OUTPUT_FLAG_OUTPUT_DONE;

    if (lin_enc_flags & (RLE_FLAG_ENC_OVERFLOW)) {
        flags |= OUTPUT_FLAG_RX_OVERFLOW;
    } else {
        flags |= OUTPUT_FLAG_ERROR;
    }

    __atomic_or_fetch(&lp.lin.signal_flags, flags, __ATOMIC_ACQ_REL);
}

LP_RAMFUNC static void lp_signal_load_error(unsigned lin_dec_flags)
{
    lp_timer_stop();

    if (lin_dec_flags & RLE_FLAG_DEC_EOS) {
        rle_encode_flush(&lp.lin.encoder, NULL, &rle_write_bits);
        uint8_t lin_enc_flags = lp.lin.encoder.flags & ~RLE_FLAG_ENC_VALUE;
        if (unlikely(lin_enc_flags)) {
            lp_signal_store_error(lin_enc_flags);
        } else {
            // terminate input
            size_t eos[2] = {0, 0};
            int w = rle_write_bits(NULL, (uint8_t*)eos, sizeof(eos) * 8);
            if (unlikely((unsigned)w != sizeof(eos) * 8)) {
                if (w < 0) {
                    __atomic_or_fetch(&lp.lin.signal_flags, OUTPUT_FLAG_OUTPUT_DONE | OUTPUT_FLAG_ERROR, __ATOMIC_ACQ_REL);
                } else  {
                    __atomic_or_fetch(&lp.lin.signal_flags, OUTPUT_FLAG_OUTPUT_DONE | OUTPUT_FLAG_RX_OVERFLOW, __ATOMIC_ACQ_REL);
                }
            } else {
                __atomic_or_fetch(&lp.lin.signal_flags, OUTPUT_FLAG_OUTPUT_DONE, __ATOMIC_ACQ_REL);
            }
        }
    } else if (lin_dec_flags & RLE_FLAG_DEC_UNDERFLOW) {
        __atomic_or_fetch(&lp.lin.signal_flags, OUTPUT_FLAG_OUTPUT_DONE | OUTPUT_FLAG_TX_STALLED, __ATOMIC_ACQ_REL);
    } else {
        __atomic_or_fetch(&lp.lin.signal_flags, OUTPUT_FLAG_OUTPUT_DONE | OUTPUT_FLAG_ERROR, __ATOMIC_ACQ_REL);
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
    int output_bit = 0;
    int input_bit = lp_rx_pin_read();
    uint8_t mode = __atomic_load_n(&lp.lin.mode, __ATOMIC_RELAXED);

    if (LP_LIN_MODE_RELAXED == mode) {
        rle_decode_bit(&lp.lin.decoder, NULL, &rle_read_bits, &output_bit);
        if (lp.lin.decoder.flags & (RLE_FLAG_DEC_UNDERFLOW | RLE_FLAG_DEC_EOS | RLE_FLAG_DEC_ERROR)) {
            lp.lin.decoder.flags = 0;
            lp_tx_pin_set();
        } else {
            if (output_bit) {
                lp_tx_pin_set();
            } else {
                lp_tx_pin_clear();
            }
        }

        rle_encode_bit(&lp.lin.encoder, NULL, &rle_write_bits, input_bit);
        lp.lin.encoder.flags &= ~RLE_FLAG_ENC_VALUE;
    } else {
        rle_decode_bit(&lp.lin.decoder, NULL, &rle_read_bits, &output_bit);
        uint8_t lin_dec_flags = lp.lin.decoder.flags & ~RLE_FLAG_DEC_AVAILABLE;
        if (unlikely(lin_dec_flags)) {
            lp_signal_load_error(lin_dec_flags);
        } else {
            if (output_bit) {
                lp_tx_pin_set();
            } else {
                lp_tx_pin_clear();
            }

            rle_encode_bit(&lp.lin.encoder, NULL, &rle_write_bits, input_bit);
            uint8_t lin_enc_flags = lp.lin.encoder.flags & ~RLE_FLAG_ENC_VALUE;
            if (unlikely(lin_enc_flags)) {
                lp_signal_store_error(lin_enc_flags);
            }
        }
    }

    goto out; // prevent compiler warning



out:
    ;
}
