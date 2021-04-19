#include <gtest/gtest.h>
#include <gmock/gmock.h>


#include <linchpin.h>
#include <bitstream.h>




#define RLE_C
#include <rle.h>


#include <string>
#include <algorithm>
#include <deque>
#include <vector>
#include <cstring>

using ::testing::StartsWith;



namespace
{
static inline void lp_signal_next_bit()
{
    lp_timer_callback();
}



struct serial_fixture : public ::testing::Test
{
    static serial_fixture* self;

    bool connected;
    int pin;
    int cmd_output_capacity;
    std::string cmd_input;
    std::string cmd_output;
    std::vector<uint8_t> lin_output;
    std::deque<uint8_t> lin_input;
    std::vector<std::tuple<int, bool>> pins_set;
    std::deque<bool> tx_pin_values;
    std::deque<bool> rx_pin_values;
    bool pin_set_success;
    bool timer_started;
    int flush_count;
    int rx_clear_count;
    int tx_clear_count;
    struct bitstream lin_input_bs;


    ~serial_fixture()
    {
        self = nullptr;
    }

    serial_fixture()
    {
        self = this;

        bs_init(&lin_input_bs);

        connected = false;
        pin = -1;
        cmd_output_capacity = -1;
        pin_set_success = true;
        timer_started = false;
        flush_count = 0;
        rx_clear_count = 0;
        tx_clear_count = 0;

        lp_init();
    }

    void connect()
    {
        connected = true;
        lp_cdc_cmd_task();
        lp_cdc_lin_task();
    }

    void set_lin_mode(const char *mode)
    {
        cmd_input = mode;
        lp_cdc_cmd_task();
        cmd_output.clear();
    }

    void set_strict_mode()
    {
        set_lin_mode("!lms\n");
    }

    void set_relaxed_mode()
    {
        set_lin_mode("!lmR\n");
    }

    bool cmd_output_has_error_code(int code) const
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%d ", code);
        return cmd_output.find(buf) != std::string::npos;
    }

    void append_lin_input(char const *str)
    {
        struct rle rle;
        rle_init(&rle);

        while (*str) {
            bool bit = *str++ == '1';
            rle_encode_bit(&rle, this, &static_append_lin_input_bits, bit);
        }

        rle_encode_flush(&rle, this, &static_append_lin_input_bits);
    }

    void set_lin_input(char const *str)
    {
        append_lin_input(str);
        while (lin_input_bs.incoming_count != 0) {
            bs_write(&lin_input_bs, this, &static_append_lin_input_byte, 1);
        }

        size_t term = ~0;
        lin_input.insert(lin_input.end(), (uint8_t*)&term, (uint8_t*)(&term + 1));

//        for (auto it = lin_input.begin(); it != lin_input.end(); ++it) {
//            fprintf(stdout, "%02x ", *it);
//        }
//        fprintf(stdout, "\n");
    }

    static int static_append_lin_input_byte(void* ctx, uint8_t byte)
    {
        static_cast<serial_fixture*>(ctx)->lin_input.push_back(byte);
        return 0;
    }

    static int static_append_lin_input_bits(void* ctx, uint8_t const* ptr, unsigned bit_count)
    {
        static_cast<serial_fixture*>(ctx)->append_lin_input_bits(ptr, bit_count);
        return bit_count;
    }

    void append_lin_input_bits(uint8_t const* ptr, unsigned bit_count)
    {
        size_t index = 0;
        size_t x = ptr[index++];
        while (bit_count >= sizeof(*ptr) * 8) {
            for (unsigned i = sizeof(*ptr) * 8 - 1; i < sizeof(*ptr) * 8; --i) {
                auto bit = (x & (static_cast<decltype (*ptr)>(1) << i)) ? '1' : '0';
                bs_write(&lin_input_bs, this, &static_append_lin_input_byte, bit);
            }

            x = ptr[index++];
            bit_count -= sizeof(*ptr) * 8;
        }

        for (unsigned i = sizeof(*ptr) * 8 - 1; bit_count && i < sizeof(*ptr) * 8; --i, --bit_count) {
            auto bit = (x & (static_cast<decltype (*ptr)>(1) << i)) ? '1' : '0';
            bs_write(&lin_input_bs, this, &static_append_lin_input_byte, bit);
        }
    }
};

serial_fixture* serial_fixture::self;



} // anon

#ifdef __cplusplus
extern "C" {
#endif

bool test_cdc_is_connected(uint8_t itf)
{
    return serial_fixture::self->connected;
}

uint32_t test_cdc_rx_available(uint8_t itf)
{
    switch (itf) {
    case 0:
        return static_cast<uint32_t>(serial_fixture::self->cmd_input.size());
    default:
        return static_cast<uint32_t>(serial_fixture::self->lin_input.size());
    }


}

uint32_t test_cdc_tx_available(uint8_t itf)
{
    switch (itf) {
    case 0:
        if (serial_fixture::self->cmd_output_capacity >= 0) {
            return static_cast<uint32_t>(serial_fixture::self->cmd_output_capacity);
        }
        break;
    default:
        break;
    }


    return ~0;
}

uint32_t test_cdc_rx(uint8_t itf, uint8_t *ptr, uint32_t count)
{
    TEST_ASSERT(ptr);

    switch (itf) {
    case 0: {
        uint32_t bytes = std::min<uint32_t>(count, serial_fixture::self->cmd_input.size());

        if (bytes) {
            memcpy(ptr, serial_fixture::self->cmd_input.data(), bytes);
            serial_fixture::self->cmd_input.erase(0, bytes);
        }

        return bytes;
    } break;
    default: {
        uint32_t bytes = std::min<uint32_t>(count, serial_fixture::self->lin_input.size());

        if (bytes) {
            std::copy(serial_fixture::self->lin_input.begin(), serial_fixture::self->lin_input.begin() + bytes, ptr);
            serial_fixture::self->lin_input.erase(serial_fixture::self->lin_input.begin(), serial_fixture::self->lin_input.begin()+bytes);
        }

        return bytes;
    } break;
    }

    return 0;
}

uint32_t test_cdc_tx(uint8_t itf, uint8_t const *ptr, uint32_t count)
{
    TEST_ASSERT(ptr);

    switch (itf) {
    case 0: {
        if (serial_fixture::self->cmd_output_capacity >= 0) {
            count = std::min<uint32_t>(count, serial_fixture::self->cmd_output_capacity);
        }

        if (count) {
            serial_fixture::self->cmd_output.append(reinterpret_cast<char const*>(ptr), count);
        }

        return count;
    } break;
    default: {
        serial_fixture::self->lin_output.insert(serial_fixture::self->lin_output.end(), ptr, ptr+count);
        return count;
    } break;
    }

    return 0;
}

void test_cdc_tx_flush(uint8_t itf)
{
    ++serial_fixture::self->flush_count;
}

void lp_delay_ms(uint32_t ms)
{
    (void)ms;
}

bool lp_pin_set(uint32_t pin, bool value)
{
    serial_fixture::self->pins_set.emplace_back(decltype (serial_fixture::self->pins_set)::value_type{pin, value});
    return serial_fixture::self->pin_set_success;
}

void test_timer_stop(void)
{
    serial_fixture::self->timer_started = false;
}

void test_timer_start(void)
{
    serial_fixture::self->timer_started = true;
}

void lp_tx_pin_set(void)
{
    serial_fixture::self->tx_pin_values.emplace_back(true);
}

void lp_tx_pin_clear(void)
{
    serial_fixture::self->tx_pin_values.emplace_back(false);
}

bool lp_rx_pin_read(void)
{
    TEST_ASSERT(!serial_fixture::self->rx_pin_values.empty());
    auto value = serial_fixture::self->rx_pin_values.front();
    serial_fixture::self->rx_pin_values.pop_front();
    return value;
}

void lp_version(char* ptr, uint32_t capacity)
{
    TEST_ASSERT(ptr);
    TEST_ASSERT(capacity);

    snprintf(ptr, capacity, "VERSION 42");
}

void test_cdc_rx_clear(uint8_t itf)
{
    ++serial_fixture::self->rx_clear_count;
}

void test_cdc_tx_clear(uint8_t itf)
{
    ++serial_fixture::self->tx_clear_count;
}

#ifdef __cplusplus
} // extern "C"
#endif

#if 0
TEST_F(serial_fixture, init_sets_initial_state)
{
    EXPECT_EQ(1250000u, lp.lin.signal_frequency);
    EXPECT_EQ(LP_CMD_DISCONNECTED, lp.cmd.state);
    EXPECT_EQ(LP_LIN_DISCONNECTED, lp.lin.state);
    EXPECT_EQ(LP_LIN_MODE_OFF, lp.lin.mode);
}

TEST_F(serial_fixture, after_the_host_connects_the_connected_state_is_active)
{
    connect();

    EXPECT_EQ(LP_CMD_CONNECTED, lp.cmd.state);
    EXPECT_EQ(LP_LIN_CONNECTED, lp.lin.state);
    EXPECT_EQ(LP_LIN_MODE_OFF, lp.lin.mode);
}



TEST_F(serial_fixture, cmd_connected_state_accepts_input)
{
    connect();

    cmd_input = "hello";
    lp_cdc_cmd_task();
    EXPECT_STREQ("", cmd_input.c_str());
}

TEST_F(serial_fixture, cmd_connected_state_flushes_usb_tx_buffer_on_lr_or_cr)
{
    connect();

    cmd_input = "\n";
    lp_cdc_cmd_task();
    EXPECT_EQ(1, flush_count);

    cmd_input = "\r";
    lp_cdc_cmd_task();
    EXPECT_EQ(2, flush_count);
}

TEST_F(serial_fixture, cmd_connected_state_processes_input_terminated_with_newline_or_carridge_return_as_command)
{
    connect();
    EXPECT_EQ(0u, cmd_output.size());

    cmd_input = "hello\n";
    lp_cdc_cmd_task();
    EXPECT_LT(0u, cmd_output.size());
    cmd_output.clear();


    cmd_input = "hello\r";
    lp_cdc_cmd_task();
    EXPECT_LT(0u, cmd_output.size());
}

TEST_F(serial_fixture, cmd_connected_state_empty_commands_yield_no_result)
{
    connect();
    EXPECT_EQ(0u, cmd_output.size());

    cmd_input = "\n";
    lp_cdc_cmd_task();
    EXPECT_EQ(0u, cmd_output.size());

    cmd_input = "\r";
    lp_cdc_cmd_task();
    EXPECT_EQ(0u, cmd_output.size());
}

TEST_F(serial_fixture, cmd_connected_state_invalid_commands_return_error_code)
{
    connect();

    cmd_input = "hello\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("-1 "));
}

TEST_F(serial_fixture, cmd_connected_state_can_get_version)
{
    connect();

    cmd_input = "?V\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("0 "));
    EXPECT_LT(2u, cmd_output.size());
}

TEST_F(serial_fixture, cmd_disconnect_in_command_mode_clears_any_pending_cmd_output_message)
{
    connect();

    cmd_output_capacity = 0;
    cmd_input = "hi\n";
    lp_cdc_cmd_task();
    EXPECT_EQ(0, tx_clear_count);

    cmd_output_capacity = -1;
    connected = 0;
    lp_cdc_cmd_task();
    EXPECT_EQ(LP_CMD_DISCONNECTED, lp.cmd.state);
    EXPECT_EQ(1, tx_clear_count);
    EXPECT_EQ(0u, cmd_output.size());
}

TEST_F(serial_fixture, cmd_connected_state_can_get_frequency)
{
    connect();


    cmd_input = "?F\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("0 "));

    const char* start = cmd_output.c_str() + 2;
    char* end = nullptr;
    unsigned long f = std::strtoul(start, &end, 10);
    EXPECT_TRUE(end != nullptr && end != start);
    EXPECT_LT(0u, f);
}

TEST_F(serial_fixture, cmd_connected_state_can_set_frequency)
{
    connect();


    cmd_input = "!F 12345\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("0 "));
    cmd_output.clear();

    cmd_input = "?F\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("0 "));

    const char* start = cmd_output.c_str() + 2;
    char* end = nullptr;
    unsigned long f = std::strtoul(start, &end, 10);
    EXPECT_TRUE(end != nullptr && end != start);
    EXPECT_EQ(12345u, f);
}

TEST_F(serial_fixture, cmd_connected_state_set_frequency_handles_invalid_args)
{
    connect();


    cmd_input = "!F 0\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("-3 "));
    cmd_output.clear();

    cmd_input = "!F 1000000000\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("-3 "));
    cmd_output.clear();

    cmd_input = "!F foo\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("-2 "));
    cmd_output.clear();

    cmd_input = "!F\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("-4 "));
    cmd_output.clear();

    cmd_input = "!F\t\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("-4 "));
    cmd_output.clear();
}

TEST_F(serial_fixture, cmd_connected_state_can_set_pin)
{
    connect();


    cmd_input = "!P 1 0\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("0 "));
    cmd_output.clear();
    ASSERT_EQ(1u, pins_set.size());
    EXPECT_EQ(1, std::get<0>(pins_set[0]));
    EXPECT_EQ(false, std::get<1>(pins_set[0]));

    cmd_input = "!P\t  2  \t1  \n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("0 "));
    cmd_output.clear();
    ASSERT_EQ(2u, pins_set.size());
    EXPECT_EQ(2, std::get<0>(pins_set[1]));
    EXPECT_EQ(true, std::get<1>(pins_set[1]));


    cmd_input = "!P 104 1\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("0 "));
    cmd_output.clear();
    ASSERT_EQ(3u, pins_set.size());
    EXPECT_EQ(104, std::get<0>(pins_set[2]));
    EXPECT_EQ(true, std::get<1>(pins_set[2]));
}

TEST_F(serial_fixture, cmd_connected_state_set_pin_handles_invalid_args)
{
    connect();


    cmd_input = "!P foo\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("-2 "));
    cmd_output.clear();

    cmd_input = "!P\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("-4 "));
    cmd_output.clear();

    cmd_input = "!P\t\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("-4 "));
    cmd_output.clear();
}

TEST_F(serial_fixture, cmd_connected_state_set_lin_mode)
{
    connect();


    cmd_input = "!LMR\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("0 "));
    cmd_output.clear();
    EXPECT_EQ(LP_LIN_MODE_RELAXED, lp.lin.mode);
    EXPECT_EQ(true, timer_started);

    cmd_input = "!LmO\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("0 "));
    cmd_output.clear();
    EXPECT_EQ(LP_LIN_MODE_OFF, lp.lin.mode);
    EXPECT_EQ(false, timer_started);

    cmd_input = "!lms\n";
    lp_cdc_cmd_task();
    EXPECT_THAT(cmd_output, StartsWith("0 "));
    cmd_output.clear();
    EXPECT_EQ(LP_LIN_MODE_STRICT, lp.lin.mode);
    EXPECT_EQ(true, timer_started);
}



TEST_F(serial_fixture, lin_relaxed_mode_outputs_1s_if_no_data_is_present)
{
    connect();
    set_relaxed_mode();

    rx_pin_values.emplace_back(false);
    rx_pin_values.emplace_back(false);


    lp_signal_next_bit();
    lp_signal_next_bit();

    ASSERT_EQ(0, lp.lin.signal_flags);
    ASSERT_EQ(true, timer_started);

    ASSERT_EQ(2u, tx_pin_values.size());
    EXPECT_EQ(true, tx_pin_values[0]);
    EXPECT_EQ(true, tx_pin_values[1]);

}

TEST_F(serial_fixture, lin_relaxed_mode_outputs_input_if_present)
{
    connect();
    set_relaxed_mode();

    rx_pin_values.emplace_back(true);
    rx_pin_values.emplace_back(true);
    rx_pin_values.emplace_back(true);
    rx_pin_values.emplace_back(true);
    rx_pin_values.emplace_back(false);

    append_lin_input("1010");

    lp_cdc_lin_task();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_signal_next_bit();

    ASSERT_EQ(0, lp.lin.signal_flags);
    ASSERT_EQ(true, timer_started);

    ASSERT_EQ(5u, tx_pin_values.size());
    EXPECT_EQ(true, tx_pin_values[0]);
    EXPECT_EQ(false, tx_pin_values[1]);
    EXPECT_EQ(true, tx_pin_values[2]);
    EXPECT_EQ(false, tx_pin_values[3]);
    EXPECT_EQ(true, tx_pin_values[4]);
}


TEST_F(serial_fixture, lin_strict_mode_outputs_received_bits)
{
    connect();
    set_strict_mode();

    rx_pin_values.emplace_back(true);
    rx_pin_values.emplace_back(false);

    set_lin_input("01");
    lp_cdc_lin_task();
    lp_signal_next_bit();
    lp_signal_next_bit();

    EXPECT_EQ(true, timer_started);
    ASSERT_EQ(2u, tx_pin_values.size());
    EXPECT_EQ(false, tx_pin_values[0]);
    EXPECT_EQ(true, tx_pin_values[1]);
}

TEST_F(serial_fixture, lin_strict_mode_gracefully_terminates_output_when_reading_eos)
{
    connect();
    set_strict_mode();

    rx_pin_values.emplace_back(true);
    rx_pin_values.emplace_back(false);
    rx_pin_values.emplace_back(true);

    set_lin_input("01");
    lp_cdc_lin_task();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_signal_next_bit();

    EXPECT_EQ(false, timer_started);
    EXPECT_EQ(OUTPUT_FLAG_OUTPUT_DONE, lp.lin.signal_flags);
    ASSERT_EQ(2u, tx_pin_values.size());
    EXPECT_EQ(false, tx_pin_values[0]);
    EXPECT_EQ(true, tx_pin_values[1]);
}

TEST_F(serial_fixture, lin_strict_mode_sets_flags_on_tx_stall)
{
    connect();
    set_strict_mode();

    rx_pin_values.emplace_back(true);
    rx_pin_values.emplace_back(false);
    rx_pin_values.emplace_back(false);
    rx_pin_values.emplace_back(false);
    rx_pin_values.emplace_back(false);
    rx_pin_values.emplace_back(false);
    rx_pin_values.emplace_back(false);
    rx_pin_values.emplace_back(false);
    rx_pin_values.emplace_back(true);

    append_lin_input("01010101");
    lp_cdc_lin_task();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_signal_next_bit();
    lp_cdc_lin_task();

    EXPECT_EQ(false, timer_started);
    EXPECT_EQ(OUTPUT_FLAG_OUTPUT_DONE | OUTPUT_FLAG_TX_STALLED, lp.lin.signal_flags);
    EXPECT_EQ(LP_LIN_CONNECTED, lp.lin.state);
    ASSERT_EQ(8u, tx_pin_values.size());
    EXPECT_EQ(false, tx_pin_values[0]);
    EXPECT_EQ(true, tx_pin_values[1]);
    EXPECT_EQ(false, tx_pin_values[2]);
    EXPECT_EQ(true, tx_pin_values[3]);
    EXPECT_EQ(false, tx_pin_values[4]);
    EXPECT_EQ(true, tx_pin_values[5]);
    EXPECT_EQ(false, tx_pin_values[6]);
    EXPECT_EQ(true, tx_pin_values[7]);

}

#endif
