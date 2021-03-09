#include <gtest/gtest.h>
#include <gmock/gmock.h>


#include <linchpin.h>
#include "../inc/fastlz.h"


#define BASE64_C
#include <base64.h>

#include <string>
#include <algorithm>
#include <deque>
#include <vector>
#include <cstring>

using ::testing::StartsWith;

namespace
{

struct serial_fixture : public ::testing::Test
{
    static serial_fixture* self;

    bool connected;
    int pin;
    int output_capacity;
    std::string input;
    std::string output;
    std::vector<std::tuple<int, bool>> pins_set;
    std::deque<bool> tx_pin_values;
    std::deque<bool> rx_pin_values;
    bool pin_set_success;
    bool timer_started;
    int flush_count;


    ~serial_fixture()
    {
        self = nullptr;
    }

    serial_fixture()
    {
        self = this;

        connected = false;
        pin = -1;
        output_capacity = -1;
        pin_set_success = true;
        timer_started = false;
        flush_count = 0;

        lp_init();
    }

    void connect()
    {
        connected = true;
        lp_cdc_task();
    }

    void run()
    {
        connected = true;
        lp_cdc_task();
        input = "!R\n";
        lp_cdc_task();
        output.clear();
    }

    static std::string make_input(const char* str)
    {
        std::vector<char> str_vec(str, str + std::strlen(str));
        if (str_vec.size() & 15) {
            str_vec.reserve(str_vec.size() + 16);

            while (str_vec.size() & 15) {
                str_vec.emplace_back(0);
            }
        }

        std::vector<char> compressed;
        compressed.resize(str_vec.size() * 2);
        auto bytes = fastlz_compress(str_vec.data(), str_vec.size(), compressed.data());
        compressed.resize(bytes);

        str_vec.resize(2*compressed.size());
        struct base64_state s;
        base64_init(&s);
        size_t gi = 0;
        size_t pi = 0;
        for (auto it = begin(compressed), e = end(compressed); it != e; ++it) {
            base64_encode_shift(
                *it,
                &s,
                &gi,
                &pi,
                (uint8_t*)str_vec.data(),
                str_vec.size());
        }

        base64_encode_finalize(
            &s,
            &gi,
            &pi,
            (uint8_t*)str_vec.data(),
            str_vec.size());

        return std::string(str_vec.data(), pi);
    }

};

serial_fixture* serial_fixture::self;



} // anon

#ifdef __cplusplus
extern "C" {
#endif

bool test_cdc_is_connected(void)
{
    return serial_fixture::self->connected;
}

uint32_t test_cdc_rx_available(void)
{
    return static_cast<uint32_t>(serial_fixture::self->input.size());
}

uint32_t test_cdc_tx_available(void)
{
    if (serial_fixture::self->output_capacity >= 0) {
        return static_cast<uint32_t>(serial_fixture::self->output_capacity);
    }

    return ~0;
}

uint32_t test_cdc_rx(uint8_t *ptr, uint32_t count)
{
    TEST_ASSERT(ptr);

    uint32_t bytes = std::min<uint32_t>(count, serial_fixture::self->input.size());

    if (bytes) {
        memcpy(ptr, serial_fixture::self->input.data(), bytes);
        serial_fixture::self->input.erase(0, bytes);
    }

    return bytes;
}

uint32_t test_cdc_tx(uint8_t const *ptr, uint32_t count)
{
    TEST_ASSERT(ptr);

    if (serial_fixture::self->output_capacity >= 0) {
        count = std::min<uint32_t>(count, serial_fixture::self->output_capacity);
    }

    if (count) {
        serial_fixture::self->output.append(reinterpret_cast<char const*>(ptr), count);
    }

    return count;
}

void test_cdc_tx_flush(void)
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

void test_cdc_rx_clear(void)
{

}

#ifdef __cplusplus
} // extern "C"
#endif

TEST_F(serial_fixture, init_sets_initial_state)
{
    EXPECT_EQ(1250000, lp.signal_frequency);
    EXPECT_EQ(LP_DISCONNECTED, lp.state);
}

TEST_F(serial_fixture, after_the_host_connects_the_connected_state_is_active)
{
    connect();

    EXPECT_EQ(LP_CONNECTED, lp.state);
}

TEST_F(serial_fixture, connected_state_accepts_input)
{
    connect();

    input = "hello";
    lp_cdc_task();
    EXPECT_STREQ("", input.c_str());
}

TEST_F(serial_fixture, connected_state_flushes_usb_tx_buffer_on_lr_or_cr)
{
    connect();

    input = "\n";
    lp_cdc_task();
    EXPECT_EQ(1, flush_count);

    input = "\r";
    lp_cdc_task();
    EXPECT_EQ(2, flush_count);
}

TEST_F(serial_fixture, connected_state_processes_input_terminated_with_newline_or_carridge_return_as_command)
{
    connect();
    EXPECT_EQ(0, output.size());

    input = "hello\n";
    lp_cdc_task();
    EXPECT_LT(0, output.size());
    output.clear();


    input = "hello\r";
    lp_cdc_task();
    EXPECT_LT(0, output.size());
}

TEST_F(serial_fixture, connected_state_empty_commands_yield_no_result)
{
    connect();
    EXPECT_EQ(0, output.size());

    input = "\n";
    lp_cdc_task();
    EXPECT_EQ(0, output.size());

    input = "\r";
    lp_cdc_task();
    EXPECT_EQ(0, output.size());
}

TEST_F(serial_fixture, connected_state_invalid_commands_return_error_code)
{
    connect();

    input = "hello\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("-1 "));
}

TEST_F(serial_fixture, connected_state_can_get_version)
{
    connect();

    input = "?V\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("0 "));
    EXPECT_LT(2, output.size());
}

TEST_F(serial_fixture, connected_state_can_get_frequency)
{
    connect();


    input = "?F\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("0 "));

    const char* start = output.c_str() + 2;
    char* end = nullptr;
    unsigned long f = std::strtoul(start, &end, 10);
    EXPECT_TRUE(end != nullptr && end != start);
    EXPECT_LT(0, f);
}

TEST_F(serial_fixture, connected_state_can_set_frequency)
{
    connect();


    input = "!F 12345\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("0 "));
    output.clear();

    input = "?F\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("0 "));

    const char* start = output.c_str() + 2;
    char* end = nullptr;
    unsigned long f = std::strtoul(start, &end, 10);
    EXPECT_TRUE(end != nullptr && end != start);
    EXPECT_EQ(12345, f);
}

TEST_F(serial_fixture, connected_state_set_frequency_handles_invalid_args)
{
    connect();


    input = "!F 0\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("-3 "));
    output.clear();

    input = "!F 1000000000\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("-3 "));
    output.clear();

    input = "!F foo\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("-2 "));
    output.clear();

    input = "!F\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("-4 "));
    output.clear();

    input = "!F\t\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("-4 "));
    output.clear();
}

TEST_F(serial_fixture, connected_state_can_set_pin)
{
    connect();


    input = "!P 1 0\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("0 "));
    output.clear();
    ASSERT_EQ(1, pins_set.size());
    EXPECT_EQ(1, std::get<0>(pins_set[0]));
    EXPECT_EQ(false, std::get<1>(pins_set[0]));

    input = "!P\t  2  \t1  \n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("0 "));
    output.clear();
    ASSERT_EQ(2, pins_set.size());
    EXPECT_EQ(2, std::get<0>(pins_set[1]));
    EXPECT_EQ(true, std::get<1>(pins_set[1]));


}

TEST_F(serial_fixture, connected_state_set_pin_handles_invalid_args)
{
    connect();


    input = "!P foo\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("-2 "));
    output.clear();

    input = "!P\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("-4 "));
    output.clear();

    input = "!P\t\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("-4 "));
    output.clear();
}



TEST_F(serial_fixture, connected_state_can_switch_to_running)
{
    connect();

    input = "!R\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("0 "));
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(LP_RUN_REQUESTED, lp.run_state);
    EXPECT_EQ(false, timer_started);
}

TEST_F(serial_fixture, running_state_starts_with_first_data_packet)
{
    run();

    input = make_input("f") + "\n";
    fprintf(stdout, "%s\n", input.c_str());
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(true, timer_started);
    EXPECT_EQ(0, lp.signal_flags);
}

TEST_F(serial_fixture, running_state_ignores_line_breaks)
{
    run();

    input = "f";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_EQ(true, timer_started);

    input = "\r\n";
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_EQ(true, timer_started);
}

TEST_F(serial_fixture, running_state_finishes_on_invalid_chars)
{
    run();

    input = "f";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_EQ(true, timer_started);

    input = "\x80";
    lp_cdc_task();
    EXPECT_EQ(LP_CONNECTED, lp.state);
    EXPECT_EQ(false, timer_started);
    EXPECT_STREQ("\n-5 USBRXB64INV \n", output.c_str());
}

TEST_F(serial_fixture, running_state_finishes_on_tx_stall)
{
    run();

    input = "f";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_EQ(true, timer_started);

    lp.signal_flags |= OUTPUT_FLAG_TX_STALLED;

    lp_cdc_task();
    EXPECT_EQ(LP_CONNECTED, lp.state);
    EXPECT_EQ(false, timer_started);
    EXPECT_STREQ("\n-5 SIGTXSTALL \n", output.c_str());
}

TEST_F(serial_fixture, running_state_finishes_on_rx_overflow)
{
    run();

    input = "f";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_EQ(true, timer_started);

    lp.signal_flags |= OUTPUT_FLAG_RX_OVERFLOW;

    lp_cdc_task();
    EXPECT_EQ(LP_CONNECTED, lp.state);
    EXPECT_EQ(false, timer_started);
    EXPECT_STREQ("\n-5 SIGRXOVRFL \n", output.c_str());
}

TEST_F(serial_fixture, running_state_reads_input)
{
    run();

    input = "f";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_EQ(true, timer_started);

    lp.signal_rx_buffer_pi = 4;
    lp.signal_rx_buffer_gi = 0;
    lp.signal_rx_buffer[0] = 0xde;
    lp.signal_rx_buffer[1] = 0xad;
    lp.signal_rx_buffer[2] = 0xbe;
    lp.signal_rx_buffer[3] = 0xef;

    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_STREQ("3q2+7", output.c_str());
}

TEST_F(serial_fixture, running_state_returns_to_connected_when_input_terminates)
{
    run();

    input = "AYEBgQ==\n";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
//    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_EQ(true, timer_started);
    EXPECT_TRUE(lp.signal_flags & OUTPUT_FLAG_INPUT_DONE);

    const int runs = 4;

    rx_pin_values.emplace_back(true);
    for (int i = 0; i < runs; ++i) {
        rx_pin_values.emplace_back(true);
    }

    lp_signal_next_bit(); // load run
    lp_cdc_task();

    for (int i = 0; i < runs; ++i) {
        lp_signal_next_bit();
        lp_cdc_task();
    }


    EXPECT_EQ(LP_CONNECTED, lp.state);
    EXPECT_EQ(LP_RUN_STOPPED, lp.run_state);
    EXPECT_STREQ("hA==\n0 \n", output.c_str());
    ASSERT_EQ(4, tx_pin_values.size());
    EXPECT_EQ(false, tx_pin_values[0]);
    EXPECT_EQ(true, tx_pin_values[1]);
    EXPECT_EQ(false, tx_pin_values[2]);
    EXPECT_EQ(true, tx_pin_values[3]);
}

TEST_F(serial_fixture, running_state_handles_more_that_one_usb_buffer_worth_of_data)
{
    run();

    // 40x 0 -> 1 -> 0 ...
    input = "AYEBgQGBAYEBgQGBAYEBgQGBAYEBgQGBAYEBgQGBAYEBgQGBAYEBgQ==";
    lp_cdc_task();
//    EXPECT_EQ(LP_RUNNING, lp.state);

    const int runs = 40;

    rx_pin_values.emplace_back(true);
    for (int i = 0; i < runs; ++i) {
        rx_pin_values.emplace_back(true);
    }

    lp_signal_next_bit(); // load run
    lp_cdc_task();

    for (int i = 0; i < runs; ++i) {
        lp_signal_next_bit();
        lp_cdc_task();
    }


    EXPECT_EQ(LP_CONNECTED, lp.state);
    EXPECT_EQ(LP_RUN_STOPPED, lp.run_state);
    EXPECT_STREQ("qA==\n0 \n", output.c_str());
    ASSERT_EQ(40, tx_pin_values.size());
    for (int i = 0; i < runs; ++i) {
        EXPECT_EQ(i & 1, tx_pin_values[i]);
    }
}

//TEST_F(serial_fixture, a_new_run_can_started_after_tx_stall)
//{
//    run();

//    // must be larger than size of USB rx buffer
//    input = "dead0000000000000000000000000000000000000000000000000000000000";
//    lp_cdc_task();
//    EXPECT_EQ(LP_RUNNING, lp.state);
//    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
//    EXPECT_EQ(true, timer_started);
//    EXPECT_STREQ("", input.c_str());

//    lp.signal_flags |= OUTPUT_FLAG_TX_STALLED;

//    lp_cdc_task();
//    EXPECT_EQ(LP_CONNECTED, lp.state);
//    EXPECT_EQ(false, timer_started);
//    EXPECT_STREQ("\n-5 SIGTXSTALL \n", output.c_str());

//    input = "!R\n";
//    lp_cdc_task();
//    EXPECT_EQ(LP_RUNNING, lp.state);
//}


