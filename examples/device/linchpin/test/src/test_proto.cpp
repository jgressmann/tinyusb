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
        if (str_vec.size() % FASTLZ_MIN_INPUT_SIZE) {
            str_vec.reserve(str_vec.size() + FASTLZ_MIN_INPUT_SIZE);

            while (str_vec.size() % FASTLZ_MIN_INPUT_SIZE) {
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

    bool output_has_error_code(int code) const
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%d ", code);
        return output.find(buf) != std::string::npos;
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
//    if (-1 == serial_fixture::self->tx_bit || !serial_fixture::self->tx_bit) {
//        serial_fixture::self->tx_bit = true;
//        serial_fixture::self->tx_pin_values.emplace_back(true);
//    }
}

void lp_tx_pin_clear(void)
{
    serial_fixture::self->tx_pin_values.emplace_back(false);
//    if (-1 == serial_fixture::self->tx_bit || serial_fixture::self->tx_bit) {
//        serial_fixture::self->tx_bit = false;
//        serial_fixture::self->tx_pin_values.emplace_back(false);
//    }
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
    EXPECT_EQ(1250000u, lp.signal_frequency);
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
    EXPECT_EQ(0u, output.size());

    input = "hello\n";
    lp_cdc_task();
    EXPECT_LT(0u, output.size());
    output.clear();


    input = "hello\r";
    lp_cdc_task();
    EXPECT_LT(0u, output.size());
}

TEST_F(serial_fixture, connected_state_empty_commands_yield_no_result)
{
    connect();
    EXPECT_EQ(0u, output.size());

    input = "\n";
    lp_cdc_task();
    EXPECT_EQ(0u, output.size());

    input = "\r";
    lp_cdc_task();
    EXPECT_EQ(0u, output.size());
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
    EXPECT_LT(2u, output.size());
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
    EXPECT_LT(0u, f);
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
    EXPECT_EQ(12345u, f);
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
    ASSERT_EQ(1u, pins_set.size());
    EXPECT_EQ(1, std::get<0>(pins_set[0]));
    EXPECT_EQ(false, std::get<1>(pins_set[0]));

    input = "!P\t  2  \t1  \n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("0 "));
    output.clear();
    ASSERT_EQ(2u, pins_set.size());
    EXPECT_EQ(2, std::get<0>(pins_set[1]));
    EXPECT_EQ(true, std::get<1>(pins_set[1]));


    input = "!P 104 1\n";
    lp_cdc_task();
    EXPECT_THAT(output, StartsWith("0 "));
    output.clear();
    ASSERT_EQ(3u, pins_set.size());
    EXPECT_EQ(104, std::get<0>(pins_set[2]));
    EXPECT_EQ(true, std::get<1>(pins_set[2]));
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
    //fprintf(stdout, "%s\n", input.c_str());
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(true, timer_started);
    EXPECT_EQ(OUTPUT_FLAG_INPUT_DONE, lp.signal_flags);
}


TEST_F(serial_fixture, running_state_finishes_on_invalid_chars)
{
    run();

    input = make_input("f") + "!";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_EQ(true, timer_started);

    input = "\x80";
    lp_cdc_task();
    EXPECT_EQ(LP_CONNECTED, lp.state);
    EXPECT_EQ(false, timer_started);
    EXPECT_TRUE(output_has_error_code(LP_ERROR_MALFORMED));
//    EXPECT_STREQ("-4 USBRXB64INV \n", output.c_str());
}

TEST_F(serial_fixture, running_state_aborts_on_too_large_of_an_input_packet)
{
    run();

    input = make_input("1234567890qwertyuiop[]\asdfghjkl;źxcvbnm,./!@#$%&*()_+QWERTYUIOP{}|ASDFGHJKL:ZXCVBNM<>?") + "\n";
    lp_cdc_task();
    EXPECT_EQ(LP_CONNECTED, lp.state);
    EXPECT_EQ(false, timer_started);
    EXPECT_TRUE(output_has_error_code(LP_ERROR_MALFORMED));
}

TEST_F(serial_fixture, running_state_finishes_on_tx_stall)
{
    run();

    input = make_input("f") + "\r";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_EQ(true, timer_started);

    lp.signal_flags |= OUTPUT_FLAG_TX_STALLED | OUTPUT_FLAG_OUTPUT_DONE;

    lp_cdc_task();
    EXPECT_EQ(LP_CONNECTED, lp.state);
//    EXPECT_EQ(false, timer_started);
    EXPECT_STREQ("\n-5 SIGTXSTALL \n", output.c_str());
}

TEST_F(serial_fixture, running_state_finishes_on_rx_overflow)
{
    run();

    input = make_input("f") + "\r";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_EQ(true, timer_started);

    lp.signal_flags |= OUTPUT_FLAG_RX_OVERFLOW | OUTPUT_FLAG_OUTPUT_DONE;

    lp_cdc_task();
    EXPECT_EQ(LP_CONNECTED, lp.state);
//    EXPECT_EQ(false, timer_started);
    EXPECT_STREQ("\n-5 SIGRXOVRFL \n", output.c_str());
}

//TEST_F(serial_fixture, running_state_reads_input)
//{
//    run();

//    input = make_input("f") + "\r";
//    lp_cdc_task();
//    EXPECT_EQ(LP_RUNNING, lp.state);
//    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
//    EXPECT_EQ(true, timer_started);

//    lp.signal_rx_buffer_pi = 4;
//    lp.signal_rx_buffer_gi = 0;
//    lp.signal_rx_buffer[0] = 0xde;
//    lp.signal_rx_buffer[1] = 0xad;
//    lp.signal_rx_buffer[2] = 0xbe;
//    lp.signal_rx_buffer[3] = 0xef;

//    lp_cdc_task();
//    EXPECT_EQ(LP_CONNECTED, lp.state);
////    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
//    EXPECT_STREQ("3q2+7", output.c_str());
//}

TEST_F(serial_fixture, running_state_returns_to_connected_when_input_terminates)
{
    run();

    input = make_input("f") + "\r";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
//    EXPECT_EQ(LP_RUN_STARTED, lp.run_state);
    EXPECT_EQ(true, timer_started);
    EXPECT_TRUE(lp.signal_flags & OUTPUT_FLAG_INPUT_DONE);

    const int runs = 128; // best guess

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
    EXPECT_TRUE(!output.empty());
}

TEST_F(serial_fixture, running_state_outputs_the_expected_bits)
{
    run();

    input = make_input("\x81\x81\x81\x81\x01\x01\x81\x01\x81\x01\x01\x01\x01\x01") + "\r";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);
    EXPECT_EQ(true, timer_started);
    EXPECT_TRUE(lp.signal_flags & OUTPUT_FLAG_INPUT_DONE);

    const int runs = 128; // best guess

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
    ASSERT_EQ(14u, tx_pin_values.size());
    EXPECT_EQ(true, tx_pin_values[0]);
    EXPECT_EQ(true, tx_pin_values[1]);
    EXPECT_EQ(true, tx_pin_values[2]);
    EXPECT_EQ(true, tx_pin_values[3]);
    EXPECT_EQ(false, tx_pin_values[4]);
    EXPECT_EQ(false, tx_pin_values[5]);
    EXPECT_EQ(true, tx_pin_values[6]);
    EXPECT_EQ(false, tx_pin_values[7]);
    EXPECT_EQ(true, tx_pin_values[8]);
    EXPECT_EQ(false, tx_pin_values[9]);
    EXPECT_EQ(false, tx_pin_values[10]);
    EXPECT_EQ(false, tx_pin_values[11]);
    EXPECT_EQ(false, tx_pin_values[12]);
    EXPECT_EQ(false, tx_pin_values[13]);
}

TEST_F(serial_fixture, running_state_handles_more_that_one_packet_worth_of_data)
{
    run();

    input = make_input("\x81") + "!" + make_input("\x01") + "\n";
    lp_cdc_task();
    EXPECT_EQ(LP_RUNNING, lp.state);

    const int runs = 2;

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
    EXPECT_TRUE(!output.empty());
    ASSERT_EQ(2u, tx_pin_values.size());
    EXPECT_EQ(true, tx_pin_values[0]);
    EXPECT_EQ(false, tx_pin_values[1]);
}

TEST_F(serial_fixture, running_state_aborts_on_some_packet_being_too_large)
{
    run();

    input = make_input("\x81") + "!" + make_input("1234567890qwertyuiop[]\asdfghjkl;źxcvbnm,./!@#$%&*()_+QWERTYUIOP{}|ASDFGHJKL:ZXCVBNM<>?") + "\n";
    lp_cdc_task();
    EXPECT_EQ(LP_CONNECTED, lp.state);
    EXPECT_EQ(false, timer_started);
    EXPECT_TRUE(output_has_error_code(LP_ERROR_MALFORMED));
}
