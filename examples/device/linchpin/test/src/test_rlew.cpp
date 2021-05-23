#include <gtest/gtest.h>

#define STR2(x) #x
#define STR(x) STR2(x)

#define RLEW_ASSERT(x) if (!(x)) throw std::runtime_error(__FILE__ ":" STR(__LINE__))
#define RLEW_H
#define RLEW_C
#define RLEW_INT_TYPE uint32_t
#define RLEW_STATIC
#include <rlew.h>
//#undef RLEW_INT_TYPE

#include <string>
#include <cstring>
#include <deque>
#include <cstdio>



namespace
{

struct rlew_fixture : public ::testing::Test
{
    struct rlew_decoder d;
    struct rlew_encoder e;

    std::string output;
    std::deque<char> input;
    int write_error;
    int read_error;


    ~rlew_fixture()
    {

    }

    rlew_fixture()
    {
        rlew_dec_init(&d);
        rlew_enc_init(&e);
        write_error = std::numeric_limits<int>::min();
        read_error = std::numeric_limits<int>::min();
    }

    void set_input(char const *str)
    {
        size_t len = std::strlen(str);
        input.assign(str, str+len);
        while (input.size() & (sizeof(RLEW_INT_TYPE) * 8 - 1)) {
            input.push_back('0');
        }
    }

    void add_to_output(RLEW_INT_TYPE value)
    {
        output.reserve(output.size() + sizeof(value) * 8);

        for (size_t i = sizeof(value) * 8 - 1; i < sizeof(value) * 8; --i) {
            output += (value & (static_cast<RLEW_INT_TYPE>(1) << i)) ? '1' : '0';
        }
    }

    static int static_read(void* ctx, RLEW_INT_TYPE *ptr)
    {
        return static_cast<rlew_fixture*>(ctx)->read(ptr);
    }

    int read(RLEW_INT_TYPE *ptr)
    {
        if (read_error != std::numeric_limits<int>::min()) {
            return read_error;
        }

        auto input_size = input.size();
        if (input_size < sizeof(*ptr) * 8) {
            return 1;
        }

        for (unsigned i = 0; i < sizeof(*ptr) * 8; ++i) {
            RLEW_INT_TYPE bit = input[0] == '1' ? 1 : 0;
            input.pop_front();
            *ptr <<= 1;
            *ptr |= bit;
        }

        return 0;
    }

    static int static_write(void* ctx, RLEW_INT_TYPE value)
    {
        return static_cast<rlew_fixture*>(ctx)->write(value);
    }

    int write(RLEW_INT_TYPE value)
    {
        if (write_error != std::numeric_limits<int>::min()) {
            return write_error;
        }

        for (unsigned i = sizeof(value) * 8 - 1; i < sizeof(value) * 8; --i) {
            output += (value & (static_cast<RLEW_INT_TYPE>(1) << i)) ? '1' : '0';
        }

        return 0;
    }

    void dump_ints()
    {
        size_t int_count = output.size() / 32;
        size_t offset = 0;
        for (size_t i = 0; i < int_count; ++i) {
            uint32_t value = 0;
            for (size_t j = 0; j < 32; ++j, ++offset) {
                uint32_t bit = output[offset] == '1';
                bit <<= 31 - j;
                value |= bit;
            }

            fprintf(stdout, "%u. %08x %u\n", (unsigned)i, value, value);
        }
    }
};



} // anon

TEST(rlew, init_asserts_pointers)
{
    EXPECT_ANY_THROW(rlew_dec_init(NULL));
    EXPECT_ANY_THROW(rlew_enc_init(NULL));
}

TEST(rlew, init_sets_default_values)
{
    struct rlew_decoder d;
    struct rlew_encoder e;
    rlew_dec_init(&d);
    rlew_enc_init(&e);
    EXPECT_EQ(0u, d.count);
    EXPECT_EQ(0u, d.input_gi);
    EXPECT_EQ(0u, d.input_pi);

    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(0u, e.output_gi);
    EXPECT_EQ(0u, e.output_pi);
}

//TEST_F(rlew_fixture, rlew_enc_flush_asserts_params)
//{
//    EXPECT_ANY_THROW(rlew_enc_flush(NULL));
//}

//TEST_F(rlew_fixture, rlew_enc_flush_is_a_nop_if_count_is_zero)
//{
//    EXPECT_EQ(0u, e.count);
//    rlew_enc_flush(&e);
//    EXPECT_EQ(0u, output.size());
//}

TEST_F(rlew_fixture, rlew_enc_bit_asserts_params)
{
    EXPECT_ANY_THROW(rlew_enc_bit(NULL, 1));
}

TEST_F(rlew_fixture, encode_0_yields_expected_output)
{
    EXPECT_EQ(RLEW_ERROR_NONE, rlew_enc_bit(&e, 0));
    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(1u, e.count);

    rlew_enc_flush(&e);
    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(1u, e.state);
}

TEST_F(rlew_fixture, encode_1_yields_expected_output)
{
    EXPECT_EQ(RLEW_ERROR_NONE, rlew_enc_bit(&e, 1));

    EXPECT_EQ(1u, e.value);
    EXPECT_EQ(1u, e.count);

    rlew_enc_flush(&e);
    EXPECT_EQ(0u, e.count);

    EXPECT_EQ(0x0002u, e.state);
}

//TEST_F(rlew_fixture, encode_00_yields_expected_output)
//{
//    rlew_enc_bit(&e, this, &static_write, 0);
//    rlew_enc_bit(&e, this, &static_write, 0);

//    EXPECT_EQ(0u, e.value);
//    EXPECT_EQ(2u, e.count);


//    rlew_enc_flush(&e, this, &static_write);
//    EXPECT_EQ(0u, e.count);

//    EXPECT_EQ(0x0002u, e.state);
//}

//TEST_F(rlew_fixture, encode_11_yields_expected_output)
//{
//    rlew_enc_bit(&e, this, &static_write, 1);
//    rlew_enc_bit(&e, this, &static_write, 1);


//    EXPECT_EQ(1u, e.value);
//    EXPECT_EQ(2u, e.count);


//    rlew_enc_flush(&e, this, &static_write);
//    EXPECT_EQ(0u, e.count);

//    EXPECT_EQ(0x000cu, e.state);
//}

//TEST_F(rlew_fixture, encode_000_yields_expected_output)
//{
//    rlew_enc_bit(&e, this, &static_write, 0);
//    rlew_enc_bit(&e, this, &static_write, 0);
//    rlew_enc_bit(&e, this, &static_write, 0);

//    EXPECT_EQ(0u, e.value);
//    EXPECT_EQ(3u, e.count);


//    rlew_enc_flush(&e, this, &static_write);
//    EXPECT_EQ(0u, e.count);

//    EXPECT_EQ(0x0003u, e.state);
//}

//TEST_F(rlew_fixture, encode_0000_yields_expected_output)
//{
//    for (int i = 0; i < 4; ++i) {
//        rlew_enc_bit(&e, this, &static_write, 0);
//    }

//    EXPECT_EQ(0u, e.value);
//    EXPECT_EQ(4u, e.count);


//    rlew_enc_flush(&e, this, &static_write);
//    EXPECT_EQ(0u, e.count);

//    EXPECT_EQ(0x0004u, e.state);
//}

//TEST_F(rlew_fixture, encode_0000000_yields_expected_output)
//{
//    const size_t LIMIT = 7;
//    for (size_t i = 0; i < LIMIT; ++i) {
//        rlew_enc_bit(&e, this, &static_write, 0);
//    }

//    EXPECT_EQ(0u, e.value);
//    EXPECT_EQ(LIMIT, e.count);


//    rlew_enc_flush(&e, this, &static_write);
//    EXPECT_EQ(0u, e.count);

//    EXPECT_EQ(0x0007u, e.state);
//}

TEST_F(rlew_fixture, encode_finish_is_a_nop_if_no_count_or_used_state_bits)
{
    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(0u, e.used);
    EXPECT_EQ(RLEW_ERROR_NONE, rlew_enc_finish(&e));
}

TEST_F(rlew_fixture, encode_finish_aligns_and_stores_current_state_then_terminates_with_zeros)
{
    EXPECT_EQ(RLEW_ERROR_NONE, rlew_enc_bit(&e, 1));
    EXPECT_EQ(1u, e.count);

    EXPECT_EQ(RLEW_ERROR_OVERFLOW, rlew_enc_finish(&e));
    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(0u, e.used);

    while (e.output_gi != e.output_pi) {
        add_to_output(e.output_buffer[e.output_gi++ % RLEW_ARRAY_SIZE(e.output_buffer)]);
    }

    EXPECT_EQ(RLEW_ERROR_NONE, rlew_enc_finish(&e));

    if (1 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("10000000", output.c_str());
    } else if (2 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("1000000000000000", output.c_str());
    } else {
        EXPECT_STREQ("10000000000000000000000000000000", output.c_str());
    }
}

TEST_F(rlew_fixture, encode_0xLIMIT_yields_expected_output)
{
    e.value = 0;
    e.count = RLEW_MAX_COUNT - 1;
    EXPECT_EQ(RLEW_ERROR_NONE, rlew_enc_bit(&e, 0));

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(RLEW_MAX_COUNT, e.count);

    EXPECT_EQ(RLEW_ERROR_OVERFLOW, rlew_enc_finish(&e));
    EXPECT_EQ(0u, e.count);

    while (e.output_gi != e.output_pi) {
        add_to_output(e.output_buffer[e.output_gi++ % RLEW_ARRAY_SIZE(e.output_buffer)]);
    }

    EXPECT_EQ(RLEW_ERROR_OVERFLOW, rlew_enc_finish(&e));
    EXPECT_EQ(0u, e.count);

    while (e.output_gi != e.output_pi) {
        add_to_output(e.output_buffer[e.output_gi++ % RLEW_ARRAY_SIZE(e.output_buffer)]);
    }

    EXPECT_EQ(RLEW_ERROR_NONE, rlew_enc_finish(&e));

    EXPECT_EQ(0u, e.used);


    if (1 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("0000000111111111", output.c_str());
    } else if (2 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("00000000000000001111111111111111", output.c_str());
    } else if (4 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("0000000000000000000000000000000111111111111111111111111111111100", output.c_str());
    } else {
        FAIL();
    }
}

TEST_F(rlew_fixture, encode_1xLIMIT_yields_expected_output)
{
    e.value = 1;
    e.count = RLEW_MAX_COUNT - 1;
    EXPECT_EQ(RLEW_ERROR_NONE, rlew_enc_bit(&e, 1));

    EXPECT_EQ(1u, e.value);
    EXPECT_EQ(RLEW_MAX_COUNT, e.count);

    EXPECT_EQ(RLEW_ERROR_OVERFLOW, rlew_enc_finish(&e));
    EXPECT_EQ(0u, e.count);

    while (e.output_gi != e.output_pi) {
        add_to_output(e.output_buffer[e.output_gi++ % RLEW_ARRAY_SIZE(e.output_buffer)]);
    }

    EXPECT_EQ(RLEW_ERROR_OVERFLOW, rlew_enc_finish(&e));
    EXPECT_EQ(0u, e.count);

    while (e.output_gi != e.output_pi) {
        add_to_output(e.output_buffer[e.output_gi++ % RLEW_ARRAY_SIZE(e.output_buffer)]);
    }

    EXPECT_EQ(RLEW_ERROR_NONE, rlew_enc_finish(&e));

    EXPECT_EQ(0u, e.used);


    if (1 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("", output.c_str());
    } else if (2 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("", output.c_str());
    } else if (4 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("1111111111111111111111111111111011111111111111111111111111111100", output.c_str());
    } else {
        FAIL();
    }
}



TEST_F(rlew_fixture, encode_output_is_correct_for_all_valid_offsets_and_all_bit_counts)
{
    std::string prefix;
    for (size_t count_bits = 0; count_bits < RLEW_MAX_BIT_COUNT; ++count_bits) {
        for (size_t offset = 0; offset < sizeof(RLEW_INT_TYPE) * 8; offset += 2) {
            for (size_t value = 0; value <= 1; ++value) {
                for (size_t counter = 0; counter < 2; ++counter) {
                    for (size_t prefix_value = 0; prefix_value <= 1; ++prefix_value) {
                        prefix.clear();
                        output.clear();
                        rlew_enc_init(&e);

                        for (size_t j = 0; j < offset; j += 2) {
                            EXPECT_EQ(RLEW_ERROR_NONE, rlew_enc_bit(&e, prefix_value));
                            rlew_enc_flush(&e);

                            EXPECT_EQ(j + 2, e.used);

                            if (prefix_value) {
                                prefix += "10";
                            } else {
                                prefix += "01";
                            }
                        }

                        if (value) {
                            prefix += '1';

                        } else {
                            prefix += '0';
                        }

                        e.value = value;
                        e.count = 1;

                        if (count_bits) {
                            e.count <<= count_bits;
                            e.count += counter;

                            // count bits of value char
                            for (size_t c = 0; c < count_bits; ++c) {
                                prefix += value ? '1' : '0';
                            }

                            // inverted char
                            prefix += value ? '0' : '1';

                            // counter value
                            for (size_t c = 0; c < count_bits; ++c) {
                                prefix += '0';
                            }

                            if (counter) {
                                prefix.back() = '1';
                            }
                        } else {
                            // inverted char
                            prefix += value ? '0' : '1';
                        }

                        while (prefix.size() & (sizeof(RLEW_INT_TYPE) * 8 - 1)) {
                            prefix += '0';
                        }

                        for (int i = 0; i < 3; ++i) {
                            int error = rlew_enc_finish(&e);
                            if (RLEW_ERROR_NONE == error) {
                                break;
                            }

                            EXPECT_EQ(RLEW_ERROR_OVERFLOW, error);

                            while (e.output_gi != e.output_pi) {
                                add_to_output(e.output_buffer[e.output_gi++ % RLEW_ARRAY_SIZE(e.output_buffer)]);
                            }
                        }

                        EXPECT_EQ(prefix, output);
                    }
                }
            }
        }
    }
}


//TEST_F(rlew_fixture, encode_for_break_is_correct)
//{

//    const int oversampling = 16;

//    // break
//    for (int i = 0; i < 13; ++i) {
//        for (int j = 0; j < oversampling; ++j) {
//            rlew_enc_bit(&e, this, &static_write, 0);
//        }
//    }



//    rlew_enc_finish(&e, this, &static_write, 0);

//
//    EXPECT_EQ(0u, e.used);

//    if (1 == sizeof(RLEW_INT_TYPE)) {
//        // 16*13=208 => 127 + 81 = 64 + 17 (0b010001)
//        EXPECT_STREQ("00000001111111000000010100010000", output.c_str());
//    } else if (2 == sizeof(RLEW_INT_TYPE)) {
//        // 16*13=208 => 128 + 80 (0b1010000)
//        EXPECT_STREQ("0000000011010000", output.c_str());
//    } else {
//        EXPECT_STREQ("00000000110100000000000000000000", output.c_str());
//    }
//}

//TEST_F(rlew_fixture, encode_for_break_and_break_delim_is_correct)
//{

//    const int oversampling = 16;

//    // break
//    for (int i = 0; i < 13; ++i) {
//        for (int j = 0; j < oversampling; ++j) {
//            rlew_enc_bit(&e, this, &static_write, 0);
//        }
//    }

//    // 16*13=208 => 1<<7 + 80 (0b1010000)

//    // break delim
//    for (int j = 0; j < oversampling; ++j) {
//        rlew_enc_bit(&e, this, &static_write, 1);
//    }

//    // 16 => 1<<4 + 0 (0b0000)

//    rlew_enc_finish(&e, this, &static_write, 0);

//
//    EXPECT_EQ(0u, e.used);
//    EXPECT_STREQ("00000000110100001111100000000000", output.c_str());
//}


//TEST_F(rlew_fixture, encode_for_break_and_break_delim_an_sync_start_is_correct)
//{

//    const int oversampling = 16;

//    // break
//    for (int i = 0; i < 13; ++i) {
//        for (int j = 0; j < oversampling; ++j) {
//            rlew_enc_bit(&e, this, &static_write, 0);
//        }
//    }

//    // 16*13=208 => 1<<7 + 80 (0b1010000)

//    // break delim
//    for (int j = 0; j < oversampling; ++j) {
//        rlew_enc_bit(&e, this, &static_write, 1);
//    }

//    // 16 => 1<<4 + 0 (0b0000)

//    // sync start bit
//    for (int j = 0; j < oversampling; ++j) {
//        rlew_enc_bit(&e, this, &static_write, 0);
//    }

//    // 16 => 1<<4 + 0 (0b0000)

//    rlew_enc_finish(&e, this, &static_write, 0);

//
//    EXPECT_EQ(0u, e.used);
//    if (sizeof(RLEW_INT_TYPE) == 2) {
//        EXPECT_STREQ("000000001101000011111000000000010000000000000000", output.c_str());
//    } else {
//        EXPECT_STREQ("0000000011010000111110000000000100000000000000000000000000000000", output.c_str());
//    }
//}

//TEST_F(rlew_fixture, encode_for_break_and_break_delim_and_sync_start_and_sync_field_is_correct)
//{

//    const int oversampling = 16;

//    // break
//    for (int i = 0; i < 13; ++i) {
//        for (int j = 0; j < oversampling; ++j) {
//            rlew_enc_bit(&e, this, &static_write, 0);
//        }
//    }

//    // 16*13=208 => 1<<7 + 80 (0b1010000)

//    // break delim
//    for (int j = 0; j < oversampling; ++j) {
//        rlew_enc_bit(&e, this, &static_write, 1);
//    }

//    // 16 => 1<<4 + 0 (0b0000)

//    // sync start bit
//    for (int j = 0; j < oversampling; ++j) {
//        rlew_enc_bit(&e, this, &static_write, 0);
//    }

//    // 16 => 1<<4 + 0 (0b0000)

//    for (int i = 0; i < 8; ++i) {
//        int bit = (1 << i);
//        int value = (0x55 & bit) == bit;

//        for (int j = 0; j < oversampling; ++j) {
//            rlew_enc_bit(&e, this, &static_write, value);
//        }
//    }

//    // 0x55 (0b01010101), 16x oversample  => 1<<4 + 0 (0b0000000)

//    rlew_enc_finish(&e, this, &static_write, 0);

//
//    EXPECT_EQ(0u, e.used);
//    EXPECT_STREQ("00000000110100001111100000000001000011111000000000010000111110000000000100001111100000000001000011111000000000010000000000000000", output.c_str());
//}

//TEST_F(rlew_fixture, encode_break_sync)
//{
//    const int oversampling = 16;

//    // break
//    for (int i = 0; i < 13; ++i) {
//        for (int j = 0; j < oversampling; ++j) {
//            rlew_enc_bit(&e, this, &static_write, 0);
//        }
//    }

//    // break delim
//    for (int i = 0; i < 1; ++i) {
//        for (int j = 0; j < oversampling; ++j) {
//            rlew_enc_bit(&e, this, &static_write, 1);
//        }
//    }

//    // sync start bit
//    for (int j = 0; j < oversampling; ++j) {
//        rlew_enc_bit(&e, this, &static_write, 0);
//    }

//    for (int i = 0; i < 8; ++i) {
//        int bit = (1 << i);
//        int value = (0x55 & bit) == bit;

//        for (int j = 0; j < oversampling; ++j) {
//            rlew_enc_bit(&e, this, &static_write, value);
//        }
//    }

//    // sync stop bit
//    for (int j = 0; j < oversampling; ++j) {
//        rlew_enc_bit(&e, this, &static_write, 1);
//    }

//    rlew_enc_finish(&e, this, &static_write, 0);
//
//    EXPECT_EQ(0u, e.used);
//    EXPECT_STREQ(
//                "00000000110100001111100000000001000011111000000000010000111110000000000100001111100000000001000011111000000000010000111110000000",
//                output.c_str());

//    dump_ints();
//}


TEST_F(rlew_fixture, rlew_dec_bit_asserts_params)
{
    EXPECT_ANY_THROW(rlew_dec_bit(NULL));
}


TEST_F(rlew_fixture, rlew_dec_bit_underflows_if_no_or_insufficient_input_is_available)
{
    EXPECT_EQ(RLEW_ERROR_UNDERFLOW, rlew_dec_bit(&d));

    for (size_t i = 0; i < RLEW_ARRAY_SIZE(d.input_buffer)-1; ++i) {
        ++d.input_pi;
        EXPECT_EQ(RLEW_ERROR_UNDERFLOW, rlew_dec_bit(&d));
    }
}


TEST_F(rlew_fixture, rlew_dec_bit_decodes_0_bit_counter_of_value_0_properly)
{
    d.input_buffer[0] = RLEW_INT_TYPE(1) << (sizeof(RLEW_INT_TYPE) * 8 - 2);
    d.input_gi = 0;
    d.input_pi = 3;

    EXPECT_EQ(RLEW_ERROR_FALSE, rlew_dec_bit(&d));

    d.input_buffer[0] = 0;
    d.input_gi = 0;
    d.input_pi = 3;
    EXPECT_EQ(RLEW_ERROR_EOS, rlew_dec_bit(&d));
}

TEST_F(rlew_fixture, rlew_dec_bit_decodes_0_bit_counter_of_value_1_properly)
{
    d.input_buffer[0] = RLEW_INT_TYPE(1) << (sizeof(RLEW_INT_TYPE) * 8 - 1);
    d.input_gi = 0;
    d.input_pi = 3;

    EXPECT_EQ(RLEW_ERROR_TRUE, rlew_dec_bit(&d));

    d.input_buffer[0] = 0;
    d.input_gi = 0;
    d.input_pi = 3;
    EXPECT_EQ(RLEW_ERROR_EOS, rlew_dec_bit(&d));
}

TEST_F(rlew_fixture, rlew_dec_bit_decodes_max_bit_counter_of_value_0_properly)
{
    d.input_buffer[0] = 1;
    d.input_buffer[1] = RLEW_INT_TYPE(~0) << 2;
    d.input_gi = 0;
    d.input_pi = 3;
    EXPECT_EQ(RLEW_ERROR_FALSE, rlew_dec_bit(&d));
    EXPECT_EQ(RLEW_MAX_COUNT - 1, d.count);

    d.count = 0;
    d.input_buffer[0] = 0;
    d.input_buffer[1] = 0;
    d.input_gi = 0;
    d.input_pi = 3;
    EXPECT_EQ(RLEW_ERROR_EOS, rlew_dec_bit(&d));
}

TEST_F(rlew_fixture, rlew_dec_bit_decodes_max_bit_counter_of_value_1_properly)
{
    d.input_buffer[0] = RLEW_INT_TYPE(~0) << 1;
    d.input_buffer[1] = RLEW_INT_TYPE(~0) << 2;
    d.input_gi = 0;
    d.input_pi = 3;
    EXPECT_EQ(RLEW_ERROR_TRUE, rlew_dec_bit(&d));
    EXPECT_EQ(RLEW_MAX_COUNT - 1, d.count);

    d.count = 0;
    d.input_buffer[0] = 0;
    d.input_buffer[1] = 0;
    d.input_gi = 0;
    d.input_pi = 3;
    EXPECT_EQ(RLEW_ERROR_EOS, rlew_dec_bit(&d));
}

#if 0


TEST_F(rlew_fixture, rlew_dec_bit_decodes_1_bit_counter_properly)
{
    EXPECT_EQ(0u, d.flags);
    set_input("00101101");

    int bit;
    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0u, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0u, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0u, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(RLEW_FLAG_DEC_UNDERFLOW, d.flags);
}




TEST_F(rlew_fixture, rlew_dec_bit_decodes_max_bit_counter_properly0)
{
    int bit;

    if (1 == sizeof(RLEW_INT_TYPE)) {
        set_input("0000000111111100");

        bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0, bit);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(126u, d.count);
    } else if (2 == sizeof(RLEW_INT_TYPE)) {
        set_input("00000000000000001000000000000000");

        bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0, bit);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(32767u, d.count);
    } else if (4 == sizeof(RLEW_INT_TYPE)) {
        set_input("00000000000000000000000000000000100000000000000000000000000000");

        bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0, bit);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(2147483647u, d.count);
    } else {
        FAIL();
    }
}




TEST_F(rlew_fixture, rlew_dec_bit_decodes_max_bit_counter_properly1)
{
    int bit;
    if (1 == sizeof(RLEW_INT_TYPE)) {
        set_input("1111111011111100");

        bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(1, bit);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(126u, d.count);
    } else if (2 == sizeof(RLEW_INT_TYPE)) {

        set_input("111111111111111000000000000000");

        bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(1, bit);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(16383u, d.count);
    } else if (4 == sizeof(RLEW_INT_TYPE)) {
        set_input("111111111111111111111111111111100000000000000000000000000000");

        bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(1, bit);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(1073741823u, d.count);
    } else {
        FAIL();
    }


}




TEST_F(rlew_fixture, rlew_dec_bit_decodes_max_bit_counter_properly_accross_integer_boundaries)
{
    int bit;

    EXPECT_EQ(0u, d.flags);
    set_input("01010101000000000000000100000000000000");

    for (auto i = 0; i < 4; ++i) {
        bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0, bit);
        EXPECT_EQ(0u, d.flags);
    }

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0u, d.flags);
    EXPECT_EQ(16383u, d.count);
}

TEST_F(rlew_fixture, rlew_dec_bit_ends_after_sizeof_int_len_of_0)
{
    if (sizeof(RLEW_INT_TYPE) != 2) {
        return;
    }

    EXPECT_EQ(0u, d.flags);
    set_input("01100000000000000000");

    int bit;
    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0u, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0u, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(RLEW_FLAG_DEC_EOS, d.flags);
}

TEST_F(rlew_fixture, rlew_dec_bit_ends_after_sizeof_int_len_of_1)
{
    if (sizeof(RLEW_INT_TYPE) != 2) {
        return;
    }

    EXPECT_EQ(0u, d.flags);
    set_input("01101111111111111111");

    int bit;
    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0u, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0u, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(RLEW_FLAG_DEC_EOS, d.flags);
}

TEST_F(rlew_fixture, rlew_decode_keeps_underflow_flag)
{
    if (sizeof(RLEW_INT_TYPE) != 2) {
        return;
    }

    int bit;
    set_input("000000000100000000");

    EXPECT_EQ(0u, d.flags);
    read_error = 1;

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(-1, bit);
    EXPECT_EQ(RLEW_FLAG_DEC_UNDERFLOW, d.flags);

    read_error = std::numeric_limits<int>::min();

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(-1, bit);
    EXPECT_EQ(RLEW_FLAG_DEC_UNDERFLOW, d.flags);
}

TEST_F(rlew_fixture, rlew_decode_keeps_eos_flag)
{
    if (sizeof(RLEW_INT_TYPE) != 2) {
        return;
    }

    EXPECT_EQ(0u, d.flags);
    set_input("011011111111111111111000000");

    int bit;
    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0u, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0u, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(-1, bit);
    EXPECT_EQ(RLEW_FLAG_DEC_EOS, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(-1, bit);
    EXPECT_EQ(RLEW_FLAG_DEC_EOS, d.flags);
}

TEST_F(rlew_fixture, rlew_decode_works_for_all_numbers)
{
    if (sizeof(RLEW_INT_TYPE) > 2) {
        return;
    }

    for (RLEW_INT_TYPE i = 0; i < RLEW_MAX_BIT_COUNT; ++i) {
        RLEW_INT_TYPE count = RLEW_INT_TYPE(1) << i;

        for (int value = 0; value <= 1; ++value) {
            for (RLEW_INT_TYPE j = 0; j < count; ++j) {
                rlew_enc_bit(&e, this, &static_write, value);

            }

            rlew_enc_finish(&e, this, &static_write, 1);


            input.assign(output.begin(), output.end());
            output.clear();
            rlew_enc_init(&e);

            for (RLEW_INT_TYPE j = 0; j < count; ++j) {
                auto bit = rlew_dec_bit(&d, this, &static_read);
                EXPECT_EQ(0u, d.flags);
                EXPECT_EQ(value, bit);
            }

            rlew_dec_bit(&d, this, &static_read);
            EXPECT_EQ(RLEW_FLAG_DEC_EOS, d.flags);
            EXPECT_EQ(0u, input.size());

            rlew_dec_init(&d);
        }
    }
}



//TEST_F(rlew_fixture, decode_produces_right_output)
//{
//    if (sizeof(RLEW_INT_TYPE) != 4) {
//        return;
//    }

//    const int oversampling = 8;
//    const int break_bits = 13;
//    const int data_bits = 174;

//    const uint8_t pids[] = {0x20, 0x61, 0x32};

//    //
//    set_input("00000001101000111100000000100011110000000010001111000000001000111100000000100011110000000010001111000000000011000011110000000001000011111111111001011110000000000110100011110000000010001111000000001000111100000000100011110000000010001111000000001000111100000000100011110000000000100000111110000000001000111111111110010111100000000001101000111100000000100011110000000010001111000000001000111100000000100011110000000010001111000000000100001111000000000100001111100000000001000011111111111001011110000000000110100011110000000010001111000000001000111100000000100011110000000010001111000000001000111100000000100011111000000000010000111110100000001000111111111110010111100000000000000000000000000000000000000000");

//    for (size_t a = 0; a < GTEST_ARRAY_SIZE_(pids); ++a) {
//        uint8_t pid = pids[a];


//        // break low
//        for (int i = 0; i < oversampling * break_bits; ++i) {
//            auto bit = rlew_dec_bit(&d, this, &static_read);
//            EXPECT_EQ(0u, d.flags);
//            EXPECT_EQ(0, bit);
//        }

//        // break high
//        for (int i = 0; i < oversampling; ++i) {
//            auto bit = rlew_dec_bit(&d, this, &static_read);
//            EXPECT_EQ(0u, d.flags);
//            EXPECT_EQ(1, bit);
//        }

//        // sync start bit
//        for (int i = 0; i < oversampling; ++i) {
//            auto bit = rlew_dec_bit(&d, this, &static_read);
//            EXPECT_EQ(0u, d.flags);
//            EXPECT_EQ(0, bit);
//        }

//        // sync byte
//        for (int i = 0; i < 8; ++i) {
//            int flag = 1<<i;
//            int value = (0x55 & flag) == flag;
//            for (int j = 0; j < oversampling; ++j) {
//                auto bit = rlew_dec_bit(&d, this, &static_read);
//                EXPECT_EQ(0u, d.flags);
//                EXPECT_EQ(value, bit);
//            }
//        }

//        // sync stop bit
//        for (int i = 0; i < oversampling; ++i) {
//            auto bit = rlew_dec_bit(&d, this, &static_read);
//            EXPECT_EQ(0u, d.flags);
//            EXPECT_EQ(1, bit);
//        }

//        // pid start bit
//        for (int i = 0; i < oversampling; ++i) {
//            auto bit = rlew_dec_bit(&d, this, &static_read);
//            EXPECT_EQ(0u, d.flags);
//            EXPECT_EQ(0, bit);
//        }

//        // pid byte
//        for (int i = 0; i < 8; ++i) {
//            int flag = 1<<i;
//            // 3nd load
//            int value = (pid & flag) == flag;
//            for (int j = 0; j < oversampling; ++j) {
//                auto bit = rlew_dec_bit(&d, this, &static_read);
//                EXPECT_EQ(0u, d.flags);
//                EXPECT_EQ(value, bit);
//            }
//        }

//        // pid stop bit
//        for (int i = 0; i < oversampling; ++i) {
//            auto bit = rlew_dec_bit(&d, this, &static_read);
//            EXPECT_EQ(0u, d.flags);
//            EXPECT_EQ(1, bit);
//        }

//        for (int i = 0; i < oversampling * data_bits; ++i) {
//            auto bit = rlew_dec_bit(&d, this, &static_read);
//            EXPECT_EQ(0u, d.flags);
//            EXPECT_EQ(1, bit);
//        }
//    }
//}

TEST_F(rlew_fixture, encode_decode_for_break_is_correct)
{

    const int oversampling = 16;

    // break
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            rlew_enc_bit(&e, this, &static_write, 0);
        }
    }

    // 16*13=208 => 128 + 80 (0b1010000)

    rlew_enc_finish(&e, this, &static_write, 1);


    EXPECT_EQ(0u, e.used);
    if (sizeof(RLEW_INT_TYPE) == 2) {
        EXPECT_STREQ("00000000110100000000000000000000", output.c_str());
    } else {
        EXPECT_STREQ("0000000011010000000000000000000000000000000000000000000000000000", output.c_str());
    }

    input.assign(output.begin(), output.end());

    // break
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            auto bit = rlew_dec_bit(&d, this, &static_read);
            EXPECT_EQ(0u, d.flags);
            EXPECT_EQ(0, bit);
        }
    }

    rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(RLEW_FLAG_DEC_EOS, d.flags);
}



TEST_F(rlew_fixture, encode_decode_for_break_and_break_delim_is_correct)
{

    const int oversampling = 16;

    // break
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            rlew_enc_bit(&e, this, &static_write, 0);
        }
    }

    // 16*13=208 => 1<<7 + 80 (0b1010000)

    // break delim
    for (int j = 0; j < oversampling; ++j) {
        rlew_enc_bit(&e, this, &static_write, 1);
    }

    // 16 => 1<<4 + 0 (0b0000)

    rlew_enc_finish(&e, this, &static_write, 1);


    EXPECT_EQ(0u, e.used);
    if (sizeof(RLEW_INT_TYPE) == 2) {
        EXPECT_STREQ("000000001101000011111000000000000000000000000000", output.c_str());
    } else {
        EXPECT_STREQ("0000000011010000111110000000000000000000000000000000000000000000", output.c_str());
    }

    input.assign(output.begin(), output.end());

    // break
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            auto bit = rlew_dec_bit(&d, this, &static_read);
            EXPECT_EQ(0u, d.flags);
            EXPECT_EQ(0, bit);
        }
    }

    // break delim
    for (int j = 0; j < oversampling; ++j) {
        auto bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(1, bit);
    }

    rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(RLEW_FLAG_DEC_EOS, d.flags);
}


TEST_F(rlew_fixture, encode_decode_for_break_and_break_delim_an_sync_start_is_correct)
{

    const int oversampling = 16;

    // break
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            rlew_enc_bit(&e, this, &static_write, 0);
        }
    }

    // 16*13=208 => 1<<7 + 80 (0b1010000)

    // break delim
    for (int j = 0; j < oversampling; ++j) {
        rlew_enc_bit(&e, this, &static_write, 1);
    }

    // 16 => 1<<4 + 0 (0b0000)

    // sync start bit
    for (int j = 0; j < oversampling; ++j) {
        rlew_enc_bit(&e, this, &static_write, 0);
    }

    // 16 => 1<<4 + 0 (0b0000)

    rlew_enc_finish(&e, this, &static_write, 1);


    EXPECT_EQ(0u, e.used);
    if (sizeof(RLEW_INT_TYPE) == 2) {
        EXPECT_STREQ("0000000011010000111110000000000100000000000000000000000000000000", output.c_str());
    } else {
        EXPECT_STREQ("000000001101000011111000000000010000000000000000000000000000000000000000000000000000000000000000", output.c_str());
    }

    input.assign(output.begin(), output.end());

    // break
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            auto bit = rlew_dec_bit(&d, this, &static_read);
            EXPECT_EQ(0u, d.flags);
            EXPECT_EQ(0, bit);
        }
    }

    // break delim
    for (int j = 0; j < oversampling; ++j) {
        auto bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(1, bit);
    }

    // start sync
    for (int j = 0; j < oversampling; ++j) {
        auto bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(0, bit);
    }

    rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(RLEW_FLAG_DEC_EOS, d.flags);
}


TEST_F(rlew_fixture, encode_decode_for_break_and_break_delim_and_sync_start_and_sync_field_is_correct)
{

    const int oversampling = 16;

    // break
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            rlew_enc_bit(&e, this, &static_write, 0);
        }
    }

    // 16*13=208 => 1<<7 + 80 (0b1010000)

    // break delim
    for (int j = 0; j < oversampling; ++j) {
        rlew_enc_bit(&e, this, &static_write, 1);
    }

    // 16 => 1<<4 + 0 (0b0000)

    // sync start bit
    for (int j = 0; j < oversampling; ++j) {
        rlew_enc_bit(&e, this, &static_write, 0);
    }

    // 16 => 1<<4 + 0 (0b0000)

    for (int i = 0; i < 8; ++i) {
        int bit = (1 << i);
        int value = (0x55 & bit) == bit;

        for (int j = 0; j < oversampling; ++j) {
            rlew_enc_bit(&e, this, &static_write, value);
        }
    }

    // 0x55 (0b01010101), 16x oversample  => 1<<4 + 0 (0b0000000)

    rlew_enc_finish(&e, this, &static_write, 1);


    EXPECT_EQ(0u, e.used);
    if (sizeof(RLEW_INT_TYPE) == 2) {
        EXPECT_STREQ("000000001101000011111000000000010000111110000000000100001111100000000001000011111000000000010000111110000000000100000000000000000000000000000000", output.c_str());
    } else {
        EXPECT_STREQ("0000000011010000111110000000000100001111100000000001000011111000000000010000111110000000000100001111100000000001000000000000000000000000000000000000000000000000", output.c_str());
    }


    input.assign(output.begin(), output.end());

    // break
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            auto bit = rlew_dec_bit(&d, this, &static_read);
            EXPECT_EQ(0u, d.flags);
            EXPECT_EQ(0, bit);
        }
    }

    // break delim
    for (int j = 0; j < oversampling; ++j) {
        auto bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(1, bit);
    }

    // start sync
    for (int j = 0; j < oversampling; ++j) {
        auto bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(0, bit);
    }

    // sync byte
    for (int i = 0; i < 8; ++i) {
        int bit = (1 << i);
        int value = (0x55 & bit) == bit;

        for (int j = 0; j < oversampling; ++j) {
            auto bit = rlew_dec_bit(&d, this, &static_read);
            EXPECT_EQ(0u, d.flags);
            EXPECT_EQ(value, bit);
        }
    }

    rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(RLEW_FLAG_DEC_EOS, d.flags);

}



TEST_F(rlew_fixture, test_round_trip)
{
    const int break_bits = 13;
    const int break_delim_bits = 1;
    const int oversampling = 16;
    const uint8_t pid = 0x20;

    // break
    for (int i = 0; i < break_bits; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            rlew_enc_bit(&e, this, &static_write, 0);
        }
    }

    // break delim
    for (int i = 0; i < break_delim_bits; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            rlew_enc_bit(&e, this, &static_write, 1);
        }
    }

    // sync start bit
    for (int j = 0; j < oversampling; ++j) {
        rlew_enc_bit(&e, this, &static_write, 0);
    }

    for (int i = 0; i < 8; ++i) {
        int bit = (1 << i);
        int value = (0x55 & bit) == bit;

        for (int j = 0; j < oversampling; ++j) {
            rlew_enc_bit(&e, this, &static_write, value);
        }
    }

    // sync stop bit
    for (int j = 0; j < oversampling; ++j) {
        rlew_enc_bit(&e, this, &static_write, 1);
    }

    // pid start bit
    for (int j = 0; j < oversampling; ++j) {
        rlew_enc_bit(&e, this, &static_write, 0);
    }

    for (int i = 0; i < 8; ++i) {
        int bit = (1 << i);
        int value = (pid & bit) == bit;

        for (int j = 0; j < oversampling; ++j) {
            rlew_enc_bit(&e, this, &static_write, value);
        }
    }

    // pid stop bit
    for (int j = 0; j < oversampling; ++j) {
        rlew_enc_bit(&e, this, &static_write, 1);
    }

    rlew_enc_finish(&e, this, &static_write, 1);

    EXPECT_EQ(0u, e.used);

    input.assign(output.begin(), output.end());

    /// READ IT ALL BACK

    // break
    for (int i = 0; i < break_bits; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            auto bit = rlew_dec_bit(&d, this, &static_read);
            EXPECT_EQ(0u, d.flags);
            EXPECT_EQ(0, bit);
        }
    }

    // break delim
    for (int i = 0; i < break_delim_bits; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            auto bit = rlew_dec_bit(&d, this, &static_read);
            EXPECT_EQ(0u, d.flags);
            EXPECT_EQ(1, bit);
        }
    }

    // sync start bit
    for (int j = 0; j < oversampling; ++j) {
        auto bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(0, bit);
    }

    for (int i = 0; i < 8; ++i) {
        int bit = (1 << i);
        int value = (0x55 & bit) == bit;

        for (int j = 0; j < oversampling; ++j) {
            auto bit = rlew_dec_bit(&d, this, &static_read);
            EXPECT_EQ(0u, d.flags);
            EXPECT_EQ(value, bit);
        }
    }

    // sync stop bit
    for (int j = 0; j < oversampling; ++j) {
        auto bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(1, bit);
    }

    // pid start bit
    for (int j = 0; j < oversampling; ++j) {
        auto bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(0, bit);
    }

    for (int i = 0; i < 8; ++i) {
        int bit = (1 << i);
        int value = (pid & bit) == bit;

        for (int j = 0; j < oversampling; ++j) {
            auto bit = rlew_dec_bit(&d, this, &static_read);
            EXPECT_EQ(0u, d.flags);
            EXPECT_EQ(value, bit);
        }
    }

    // pid stop bit
    for (int j = 0; j < oversampling; ++j) {
        auto bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(1, bit);
    }
}
#endif


TEST_F(rlew_fixture, decode_this1)
{
    const uint32_t words[] = {
        0x01a3c023,
        0xc023c023,
        0xc023c023,
        0xc00c3c01,
        0x0ffe5e00,
        0x68f008f0,
        0x08f008f0,
        0x08f008f0,
        0x030f0043,
        0xff97801a,
        0x3c023c02,
        0x3c023c02,
        0x3c023c00,
        0xc3c010ff,
        0xe5e0068f,
        0x008f008f,
        0x008f008f,
        0x008f0030,
        0xf0043ff9,
        0x78000000,
        0x00000000,
        0x00000000,
        0x00000000,
    };

    size_t count = 0;
    bool value = false;
    size_t word_offset = 0;
    size_t decoded_bits_counter = 0;

    for (bool done = false; !done; ) {
        int error = 0;
        if (3328 == decoded_bits_counter) {
            error = rlew_dec_bit(&d);
        } else {
            error = rlew_dec_bit(&d);
        }

        switch (error) {
        case RLEW_ERROR_FALSE:
        case RLEW_ERROR_TRUE: {
            ++decoded_bits_counter;
            auto new_value = error == RLEW_ERROR_TRUE;
            if (count == 0) {
                value = new_value;
                count = 1;
            } else if (value != new_value) {
                fprintf(stdout, "%d: %zu\n", value, count);
                value = new_value;
                count = 1;
            } else {
                ++count;
            }
        } break;
        case RLEW_ERROR_EOS:
            done = true;
            break;
        case RLEW_ERROR_UNDERFLOW:
            ASSERT_TRUE(word_offset < RLEW_ARRAY_SIZE(words));
            ASSERT_TRUE(size_t(d.input_pi - d.input_gi) < RLEW_ARRAY_SIZE(d.input_buffer));
            d.input_buffer[d.input_pi++ % RLEW_ARRAY_SIZE(d.input_buffer)] = words[word_offset++];
            break;
        default:
            FAIL();
            break;
        }
    }

    if (count) {
        fprintf(stdout, "%d: %zu\n", value, count);
    }
}

TEST_F(rlew_fixture, decode_is_correct_for_all_valid_offsets_and_all_bit_counts)
{
    std::string prefix;

    for (size_t count_bits = 0; count_bits < RLEW_MAX_BIT_COUNT; ++count_bits) {
        for (size_t offset = 0; offset < sizeof(RLEW_INT_TYPE) * 8; offset += 2) {
            for (size_t value = 0; value <= 1; ++value) {
                for (size_t counter = 0; counter < 2; ++counter) {
                    for (size_t prefix_value = 0; prefix_value <= 1; ++prefix_value) {
                        prefix.clear();
                        output.clear();
                        rlew_enc_init(&e);
                        rlew_dec_init(&d);

                        //count_bits=0 offset=4 value=0 counter=0
                        //count_bits=1 offset=0 value=0 counter=0
                        //count_bits=1 offset=30 value=1 counter=0
                        //count_bits=3 offset=26 value=0 counter=0
    //                    if (count_bits != 3 || offset != 26 || value != 1 || counter != 0) {
    //                        continue;
    //                    }

                        for (size_t j = 0; j < offset; j += 2) {
                            EXPECT_EQ(RLEW_ERROR_NONE, rlew_enc_bit(&e, prefix_value));
                            rlew_enc_flush(&e);

                            EXPECT_EQ(j + 2, e.used);

                            if (prefix_value) {
                                prefix += "10";
                            } else {
                                prefix += "01";
                            }
                        }

                        if (value) {
                            prefix += '1';

                        } else {
                            prefix += '0';
                        }

                        e.value = value;
                        e.count = 1;

                        if (count_bits) {
                            e.count <<= count_bits;
                            e.count += counter;

                            // count bits of value char
                            for (size_t c = 0; c < count_bits; ++c) {
                                prefix += value ? '1' : '0';
                            }

                            // inverted char
                            prefix += value ? '0' : '1';

                            // counter value
                            for (size_t c = 0; c < count_bits; ++c) {
                                prefix += '0';
                            }

                            if (counter) {
                                prefix.back() = '1';
                            }
                        } else {
                            // inverted char
                            prefix += value ? '0' : '1';
                        }

                        while (prefix.size() & (sizeof(RLEW_INT_TYPE) * 8 - 1)) {
                            prefix += '0';
                        }

                        for (int i = 0; i < 3; ++i) {
                            int error = rlew_enc_finish(&e);
                            if (RLEW_ERROR_NONE == error) {
                                break;
                            }

                            EXPECT_EQ(RLEW_ERROR_OVERFLOW, error);

                            while (e.output_gi != e.output_pi) {
                                RLEW_INT_TYPE o = e.output_buffer[e.output_gi++ % RLEW_ARRAY_SIZE(e.output_buffer)];
                                d.input_buffer[d.input_pi++ % RLEW_ARRAY_SIZE(d.input_buffer)] = o;
                                add_to_output(o);
                            }
                        }

                        EXPECT_EQ(prefix, output);
    //                    fprintf(stdout, "output: %s\n", output.c_str());



                        // consume alignment
                        int expected_alignment_error = prefix_value ? RLEW_ERROR_TRUE : RLEW_ERROR_FALSE;
                        for (size_t j = 0; j < offset; j += 2) {
                            while (size_t(d.input_pi - d.input_gi) < RLEW_ARRAY_SIZE(d.input_buffer)) {
                                d.input_buffer[d.input_pi++ % RLEW_ARRAY_SIZE(d.input_buffer)] = 0;
                            }

                            int error = rlew_dec_bit(&d);
                            if (error != expected_alignment_error) {
                                fprintf(stdout, "count_bits=%zu offset=%zu value=%zu counter=%zu\n", count_bits, offset, value, counter);
                            }
                            EXPECT_EQ(expected_alignment_error, error);
                        }

                        while (size_t(d.input_pi - d.input_gi) < RLEW_ARRAY_SIZE(d.input_buffer)) {
                            d.input_buffer[d.input_pi++ % RLEW_ARRAY_SIZE(d.input_buffer)] = 0;
                        }

                        // consume value
                        int value_error = rlew_dec_bit(&d);
                        int expected_value_error = value ? RLEW_ERROR_TRUE : RLEW_ERROR_FALSE;
                        if (value_error != expected_value_error) {
                            fprintf(stdout, "count_bits=%zu offset=%zu value=%zu counter=%zu\n", count_bits, offset, value, counter);
                        }
                        EXPECT_EQ(expected_value_error, value_error);

                        if (count_bits) {
                            size_t expected = (size_t(1) << count_bits) + counter - 1;
                            EXPECT_EQ(expected, d.count);
                            d.count = 0;
                        }

                        while (size_t(d.input_pi - d.input_gi) < RLEW_ARRAY_SIZE(d.input_buffer)) {
                            d.input_buffer[d.input_pi++ % RLEW_ARRAY_SIZE(d.input_buffer)] = 0;
                        }

                        EXPECT_EQ(RLEW_ERROR_EOS, rlew_dec_bit(&d));
                    }
                }
            }
        }
    }
}

