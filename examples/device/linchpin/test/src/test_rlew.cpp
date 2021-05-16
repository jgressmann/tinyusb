#include <gtest/gtest.h>


#define RLEW_ASSERT(x) if (!(x)) throw 1
#define RLEW_H
#define RLEW_C
#define RLEW_INT_TYPE uint16_t
#define RLEW_STATIC
#include <rlew.h>
//#undef RLEW_INT_TYPE

#include <string>
#include <cstring>
#include <deque>



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
    EXPECT_EQ(0u, d.flags);
    EXPECT_EQ(0u, d.count);
    EXPECT_EQ(0u, d.value);
}

TEST_F(rlew_fixture, rlew_enc_flush_asserts_params)
{
    EXPECT_ANY_THROW(rlew_enc_flush(NULL, this, &static_write));
    EXPECT_ANY_THROW(rlew_enc_flush(&e, this, NULL));
}

TEST_F(rlew_fixture, rlew_enc_flush_is_a_nop_if_count_is_zero)
{
    EXPECT_EQ(0u, e.count);
    rlew_enc_flush(&e, this, &static_write);
    EXPECT_EQ(0u, output.size());
}

TEST_F(rlew_fixture, rlew_en_bit_asserts_params)
{
    EXPECT_ANY_THROW(rlew_enc_bit(NULL, this, &static_write, 1));
    EXPECT_ANY_THROW(rlew_enc_bit(&e, this, NULL, 0));
}

TEST_F(rlew_fixture, encode_0_yields_expected_output)
{
    rlew_enc_bit(&e, this, &static_write, 0);

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(1u, e.count);
    EXPECT_EQ(0u, e.flags);

    rlew_enc_flush(&e, this, &static_write);
    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0x0001, e.state);
}

TEST_F(rlew_fixture, encode_1_yields_expected_output)
{
    rlew_enc_bit(&e, this, &static_write, 1);

    EXPECT_EQ(1u, e.value);
    EXPECT_EQ(1u, e.count);
    EXPECT_EQ(0, e.flags);

    rlew_enc_flush(&e, this, &static_write);
    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0x0002, e.state);
}

TEST_F(rlew_fixture, encode_00_yields_expected_output)
{
    rlew_enc_bit(&e, this, &static_write, 0);
    rlew_enc_bit(&e, this, &static_write, 0);

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(2u, e.count);
    EXPECT_EQ(0u, e.flags);

    rlew_enc_flush(&e, this, &static_write);
    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0x0002, e.state);
}

TEST_F(rlew_fixture, encode_11_yields_expected_output)
{
    rlew_enc_bit(&e, this, &static_write, 1);
    rlew_enc_bit(&e, this, &static_write, 1);


    EXPECT_EQ(1u, e.value);
    EXPECT_EQ(2u, e.count);
    EXPECT_EQ(0u, e.flags);

    rlew_enc_flush(&e, this, &static_write);
    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0x000c, e.state);
}

TEST_F(rlew_fixture, encode_000_yields_expected_output)
{
    rlew_enc_bit(&e, this, &static_write, 0);
    rlew_enc_bit(&e, this, &static_write, 0);
    rlew_enc_bit(&e, this, &static_write, 0);

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(3u, e.count);
    EXPECT_EQ(0u, e.flags);

    rlew_enc_flush(&e, this, &static_write);
    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0x0003, e.state);
}

TEST_F(rlew_fixture, encode_0000_yields_expected_output)
{
    for (int i = 0; i < 4; ++i) {
        rlew_enc_bit(&e, this, &static_write, 0);
    }

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(4u, e.count);
    EXPECT_EQ(0u, e.flags);

    rlew_enc_flush(&e, this, &static_write);
    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0x0004, e.state);
}

TEST_F(rlew_fixture, encode_0000000_yields_expected_output)
{
    const size_t LIMIT = 7;
    for (size_t i = 0; i < LIMIT; ++i) {
        rlew_enc_bit(&e, this, &static_write, 0);
    }

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(LIMIT, e.count);
    EXPECT_EQ(0u, e.flags);

    rlew_enc_flush(&e, this, &static_write);
    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0x0007, e.state);
}


TEST_F(rlew_fixture, encode_0x0xLIMIT_yields_expected_output)
{
    for (size_t i = 0; i < RLEW_MAX_COUNT; ++i) {
        rlew_enc_bit(&e, this, &static_write, 0);
        EXPECT_EQ(0u, e.flags);
    }

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(RLEW_MAX_COUNT, e.count);

    rlew_enc_finish(&e, this, &static_write, 0);
    EXPECT_EQ(0u, e.count);
    EXPECT_EQ(0u, e.flags);

    if (1 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("0000000111111100", output.c_str());
    } else if (2 == sizeof(RLEW_INT_TYPE)){
        EXPECT_STREQ("00000000000000011111111111111100", output.c_str());
    } else {
        FAIL();
    }
}

TEST_F(rlew_fixture, encode_sets_overflow_flag_if_output_cannot_be_written)
{
    e.count = 1;
    e.used = sizeof(RLEW_INT_TYPE) * 8;
    write_error = 1;

    rlew_enc_flush(&e, this, &static_write);
    EXPECT_EQ(RLEW_FLAG_ENC_OVERFLOW, e.flags);
}

TEST_F(rlew_fixture, encode_keeps_a_set_overflow_flag)
{
    e.count = 1;
    e.used = sizeof(RLEW_INT_TYPE) * 8;
    write_error = 1;

    rlew_enc_flush(&e, this, &static_write);
    EXPECT_EQ(RLEW_FLAG_ENC_OVERFLOW, e.flags);

    write_error = std::numeric_limits<int>::min();

    rlew_enc_bit(&e, this, &static_write, 1);
    EXPECT_EQ(RLEW_FLAG_ENC_OVERFLOW, e.flags);

    rlew_enc_flush(&e, this, &static_write);
    EXPECT_EQ(RLEW_FLAG_ENC_OVERFLOW, e.flags);
}

TEST_F(rlew_fixture, encode_finish_writes_at_least_one_int_worth_of_zeros)
{
    rlew_enc_finish(&e, this, &static_write, 1);
    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0u, e.used);

    if (1 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("00000000", output.c_str());
    } else if (2 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("0000000000000000", output.c_str());
    } else {
        FAIL();
    }
}

TEST_F(rlew_fixture, encode_finish_aligns_and_stores_current_state_then_terminates_with_zeros)
{
    rlew_enc_bit(&e, this, &static_write, 1);

    rlew_enc_finish(&e, this, &static_write, 1);
    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0u, e.used);
    if (1 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("1000000000000000", output.c_str());
    } else if (2 == sizeof(RLEW_INT_TYPE)) {
        EXPECT_STREQ("10000000000000000000000000000000", output.c_str());
    } else {
        FAIL();
    }
}

TEST_F(rlew_fixture, encode_for_break_is_correct)
{

    const int oversampling = 16;

    // break
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            rlew_enc_bit(&e, this, &static_write, 0);
        }
    }



    rlew_enc_finish(&e, this, &static_write, 0);

    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0u, e.used);

    if (1 == sizeof(RLEW_INT_TYPE)) {
        // 16*13=208 => 127 + 81 = 64 + 17 (0b010001)
        EXPECT_STREQ("00000001111111000000010100010000", output.c_str());
    } else if (2 == sizeof(RLEW_INT_TYPE)) {
        // 16*13=208 => 128 + 80 (0b1010000)
        EXPECT_STREQ("0000000011010000", output.c_str());
    } else {
        FAIL();
    }
}

TEST_F(rlew_fixture, encode_for_break_and_break_delim_is_correct)
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

    rlew_enc_finish(&e, this, &static_write, 0);

    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0u, e.used);
    EXPECT_STREQ("00000000110100001111100000000000", output.c_str());
}


TEST_F(rlew_fixture, encode_for_break_and_break_delim_an_sync_start_is_correct)
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

    rlew_enc_finish(&e, this, &static_write, 0);

    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0u, e.used);
    EXPECT_STREQ("000000001101000011111000000000010000000000000000", output.c_str());
}

TEST_F(rlew_fixture, encode_for_break_and_break_delim_and_sync_start_and_sync_field_is_correct)
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

    rlew_enc_finish(&e, this, &static_write, 0);

    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0u, e.used);
    EXPECT_STREQ("00000000110100001111100000000001000011111000000000010000111110000000000100001111100000000001000011111000000000010000000000000000", output.c_str());
}

TEST_F(rlew_fixture, encode_break_sync)
{
    const int oversampling = 16;

    // break
    for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < oversampling; ++j) {
            rlew_enc_bit(&e, this, &static_write, 0);
        }
    }

    // break delim
    for (int i = 0; i < 1; ++i) {
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

    rlew_enc_finish(&e, this, &static_write, 0);
    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0u, e.used);
    EXPECT_STREQ(
                "00000000110100001111100000000001000011111000000000010000111110000000000100001111100000000001000011111000000000010000111110000000",
                output.c_str());

    dump_ints();
}


TEST_F(rlew_fixture, rlew_dec_bit_asserts_params)
{
    EXPECT_ANY_THROW(rlew_dec_bit(NULL, this, &static_read));
    EXPECT_ANY_THROW(rlew_dec_bit(&d, this, NULL));
}


TEST_F(rlew_fixture, rlew_dec_bit_underflows_if_no_input_is_available)
{
    EXPECT_EQ(0u, d.flags);

    int bit;
    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(-1, bit);
    EXPECT_EQ(RLEW_FLAG_DEC_UNDERFLOW, d.flags);
}


TEST_F(rlew_fixture, rlew_dec_bit_decodes_0_bit_counter_properly)
{
    EXPECT_EQ(0u, d.flags);
    set_input("0110");

    int bit;
    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0u, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0u, d.flags);

    bit = rlew_dec_bit(&d, this, &static_read);
    EXPECT_EQ(RLEW_FLAG_DEC_UNDERFLOW, d.flags);
}




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
        set_input("000000000000000100000000000000");

        bit = rlew_dec_bit(&d, this, &static_read);
        EXPECT_EQ(0, bit);
        EXPECT_EQ(0u, d.flags);
        EXPECT_EQ(16383u, d.count);
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
    for (RLEW_INT_TYPE i = 0; i < RLEW_MAX_BIT_COUNT; ++i) {
        RLEW_INT_TYPE count = RLEW_INT_TYPE(1) << i;

        for (int value = 0; value <= 1; ++value) {
            for (RLEW_INT_TYPE j = 0; j < count; ++j) {
                rlew_enc_bit(&e, this, &static_write, value);
                EXPECT_EQ(0u, e.flags);
            }

            rlew_enc_finish(&e, this, &static_write, 1);
            EXPECT_EQ(0u, e.flags);

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

    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0u, e.used);
    EXPECT_STREQ("00000000110100000000000000000000", output.c_str());

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

    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0u, e.used);
    EXPECT_STREQ("000000001101000011111000000000000000000000000000", output.c_str());

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

    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0u, e.used);
    EXPECT_STREQ("0000000011010000111110000000000100000000000000000000000000000000", output.c_str());

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

    EXPECT_EQ(0u, e.flags);
    EXPECT_EQ(0u, e.used);
    EXPECT_STREQ("000000001101000011111000000000010000111110000000000100001111100000000001000011111000000000010000111110000000000100000000000000000000000000000000", output.c_str());


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
    EXPECT_EQ(0u, e.flags);
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


