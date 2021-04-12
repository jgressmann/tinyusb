#include <gtest/gtest.h>


#define RLE_ASSERT(x) if (!(x)) throw 1
#define RLE_H
#define RLE_C
#define RLE_INT_TYPE uint16_t
#define RLE_STATIC
#include <rle.h>
#undef RLE_INT_TYPE

#include <string>
#include <cstring>
#include <deque>



namespace
{

struct rle_fixture : public ::testing::Test
{
    struct rle r;

    std::string output;
    std::deque<char> input;
    int write_error;
    int read_error;


    ~rle_fixture()
    {

    }

    rle_fixture()
    {
        rle_init(&r);
        write_error = std::numeric_limits<int>::min();
        read_error = std::numeric_limits<int>::min();
    }

    void set_input(char const *str)
    {
        size_t len = std::strlen(str);
        input.assign(str, str+len);
    }

    static int static_read(void* ctx, uint8_t * ptr, unsigned count)
    {
        return static_cast<rle_fixture*>(ctx)->read(ptr, count);
    }

    int read(uint8_t * ptr, unsigned count)
    {
        if (read_error != std::numeric_limits<int>::min()) {
            return read_error;
        }

        count = std::min<size_t>(count, input.size());

        unsigned bits_left_in_byte = 8;

        for (unsigned i = 0, bit_count = 0; i < count; ++i) {
            *ptr <<= 1;
            *ptr |= input[0] == '1' ? 1 : 0;
            input.pop_front();
            ++bit_count;
            --bits_left_in_byte;

            if (8 == bit_count) {
                ++ptr;
                bit_count = 0;
                bits_left_in_byte = 8;
            }
        }

        *ptr <<= bits_left_in_byte;



        return count;
    }

    static int static_write(void* ctx, uint8_t const* ptr, unsigned count)
    {
        return static_cast<rle_fixture*>(ctx)->write(ptr, count);
    }

    int write(uint8_t const* ptr, unsigned bit_count)
    {
        if (write_error != std::numeric_limits<int>::min()) {
            return write_error;
        }

        int r = bit_count;
        size_t index = 0;
        size_t x = ptr[index++];
        while (bit_count >= sizeof(*ptr) * 8) {
            for (unsigned i = sizeof(*ptr) * 8 - 1; i < sizeof(*ptr) * 8; --i) {
                output += (x & (static_cast<decltype (*ptr)>(1) << i)) ? '1' : '0';
            }

            x = ptr[index++];
            bit_count -= sizeof(*ptr) * 8;
        }

        for (unsigned i = sizeof(*ptr) * 8 - 1; bit_count && i < sizeof(*ptr) * 8; --i, --bit_count) {
            output += (x & (static_cast<decltype (*ptr)>(1) << i)) ? '1' : '0';
        }



        return r;
    }
};



} // anon

TEST(rle, init_asserts_pointers)
{
    EXPECT_ANY_THROW(rle_init(NULL));
}

TEST(rle, init_sets_default_values)
{
    struct rle s;
    rle_init(&s);
    EXPECT_EQ(0u, s.flags);
    EXPECT_EQ(0u, s.count);
    EXPECT_EQ(0u, s.value);
}

TEST_F(rle_fixture, rle_encode_flush_asserts_params)
{
    EXPECT_ANY_THROW(rle_encode_flush(NULL, this, &static_write));
    EXPECT_ANY_THROW(rle_encode_flush(&r, this, NULL));
}

TEST_F(rle_fixture, rle_encode_flush_is_a_nop_if_count_is_zero)
{
    auto x = r;
    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(0, memcmp(&x, &r, sizeof(r)));
    EXPECT_EQ(0u, output.size());
}

TEST_F(rle_fixture, rle_encode_bit_asserts_params)
{
    EXPECT_ANY_THROW(rle_encode_bit(NULL, this, &static_write, 1));
    EXPECT_ANY_THROW(rle_encode_bit(&r, this, NULL, 0));
}

TEST_F(rle_fixture, encode_0_yields_expected_output)
{
    rle_encode_bit(&r, this, &static_write, 0);

    EXPECT_EQ(0u, r.value);
    EXPECT_EQ(1u, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(0u, r.count);
    EXPECT_EQ(0u, r.flags);
    EXPECT_STREQ("01", output.c_str());
}

TEST_F(rle_fixture, encode_1_yields_expected_output)
{
    rle_encode_bit(&r, this, &static_write, 1);

    EXPECT_EQ(1u, r.value);
    EXPECT_EQ(1u, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(0u, r.count);
    EXPECT_EQ(0u, r.flags);
    EXPECT_STREQ("10", output.c_str());
}

TEST_F(rle_fixture, encode_00_yields_expected_output)
{
    rle_encode_bit(&r, this, &static_write, 0);
    rle_encode_bit(&r, this, &static_write, 0);

    EXPECT_EQ(0u, r.value);
    EXPECT_EQ(2u, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(0u, r.count);
    EXPECT_EQ(0u, r.flags);
    EXPECT_STREQ("0010", output.c_str());
}

TEST_F(rle_fixture, encode_11_yields_expected_output)
{
    rle_encode_bit(&r, this, &static_write, 1);
    rle_encode_bit(&r, this, &static_write, 1);


    EXPECT_EQ(1u, r.value);
    EXPECT_EQ(2u, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(0u, r.count);
    EXPECT_EQ(0u, r.flags);
    EXPECT_STREQ("1100", output.c_str());
}

TEST_F(rle_fixture, encode_000_yields_expected_output)
{
    rle_encode_bit(&r, this, &static_write, 0);
    rle_encode_bit(&r, this, &static_write, 0);
    rle_encode_bit(&r, this, &static_write, 0);

    EXPECT_EQ(0u, r.value);
    EXPECT_EQ(3u, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(0u, r.count);
    EXPECT_EQ(0u, r.flags);
    EXPECT_STREQ("0011", output.c_str());
}

TEST_F(rle_fixture, encode_0000_yields_expected_output)
{
    for (int i = 0; i < 4; ++i) {
        rle_encode_bit(&r, this, &static_write, 0);
    }

    EXPECT_EQ(0u, r.value);
    EXPECT_EQ(4u, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(0u, r.count);
    EXPECT_EQ(0u, r.flags);
    EXPECT_STREQ("000100", output.c_str());
}

TEST_F(rle_fixture, encode_0000000_yields_expected_output)
{
    const size_t LIMIT = 7;
    for (size_t i = 0; i < LIMIT; ++i) {
        rle_encode_bit(&r, this, &static_write, 0);
    }

    EXPECT_EQ(0u, r.value);
    EXPECT_EQ(LIMIT, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(0u, r.count);
    EXPECT_EQ(0u, r.flags);
    EXPECT_STREQ("000111", output.c_str());
}


TEST_F(rle_fixture, encode_0x0x14_yields_expected_output)
{
    const size_t LIMIT = (1u<<15)-1;
    for (size_t i = 0; i < LIMIT; ++i) {
        rle_encode_bit(&r, this, &static_write, 0);
    }

    EXPECT_EQ(0u, r.value);
    EXPECT_EQ(LIMIT, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(0u, r.count);
    EXPECT_EQ(0u, r.flags);
    EXPECT_STREQ("000000000000000111111111111111", output.c_str());
}

TEST_F(rle_fixture, encode_sets_overflow_flag_if_output_cannot_be_written)
{
    rle_encode_bit(&r, this, &static_write, 0);

    EXPECT_EQ(0u, r.value);
    EXPECT_EQ(1u, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    write_error = 0;

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(RLE_FLAG_ENC_OVERFLOW | RLE_FLAG_ENC_VALUE, r.flags);
}



TEST_F(rle_fixture, encode_keeps_a_set_overflow_flag)
{
    rle_encode_bit(&r, this, &static_write, 0);

    EXPECT_EQ(0u, r.value);
    EXPECT_EQ(1u, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    write_error = 0;

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(RLE_FLAG_ENC_OVERFLOW | RLE_FLAG_ENC_VALUE, r.flags);

    write_error = std::numeric_limits<int>::min();

    rle_encode_bit(&r, this, &static_write, 1);
    EXPECT_EQ(RLE_FLAG_ENC_OVERFLOW | RLE_FLAG_ENC_VALUE, r.flags);

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(RLE_FLAG_ENC_OVERFLOW | RLE_FLAG_ENC_VALUE, r.flags);
}


TEST_F(rle_fixture, encode_sets_error_flag_on_error)
{
    rle_encode_bit(&r, this, &static_write, 0);

    EXPECT_EQ(0u, r.value);
    EXPECT_EQ(1u, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    write_error = -1;

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(RLE_FLAG_ENC_ERROR | RLE_FLAG_ENC_VALUE, r.flags);
}

TEST_F(rle_fixture, encode_keeps_a_set_error_flag)
{
    rle_encode_bit(&r, this, &static_write, 0);

    EXPECT_EQ(0u, r.value);
    EXPECT_EQ(1u, r.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, r.flags);

    write_error = -1;

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(RLE_FLAG_ENC_ERROR | RLE_FLAG_ENC_VALUE, r.flags);

    write_error = std::numeric_limits<int>::min();

    rle_encode_bit(&r, this, &static_write, 1);
    EXPECT_EQ(RLE_FLAG_ENC_ERROR | RLE_FLAG_ENC_VALUE, r.flags);

    rle_encode_flush(&r, this, &static_write);
    EXPECT_EQ(RLE_FLAG_ENC_ERROR | RLE_FLAG_ENC_VALUE, r.flags);
}


TEST_F(rle_fixture, rle_decode_bit_asserts_params)
{
    unsigned bit;
    EXPECT_ANY_THROW(rle_decode_bit(NULL, this, &static_read, &bit));
    EXPECT_ANY_THROW(rle_decode_bit(&r, this, NULL, &bit));
    EXPECT_ANY_THROW(rle_decode_bit(&r, this, &static_read, NULL));
}

TEST_F(rle_fixture, rle_decode_bit_underflows_if_no_input_is_available)
{
    EXPECT_EQ(0u, r.flags);

    unsigned bit;
    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, r.flags);
}

TEST_F(rle_fixture, rle_decode_bit_underflows_if_insufficient_insufficient_input_is_available_unary)
{
    EXPECT_EQ(0u, r.flags);
    set_input("0");

    unsigned bit;
    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, r.flags);
}

TEST_F(rle_fixture, rle_decode_bit_underflows_if_insufficient_input_is_available_negated)
{
    EXPECT_EQ(0u, r.flags);
    set_input("00");

    unsigned bit;
    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, r.flags);
}

TEST_F(rle_fixture, rle_decode_bit_underflows_if_insufficient_input_is_available_counter)
{
    EXPECT_EQ(0u, r.flags);
    set_input("001");

    unsigned bit;
    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, r.flags);
}

TEST_F(rle_fixture, rle_decode_bit_ends_after_sizeof_int_len_of_0)
{
    EXPECT_EQ(0u, r.flags);
    set_input("01100000000000000000");

    unsigned bit;
    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(0u, bit);
    EXPECT_EQ(0u, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(1u, bit);
    EXPECT_EQ(0u, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_EOS, r.flags);
}

TEST_F(rle_fixture, rle_decode_bit_ends_after_sizeof_int_len_of_1)
{
    EXPECT_EQ(0u, r.flags);
    set_input("01101111111111111111");

    unsigned bit;
    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(0u, bit);
    EXPECT_EQ(0u, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(1u, bit);
    EXPECT_EQ(0u, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_EOS, r.flags);
}

TEST_F(rle_fixture, rle_decode_bit_decodes_0_bit_counter_properly)
{
    EXPECT_EQ(0u, r.flags);
    set_input("0110");

    unsigned bit;
    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(0u, bit);
    EXPECT_EQ(0u, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(1u, bit);
    EXPECT_EQ(0u, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, r.flags);
}

TEST_F(rle_fixture, rle_decode_bit_decodes_1_bit_counter_properly)
{
    EXPECT_EQ(0u, r.flags);
    set_input("00101101");

    unsigned bit;
    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(0u, bit);
    EXPECT_EQ(RLE_FLAG_DEC_AVAILABLE, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(0u, bit);
    EXPECT_EQ(0, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(1u, bit);
    EXPECT_EQ(RLE_FLAG_DEC_AVAILABLE, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(1u, bit);
    EXPECT_EQ(RLE_FLAG_DEC_AVAILABLE, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(1u, bit);
    EXPECT_EQ(0, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, r.flags);
}

TEST_F(rle_fixture, rle_decode_bit_decodes_8_bit_counter_properly)
{
    EXPECT_EQ(0u, r.flags);
    set_input("000000000100000000");

    unsigned bit;
    for (size_t i = 0; i < 255; ++i) {
        rle_decode_bit(&r, this, &static_read, &bit);
        EXPECT_EQ(0u, bit);
        EXPECT_EQ(RLE_FLAG_DEC_AVAILABLE, r.flags);
    }

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(0u, bit);
    EXPECT_EQ(0u, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, r.flags);
}

TEST_F(rle_fixture, rle_decode_keeps_underflow_flag)
{
    unsigned bit;
    set_input("000000000100000000");

    EXPECT_EQ(0u, r.flags);
    read_error = 0;

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, r.flags);

    read_error = std::numeric_limits<int>::min();

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, r.flags);
}

TEST_F(rle_fixture, rle_decode_keeps_error_flag)
{
    unsigned bit;
    set_input("000000000100000000");

    EXPECT_EQ(0u, r.flags);
    read_error = -1;

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_ERROR, r.flags);

    read_error = std::numeric_limits<int>::min();

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_ERROR, r.flags);
}



TEST_F(rle_fixture, rle_decode_keeps_eos_flag)
{
    EXPECT_EQ(0u, r.flags);
    set_input("011011111111111111111000000");

    unsigned bit;
    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(0u, bit);
    EXPECT_EQ(0u, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(1u, bit);
    EXPECT_EQ(0u, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_EOS, r.flags);

    rle_decode_bit(&r, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_EOS, r.flags);
}


