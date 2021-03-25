#include <gtest/gtest.h>

#define RLE_ASSERT(x) if (!(x)) throw 1
#define RLE_H
#define RLE_C
#define RLE_INT_TYPE size_t
#include <rle.h>

#include <string>
#include <cstring>
#include <deque>



namespace
{

struct rle_fixture : public ::testing::Test
{
    struct rle e;
    struct rle d;

    std::string output;
    std::deque<char> input;
    static rle_fixture* self;
    volatile size_t gi;
    volatile size_t pi;
    size_t read_bit_offset;
    uint8_t decode_buffer[1<<16];


    ~rle_fixture()
    {
        self = nullptr;
    }

    rle_fixture()
    {
        self = this;

        rle_init(&e);
        rle_init(&d);
        e.generator = &static_enc_put;
        pi = 0;
        gi = 0;
        read_bit_offset = 0;
    }

    static int static_enc_put(size_t* bits, unsigned bit_count)
    {
        return self->enc_put(bits, bit_count);
    }

    int enc_put(size_t* bits, unsigned bit_count)
    {
        size_t index = 0;
        size_t x = bits[index++];
        while (bit_count >= sizeof(size_t) * 8) {
            for (unsigned i = sizeof(size_t) * 8 - 1; i < sizeof(size_t) * 8; --i) {
                output += (x & (static_cast<size_t>(1) << i)) ? '1' : '0';
            }

            x = bits[index++];
            bit_count -= sizeof(size_t) * 8;
        }

        for (unsigned i = bit_count - 1; i < bit_count; --i) {
            output += (x & (static_cast<size_t>(1) << i)) ? '1' : '0';
        }

        return 0;
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
        count = std::min<size_t>(count, input.size());

        for (unsigned i = 0, bit_count = 0; i < count; ++i) {
            *ptr <<= 1;
            *ptr |= input[0] == '1' ? 1 : 0;
            input.pop_front();
            ++bit_count;

            if (8 == bit_count) {
                ++ptr;
                bit_count = 0;
            }
        }

        return count;
    }
};

rle_fixture* rle_fixture::self;

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
}

TEST_F(rle_fixture, rle_encode_flush_asserts_params)
{
    EXPECT_ANY_THROW(rle_encode_flush(NULL));
}

TEST_F(rle_fixture, rle_encode_flush_is_a_nop_if_count_is_zero)
{
    auto x = e;
    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_EQ(0, memcmp(&x, &e, sizeof(e)));

    EXPECT_EQ(0u, output.size());
}

TEST_F(rle_fixture, rle_encode_push_bit_asserts_params)
{
    EXPECT_ANY_THROW(rle_encode_push_bit(NULL, 1));
    EXPECT_ANY_THROW(rle_encode_push_bit(NULL, 0));
}

TEST_F(rle_fixture, encode_0_yields_expected_output)
{
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));
    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(1u, e.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("01", output.c_str());
}

TEST_F(rle_fixture, encode_1_yields_expected_output)
{
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 1));
    EXPECT_EQ(1u, e.value);
    EXPECT_EQ(1u, e.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("10", output.c_str());
}

TEST_F(rle_fixture, encode_00_yields_expected_output)
{
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(2u, e.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("0010", output.c_str());
}

TEST_F(rle_fixture, encode_11_yields_expected_output)
{
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 1));
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 1));

    EXPECT_EQ(1u, e.value);
    EXPECT_EQ(2u, e.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("1100", output.c_str());
}

TEST_F(rle_fixture, encode_000_yields_expected_output)
{
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(3u, e.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("0011", output.c_str());
}

TEST_F(rle_fixture, encode_0000_yields_expected_output)
{
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));
    }

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(4u, e.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("000100", output.c_str());
}

TEST_F(rle_fixture, encode_0000000_yields_expected_output)
{
    const size_t LIMIT = 7;
    for (size_t i = 0; i < LIMIT; ++i) {
        EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));
    }

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(LIMIT, e.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("000111", output.c_str());
}


TEST_F(rle_fixture, encode_0x0x10000_yields_expected_output)
{
    const size_t LIMIT = 1u<<20;
    for (size_t i = 0; i < LIMIT; ++i) {
        EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));
    }

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(LIMIT, e.count);
    EXPECT_EQ(RLE_FLAG_ENC_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("000000000000000000000100000000000000000000", output.c_str());
}

TEST_F(rle_fixture, rle_decode_bit_asserts_params)
{
    int bit;
    EXPECT_ANY_THROW(rle_decode_bit(NULL, this, &static_read, &bit));
    EXPECT_ANY_THROW(rle_decode_bit(&d, this, NULL, &bit));
    EXPECT_ANY_THROW(rle_decode_bit(&d, this, &static_read, NULL));

//    EXPECT_ANY_THROW(rle_decode_bit(NULL, &gi, &pi, decode_buffer, sizeof(decode_buffer), &bit));
//    EXPECT_ANY_THROW(rle_decode_bit(&d, NULL, &pi, decode_buffer, sizeof(decode_buffer), &bit));
//    EXPECT_ANY_THROW(rle_decode_bit(&d, &gi, NULL, decode_buffer, sizeof(decode_buffer), &bit));
//    EXPECT_ANY_THROW(rle_decode_bit(&d, &gi, &pi, NULL, sizeof(decode_buffer), &bit));
//    EXPECT_ANY_THROW(rle_decode_bit(&d, &gi, &pi, decode_buffer, 0, &bit));
//    EXPECT_ANY_THROW(rle_decode_bit(&d, &gi, &pi, decode_buffer, sizeof(decode_buffer), NULL));
}

TEST_F(rle_fixture, rle_decode_bit_underflows_if_no_input_is_available)
{
    EXPECT_EQ(0u, d.flags);

    int bit;
    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, d.flags);
}

TEST_F(rle_fixture, rle_decode_bit_underflows_if_insufficient_insufficient_input_is_available_unary)
{
    EXPECT_EQ(0u, d.flags);
    set_input("0");

    int bit;
    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, d.flags);
}

TEST_F(rle_fixture, rle_decode_bit_underflows_if_insufficient_insufficient_input_is_available_negated)
{
    EXPECT_EQ(0u, d.flags);
    set_input("00");

    int bit;
    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, d.flags);
}

TEST_F(rle_fixture, rle_decode_bit_underflows_if_insufficient_insufficient_input_is_available_counter)
{
    EXPECT_EQ(0u, d.flags);
    set_input("001");

    int bit;
    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, d.flags);
}

TEST_F(rle_fixture, rle_decode_bit_set_bad_flag_on_invalid_input)
{
    EXPECT_EQ(0u, d.flags);
    set_input("0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");

    int bit;
    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_BAD, d.flags);
}

TEST_F(rle_fixture, rle_decode_bit_decodes_0_bit_counter_properly)
{
    EXPECT_EQ(0u, d.flags);
    set_input("0110");

    int bit;
    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0u, d.flags);

    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0u, d.flags);

    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, d.flags);
}

TEST_F(rle_fixture, rle_decode_bit_decodes_1_bit_counter_properly)
{
    EXPECT_EQ(0u, d.flags);
    set_input("00101101");

    int bit;
    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(0, bit);
    EXPECT_EQ(RLE_FLAG_DEC_AVAILABLE, d.flags);

    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0, d.flags);

    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(1, bit);
    EXPECT_EQ(RLE_FLAG_DEC_AVAILABLE, d.flags);

    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(1, bit);
    EXPECT_EQ(RLE_FLAG_DEC_AVAILABLE, d.flags);

    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0, d.flags);

    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, d.flags);
}

TEST_F(rle_fixture, rle_decode_bit_decodes_8_bit_counter_properly)
{
    EXPECT_EQ(0u, d.flags);
    set_input("000000000100000000");

    int bit;
    for (size_t i = 0; i < 255; ++i) {
        rle_decode_bit(&d, this, &static_read, &bit);
        EXPECT_EQ(0, bit);
        EXPECT_EQ(RLE_FLAG_DEC_AVAILABLE, d.flags);
    }

    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0u, d.flags);

    rle_decode_bit(&d, this, &static_read, &bit);
    EXPECT_EQ(RLE_FLAG_DEC_UNDERFLOW, d.flags);
}

