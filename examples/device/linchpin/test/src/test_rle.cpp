#include <gtest/gtest.h>

#define RLE_ASSERT(x) if (!(x)) throw 1
#define RLE_H
#define RLE_C
#include <rle.h>

#include <string>



namespace
{

struct rle_fixture : public ::testing::Test
{
    struct rle e;
    struct rle d;

    std::string output;
    static rle_fixture* self;


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

TEST_F(rle_fixture, rle_encode_flush_throws_on_invalid_params)
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

TEST_F(rle_fixture, rle_encode_push_bit_throws_on_invalid_params)
{
    EXPECT_ANY_THROW(rle_encode_push_bit(NULL, 1));
    EXPECT_ANY_THROW(rle_encode_push_bit(NULL, 0));
}

TEST_F(rle_fixture, encode_0_yields_expected_output)
{
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));
    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(1u, e.count);
    EXPECT_EQ(RLE_FLAG_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("01", output.c_str());
}

TEST_F(rle_fixture, encode_1_yields_expected_output)
{
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 1));
    EXPECT_EQ(1u, e.value);
    EXPECT_EQ(1u, e.count);
    EXPECT_EQ(RLE_FLAG_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("10", output.c_str());
}

TEST_F(rle_fixture, encode_00_yields_expected_output)
{
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 0));

    EXPECT_EQ(0u, e.value);
    EXPECT_EQ(2u, e.count);
    EXPECT_EQ(RLE_FLAG_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("0010", output.c_str());
}

TEST_F(rle_fixture, encode_11_yields_expected_output)
{
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 1));
    EXPECT_EQ(RLEE_NONE, rle_encode_push_bit(&e, 1));

    EXPECT_EQ(1u, e.value);
    EXPECT_EQ(2u, e.count);
    EXPECT_EQ(RLE_FLAG_VALUE, e.flags);

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
    EXPECT_EQ(RLE_FLAG_VALUE, e.flags);

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
    EXPECT_EQ(RLE_FLAG_VALUE, e.flags);

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
    EXPECT_EQ(RLE_FLAG_VALUE, e.flags);

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
    EXPECT_EQ(RLE_FLAG_VALUE, e.flags);

    EXPECT_EQ(RLEE_NONE, rle_encode_flush(&e));
    EXPECT_STREQ("000000000000000000000100000000000000000000", output.c_str());
}

