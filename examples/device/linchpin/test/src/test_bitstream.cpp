#include <gtest/gtest.h>

#define BS_ASSERT(x) if (!(x)) throw 1
#include <bitstream.h>

namespace
{
int error_code = 0;
uint8_t byte;

int read_byte(void*, uint8_t* byte_ptr)
{
    *byte_ptr = byte;
    return error_code;
}

int write_byte(void*, uint8_t _byte)
{
    byte = _byte;
    return error_code;
}

} // anon

TEST(bitstream, init_asserts_pointers)
{
    EXPECT_ANY_THROW(bs_init(nullptr));
}

TEST(bitstream, read_asserts_pointers)
{
    int bit;
    struct bitstream s;

    bs_init(&s);

    EXPECT_ANY_THROW(bs_read(nullptr, nullptr, &read_byte, &bit));
    EXPECT_ANY_THROW(bs_read(&s, nullptr, nullptr, &bit));
    EXPECT_ANY_THROW(bs_read(&s, nullptr, &read_byte, nullptr));
}

TEST(bitstream, read_returns_the_error_code_from_the_callback)
{
    int bit;
    struct bitstream s;
    bs_init(&s);

    error_code = 42;

    EXPECT_EQ(42, bs_read(&s, nullptr, &read_byte, &bit));

    error_code = 0;
}


TEST(bitstream, read_unpacks_bytes)
{
    int bit;
    struct bitstream s;
    bs_init(&s);

    byte = 0xe2;



    EXPECT_EQ(0, bs_read(&s, nullptr, &read_byte, &bit));
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0, bs_read(&s, nullptr, &read_byte, &bit));
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0, bs_read(&s, nullptr, &read_byte, &bit));
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0, bs_read(&s, nullptr, &read_byte, &bit));
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0, bs_read(&s, nullptr, &read_byte, &bit));
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0, bs_read(&s, nullptr, &read_byte, &bit));
    EXPECT_EQ(0, bit);
    EXPECT_EQ(0, bs_read(&s, nullptr, &read_byte, &bit));
    EXPECT_EQ(1, bit);
    EXPECT_EQ(0, bs_read(&s, nullptr, &read_byte, &bit));
    EXPECT_EQ(0, bit);
    error_code = 22;
    EXPECT_EQ(22, bs_read(&s, nullptr, &read_byte, &bit));
    error_code = 0;
}

TEST(bitstream, write_asserts_pointers)
{
    struct bitstream s;

    bs_init(&s);

    EXPECT_ANY_THROW(bs_write(nullptr, nullptr, &write_byte, 0));
    EXPECT_ANY_THROW(bs_write(&s, nullptr, nullptr, 0));
}

TEST(bitstream, write_packs_bytes)
{
    struct bitstream s;
    bs_init(&s);


    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 0));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 0));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 0));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 1));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 1));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 1));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 1));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 0));
    EXPECT_EQ(0x1e, byte);
}

TEST(bitstream, write_returns_the_error_code_from_the_callback)
{
    struct bitstream s;
    bs_init(&s);

    error_code = 42;
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 0));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 0));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 0));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 1));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 1));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 1));
    EXPECT_EQ(0, bs_write(&s, nullptr, &write_byte, 1));
    EXPECT_EQ(42, bs_write(&s, nullptr, &write_byte, 0));
    error_code = 0;
}

