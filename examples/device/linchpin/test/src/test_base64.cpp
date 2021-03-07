#include <gtest/gtest.h>

#define BASE64_ASSERT(x) if (!(x)) throw 1
#define BASE64_STATIC
#define BASE64_H
#define BASE64_C
#include <base64.h>


namespace
{

struct base64_fixture : public ::testing::Test
{
    struct base64_state e;
    struct base64_state d;

    uint8_t buf[64];
    volatile uint8_t pi;
    volatile uint8_t gi;

    ~base64_fixture()
    {

    }

    base64_fixture()
    {
        base64_init(&e);
        base64_init(&d);
        gi = 0;
        pi = 0;
    }
};

} // anon

TEST(base64, init_asserts_pointers)
{
    EXPECT_ANY_THROW(base64_init(NULL));
}

TEST(base64, init_sets_default_values)
{
    struct base64_state s;
    base64_init(&s);
    EXPECT_EQ(0, s.flags);
    EXPECT_EQ(0, s.state);
    EXPECT_EQ(0, s.bits);
    EXPECT_EQ(0, s.rem);
}

TEST(base64, base64_to_ascii)
{
    for (unsigned i = 0; i < 26; ++i) {
        EXPECT_EQ('A' + i, base64_to_ascii(i));
    }

    for (unsigned i = 26; i < 52; ++i) {
        EXPECT_EQ('a' + i - 26, base64_to_ascii(i));
    }

    for (unsigned i = 52; i < 62; ++i) {
        EXPECT_EQ('0' + i - 52, base64_to_ascii(i));
    }

    EXPECT_EQ('+', base64_to_ascii(62));
    EXPECT_EQ('/', base64_to_ascii(63));
}



TEST_F(base64_fixture, encode_shift_asserts_arguments)
{
    EXPECT_ANY_THROW(base64_encode_shift(0, NULL, (volatile uint8_t*)buf, (volatile uint8_t*)buf, buf, sizeof(buf)));
    EXPECT_ANY_THROW(base64_encode_shift(0, &e, NULL, (volatile uint8_t*)buf, buf, sizeof(buf)));
    EXPECT_ANY_THROW(base64_encode_shift(0, &e, (volatile uint8_t*)buf, NULL, buf, sizeof(buf)));
    EXPECT_ANY_THROW(base64_encode_shift(0, &e, (volatile uint8_t*)buf, (volatile uint8_t*)buf, NULL, sizeof(buf)));
    EXPECT_ANY_THROW(base64_encode_shift(0, &e, (volatile uint8_t*)buf, (volatile uint8_t*)buf, buf, 0));
}

TEST_F(base64_fixture, encode_sets_overflow_if_no_space_left_in_output_buffer)
{
    EXPECT_EQ(0, e.flags);
    pi = sizeof(buf);
    base64_encode_shift(0xff, &e, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(BASE64_FLAG_OVERFLOW, e.flags);
}

TEST_F(base64_fixture, encode_shifts_in_bytes)
{
    base64_encode_shift(0xff, &e, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, e.flags);
    EXPECT_EQ(1, e.rem);
    // EXPECT_EQ(0x03, e.state);
    EXPECT_EQ(2, e.bits);
    EXPECT_EQ(1, pi);
    EXPECT_EQ('/', buf[0]);

    base64_encode_shift(0x00, &e, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, e.flags);
    EXPECT_EQ(2, e.rem);
    // EXPECT_EQ(0x00, e.state);
    EXPECT_EQ(4, e.bits);
    EXPECT_EQ(2, pi);
    EXPECT_EQ('w', buf[1]);

    base64_encode_shift(0x55, &e, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, e.flags);
    EXPECT_EQ(0, e.rem);
    // EXPECT_EQ(0x00, e.state);
    EXPECT_EQ(0, e.bits);
    EXPECT_EQ(4, pi);
    EXPECT_EQ('B', buf[2]);
    EXPECT_EQ('V', buf[3]);
}

TEST_F(base64_fixture, encode_finalize_asserts_arguments)
{
    EXPECT_ANY_THROW(base64_encode_finalize(NULL, (volatile uint8_t*)buf, (volatile uint8_t*)buf, buf, sizeof(buf)));
    EXPECT_ANY_THROW(base64_encode_finalize(&e, NULL, (volatile uint8_t*)buf, buf, sizeof(buf)));
    EXPECT_ANY_THROW(base64_encode_finalize(&e, (volatile uint8_t*)buf, NULL, buf, sizeof(buf)));
    EXPECT_ANY_THROW(base64_encode_finalize(&e, (volatile uint8_t*)buf, (volatile uint8_t*)buf, NULL, sizeof(buf)));
    EXPECT_ANY_THROW(base64_encode_finalize(&e, (volatile uint8_t*)buf, (volatile uint8_t*)buf, buf, 0));
}


TEST_F(base64_fixture, encode_finalize2)
{
    base64_encode_shift(0xff, &e, &gi, &pi, buf, sizeof(buf));
    base64_encode_finalize(&e, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, e.flags);
    EXPECT_EQ(0, e.rem);
    // EXPECT_EQ(0x00, e.state);
    // EXPECT_EQ(4, e.bits);
    EXPECT_EQ(4, pi);
    EXPECT_EQ('/', buf[0]);
    EXPECT_EQ('w', buf[1]);
    EXPECT_EQ('=', buf[2]);
    EXPECT_EQ('=', buf[3]);
}

TEST_F(base64_fixture, encode_finalize2_sets_overflow_if_no_space_left_in_output_buffer)
{
    base64_encode_shift(0xff, &e, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, e.flags);
    pi = sizeof(buf);
    base64_encode_finalize(&e, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(BASE64_FLAG_OVERFLOW, e.flags);
}


TEST_F(base64_fixture, encode_finalize1)
{
    base64_encode_shift(0xf0, &e, &gi, &pi, buf, sizeof(buf));
    base64_encode_shift(0x0f, &e, &gi, &pi, buf, sizeof(buf));
    base64_encode_finalize(&e, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, e.flags);
    EXPECT_EQ(0, e.rem);
    // EXPECT_EQ(0x00, e.state);
    // EXPECT_EQ(4, e.bits);
    EXPECT_EQ(4, pi);
    EXPECT_EQ('8', buf[0]);
    EXPECT_EQ('A', buf[1]);
    EXPECT_EQ('8', buf[2]);
    EXPECT_EQ('=', buf[3]);
}

TEST_F(base64_fixture, encode_finalize1_sets_overflow_if_no_space_left_in_output_buffer)
{
    base64_encode_shift(0xf0, &e, &gi, &pi, buf, sizeof(buf));
    base64_encode_shift(0x0f, &e, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, e.flags);
    pi = sizeof(buf);
    base64_encode_finalize(&e, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(BASE64_FLAG_OVERFLOW, e.flags);
}

TEST_F(base64_fixture, decode_shift_asserts_arguments)
{
    EXPECT_ANY_THROW(base64_decode_shift(0, NULL, (volatile uint8_t*)buf, (volatile uint8_t*)buf, buf, sizeof(buf)));
    EXPECT_ANY_THROW(base64_decode_shift(0, &d, NULL, (volatile uint8_t*)buf, buf, sizeof(buf)));
    EXPECT_ANY_THROW(base64_decode_shift(0, &d, (volatile uint8_t*)buf, NULL, buf, sizeof(buf)));
    EXPECT_ANY_THROW(base64_decode_shift(0, &d, (volatile uint8_t*)buf, (volatile uint8_t*)buf, NULL, sizeof(buf)));
    EXPECT_ANY_THROW(base64_decode_shift(0, &d, (volatile uint8_t*)buf, (volatile uint8_t*)buf, buf, 0));
}

TEST_F(base64_fixture, decode_sets_overflow_if_no_space_left_in_output_buffer)
{
    EXPECT_EQ(0, d.flags);
    pi = sizeof(buf);
    base64_decode_shift('A', &d, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, d.flags);
    base64_decode_shift('B', &d, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(BASE64_FLAG_OVERFLOW, d.flags);
}

TEST_F(base64_fixture, decode_works)
{
    EXPECT_EQ(0, d.flags);
    base64_decode_shift('A', &d, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, d.flags);
    EXPECT_EQ(6, d.bits);
    // EXPECT_EQ(0x00, d.state);
    EXPECT_EQ(0, pi);
    base64_decode_shift('B', &d, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, d.flags);
    EXPECT_EQ(4, d.bits);
    // EXPECT_EQ(0x00, d.state);
    EXPECT_EQ(1, pi);
    EXPECT_EQ(0x00, buf[0]);
    base64_decode_shift('C', &d, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, d.flags);
    EXPECT_EQ(2, d.bits);
    // EXPECT_EQ(0x00, d.state);
    EXPECT_EQ(2, pi);
    EXPECT_EQ(0x10, buf[1]);
    base64_decode_shift('D', &d, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(0, d.flags);
    EXPECT_EQ(0, d.bits);
    // EXPECT_EQ(0x00, d.state);
    EXPECT_EQ(3, pi);
    EXPECT_EQ(0x83, buf[2]);
}

// TEST_F(base64_fixture, decode_short3)
// {
// 	base64_decode_shift('/', &d, &gi, &pi, buf, sizeof(buf));
// 	base64_decode_shift('=', &d, &gi, &pi, buf, sizeof(buf));
// 	EXPECT_EQ(0, d.flags);
// 	EXPECT_EQ(0, d.bits);
// 	// EXPECT_EQ(0x00, d.state);
// 	EXPECT_EQ(1, pi);
// 	EXPECT_EQ(0xfc, buf[0]);
// 	base64_decode_shift('=', &d, &gi, &pi, buf, sizeof(buf));
// 	EXPECT_EQ(0, d.flags);
// 	EXPECT_EQ(0, d.bits);
// 	// EXPECT_EQ(0x00, d.state);
// 	EXPECT_EQ(1, pi);
// 	EXPECT_EQ(0xfc, buf[0]);

// 	// extra = does nothing
// 	base64_decode_shift('=', &d, &gi, &pi, buf, sizeof(buf));
// 	EXPECT_EQ(0, d.flags);
// 	EXPECT_EQ(0, d.bits);
// 	// EXPECT_EQ(0x00, d.state);
// 	EXPECT_EQ(1, pi);
// 	EXPECT_EQ(0xfc, buf[0]);
// }

TEST_F(base64_fixture, decode_short2)
{
    EXPECT_EQ(0, d.flags);
    base64_decode_shift('/', &d, &gi, &pi, buf, sizeof(buf));
    base64_decode_shift('B', &d, &gi, &pi, buf, sizeof(buf));
    base64_decode_shift('=', &d, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(BASE64_FLAG_DONE, d.flags);
    EXPECT_EQ(0, d.bits);
    // EXPECT_EQ(0x00, d.state);
    EXPECT_EQ(1, pi);
    EXPECT_EQ(0xfc, buf[0]);
    // EXPECT_EQ(0x10, buf[1]);
    base64_decode_shift('=', &d, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(BASE64_FLAG_DONE, d.flags);
    // EXPECT_EQ(0, d.bits);
    // EXPECT_EQ(0x00, d.state);
    EXPECT_EQ(1, pi);
    EXPECT_EQ(0xfc, buf[0]);
    // EXPECT_EQ(0x10, buf[1]);

    // extra = does nothing
    base64_decode_shift('=', &d, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(BASE64_FLAG_DONE, d.flags);
    EXPECT_EQ(0, d.bits);
    // EXPECT_EQ(0x00, d.state);
    EXPECT_EQ(1, pi);
    EXPECT_EQ(0xfc, buf[0]);
    // EXPECT_EQ(0x10, buf[1]);
}

TEST_F(base64_fixture, decode_short1)
{
    EXPECT_EQ(0, d.flags);
    base64_decode_shift('/', &d, &gi, &pi, buf, sizeof(buf));
    base64_decode_shift('B', &d, &gi, &pi, buf, sizeof(buf));
    base64_decode_shift('C', &d, &gi, &pi, buf, sizeof(buf));
    base64_decode_shift('=', &d, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(BASE64_FLAG_DONE, d.flags);
    EXPECT_EQ(0, d.bits);
    // EXPECT_EQ(0x00, d.state);
    EXPECT_EQ(2, pi);
    EXPECT_EQ(0xfc, buf[0]);
    EXPECT_EQ(0x10, buf[1]);

    // extra = does nothing
    base64_decode_shift('=', &d, &gi, &pi, buf, sizeof(buf));
    EXPECT_EQ(BASE64_FLAG_DONE, d.flags);
    EXPECT_EQ(0, d.bits);
    // EXPECT_EQ(0x00, d.state);
    EXPECT_EQ(2, pi);
    EXPECT_EQ(0xfc, buf[0]);
    EXPECT_EQ(0x10, buf[1]);
}


