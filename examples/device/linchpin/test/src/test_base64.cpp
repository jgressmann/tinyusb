#include <gtest/gtest.h>

#define BASE64_ASSERT(x) if (!(x)) throw 1
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