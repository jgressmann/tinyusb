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

	~base64_fixture()
	{

	}

	base64_fixture()
	{
		base64_init(&e);
		base64_init(&d);
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
}



TEST_F(base64_fixture, encode_shift_asserts_arguments)
{
	EXPECT_ANY_THROW(base64_encode_shift(0, NULL, (volatile uint8_t*)buf, (volatile uint8_t*)buf, buf, sizeof(buf)));
	EXPECT_ANY_THROW(base64_encode_shift(0, &e, NULL, (volatile uint8_t*)buf, buf, sizeof(buf)));
	EXPECT_ANY_THROW(base64_encode_shift(0, &e, (volatile uint8_t*)buf, NULL, buf, sizeof(buf)));
	EXPECT_ANY_THROW(base64_encode_shift(0, &e, (volatile uint8_t*)buf, (volatile uint8_t*)buf, NULL, sizeof(buf)));
	EXPECT_ANY_THROW(base64_encode_shift(0, &e, (volatile uint8_t*)buf, (volatile uint8_t*)buf, buf, 0));
}

TEST_F(base64_fixture, encode_shift_asserts_arguments)
{
	EXPECT_ANY_THROW(base64_encode_shift(0, NULL, (volatile uint8_t*)buf, (volatile uint8_t*)buf, buf, sizeof(buf)));
	EXPECT_ANY_THROW(base64_encode_shift(0, &e, NULL, (volatile uint8_t*)buf, buf, sizeof(buf)));
	EXPECT_ANY_THROW(base64_encode_shift(0, &e, (volatile uint8_t*)buf, NULL, buf, sizeof(buf)));
	EXPECT_ANY_THROW(base64_encode_shift(0, &e, (volatile uint8_t*)buf, (volatile uint8_t*)buf, NULL, sizeof(buf)));
	EXPECT_ANY_THROW(base64_encode_shift(0, &e, (volatile uint8_t*)buf, (volatile uint8_t*)buf, buf, 0));
}