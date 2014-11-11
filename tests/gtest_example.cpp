#include <limits.h>
#include "gtest/gtest.h"

TEST(SimpleTest, Positive) {
	EXPECT_FALSE(4 > 5);
	EXPECT_TRUE(5 == 5);
}
