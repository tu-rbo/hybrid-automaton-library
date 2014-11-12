#include <limits.h>
#include "gtest/gtest.h"

#include "hybrid_automaton/DescriptionTree.h"

TEST(TestDescriptionTree, Positive) {
	EXPECT_FALSE(4 > 5);
	EXPECT_TRUE(5 == 5);
}
 