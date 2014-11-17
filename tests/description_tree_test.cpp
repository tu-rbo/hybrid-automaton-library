#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <string>

#include "hybrid_automaton/DescriptionTreeNode.h"
#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::_;

using namespace ::ha;

TEST(DescriptionTree, setAttribute) {
	using namespace ha;
	using ::testing::Return;

	MockDescriptionTreeNode mock_dtn;

	int dimensionality = 6;
	std::string dimensionality_exp = "6";

	EXPECT_CALL(mock_dtn, setAttributeString(std::string("dimensionality"), dimensionality_exp));

	mock_dtn.setAttribute<int>("dimensionality", dimensionality);
}

TEST(DescriptionTree, getAttribute) {
	using namespace ha;

	MockDescriptionTreeNode mock_dtn;

	int dimensionality = 6;
	std::string dimensionality_exp = "6";

	EXPECT_CALL(mock_dtn, getAttributeString(std::string("dimensionality"), _)).WillOnce(DoAll(SetArgReferee<1>(dimensionality_exp), Return(true)));

	int dimensionality_result;
	mock_dtn.getAttribute<int>("dimensionality", dimensionality_result);

	EXPECT_EQ(dimensionality, dimensionality_result);
}