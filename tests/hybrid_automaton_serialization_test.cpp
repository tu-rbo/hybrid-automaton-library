#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/DescriptionTreeNode.h"

#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"

#include <string>

//namespace ha {
//	class MockControlMode : public ControlMode {
//		MOCK_METHOD2(serialize, void (DescriptionTreeNode) );
//
//	};
//}

TEST(HybridAutomatonSerialization, deserializeSimple) {
	using namespace ha;
	using ::testing::Return;

	HybridAutomaton hybaut;

	//MockDescriptionTreeNode mock_dtn;
	//EXPECT_CALL(mock_dtn, getAttribute(std::string("name")))
	//	.WillOnce(Return("example_ha"));

	//hybaut.deserialize(mock_dtn);

}

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::_;

TEST(HybridAutomatonSerialization, setAttribute) {
	using namespace ha;
	using ::testing::Return;

	MockDescriptionTreeNode mock_dtn;

	int dimensionality = 6;
	std::string dimensionality_exp = "6";

	EXPECT_CALL(mock_dtn, setAttribute(std::string("dimensionality"), dimensionality_exp))
		.WillOnce();

	mock_dtn.setAttribute<int>("dimensionality", dimensionality);
}

TEST(HybridAutomatonSerialization, getAttribute) {
	using namespace ha;

	MockDescriptionTreeNode mock_dtn;

	int dimensionality = 6;
	std::string dimensionality_exp = "6";

	EXPECT_CALL(mock_dtn, getAttribute(std::string("dimensionality"), _))
		.WillOnce(SetArgReferee<1>(dimensionality_exp));

	int dimensionality_result;
	mock_dtn.getAttribute<int>("dimensionality", dimensionality_result);

	EXPECT_EQ(dimensionality, dimensionality_result);
}