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
