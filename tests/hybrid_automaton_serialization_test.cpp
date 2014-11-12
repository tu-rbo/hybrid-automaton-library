#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/DescriptionTreeNode.h"

#include <string>

namespace ha {

	class MockDescriptionTreeNode : public DescriptionTreeNode {
	public:
		MOCK_CONST_METHOD1(getAttribute, std::string (const std::string& field_name) );
		MOCK_CONST_METHOD1(getChildNode, const DescriptionTreeNode& (const std::string& field_name) );

//		virtual std::string getAttribute(const std::string& field_name) = 0;
//		virtual const DescriptionTreeNode& getChildNode(const std::string& field_name) = 0;
	};


//class MockHybridAutomatonSerializable : public HybridAutomaton {
//
//};

}

TEST(HybridAutomatonSerialization, deserializeSimple) {
	using namespace ha;
	using ::testing::Return;

	HybridAutomaton hybaut;

	//MockDescriptionTreeNode mock_dtn;
	//EXPECT_CALL(mock_dtn, getAttribute(std::string("name")))
	//	.WillOnce(Return("example_ha"));

	//hybaut.deserialize(mock_dtn);

}
