#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <string>

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/DescriptionTreeNode.h"
#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::_;

TEST(HybridAutomatonSerialization, deserializeSimple) {
	using namespace ha;
	using ::testing::Return;

	HybridAutomaton hybaut;

	//MockDescriptionTreeNode mock_dtn;
	//EXPECT_CALL(mock_dtn, getAttribute(std::string("name")))
	//	.WillOnce(Return("example_ha"));

	//hybaut.deserialize(mock_dtn);
}

TEST(HybridAutomatonSerialization, setAttribute) {
	using namespace ha;
	using ::testing::Return;

	MockDescriptionTreeNode mock_dtn;

	int dimensionality = 6;
	std::string dimensionality_exp = "6";

	EXPECT_CALL(mock_dtn, setAttributeString(std::string("dimensionality"), dimensionality_exp));

	mock_dtn.setAttribute<int>("dimensionality", dimensionality);
}

TEST(HybridAutomatonSerialization, getAttribute) {
	using namespace ha;

	MockDescriptionTreeNode mock_dtn;

	int dimensionality = 6;
	std::string dimensionality_exp = "6";

	EXPECT_CALL(mock_dtn, getAttributeString(std::string("dimensionality"), _)).WillOnce(DoAll(SetArgReferee<1>(dimensionality_exp), Return(true)));

	int dimensionality_result;
	mock_dtn.getAttribute<int>("dimensionality", dimensionality_result);

	EXPECT_EQ(dimensionality, dimensionality_result);
}

// --------------------------------------------

class MockSerializableController : public ha::Controller {
	public:
		MOCK_CONST_METHOD1(serialize, void (const ha::DescriptionTreeNode::Ptr& tree) );
		MOCK_CONST_METHOD1(deserialize, void (const ha::DescriptionTreeNode::ConstPtr& tree) );

		HA_CONTROLLER_INSTANCE(node, system) {
			return Controller::Ptr(new MockSerializableController);
		}
};

HA_CONTROLLER_REGISTER("JointController", MockSerializableController);

/*
TEST(HybridAutomatonSerialization, minimalFullHA) {
	using namespace ha;
	using namespace std;

	//-------
	// (mocked) controller to be serialized
	Controller * _ctrl = new MockSerializableController;
	Controller::Ptr ctrl(_ctrl);
	_ctrl->setType("JointController");
	_ctrl->setName("myCtrl");

	// Mocked description returned by controller
	MockDescriptionTreeNode* _ctrl_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr ctrl_node(_ctrl_node);
	EXPECT_CALL(*_ctrl_node, getType()).WillOnce(Return("JointController"));
	EXPECT_CALL(*_ha_node, getAttribute<string>(std::string("name"), _))
		.WillOnce(DoAll(SetArgReferee<1>("myCtrl"),Return(true)));

	//-------
	// Serialized and deserialized ControlMode


	//-------
	// Serialized and deserialized HybridAutomaton
	HybridAutomaton ha;
	ha.setName("myHA");
	ha.addContr

	MockDescriptionTreeNode* _ha_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr ha_node(_ga_node);
	EXPECT_CALL(*_ha_node, getAttribute<string>(std::string("name"), _))
		.WillOnce(DoAll(SetArgReferee<1>("myHA"),Return(true)));

}
*/