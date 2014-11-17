#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <string>

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/DescriptionTreeNode.h"
#include "hybrid_automaton/tests/MockDescriptionTree.h"
#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::AtLeast;
using ::testing::_;

using namespace ::ha;

TEST(HybridAutomatonSerialization, setAttributeVector) {
	using namespace ha;
	using ::testing::Return;

	MockDescriptionTreeNode mock_dtn;

	::Eigen::MatrixXd goal(5,1);
	goal << 1.,2.,3.,4.,5;
	std::string goal_exp = "1\n2\n3\n4\n5";

	EXPECT_CALL(mock_dtn, setAttributeString(std::string("goal"), goal_exp));

	mock_dtn.setAttribute<::Eigen::MatrixXd>("goal", goal);
}

TEST(HybridAutomatonSerialization, getAttributeVector) {
	using namespace ha;

	MockDescriptionTreeNode mock_dtn;

	::Eigen::MatrixXd goal(1,1);
	goal << 1.,2.,3.,4.,5;
	std::string goal_exp = "1\n2\n3\n4\n5";

	EXPECT_CALL(mock_dtn, getAttributeString(std::string("goal"), _)).WillOnce(DoAll(SetArgReferee<1>(goal_exp), Return(true)));

	::Eigen::MatrixXd goal_result(1,1);
	goal_result << 1.;
	mock_dtn.getAttribute<::Eigen::MatrixXd>("goal", goal_result);

	EXPECT_EQ(goal, goal_result);
}

TEST(HybridAutomatonSerialization, setAttributeMatrix) {
	using namespace ha;
	using ::testing::Return;

	MockDescriptionTreeNode mock_dtn;

	::Eigen::MatrixXd goal(5,2);
	goal << 1.,2.,3.,4.,5, 6.,7.,8.,9.,10.;
	std::string goal_exp = "1 2\n3 4\n5 6\n7 8\n9 10";

	EXPECT_CALL(mock_dtn, setAttributeString(std::string("goal"), goal_exp));

	mock_dtn.setAttribute<::Eigen::MatrixXd>("goal", goal);
}

TEST(HybridAutomatonSerialization, getAttributeMatrix) {
	using namespace ha;

	MockDescriptionTreeNode mock_dtn;

	::Eigen::MatrixXd goal(5,1);
	goal << 1.,2.,3.,4.,5;
	std::string goal_exp = "1\n2\n3\n4\n5";

	EXPECT_CALL(mock_dtn, getAttributeString(std::string("goal"), _)).WillOnce(DoAll(SetArgReferee<1>(goal_exp), Return(true)));

	::Eigen::MatrixXd goal_result(5,1);
	goal_result << 1.,2.,3.,4.,5;
	mock_dtn.getAttribute<::Eigen::MatrixXd>("goal", goal_result);

	EXPECT_EQ(goal, goal_result);
}

// --------------------------------------------

class MockSerializableControlMode : public ha::ControlMode {
	public:
		MOCK_CONST_METHOD1(serialize, DescriptionTreeNode::Ptr (const DescriptionTree::ConstPtr& factory) );
		MOCK_CONST_METHOD1(deserialize, void (const ha::DescriptionTreeNode::ConstPtr& tree) );
};

TEST(HybridAutomaton, Serialization) {
	using namespace ha;
	using namespace std;

	MockDescriptionTree* _tree = new MockDescriptionTree;
	MockDescriptionTree::Ptr tree(_tree);

	//-------
	string ctrlType("MockSerializableController");
	string ctrlSetType("MockSerializableControlSet");

	//-------
	// Serialized and deserialized ControlMode
	MockSerializableControlMode* _cm1 = new MockSerializableControlMode;
	ControlMode::Ptr cm1(_cm1);	
	cm1->setName("myCM1");

	// Mocked node returned by control mode
	MockDescriptionTreeNode* _cm1_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr cm1_node(_cm1_node);
	EXPECT_CALL(*_cm1_node, getType()).WillRepeatedly(Return("ControlMode"));
	EXPECT_CALL(*_cm1_node, getAttributeString(_, _))
		.WillRepeatedly(DoAll(SetArgReferee<1>(""),Return(true)));

	EXPECT_CALL(*_cm1, serialize(_))
		.WillRepeatedly(Return(cm1_node));
	
	//-------
	// Serialize HybridAutomaton
	HybridAutomaton ha;
	ha.setName("myHA");
	ha.addControlMode(cm1);

	// this will be the node "generated" by the tree
	MockDescriptionTreeNode* _ha_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr ha_node(_ha_node);

	EXPECT_CALL(*_tree, createNode("HybridAutomaton"))
		.WillOnce(Return(ha_node));

	// asserts on what serialization of HA will do
	// -> child node
	EXPECT_CALL(*_ha_node, addChildNode(cm1_node))
		.WillOnce(Return());
	// -> some properties (don't care)
	EXPECT_CALL(*_ha_node, setAttributeString(_,_))
		.Times(AtLeast(0));

	//MockDescriptionTreeNode* _ha_node = new MockDescriptionTreeNode;
	//DescriptionTreeNode::Ptr ha_node(_ha_node);
	//EXPECT_CALL(*_ha_node, getAttributeString(std::string("name"), _))
	//	.WillOnce(DoAll(SetArgReferee<1>("myHA"),Return(true)));

	DescriptionTreeNode::Ptr ha_serialized;
	ha_serialized = ha.serialize(tree);
	
}
