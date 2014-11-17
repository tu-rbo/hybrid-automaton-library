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

TEST(HybridAutomatonSerialization, setAttributeVector) {
	using namespace ha;
	using ::testing::Return;

	MockDescriptionTreeNode mock_dtn;

	::Eigen::MatrixXd goal(5,1);
	goal << 1.,
		2.,
		3.,
		4.,
		5.;
	std::string goal_exp = "1\n2\n3\n4\n5";

	EXPECT_CALL(mock_dtn, setAttributeString(std::string("goal"), goal_exp));

	mock_dtn.setAttribute<::Eigen::MatrixXd>("goal", goal);
}

TEST(HybridAutomatonSerialization, getAttributeVector) {
	using namespace ha;

	MockDescriptionTreeNode mock_dtn;


	std::string goal_exp = "1\n2\n3\n4\n5";
	EXPECT_CALL(mock_dtn, getAttributeString(std::string("goal"), _)).WillOnce(DoAll(SetArgReferee<1>(goal_exp), Return(true)));

	::Eigen::MatrixXd goal_result(1,1);
	goal_result << 1.;
	mock_dtn.getAttribute<::Eigen::MatrixXd>("goal", goal_result);

	::Eigen::MatrixXd goal_true(5,1);
	goal_true << 1., 2., 3., 4., 5.;
	EXPECT_EQ(goal_result, goal_true);

	::Eigen::MatrixXd goal_false(5,1);
	goal_false << 2., 2., 2., 2., 2.;
	EXPECT_NE(goal_result, goal_false);
}

TEST(HybridAutomatonSerialization, setAttributeMatrix) {
	using namespace ha;
	using ::testing::Return;

	MockDescriptionTreeNode mock_dtn;

	::Eigen::MatrixXd goal(5,2);
	goal << 1.,2.,3.,4.,5, 6.,7.,8.,9.,10.;
	std::string goal_exp = " 1  2\n 3  4\n 5  6\n 7  8\n 9 10";

	EXPECT_CALL(mock_dtn, setAttributeString(std::string("goal"), goal_exp));

	mock_dtn.setAttribute<::Eigen::MatrixXd>("goal", goal);
}

TEST(HybridAutomatonSerialization, getAttributeMatrix) {
	using namespace ha;

	MockDescriptionTreeNode mock_dtn;


	std::string goal_exp = "1 2\n3 4\n5 6\n7 8\n9 10";
	EXPECT_CALL(mock_dtn, getAttributeString(std::string("goal"), _)).WillOnce(DoAll(SetArgReferee<1>(goal_exp), Return(true)));

	::Eigen::MatrixXd goal_result(1,1);
	goal_result << 1.;
	mock_dtn.getAttribute<::Eigen::MatrixXd>("goal", goal_result);

	::Eigen::MatrixXd goal_true(5,2);
	goal_true << 1.,2.,3.,4.,5, 6.,7.,8.,9.,10.;
	EXPECT_EQ(goal_result, goal_true);

	::Eigen::MatrixXd goal_false(5,2);
	goal_false << 2., 2., 2., 2., 2., 2., 2., 2., 2., 2.;
	EXPECT_NE(goal_result, goal_false);
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