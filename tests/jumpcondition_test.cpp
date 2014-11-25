#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <string>

#include "hybrid_automaton/JumpCondition.h"
#include "hybrid_automaton/DescriptionTreeNode.h"
#include "hybrid_automaton/JointConfigurationSensor.h"
#include "tests/MockDescriptionTree.h"
#include "tests/MockDescriptionTreeNode.h"

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::AtLeast;
using ::testing::_;

using namespace ::ha;

namespace JumpConditionSerialization1 {
	class MockSystem : public ha::System {
	public:
		MOCK_CONST_METHOD0(getDof, int () );
		MOCK_CONST_METHOD0(getConfiguration, ::Eigen::MatrixXd () );
		MOCK_CONST_METHOD0(getForceTorqueMeasurement, ::Eigen::MatrixXd () );
		MOCK_CONST_METHOD0(getCurrentTime, ::Eigen::MatrixXd () );
		MOCK_CONST_METHOD1(getFramePose, ::Eigen::MatrixXd (const std::string& frame_id) );
	};
}

TEST(JumpCondition, Serialization) {
	using namespace ha;
	using namespace std;

	string ctrlSetType("MockSerializableControlSet");

	//-------
	// Serialized and deserialized sensor

	System::ConstPtr _ms = System::ConstPtr(new JumpConditionSerialization1::MockSystem());	
	JointConfigurationSensorPtr js_s = JointConfigurationSensorPtr(new JointConfigurationSensor()); 

	js_s->setSystem(_ms);

	// Mocked description returned by jump condition
	MockDescriptionTreeNode* _jc_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr jc_node(_jc_node);

	EXPECT_CALL(*_jc_node, getType()).WillRepeatedly(Return("JumpCondition"));
	EXPECT_CALL(*_jc_node, getAttributeString(_, _))
		.WillRepeatedly(DoAll(SetArgReferee<1>(""),Return(true)));

	//-------
	// TEST ControlMode

	// Mocked tree factory
	MockDescriptionTree* _tree = new MockDescriptionTree;
	MockDescriptionTree::Ptr tree(_tree);

	JumpCondition* _jc1 = new JumpCondition;
	JumpCondition::Ptr jc1(_jc1);	

	MockDescriptionTreeNode::Ptr cm1_node_faulty(new MockDescriptionTreeNode);
	EXPECT_CALL(*cm1_node_faulty, setAttributeString(_,_))
		.Times(AtLeast(0)); 
	// assert throwing of exception if no sensor has been set
	EXPECT_CALL(*_tree, createNode("JumpCondition"))
		.WillOnce(Return(cm1_node_faulty));

	ASSERT_ANY_THROW(jc1->serialize(tree));
	
	jc1->setSensor(js_s);

	// this will be the node "generated" by the tree
	MockDescriptionTreeNode* _jc1_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr jc1_node(_jc1_node);

	// and one for the sensot
	MockDescriptionTreeNode* _sensor_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr sensor_node(_sensor_node);

	EXPECT_CALL(*_tree, createNode("JumpCondition"))
	.WillOnce(Return(jc1_node));

	EXPECT_CALL(*_tree, createNode("Sensor"))
	.WillOnce(Return(sensor_node));

	// Expect that exactly one child node = sensor will be added as child node
	EXPECT_CALL(*_jc1_node, addChildNode(_))
	.Times(AtLeast(1));
	EXPECT_CALL(*_sensor_node, addChildNode(_))
	.Times(AtLeast(0));

	// -> some properties (don't care)
	EXPECT_CALL(*_jc1_node, setAttributeString(_,_))
	.Times(AtLeast(0));
	EXPECT_CALL(*_sensor_node, setAttributeString(_,_))
	.Times(AtLeast(0));


	DescriptionTreeNode::Ptr jc_serialized;
	jc_serialized = jc1->serialize(tree);
}
