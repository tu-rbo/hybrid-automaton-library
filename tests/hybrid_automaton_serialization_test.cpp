#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <string>

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/DescriptionTreeNode.h"
#include "hybrid_automaton/tests/MockDescriptionTree.h"
#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"
#include "hybrid_automaton/JointConfigurationSensor.h"

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::AtLeast;
using ::testing::_;

using namespace ::ha;

// --------------------------------------------

class MockSerializableControlMode : public ha::ControlMode {
public:
	typedef boost::shared_ptr<MockSerializableControlMode> Ptr;

	MOCK_CONST_METHOD1(serialize, DescriptionTreeNode::Ptr (const DescriptionTree::ConstPtr& factory) );
	MOCK_CONST_METHOD1(deserialize, void (const ha::DescriptionTreeNode::ConstPtr& tree) );
};

TEST(HybridAutomaton, Serialization) {
	using namespace ha;
	using namespace std;

	MockDescriptionTree::Ptr tree(new MockDescriptionTree);

	//-------
	string ctrlType("MockSerializableController");
	string ctrlSetType("MockSerializableControlSet");

	//-------
	// Serialized and deserialized ControlMode
	MockSerializableControlMode::Ptr cm1(new MockSerializableControlMode);	
	cm1->setName("myCM1");

	// Mocked node returned by control mode
	MockDescriptionTreeNode::Ptr cm1_node(new MockDescriptionTreeNode);
	EXPECT_CALL(*cm1_node, getType()).WillRepeatedly(Return("ControlMode"));
	EXPECT_CALL(*cm1_node, getAttributeString(_, _))
		.WillRepeatedly(DoAll(SetArgReferee<1>(""),Return(true)));

	EXPECT_CALL(*cm1, serialize(_))
		.WillRepeatedly(Return(cm1_node));

	//-------
	// Serialize HybridAutomaton
	HybridAutomaton ha;
	ha.setName("myHA");
	ha.addControlMode(cm1);

	// this will be the node "generated" by the tree
	MockDescriptionTreeNode::Ptr ha_node(new MockDescriptionTreeNode);

	EXPECT_CALL(*tree, createNode("HybridAutomaton"))
		.WillOnce(Return(ha_node));

	// asserts on what serialization of HA will do
	// -> child node
	EXPECT_CALL(*ha_node, addChildNode(boost::dynamic_pointer_cast<DescriptionTreeNode>(cm1_node)))
		.WillOnce(Return());
	// -> some properties (don't care)
	EXPECT_CALL(*ha_node, setAttributeString(_,_))
		.Times(AtLeast(0));

	DescriptionTreeNode::Ptr ha_serialized;
	ha_serialized = ha.serialize(tree);

}


// ------------------------------------------------

// ----------------------------------
namespace DeserializationOnlyControlMode {

class MockRegisteredController : public ha::Controller {
public:
	MockRegisteredController() : ha::Controller() {
	}

	//MOCK_METHOD0(deserialize, void (const DescriptionTreeNode::ConstPtr& tree) );
	MOCK_CONST_METHOD0(getName, std::string () );

	HA_CONTROLLER_INSTANCE(node, system, ha) {
		Controller::Ptr ctrl(new MockRegisteredController);
		return ctrl;
	}
};

HA_CONTROLLER_REGISTER("DeserializationOnlyControlModeMockRegisteredController", MockRegisteredController)
// Attention: Only use this controller in ONE test and de-register 
// in this test. Otherwise you might have weird side effects with other tests

}

TEST(HybridAutomaton, DeserializationOnlyControlMode) {
	DescriptionTreeNode::ConstNodeList cm_list;
	DescriptionTreeNode::ConstNodeList cset_list;
	DescriptionTreeNode::ConstNodeList ctrl_list;

	ha::MockDescriptionTree::Ptr tree;
	ha::MockDescriptionTreeNode::Ptr cm1_node;
	ha::MockDescriptionTreeNode::Ptr cset1_node;
	ha::MockDescriptionTreeNode::Ptr ctrl_node;
	ha::MockDescriptionTreeNode::Ptr ha_node;

	// --
	ctrl_node.reset(new MockDescriptionTreeNode);
	EXPECT_CALL(*ctrl_node, getType())
		.WillRepeatedly(Return("Controller"));
	EXPECT_CALL(*ctrl_node, getAttributeString(std::string("name"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("MyCtrl1"),Return(true)));
	EXPECT_CALL(*ctrl_node, getAttributeString(std::string("type"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("DeserializationOnlyControlModeMockRegisteredController"),Return(true)));

	ctrl_list.push_back(ctrl_node);

	// --
	cset1_node.reset(new MockDescriptionTreeNode);
	EXPECT_CALL(*cset1_node, getType())
		.WillRepeatedly(Return("ControlSet"));
	EXPECT_CALL(*cset1_node, getAttributeString(std::string("name"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("CS1"),Return(true)));
	EXPECT_CALL(*cset1_node, getChildrenNodes(std::string("Controller"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>(ctrl_list),Return(true)));

	cset_list.push_back(cset1_node);

	// --
	// Mocked ControlMode nodes
	tree.reset(new MockDescriptionTree);

	cm1_node.reset(new MockDescriptionTreeNode);
	EXPECT_CALL(*cm1_node, getType())
		.WillRepeatedly(Return("ControlMode"));
	EXPECT_CALL(*cm1_node, getAttributeString(std::string("name"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("CM1"),Return(true)));
	EXPECT_CALL(*cm1_node, getChildrenNodes(std::string("ControlSet"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>(cset_list),Return(true)));

	cm_list.push_back(cm1_node);

	//----------
	ha_node.reset(new MockDescriptionTreeNode);

	EXPECT_CALL(*ha_node, getType())
		.WillRepeatedly(Return("HybridAutomaton"));
	EXPECT_CALL(*ha_node, getAttributeString(std::string("name"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("MyHA"),Return(true)));
	
	EXPECT_CALL(*ha_node, getChildrenNodes(std::string("ControlMode"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>(cm_list),Return(true)));
	EXPECT_CALL(*ha_node, getChildrenNodes(std::string("ControlSwitch"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>(DescriptionTreeNode::ConstNodeList()),Return(true)));

}


// =============================================================

namespace DeserializationControlSet {
	
class MockRegisteredControlSet : public ha::ControlSet {
public:
	MockRegisteredControlSet() : ha::ControlSet() {
	}

	//MOCK_METHOD0(deserialize, void (const DescriptionTreeNode::ConstPtr& tree) );
	MOCK_CONST_METHOD0(getName, const std::string () );
	MOCK_CONST_METHOD1(getControllerByName, Controller::ConstPtr (const std::string& name) );

	HA_CONTROLSET_INSTANCE(node, system, ha) {
		ControlSet::Ptr ctrlSet(new MockRegisteredControlSet);
		return ctrlSet;
	}
};

HA_CONTROLSET_REGISTER("DeserializationControlSetMockRegisteredControlSet", MockRegisteredControlSet)
// Attention: Only use this controller in ONE test and de-register 
// in this test. Otherwise you might have weird side effects with other tests

}

class HybridAutomatonDeserializationTest : public ::testing::Test {
protected:
	virtual void SetUp() {
		// Mocked ControlMode nodes
		tree.reset(new MockDescriptionTree);

		// --
		ctrl_node.reset(new MockDescriptionTreeNode);
		EXPECT_CALL(*ctrl_node, getType())
			.WillRepeatedly(Return("Controller"));
		EXPECT_CALL(*ctrl_node, getAttributeString(std::string("name"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>("MyCtrl1"),Return(true)));
		EXPECT_CALL(*ctrl_node, getAttributeString(std::string("type"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>("DeserializationOnlyControlModeMockRegisteredController"),Return(true)));
		ctrl_list.push_back(ctrl_node);

		// --
		cset1_node.reset(new MockDescriptionTreeNode);
		EXPECT_CALL(*cset1_node, getType())
			.WillRepeatedly(Return("ControlSet"));
		EXPECT_CALL(*cset1_node, getAttributeString(std::string("type"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>("DeserializationControlSetMockRegisteredControlSet"),Return(true)));
		EXPECT_CALL(*cset1_node, getAttributeString(std::string("name"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>("CS1"),Return(true)));
		EXPECT_CALL(*cset1_node, getChildrenNodes(std::string("Controller"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>(ctrl_list),Return(true)));
		cset_list.push_back(cset1_node);

		// --
		cm1_node.reset(new MockDescriptionTreeNode);
		EXPECT_CALL(*cm1_node, getType())
			.WillRepeatedly(Return("ControlMode"));
		EXPECT_CALL(*cm1_node, getAttributeString(std::string("name"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>("CM1"),Return(true)));
		EXPECT_CALL(*cm1_node, getChildrenNodes(std::string("ControlSet"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>(cset_list),Return(true)));

		cm2_node.reset(new MockDescriptionTreeNode);
		EXPECT_CALL(*cm2_node, getType())
			.WillRepeatedly(Return("ControlMode"));
		EXPECT_CALL(*cm2_node, getAttributeString(std::string("name"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>("CM2"),Return(true)));
		EXPECT_CALL(*cm2_node, getChildrenNodes(std::string("ControlSet"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>(cset_list),Return(true)));

		cm_list.push_back(cm1_node);
		cm_list.push_back(cm2_node);

		// --
		cs_node.reset(new MockDescriptionTreeNode);
		EXPECT_CALL(*cs_node, getType())
			.WillRepeatedly(Return("ControlSwitch"));
		EXPECT_CALL(*cs_node, getAttributeString(std::string("name"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>(""),Return(true)));

		cs_list.push_back(cs_node);

		// --
		js_node.reset(new MockDescriptionTreeNode);
		EXPECT_CALL(*js_node, getType())
			.WillRepeatedly(Return("JumpCondition"));
		EXPECT_CALL(*js_node, getAttributeString(std::string("controller"), _))
			.WillRepeatedly(Return(false));
		EXPECT_CALL(*js_node, getAttributeString(std::string("goal"), _))
			.WillRepeatedly(Return(false));
		EXPECT_CALL(*js_node, getAttributeString(std::string("normType"), _))
			.WillRepeatedly(Return(false));
		EXPECT_CALL(*js_node, getAttributeString(std::string("epsilon"), _))
			.WillRepeatedly(Return(false));
		EXPECT_CALL(*js_node, getAttributeString(std::string("normWeights"), _))
			.WillRepeatedly(Return(false));

		js_list.push_back(js_node);

		JointConfigurationSensor jcs; // to enable registration

		ss_node.reset(new MockDescriptionTreeNode);
		EXPECT_CALL(*ss_node, getType())
			.WillRepeatedly(Return("Sensor"));
		EXPECT_CALL(*ss_node, getAttributeString(std::string("type"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>("JointConfigurationSensor"),Return(true)));
		ss_list.push_back(ss_node);


		//----------
		ha_node.reset(new MockDescriptionTreeNode);

		EXPECT_CALL(*ha_node, getType())
			.WillRepeatedly(Return("HybridAutomaton"));
		EXPECT_CALL(*ha_node, getAttributeString(std::string("name"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>("MyHA"),Return(true)));
		EXPECT_CALL(*ha_node, getAttributeString(std::string("current_control_mode"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>("CM1"),Return(true)));
		
		EXPECT_CALL(*ha_node, getChildrenNodes(std::string("ControlMode"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>(cm_list),Return(true)));
		EXPECT_CALL(*ha_node, getChildrenNodes(std::string("ControlSwitch"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>(cs_list),Return(true)));

		EXPECT_CALL(*cs_node, getChildrenNodes(std::string("JumpCondition"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>(js_list),Return(true)));

		EXPECT_CALL(*js_node, getChildrenNodes(std::string("Sensor"), _))
			.WillRepeatedly(DoAll(SetArgReferee<1>(ss_list),Return(true)));
	}

	virtual void TearDown() {
		cm_list.clear();
		cs_list.clear();
		cs_list.clear();
		ctrl_list.clear();
		ss_list.clear();
	}

	MockDescriptionTreeNode::ConstNodeList cm_list;
	MockDescriptionTreeNode::ConstNodeList ctrl_list;
	MockDescriptionTreeNode::ConstNodeList cset_list;
	MockDescriptionTreeNode::ConstNodeList cs_list;
	MockDescriptionTreeNode::ConstNodeList js_list;
	MockDescriptionTreeNode::ConstNodeList ss_list;

	MockDescriptionTree::Ptr tree;

	MockDescriptionTreeNode::Ptr cm1_node;
	MockDescriptionTreeNode::Ptr cm2_node;
	MockDescriptionTreeNode::Ptr cset1_node;
	MockDescriptionTreeNode::Ptr ctrl_node;
	MockDescriptionTreeNode::Ptr ha_node;
	MockDescriptionTreeNode::Ptr cs_node;
	MockDescriptionTreeNode::Ptr js_node;
	MockDescriptionTreeNode::Ptr ss_node;

};


TEST_F(HybridAutomatonDeserializationTest, DeserializationSuccessful) {
	using namespace ha;
	using namespace std;

	// Mocked ControlSwitch node
	EXPECT_CALL(*cs_node, getAttributeString(std::string("source"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("CM1"),Return(true)));
	EXPECT_CALL(*cs_node, getAttributeString(std::string("target"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("CM2"),Return(true)));

	// this will be the node "generated" by the tree
	HybridAutomaton ha;
	ha.deserialize(ha_node, System::Ptr());
	
}


TEST_F(HybridAutomatonDeserializationTest, GetControlModeAndGetController) {
	using namespace ha;
	using namespace std;

	// Mocked ControlSwitch node
	EXPECT_CALL(*cs_node, getAttributeString(std::string("source"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("CM1"),Return(true)));
	EXPECT_CALL(*cs_node, getAttributeString(std::string("target"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("CM2"),Return(true)));

	// this will be the node "generated" by the tree
	HybridAutomaton ha;
	
	ha.deserialize(ha_node, System::Ptr());

	ASSERT_TRUE(ha.existsControlMode("CM1"));
	ASSERT_FALSE(ha.existsControlMode("notCM1"));

	ControlMode::ConstPtr cm = ha.getControlModeByName("CM1");
	ASSERT_TRUE(cm);

	DeserializationControlSet::MockRegisteredControlSet* mockCm
		= dynamic_cast <DeserializationControlSet::MockRegisteredControlSet*>(cm->getControlSet().get());
	ASSERT_TRUE(mockCm != NULL);
	
	Controller::Ptr c(new Controller);
	c->setName("MockedControl");
	c->setType("MockedControl");

	EXPECT_CALL(*mockCm, getControllerByName(std::string("MyCtrl1")))
		.WillOnce(Return(c));

	Controller::ConstPtr cret = ha.getControllerByName("CM1", "MyCtrl1");
	EXPECT_TRUE(c);
	EXPECT_EQ("MockedControl", cret->getName());
	EXPECT_EQ("MockedControl", cret->getType());
}

TEST_F(HybridAutomatonDeserializationTest, DeserializationUnsuccessful1) {
	using namespace ha;
	using namespace std;

	// Mocked ControlSwitch node
	EXPECT_CALL(*cs_node, getAttributeString(std::string("source"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("CM1"),Return(true)));

	// setting inexistent target
	EXPECT_CALL(*cs_node, getAttributeString(std::string("target"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("Fantasia"),Return(true)));

	// this will be the node "generated" by the tree
	HybridAutomaton ha;
	
	ASSERT_ANY_THROW(ha.deserialize(ha_node, System::Ptr()));
	
}

TEST_F(HybridAutomatonDeserializationTest, DeserializationUnsuccessful2) {
	using namespace ha;
	using namespace std;

	// Mocked ControlSwitch node
	// setting inexistent target
	EXPECT_CALL(*cs_node, getAttributeString(std::string("source"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("Fantasia"),Return(true)));

	// setting inexistent target
	EXPECT_CALL(*cs_node, getAttributeString(std::string("target"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("CM2"),Return(true)));

	// this will be the node "generated" by the tree
	HybridAutomaton ha;
	
	ASSERT_ANY_THROW(ha.deserialize(ha_node, System::Ptr()));
	
}