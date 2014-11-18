#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hybrid_automaton/HybridAutomaton.h"

#include "hybrid_automaton/tests/MockDescriptionTree.h"
#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"

#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/ControlSet.h"

using namespace std;

// ----------------------------------
// create some control set which does not register itself
class MockRegisteredControlSet : public ha::ControlSet {

public:

	MockRegisteredControlSet() : ha::ControlSet() {
	}

	MOCK_METHOD0(step, void());

	HA_CONTROLSET_INSTANCE(node, system, ha) {
		return ControlSet::Ptr(new MockRegisteredControlSet);
	}

};

HA_CONTROLSET_REGISTER("MockRegisteredControlSet", MockRegisteredControlSet)


//=========================================

using namespace ha;

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::_;


TEST(ControlSet, SuccessfulRegistration) {

	System::Ptr emptySystem;

	std::string ctrlName1("MockRegisteredControlSet");

	EXPECT_TRUE(HybridAutomaton::isControlSetRegistered(ctrlName1));

	// create a MockDescriptionTreeNode object
	MockDescriptionTreeNode* mockedNode = new MockDescriptionTreeNode;

	EXPECT_CALL(*mockedNode, getType())
		.Times(1) // only once in deserialization
		.WillOnce(Return("ControlSet"));

	EXPECT_CALL(*mockedNode, getAttributeString(std::string("type"), _))
		.WillOnce(DoAll(SetArgReferee<1>(ctrlName1),Return(true)));

	// wrap mockedNode into a smart pointer to pass to 
	// HybridAutomaton::createController.
	// (google mock somewhat does not like to use EXPECT_CALL with
	// shared pointers)
	DescriptionTreeNode::Ptr mockedNodePtr(mockedNode);

	ControlSet::Ptr c = HybridAutomaton::createControlSet(mockedNodePtr, emptySystem, NULL);
	EXPECT_FALSE(c.get() == NULL);
	
	HybridAutomaton::unregisterControlSet(ctrlName1);
	EXPECT_FALSE(HybridAutomaton::isControlSetRegistered(ctrlName1));
}

//----------------------------

TEST(ControlSet, UnsuccessfulRegistration) {

	System::Ptr emptySystem;

	std::string fantasyCtrlName1("FantasyNonRegisteredControlSet");

	EXPECT_FALSE(HybridAutomaton::isControlSetRegistered(fantasyCtrlName1));

	// create a MockDescriptionTreeNode object
	MockDescriptionTreeNode* mockedNode = new MockDescriptionTreeNode;

	EXPECT_CALL(*mockedNode, getType())
		.Times(1) // only once in deserialization
		.WillOnce(Return("ControlSet"));

	// wrap mockedNode into a smart pointer to pass to 
	// HybridAutomaton::createController.
	// (google mock somewhat does not like to use EXPECT_CALL with
	// shared pointers)
	DescriptionTreeNode::Ptr mockedNodePtr(mockedNode);

	// create controller should throw an exception because
	// FantasyNonRegisteredController was not registered
	ASSERT_ANY_THROW( HybridAutomaton::createControlSet(mockedNodePtr, emptySystem, NULL));

}

//----------------------------

// we have to use different namespaces because static member "initializer" would have same name, but different types
namespace ControlSetSerialization1 {

class MockSerializableController : public ha::Controller {
	public:
		MOCK_CONST_METHOD1(serialize, DescriptionTreeNode::Ptr (const DescriptionTree::ConstPtr& factory) );
		MOCK_CONST_METHOD1(deserialize, void (const ha::DescriptionTreeNode::ConstPtr& tree) );

		HA_CONTROLLER_INSTANCE(node, system, ha) {
			return Controller::Ptr(new MockSerializableController);
		}
};
HA_CONTROLLER_REGISTER("MockSerializableController1", MockSerializableController);

}

namespace ControlSetSerialization2 {

class MockSerializableController : public ha::Controller {
	public:
		MOCK_CONST_METHOD1(serialize, DescriptionTreeNode::Ptr (const DescriptionTree::ConstPtr& factory) );
		MOCK_CONST_METHOD1(deserialize, void (const ha::DescriptionTreeNode::ConstPtr& tree) );

		HA_CONTROLLER_INSTANCE(node, system, ha) {
			return Controller::Ptr(new MockSerializableController);
		}
};
HA_CONTROLLER_REGISTER("MockSerializableController2", MockSerializableController);

}



TEST(ControlSet, Serialization) {

	std::string ctrlType1("MockSerializableController1");
	std::string ctrlType2("MockSerializableController2");

	// (mocked) controller to be serialized
	ControlSetSerialization1::MockSerializableController * _ctrl1 = new ControlSetSerialization1::MockSerializableController;
	Controller::Ptr ctrl1(_ctrl1);
	_ctrl1->setType(ctrlType1);
	_ctrl1->setName("myCtrl1");

	ControlSetSerialization2::MockSerializableController * _ctrl2 = new ControlSetSerialization2::MockSerializableController;
	Controller::Ptr ctrl2(_ctrl2);
	_ctrl2->setType(ctrlType2);
	_ctrl2->setName("myCtrl2");

	// this will be the control set node "generated" by the tree
	MockDescriptionTreeNode* _cs_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr cs_node(_cs_node);

	// this will be the controller nodes "generated" by the tree
	MockDescriptionTreeNode* _ctrl1_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr ctrl1_node(_ctrl1_node);
	EXPECT_CALL(*_ctrl1_node, getType()).WillRepeatedly(Return("Controller"));
	EXPECT_CALL(*_ctrl1, serialize(_)).WillOnce(Return(ctrl1_node));

	MockDescriptionTreeNode* _ctrl2_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr ctrl2_node(_ctrl2_node);
	EXPECT_CALL(*_ctrl2_node, getType()).WillRepeatedly(Return("Controller"));
	EXPECT_CALL(*_ctrl2, serialize(_)).WillOnce(Return(ctrl2_node));

	// Mocked tree factory
	MockDescriptionTree* _tree = new MockDescriptionTree;
	MockDescriptionTree::Ptr tree(_tree);
	// second call
	EXPECT_CALL(*_cs_node, addChildNode(ctrl2_node))
		.WillOnce(Return());
	// first call
	EXPECT_CALL(*_cs_node, addChildNode(ctrl1_node))
		.WillOnce(Return());

	// mocked node for control set
	EXPECT_CALL(*_tree, createNode("ControlSet"))
		.WillOnce(Return(cs_node));

	ControlSet::Ptr cs (new ControlSet);
	cs->appendController(ctrl1);
	cs->appendController(ctrl2);

	cs->serialize(tree);

}