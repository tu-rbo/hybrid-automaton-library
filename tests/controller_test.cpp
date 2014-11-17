#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/Controller.h"

#include "hybrid_automaton/tests/MockDescriptionTree.h"
#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"

// ----------------------------------
// create some controller which registers itself
class MockRegisteredController : public ha::Controller {

public:

	MockRegisteredController() : ha::Controller() {
	}

	//MOCK_METHOD0(step, void());

	HA_CONTROLLER_INSTANCE(node, system) {
		Controller::Ptr ctrl(new MockRegisteredController);
		ctrl->deserialize(node);
		return ctrl;
	}

};

HA_CONTROLLER_REGISTER("MockRegisteredController", MockRegisteredController)

//=========================================

using namespace std;
using namespace ha;

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::AtLeast;
using ::testing::_;


TEST(Controller, SuccessfulRegistration) {

	System::Ptr emptySystem;

	std::string ctrlType("MockRegisteredController");
	std::string ctrlName("MyCtrl");

	EXPECT_TRUE(HybridAutomaton::isControllerRegistered(ctrlType));

	// create a MockDescriptionTreeNode object
	MockDescriptionTreeNode* mockedNode = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr mockedNodePtr(mockedNode);

	EXPECT_CALL(*mockedNode, getType())
		.Times(2) // once in registration, once in deserialization
		.WillRepeatedly(Return("Controller"));

	// FIXME
	EXPECT_CALL(*mockedNode, getAttributeString(_, _))
		.WillRepeatedly(DoAll(SetArgReferee<1>(""),Return(false)));

	EXPECT_CALL(*mockedNode, getAttributeString(std::string("type"), _))
		.Times(AtLeast(1))
		.WillRepeatedly(DoAll(SetArgReferee<1>(ctrlType),Return(true)));

	EXPECT_CALL(*mockedNode, getAttributeString(std::string("name"), _))
		.Times(AtLeast(1))
		.WillRepeatedly(DoAll(SetArgReferee<1>(ctrlName),Return(true)));

	// wrap mockedNode into a smart pointer to pass to 
	// HybridAutomaton::createController.
	// (google mock somewhat does not like to use EXPECT_CALL with
	// shared pointers)

	Controller::Ptr c = HybridAutomaton::createController(mockedNodePtr, emptySystem);
	EXPECT_FALSE(c.get() == NULL);

	HybridAutomaton::unregisterController(ctrlType);
	EXPECT_FALSE(HybridAutomaton::isControllerRegistered(ctrlType));

}

//----------------------------

TEST(Controller, UnsuccessfulRegistration) {

	System::Ptr emptySystem;

	std::string fantasyCtrlType("FantasyNonRegisteredController");

	// create a MockDescriptionTreeNode object
	MockDescriptionTreeNode* mockedNode = new MockDescriptionTreeNode;

	EXPECT_CALL(*mockedNode, getType())
		.Times(1) // only once in deserialization
		.WillOnce(Return("Controller"));

	// wrap mockedNode into a smart pointer to pass to 
	// HybridAutomaton::createController.
	// (google mock somewhat does not like to use EXPECT_CALL with
	// shared pointers)
	DescriptionTreeNode::Ptr mockedNodePtr(mockedNode);

	// create controller should throw an exception because
	// FantasyNonRegisteredController was not registered
	ASSERT_ANY_THROW( HybridAutomaton::createController(mockedNodePtr, emptySystem));
}


//----------------------------



TEST(Controller, Serialization) {
	/*
	using namespace ha;
	using namespace std;

	// Controller to be serialized
	Controller * _ctrl = new Controller;
	Controller::Ptr ctrl(_ctrl);
	_ctrl->setType("JointController");
	_ctrl->setName("myCtrl");

	MockDescriptionTree* _desc_tree = new MockDescriptionTree;
	DescriptionTree::Ptr desc_tree(_desc_tree);

	// Mocked description returned by controller
	MockDescriptionTreeNode* _ctrl_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr ctrl_node(_ctrl_node);

	EXPECT_CALL(*_ctrl_node, getType()).WillOnce(Return("JointController"));

	////EXPECT_CALL(*_ctrl_node, getAttribute<string>(std::string("name"), _))
	////	.WillOnce(DoAll(SetArgReferee<1>("myCtrl"),Return(true)));

	ctrl_node = ctrl->serialize(desc_tree);
	*/
}