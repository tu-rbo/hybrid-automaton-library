#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/Controller.h"

#include "hybrid_automaton/tests/MockDescriptionTree.h"
#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"

// ----------------------------------
// create some controller which does not register itself
class MockRegisteredController : public ha::Controller {

public:

	MockRegisteredController() : ha::Controller() {
	}

	MOCK_METHOD0(step, void());

	HA_CONTROLLER_INSTANCE(node, system) {
		return Controller::Ptr(new MockRegisteredController);
	}

};

HA_CONTROLLER_REGISTER("MockRegisteredController", MockRegisteredController)


//=========================================

using namespace std;
using namespace ha;

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::_;


TEST(Controller, SuccessfulRegistration) {

	System::Ptr emptySystem;

	std::string ctrlName1("MockRegisteredController");

	// create a MockDescriptionTreeNode object
	MockDescriptionTreeNode* mockedNode = new MockDescriptionTreeNode;

	EXPECT_CALL(*mockedNode, getAttributeString(std::string("type"), _))
		.WillOnce(DoAll(SetArgReferee<1>(ctrlName1),Return(true)));

	// wrap mockedNode into a smart pointer to pass to 
	// HybridAutomaton::createController.
	// (google mock somewhat does not like to use EXPECT_CALL with
	// shared pointers)
	DescriptionTreeNode::Ptr mockedNodePtr(mockedNode);

	Controller::Ptr c = HybridAutomaton::createController(mockedNodePtr, emptySystem);
	EXPECT_FALSE(c.get() == NULL);
	
}

//----------------------------

TEST(Controller, UnsuccessfulRegistration) {

	System::Ptr emptySystem;

	std::string fantasyCtrlName1("FantasyNonRegisteredController");

	// create a MockDescriptionTreeNode object
	MockDescriptionTreeNode* mockedNode = new MockDescriptionTreeNode;

	EXPECT_CALL(*mockedNode, getAttributeString(std::string("type"), _))
		.WillOnce(DoAll(SetArgReferee<1>(fantasyCtrlName1),Return(true)));

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
	//EXPECT_CALL(*_ctrl_node, getAttribute<string>(std::string("name"), _))
	//	.WillOnce(DoAll(SetArgReferee<1>("myCtrl"),Return(true)));

	ctrl_node = ctrl->serialize(desc_tree);
}