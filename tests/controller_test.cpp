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

	HA_CONTROLLER_INSTANCE(node, system) {
		Controller::Ptr ctrl(new MockRegisteredController);
		ctrl->deserialize(node, system);
		return ctrl;
	}
};

HA_CONTROLLER_REGISTER("MockRegisteredController", MockRegisteredController)
// Attention: Only use this controller in ONE test and de-register 
// in this test. Otherwise you might have weird side effects with other tests

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
	MockDescriptionTreeNode::Ptr mockedNode(new MockDescriptionTreeNode);

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

	Controller::Ptr c = HybridAutomaton::createController(mockedNode, emptySystem);
	EXPECT_FALSE(c.get() == NULL);

	HybridAutomaton::unregisterController(ctrlType);
	EXPECT_FALSE(HybridAutomaton::isControllerRegistered(ctrlType));

}

//----------------------------


TEST(Controller, UnsuccessfulRegistration) {

	System::Ptr emptySystem;

	std::string fantasyCtrlType("FantasyNonRegisteredController");

	// create a MockDescriptionTreeNode object
	MockDescriptionTreeNode::Ptr mockedNode(new MockDescriptionTreeNode);

	EXPECT_CALL(*mockedNode, getType())
		.Times(1) // only once in deserialization
		.WillOnce(Return("Controller"));

	// create controller should throw an exception because
	// FantasyNonRegisteredController was not registered
	ASSERT_ANY_THROW( HybridAutomaton::createController(mockedNode, emptySystem));
}


// ----------------------------------
// create some controller which registers itself
namespace ControllerSerialization {

class CSMockSerializableController : public ha::Controller {
public:
	CSMockSerializableController() : ha::Controller() {
	}

	HA_CONTROLLER_INSTANCE(node, system) {
		Controller::Ptr ctrl(new CSMockSerializableController);
		ctrl->deserialize(node, system);
		return ctrl;
	}
};

HA_CONTROLLER_REGISTER("CSMockSerializableController", CSMockSerializableController)
// Attention: Only use this controller in ONE test and de-register 
// in this test. Otherwise you might have weird side effects with other tests

}

TEST(Controller, Serialization) {
	using namespace ha;
	using namespace std;

	string ctrlType("CSMockSerializableController");

	// Controller to be serialized
	Controller::Ptr ctrl(new Controller);
	ctrl->setType(ctrlType);
	ctrl->setName("myCtrl");

	MockDescriptionTree::Ptr tree(new MockDescriptionTree);

	// Mocked description returned by controller
	MockDescriptionTreeNode::Ptr ctrl_node(new MockDescriptionTreeNode);

	EXPECT_CALL(*ctrl_node, getType()).WillRepeatedly(Return("Controller"));
	EXPECT_CALL(*ctrl_node, getAttributeString(_, _))
		.WillRepeatedly(Return(false));
	EXPECT_CALL(*ctrl_node, getAttributeString(std::string("type"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("myCtrl"),Return(true)));
	EXPECT_CALL(*ctrl_node, getAttributeString(std::string("name"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>(ctrlType),Return(true)));

	EXPECT_CALL(*tree, createNode("Controller"))
		.WillOnce(Return(ctrl_node));

	ctrl->serialize(tree);

	HybridAutomaton::unregisterController(ctrlType);
	EXPECT_FALSE(HybridAutomaton::isControllerRegistered(ctrlType));
}


