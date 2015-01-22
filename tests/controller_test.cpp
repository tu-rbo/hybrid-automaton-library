#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/Controller.h"

#include "tests/MockDescriptionTree.h"
#include "tests/MockDescriptionTreeNode.h"


// ----------------------------------
// create some controller which registers itself
namespace ControllerTest {
class MockRegisteredController : public ha::Controller {
public:
	MockRegisteredController() : ha::Controller() {
	}

	HA_CONTROLLER_INSTANCE(node, system, ha) {
		Controller::Ptr ctrl(new MockRegisteredController);
		ctrl->deserialize(node, system, ha);
		return ctrl;
	}
};

HA_CONTROLLER_REGISTER("MockRegisteredController", MockRegisteredController)
// Attention: Only use this controller in ONE test and de-register 
// in this test. Otherwise you might have weird side effects with other tests
}

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

	EXPECT_CALL(*mockedNode, getAllAttributes(_))
		.Times(AtLeast(1));

	// wrap mockedNode into a smart pointer to pass to 
	// HybridAutomaton::createController.
	// (google mock somewhat does not like to use EXPECT_CALL with
	// shared pointers)

    HybridAutomaton ha;
    Controller::Ptr c = HybridAutomaton::createController(mockedNode, emptySystem, &ha);
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

	EXPECT_CALL(*mockedNode, getAttributeString(_, _))
		.Times(AtLeast(1)); // type only

	EXPECT_CALL(*mockedNode, getType())
		.Times(1) // only once in deserialization
		.WillOnce(Return("Controller"));

	// create controller should throw an exception because
	// FantasyNonRegisteredController was not registered
    HybridAutomaton ha;
    ASSERT_ANY_THROW( HybridAutomaton::createController(mockedNode, emptySystem, &ha));
}


// ----------------------------------
// create some controller which registers itself
namespace ControllerSerialization {

class CSMockSerializableController : public ha::Controller {
public:
	CSMockSerializableController() : ha::Controller() {
	}

	HA_CONTROLLER_INSTANCE(node, system, ha) {
		Controller::Ptr ctrl(new CSMockSerializableController);
		ctrl->deserialize(node, system, ha);
		return ctrl;
	}
};

HA_CONTROLLER_REGISTER("CSMockSerializableController", CSMockSerializableController)
// Attention: Only use this controller in ONE test and de-register 
// in this test. Otherwise you might have weird side effects with other tests

}

void _fillMap(std::map<std::string, std::string> & add_args) {
	add_args["extra_arg"] = "extra_val";
}

TEST(Controller, Serialization) {
	using namespace ha;
	using namespace std;

	string ctrlType("CSMockSerializableController");

	std::map<std::string, std::string> add_args;
	_fillMap(add_args);

	// Controller to be serialized
	Controller::Ptr ctrl(new Controller);
	ctrl->setType(ctrlType);
	ctrl->setName("myCtrl");
	ctrl->setArgument("extra_arg", add_args["extra_arg"]);

	MockDescriptionTree::Ptr tree(new MockDescriptionTree);

	// Mocked description returned by controller
	MockDescriptionTreeNode::Ptr ctrl_node(new MockDescriptionTreeNode);

	EXPECT_CALL(*ctrl_node, setAttributeString(_, _))
		.Times(AtLeast(0)); // we don't care about parameters yet here
	EXPECT_CALL(*ctrl_node, setAttributeString("name", "myCtrl"))
		.Times(1);
	EXPECT_CALL(*ctrl_node, setAttributeString("extra_arg", add_args["extra_arg"]))
		.Times(1);

	EXPECT_CALL(*tree, createNode("Controller"))
		.Times(1)
		.WillOnce(Return(ctrl_node));

	ctrl->serialize(tree);
}

TEST(Controller, Deserialization) {
	using ::testing::Invoke;

	string ctrlType("CSMockSerializableController");

	std::map<std::string, std::string> add_args;
	_fillMap(add_args);

	// Controller to be serialized
	MockDescriptionTree::Ptr tree(new MockDescriptionTree);

	// Mocked description returned by controller
	MockDescriptionTreeNode::Ptr ctrl_node(new MockDescriptionTreeNode);

	// deserialization - who calls all these methods?!
	EXPECT_CALL(*ctrl_node, getType()).WillRepeatedly(Return("Controller"));
	EXPECT_CALL(*ctrl_node, getAttributeString(_, _))
		.WillRepeatedly(Return(false));
	EXPECT_CALL(*ctrl_node, getAttributeString(std::string("type"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>(ctrlType),Return(true)));
	EXPECT_CALL(*ctrl_node, getAttributeString(std::string("name"), _))
		.WillRepeatedly(DoAll(SetArgReferee<1>("myCtrl"),Return(true)));
	EXPECT_CALL(*ctrl_node, getAllAttributes(_))
		.WillOnce(Invoke(_fillMap));

	Controller::Ptr ctrl(new Controller);

    HybridAutomaton ha;
    ctrl->deserialize(ctrl_node, System::Ptr(), &ha);

	EXPECT_EQ("myCtrl", ctrl->getName());
	EXPECT_EQ(ctrlType, ctrl->getType());

	std::string val;
	ctrl->getArgument("extra_arg", val);
	EXPECT_EQ(add_args["extra_arg"], val);

	HybridAutomaton::unregisterController(ctrlType);
	EXPECT_FALSE(HybridAutomaton::isControllerRegistered(ctrlType));

}

TEST(Controller, ComputeInterpolationTime) {
	Controller::Ptr ctrl(new Controller);
	ctrl->setName("myCtrl");

	Eigen::MatrixXd maxV(3,1);
	maxV<<1.0, 0.5, 1.0;
	ctrl->setMaximumVelocity(maxV);

	Eigen::MatrixXd x0(3,1);
	x0<<0.0, 2.0, -1.0;

	Eigen::MatrixXd xf(3,1);
	xf<<1.0, 1.0, -2.0;

	double tf = ctrl->computeInterpolationTime(x0, xf);
	EXPECT_DOUBLE_EQ(2.0, tf);


}
