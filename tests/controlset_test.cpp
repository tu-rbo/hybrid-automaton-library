#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hybrid_automaton/HybridAutomaton.h"

#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"

#include "hybrid_automaton/ControlSet.h"

using namespace std;

// ----------------------------------
// create some control set which does not register itself
class MockRegisteredControlSet : public ha::ControlSet {

public:

	MockRegisteredControlSet() : ha::ControlSet() {
	}

	MOCK_METHOD0(step, void());

	HA_CONTROLSET_INSTANCE(node, system) {
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

	ControlSet::Ptr c = HybridAutomaton::createControlSet(mockedNodePtr, emptySystem);
	EXPECT_FALSE(c.get() == NULL);
	
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
	ASSERT_ANY_THROW( HybridAutomaton::createControlSet(mockedNodePtr, emptySystem));

}
