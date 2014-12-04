#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <Eigen/Dense>

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/ControlMode.h"
#include "hybrid_automaton/ControlSwitch.h"


using namespace std;

// ----------------------------------
// create some control mode
class TestControlMode : public ha::ControlMode {

  public:
	TestControlMode(const std::string& s): ha::ControlMode(s) {};
	virtual void initialize() {};
	virtual void terminate() {};
	virtual ::Eigen::MatrixXd step(const double& t)	{ return ::Eigen::MatrixXd(0,0); }
};

using namespace ha;

class HybridAutomatonTest : public ::testing::Test {

protected:
	HybridAutomaton hybrid_automaton;
	TestControlMode::Ptr m1, m2;
	ControlSwitch::Ptr s1;

	virtual void SetUp() {
		m1.reset(new TestControlMode("m1"));
		m2.reset(new TestControlMode("m2"));

		s1.reset(new ControlSwitch());

		hybrid_automaton.addControlMode(m1);
		hybrid_automaton.addControlMode(m2);
		hybrid_automaton.addControlSwitch(m1->getName(), s1, m2->getName());
	}
};

//=========================================


using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::_;


TEST_F(HybridAutomatonTest, setCurrentControlMode) {
	// should throw an exception because the control mode doesn't exist
	EXPECT_ANY_THROW(hybrid_automaton.setCurrentControlMode("huhu"));

	// should not throw anything
	EXPECT_NO_THROW(hybrid_automaton.setCurrentControlMode("m1"));

	// should return the shit i added before
	EXPECT_TRUE(m1 == hybrid_automaton.getCurrentControlMode());
}

TEST_F(HybridAutomatonTest, activate) {
	// should throw an exception because no current control mode is defined
	EXPECT_ANY_THROW(hybrid_automaton.initialize(0.0));
	
	EXPECT_NO_THROW(hybrid_automaton.setCurrentControlMode("m1"));

	EXPECT_NO_THROW(hybrid_automaton.initialize(0.0));
}

TEST_F(HybridAutomatonTest, step) {
	ASSERT_NO_THROW(hybrid_automaton.setCurrentControlMode("m1"));
	ASSERT_NO_THROW(hybrid_automaton.initialize(0.0));

	::Eigen::MatrixXd expected_result(0,0);
	ASSERT_TRUE(expected_result == hybrid_automaton.step(0.0));
}

//TEST_F(HybridAutomatonTest, stepAndSwitch) {
//	double switching_time = 1.0;
//	TimeConditionPtr time_switch(new TimeCondition(switching_time));
//	s1->add(time_switch);
//
//	ASSERT_NO_THROW(hybrid_automaton.setCurrentControlMode("m1"));
//	ASSERT_NO_THROW(hybrid_automaton.initialize(0.0));
//
//	double time = 0.0;
//	while (time <= 2.0) {
//		hybrid_automaton.step(time);
//		if (time <= switching_time)
//			EXPECT_TRUE(m1 == hybrid_automaton.getCurrentControlMode());
//		else
//			EXPECT_TRUE(m2 == hybrid_automaton.getCurrentControlMode());
//
//		time += 0.1;
//	}
//}

//----------------------------
/*
TEST(ControlSet, UnsuccessfulRegistration) {

	System::Ptr emptySystem;

	std::string fantasyCtrlName1("FantasyNonRegisteredControlSet");

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
	ASSERT_ANY_THROW( HybridAutomaton::createControlSet(mockedNodePtr, emptySystem));

}
*/