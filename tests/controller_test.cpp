#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hybrid_automaton/HybridAutomaton.h"

//#include "hybrid_automaton_rlab/tests/MockRlabJointController.h"

using namespace std;

// ----------------------------------
// create some controller which does not register itself
class MockRegisteredController : public ha::Controller {

public:

	MockRegisteredController() : ha::Controller() {
	}

	MOCK_METHOD0(step, void());

	static Controller::Ptr instance() {
		return Controller::Ptr(new MockRegisteredController);
	}

	HA_RLAB_CONTROLLER_REGISTER_HEADER()

};

HA_RLAB_CONTROLLER_REGISTER_CPP("MockRegisteredController", MockRegisteredController)

//----------------------------

TEST(Controller, Registration) {

	using namespace ha;

	ASSERT_ANY_THROW( HybridAutomaton::createController("FantasyNonRegisteredController"));

	Controller::Ptr c = HybridAutomaton::createController("MockRegisteredController");
	ASSERT_FALSE(c.get() == NULL);
	
	// call step and make sure it does not crash
	c->step();

}
