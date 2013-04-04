#ifndef LOCAL_DECISION_CRITERION_
#define LOCAL_DECISION_CRITERION_

#include "hybrid_automaton\include\hybrid_automaton\HybridAutomaton.h"

/**
* This class decides which edge to switch to in a Hybrid automaton.
*/
class LocalDecisionCriterion
{
public:
	LocalDecisionCriterion();

	/**
	* Returns the next motionBehaviour to be executed. 
	* The default implementation just returns the first child of the current node and does not update during motion
	* @current The current milestone
	* @ha The hybrid automaton
	* @newBehaviour false, if this controller switch is due to a sensor update and happens regularly. In this case ha did not change from the last call.
	*/
	virtual MotionBehaviour* getNextMotionBehaviour(const Milestone* current, HybridAutomaton* ha, bool newBehaviour, double time=0.0);
};

#endif // LOCAL_DECISION_CRITERION_
