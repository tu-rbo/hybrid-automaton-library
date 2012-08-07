#ifndef LOCAL_DECISION_CRITERION_
#define LOCAL_DECISION_CRITERION_

#include "Milestone.h"

class LocalDecisionCriterion
{

public:

	virtual bool isConnected(const Milestone* ms1, const Milestone* ms2) = 0;

	virtual double getProbability(const Milestone* ms1, const Milestone* ms2) = 0;

};

#endif // LOCAL_DECISION_CRITERION_
