#include "LocalDecisionCriterion.h"

LocalDecisionCriterion::LocalDecisionCriterion()
{
}

MotionBehaviour* LocalDecisionCriterion::getNextMotionBehaviour(const Milestone* current, HybridAutomaton* ha, bool newBehaviour, double time, dVector q)
{
	if(newBehaviour)
	{
		std::vector<const Edge*> edges = ha->outgoingEdges(current);
		if(edges.size() > 0)
		{
			return (MotionBehaviour*)(edges[0]);			
		}
	}

	return NULL;
}

