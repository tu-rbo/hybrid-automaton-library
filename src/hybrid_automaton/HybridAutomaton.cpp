#include "HybridAutomaton.h"
//#include "MilestoneFactory.h"

#include "CSpaceMilestone.h"
#include "OpSpaceMilestone.h"

#include <Windows.h>

#include <ctime>
#include <iostream>

HybridAutomaton::HybridAutomaton() :
MDP(),
start_node_id_(NULL)
{
}

HybridAutomaton::~HybridAutomaton()
{
}

Milestone* HybridAutomaton::getStartNode() const
{
	return start_node_id_;
}

void HybridAutomaton::setStartNode(Milestone* nodeID)
{ 
	start_node_id_ = nodeID; 
}

Milestone* HybridAutomaton::getMilestoneByName(const std::string& name) const
{
	for(unsigned int i=0; i<this->nodeList.size(); i++)
	{
        Milestone* ms=(Milestone*)nodeList[i];
		if(ms->getName() == name)
		{
			return ms;
		}
	}
	return NULL;
}
