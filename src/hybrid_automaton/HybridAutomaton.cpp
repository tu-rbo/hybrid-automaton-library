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

MotionBehaviour* HybridAutomaton::getNextMotionBehaviour(const Milestone* currentMs, LocalDecisionCriterion* criterion, vector<pair<std::string,std::string>>* bad_edges)
{
	std::vector<const MDPEdge*> edges = getSortedOutgoingEdges(currentMs);
	if(edges.size() > 0 )
	{
		if(!criterion)
		{
			return (MotionBehaviour*)(edges[0]);
		}
		else
		{
			for(unsigned int i = 0; i < edges.size(); i++)
			{
				if(criterion->isConnected(currentMs, (Milestone*)edges[i]->getChild()))
				{
					if(bad_edges)
					{
						for(int j = 0; j < bad_edges->size(); j++)
						{
							// problem: very often the edges are from current_ms!!!
							if((*bad_edges)[j].first == currentMs->getName() && (*bad_edges)[j].second == ((Milestone*)edges[i]->getChild())->getName())
							{
								cout << "** IGNORING BAD EGDE: " << currentMs->getName() << " to " << ((Milestone*)edges[i]->getChild())->getName() <<std::endl;
								continue; // dont select bad_edge!
							}
						}
					}
					else if(edges[i]->getLength() < 0.5)
					{
						cout << "** IGNORING SHORT EDGE: " << currentMs->getName() << " to " << ((Milestone*)edges[i]->getChild())->getName() <<std::endl;
						continue;
					}
					return (MotionBehaviour*)(edges[i]);
				}
			}
		}
	}
	return NULL;
}