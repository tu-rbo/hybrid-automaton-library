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

MotionBehaviour* HybridAutomaton::getNextMotionBehaviour(const Milestone* currentMs, LocalDecisionCriterion* criterion)
{
	std::vector<MDPEdge*> edges = getSortedOutgoingEdges(currentMs);
	if(edges.size() > 0 )
	{
		if(!criterion)
		{
			return (MotionBehaviour*)(edges[0]);
		}
		else
		{
			for(int i = 0; i < edges.size(); i++)
			{
				if(criterion->isConnected(currentMs, (Milestone*)edges[i]->getChild()))
				{
					return (MotionBehaviour*)(edges[i]);
				}
			}
		}
	}
	return NULL;
}

std::string HybridAutomaton::toStringXML() const
{
	// If the HybridAutomaton is empty, return the corresponding string
	if(nodeNumber==0 && edgeNumber==0)
		return std::string("Empty HybridAutomaton");

	// Create the base document
	TiXmlDocument document;
	// Add the declaration
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	// Create the first (and only) root element and link it to the base document
	TiXmlElement * hyb = new TiXmlElement("HybridAutomaton");
	document.LinkEndChild(hyb);

	// Every new no-empty HybridAutomaton gets a new name
	static int name_counter = 0;
	std::string name_str;
	std::stringstream name_ss;
	name_ss << name_counter;
	name_str = name_ss.str();
	std::string name_complete = std::string("HS") + name_str;
	name_counter++;

	hyb->SetAttribute("Name", name_complete.c_str());

	if(start_node_id_ < 0)
		throw std::string("[HybridAutomaton::toStringXML] ERROR: Start node is non-valid.");

	hyb->SetAttribute("InitialMilestone", start_node_id_->getName().c_str());

	// Add the data of the nodes-milestones
	for(unsigned int i = 0; i<nodeNumber ; i++){
        Milestone* ms=(Milestone*)nodeList.at(i);
		if(ms!=NULL){
			TiXmlElement * mst_element = ms->toElementXML();
			hyb->LinkEndChild(mst_element);			
		}
	}

	// Add the data of the edges-motionbehaviours
	for(unsigned int i = 0; i<nodeNumber ; i++){
		for(unsigned int j = 0; j<nodeNumber ; j++){
            MotionBehaviour* mb=(MotionBehaviour*)adjacencyMatrix.at(i).at(j); 
			if( mb != NULL){
				TiXmlElement * mb_element = mb->toElementXML();
				hyb->LinkEndChild(mb_element);
			}
		}
	}

	// Declare a printer
	TiXmlPrinter printer;
	// Attach it to the document you want to convert in to a std::string
	document.Accept(&printer);
	// Create a std::string and copy your document data in to the string
	std::string ret_val = printer.CStr();

	//TODO: Memory leaks?
	return ret_val;
}

ostream& operator<<(ostream & out, const HybridAutomaton & hybrid_system){
	out << hybrid_system.toStringXML();
	return out;
}