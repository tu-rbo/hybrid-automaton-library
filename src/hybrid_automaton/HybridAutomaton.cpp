#include "HybridAutomaton.h"
//#include "MilestoneFactory.h"

#include "CSpaceMilestone.h"
#include "OpSpaceMilestone.h"

#include <Windows.h>

#include <ctime>
#include <iostream>

HybridAutomaton::HybridAutomaton() :
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
	for(int i=0; i<this->nodeList.size(); i++)
	{
		if(this->nodeList[i]->getName() == name)
		{
			return this->nodeList[i];
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
		if(nodeList.at(i)!=NULL){
			TiXmlElement * mst_element = (nodeList.at(i))->toElementXML();
			hyb->LinkEndChild(mst_element);			
		}
	}

	// Add the data of the edges-motionbehaviours
	for(unsigned int i = 0; i<nodeNumber ; i++){
		for(unsigned int j = 0; j<nodeNumber ; j++){
			if( adjacencyMatrix.at(i).at(j) != NULL){
				TiXmlElement * mb_element = (adjacencyMatrix.at(i).at(j))->toElementXML();
				hyb->LinkEndChild(mb_element);
				// Parent and Child indexes cannot be retrieved by the Edge itself, because they are define at the 
				// HybridAutomaton level. They must be added at this level.
				mb_element->SetAttribute("Parent", (adjacencyMatrix.at(i).at(j))->getParent()->getName().c_str());
				mb_element->SetAttribute("Child", (adjacencyMatrix.at(i).at(j))->getChild()->getName().c_str());
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