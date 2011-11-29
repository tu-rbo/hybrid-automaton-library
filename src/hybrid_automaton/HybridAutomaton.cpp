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
		throw std::string("ERROR [std::string HybridAutomaton::toStringXML()] Start node is non-valid.");

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

void HybridAutomaton::fromStringXML(std::string xml_string, rxSystem* robot, double dT)
{
	// Delete the current values of the HybridAutomaton
	this->clear();

	// Create the DOM-model
	TiXmlDocument document;
	document.Parse(xml_string.c_str());
	TiXmlHandle docHandle(&document);

	// Find the first (there should be the only one) HybridAutomaton element in the base document
	TiXmlElement* hs_element =
		docHandle.FirstChild("HybridAutomaton").Element();

	// Check if the HybridAutomaton element was found
	if (hs_element == NULL) {
		throw std::string("ERROR [HybridAutomaton::fromStringXML]: Child Element \"HybridAutomaton\" not found in XML element docHandle.");
		return;
	}

	std::string start_node = std::string(hs_element->Attribute("InitialMilestone"));
	if(start_node == std::string("")){
		std::cout << "WARNING [HybridAutomaton::fromStringXML]: Attribute \"InitialMilestone\" not found in XML element hsElement." << std::endl;
	}

	// Print out a message with the general properties of the new HybridAutomaton.
	std::cout << "Creating hybrid system '" << hs_element->Attribute("Name") << "'. Initial Milestone: " << start_node << std::endl;

	// Read the data of the nodes-milestones and create them
	for (TiXmlElement* mst_element = hs_element->FirstChildElement("Milestone"); mst_element
		!= 0; mst_element = mst_element->NextSiblingElement("Milestone")) 
	{
		Milestone* mst;
		std::string mst_type = std::string(mst_element->Attribute("type"));
		if(mst_type == "CSpace"){
			mst = new CSpaceMilestone(mst_element, robot, dT);
		}else if(mst_type == "OpSpace"){
			mst = new OpSpaceMilestone(mst_element, robot, dT);
		}else{
			throw std::string("ERROR [HybridAutomaton::fromStringXML]: Wrong type of milestone.");
		}
		this->addNode(mst);
	}
	// Set the start node-milestone
	for(int i=0; i<this->nodeList.size(); i++)
	{
		if(this->nodeList[i]->getName() == start_node)
		{
			this->start_node_id_ = this->nodeList[i];
			break;
		}
		throw std::string("ERROR [HybridAutomaton::fromStringXML]: The name of the initial node does not match with the name of any milestone.");
	}

	// Read the data of the edges-motionbehaviours and create them
	for (TiXmlElement* mb_element = hs_element->FirstChildElement("MotionBehaviour"); mb_element
		!= 0; mb_element = mb_element->NextSiblingElement("MotionBehaviour")) {
			std::string ms_parent = std::string(mb_element->Attribute("Parent"));
			std::string ms_child = std::string(mb_element->Attribute("Child"));
			Milestone* ms_parent_ptr = NULL;
			Milestone* ms_child_ptr = NULL;
			if(ms_parent == std::string(""))
			{
				throw std::string("ERROR [HybridAutomaton::fromStringXML]: Attribute \"Parent\" not found in XML element edgeElement.");
			}

			for(int i=0; i<this->nodeList.size(); i++)
			{
				if(this->nodeList[i]->getName() == ms_parent)
				{
					ms_parent_ptr = this->nodeList[i];
					break;
				}
			}
			if(ms_parent_ptr == NULL)
				throw std::string("ERROR [HybridAutomaton::fromStringXML]: The name of the parent node does not match with the name of any milestone.");
			
			if(ms_child == std::string(""))
			{
				throw std::string("ERROR [HybridAutomaton::fromStringXML]: Attribute \"Child\" not found in XML element edgeElement.");
			}
			for(int i=0; i<this->nodeList.size(); i++)
			{
				if(this->nodeList[i]->getName() == ms_child)
				{
					ms_child_ptr = this->nodeList[i];
					break;
				}
			}
			if(ms_child_ptr == NULL)
				throw std::string("ERROR [HybridAutomaton::fromStringXML]: The name of the child node does not match with the name of any milestone.");

			MotionBehaviour* mb = new MotionBehaviour(mb_element, ms_parent_ptr, ms_child_ptr, robot, dT);
			this->addEdge(mb);
	}
	//SYSTEMTIME systemTime3;
	//	GetSystemTime(&systemTime3);
	//	FILETIME fileTime3;
	//	SystemTimeToFileTime(&systemTime3, &fileTime3);
	//	ULARGE_INTEGER uli3;
	//	uli3.LowPart = fileTime3.dwLowDateTime;
	//	uli3.HighPart = fileTime3.dwHighDateTime;
	//	ULONGLONG systemTimeIn_ms3(uli3.QuadPart/10000);
	//std::cout << "Time consumed milestones: " << systemTimeIn_ms2 - systemTimeIn_ms1 << std::endl;
	//std::cout << "Time consumed edges: " << systemTimeIn_ms3 - systemTimeIn_ms2 << std::endl;
	//std::cout << "Time consumed total: " << systemTimeIn_ms3 - systemTimeIn_ms1 << std::endl;
}

ostream& operator<<(ostream & out, const HybridAutomaton & hybrid_system){
	out << hybrid_system.toStringXML();
	return out;
}