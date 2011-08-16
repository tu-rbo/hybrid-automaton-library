#include "elasticroadmap\include\HybridAutomaton.h"
#include "elasticroadmap\include\MilestoneFactory.h"

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

std::string HybridAutomaton::toStringXML() const{

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
	hyb->SetAttribute("NumberNodes", nodeNumber);
	hyb->SetAttribute("NumberEdges", edgeNumber);

	if(start_node_id_ < 0)
		throw std::string("ERROR [std::string HybridAutomaton::toStringXML()] Start node is non-valid.");

	hyb->SetAttribute("StartNode", this->getInternalNodeId(start_node_id_));

	// Add the data of the nodes-milestones
	for(unsigned int i = 0; i<nodeNumber ; i++){
		if(nodeList.at(i)!=NULL){
			TiXmlElement * node = new TiXmlElement("Node");
			hyb->LinkEndChild(node);
			(nodeList.at(i))->toElementXML(node);
		}
	}

	// Add the data of the edges-motionbehaviours
	for(unsigned int i = 0; i<nodeNumber ; i++){
		for(unsigned int j = 0; j<nodeNumber ; j++){
			if( adjacencyMatrix.at(i).at(j) != NULL){
				TiXmlElement * edge = new TiXmlElement("Edge");
				hyb->LinkEndChild(edge);
				// Parent and Child indexes cannot be retrieved by the Edge itself, because they are define at the 
				// HybridAutomaton level. They must be added at this level.
				edge->SetAttribute("Parent", this->getInternalNodeId((adjacencyMatrix.at(i).at(j))->getParent()));
				edge->SetAttribute("Child", this->getInternalNodeId((adjacencyMatrix.at(i).at(j))->getChild()));
				(adjacencyMatrix.at(i).at(j))->toElementXML(edge);
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

void HybridAutomaton::fromStringXML(std::string xmlString, rxSystem* robot)
{
	// Delete the current values of the HybridAutomaton
	this->clear();

	// Create the DOM-model
	TiXmlDocument document;
	document.Parse(xmlString.c_str());
	TiXmlHandle docHandle(&document);

	// Find the first (there should be the only one) HybridAutomaton element in the base document
	TiXmlElement* hsElement =
		docHandle.FirstChild("HybridAutomaton").Element();

	// Check if the HybridAutomaton element was found
	if (hsElement == NULL) {
		throw std::string("ERROR [HybridAutomaton::fromStringXML]: Child Element \"HybridAutomaton\" not found in XML element docHandle.");
		return;
	}

	int node_number_xml=-1, edge_number_xml=-1, start_node_xml = -1;
	if(!hsElement->Attribute("NumberNodes", &node_number_xml)){
		std::cout << "WARNING [HybridAutomaton::fromStringXML]: Attribute \"NumberNodes\" not found in XML element hsElement." << std::endl;
	}

	if(!hsElement->Attribute("NumberEdges", &edge_number_xml)){
		std::cout << "WARNING [HybridAutomaton::fromStringXML]: Attribute \"NumberEdges\" not found in XML element hsElement." << std::endl;
	}

	if(!hsElement->Attribute("StartNode", &start_node_xml)){
		std::cout << "WARNING [HybridAutomaton::fromStringXML]: Attribute \"StartNode\" not found in XML element hsElement." << std::endl;
	}

	// Print out a message with the general properties of the new HybridAutomaton.
	std::cout << "Creating hybrid system '" << hsElement->Attribute("Name") << "'. Number of nodes: "<<node_number_xml << ". Number of edges: " << edge_number_xml
		<< ". Starting Node ID:" << start_node_xml << std::endl;

	MilestoneFactory * milestone_factory = new MilestoneFactory();
	// Read the data of the nodes-milestones and create them
	for (TiXmlElement* nodeElement = hsElement->FirstChildElement("Node"); nodeElement
		!= 0; nodeElement = nodeElement->NextSiblingElement("Node")) {
			Milestone* mst = milestone_factory->createMilestone(nodeElement, robot);
			this->addNode(mst);
			node_number_xml--;
			if(node_number_xml<0)
				throw std::string("ERROR [HybridAutomaton::fromStringXML]: To many nodes");
	}
	delete milestone_factory;
	// Set the start node-milestone
	this->start_node_id_ = this->nodeList[start_node_xml];

	// Read the data of the edges-motionbehaviours and create them
	for (TiXmlElement* edgeElement = hsElement->FirstChildElement("Edge"); edgeElement
		!= 0; edgeElement = edgeElement->NextSiblingElement("Edge")) {
			int parent_node, child_node;
			if(!edgeElement->Attribute("Parent", &parent_node))
			{
				throw std::string("ERROR [HybridAutomaton::fromStringXML]: Attribute \"Parent\" not found in XML element edgeElement.");
			}
			if(!edgeElement->Attribute("Child", &child_node))
			{
				throw std::string("ERROR [HybridAutomaton::fromStringXML]: Attribute \"Child\" not found in XML element edgeElement.");
			}
			MotionBehaviour* motion_behaviour = new MotionBehaviour(edgeElement, nodeList.at(parent_node), nodeList.at(child_node), robot);
			//MotionBehaviour* mb = createMotionBehaviour(edgeElement, nodeList.at(parent_node), nodeList.at(child_node), robot );
			this->addEdge(motion_behaviour);
			edge_number_xml--;
			if(edge_number_xml<0)
				throw std::string("ERROR [HybridAutomaton::fromStringXML]: To many edges");
	}
}

ostream& operator<<(ostream & out, const HybridAutomaton & hybrid_system){
	out << hybrid_system.toStringXML();
	return out;
}