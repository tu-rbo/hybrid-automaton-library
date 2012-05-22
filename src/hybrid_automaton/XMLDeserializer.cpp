#include "XMLDeserializer.h"

#include "CSpaceMilestone.h"
#include "OpSpaceMilestone.h"

#include "rControlalgorithm\rControlalgorithm.h"
#include "rxControlSDK\rxControlSDK.h"

XMLDeserializer::XMLDeserializer(TiXmlElement * xml_element_in):
xml_element(xml_element_in)
{
}

XMLDeserializer::~XMLDeserializer()
{
}

HybridAutomaton* XMLDeserializer::createHybridAutomaton(const std::string& xml_string, rxSystem* robot, double dT)
{
	HybridAutomaton* automaton = new HybridAutomaton();

	// Create the DOM-model
 	TiXmlDocument document;
	if(document.Parse(xml_string.c_str()) == NULL) 	{
		std::cout << document.ErrorDesc() << std::endl;
		delete automaton;
		return NULL;
	}
	TiXmlHandle docHandle(&document);

	// Find the first (there should be the only one) HybridAutomaton element in the base document
	TiXmlElement* ha_element = docHandle.FirstChild("HybridAutomaton").Element();

	// Check if the HybridAutomaton element was found
	if (ha_element == NULL) {
		throw std::string("ERROR [HybridAutomaton::fromStringXML]: Child Element \"HybridAutomaton\" not found in XML element docHandle.");
		delete automaton;
		return NULL;
	}

	std::string start_node = std::string(ha_element->Attribute("InitialMilestone"));
	if(start_node == std::string("")) {
		std::cout << "WARNING [HybridAutomaton::fromStringXML]: Attribute \"InitialMilestone\" not found in XML element hsElement." << std::endl;
		delete automaton;
		return NULL;
	}

	// Print out a message with the general properties of the new HybridAutomaton.
	std::cout << "Creating hybrid system '" << ha_element->Attribute("Name") << "'. Initial Milestone: " << start_node << std::endl;

	// Read the data of the nodes-milestones and create them
	for (TiXmlElement* mst_element = ha_element->FirstChildElement("Milestone"); mst_element != 0; mst_element = mst_element->NextSiblingElement("Milestone")) {
		Milestone* mst;
		std::string mst_type(mst_element->Attribute("type"));
		if(mst_type == "CSpace")
		{
			mst = new CSpaceMilestone(mst_element, robot, dT);
		}
		else if(mst_type == "OpSpace")
		{
			mst = new OpSpaceMilestone(mst_element, robot, dT);
		}
		else
		{
			throw ("ERROR [HybridAutomaton::fromStringXML]: Unknown type of milestone: " + mst_type);
			delete automaton;
			return NULL;
		}
		automaton->addNode(mst);
	}

	// Set the start node-milestone
	automaton->setStartNode(automaton->getMilestoneByName(start_node));

	if (!automaton->getStartNode()) {
		throw ("ERROR [HybridAutomaton::fromStringXML]: The name of the initial node '" + start_node + "' does not match with the name of any milestone.");
		delete automaton;
		return NULL;
	}

	// Read the data of the edges-motionbehaviours and create them
	for (TiXmlElement* mb_element = ha_element->FirstChildElement("MotionBehaviour"); mb_element != 0; mb_element = mb_element->NextSiblingElement("MotionBehaviour")) {
		std::string ms_parent(mb_element->Attribute("Parent"));
		std::string ms_child(mb_element->Attribute("Child"));
		if(ms_parent == std::string(""))
		{
			throw std::string("ERROR [HybridAutomaton::fromStringXML]: Attribute \"Parent\" not found in XML element edgeElement.");
			delete automaton;
			return NULL;
		}

		Milestone* ms_parent_ptr = automaton->getMilestoneByName(ms_parent);

		if(ms_parent_ptr == NULL) {
			throw std::string("ERROR [HybridAutomaton::fromStringXML]: The name of the parent node does not match with the name of any milestone.");
			delete automaton;
			return NULL;
		}
		
		if(ms_child == std::string(""))
		{
			throw std::string("ERROR [HybridAutomaton::fromStringXML]: Attribute \"Child\" not found in XML element edgeElement.");
			delete automaton;
			return NULL;
		}
		
		Milestone* ms_child_ptr = automaton->getMilestoneByName(ms_child);

		if(ms_child_ptr == NULL)
		{
			throw ("ERROR [HybridAutomaton::fromStringXML]: The name of the child node '" + ms_child + "' does not match the name of any milestone.");
			delete automaton;
			return NULL;
		}

		MotionBehaviour* mb = new MotionBehaviour(mb_element, ms_parent_ptr, ms_child_ptr, robot, dT);
		automaton->addEdge(mb);
	}

	return automaton;
}


bool XMLDeserializer::deserializeBoolean(const char * field_name)
{
	const char* value = xml_element->Attribute(field_name);

	if (value == NULL)
	{
		std::string exception_str = std::string("ERROR: [XMLDeserializer::deserializeBoolean(const char * field_name)] Attribute ") + std::string(field_name) + std::string(" was not found in XML element.");
		throw exception_str;
	}
	else if (strcmp(value, "true") == 0)
	{
		return true;
	}
	else if (strcmp(value, "false") == 0)
	{
		return false;
	}
	std::string exception_str = std::string("ERROR: [XMLDeserializer::deserializeBoolean(const char * field_name)] Attribute ") + std::string(field_name) + std::string(" was not found in XML element.");
	throw exception_str;
}

bool XMLDeserializer::deserializeBoolean(const char * field_name, bool default_value)
{
	const char* value = xml_element->Attribute(field_name);

	if (value == NULL)
	{
		return default_value;
	}
	else if (strcmp(value, "true") == 0)
	{
		return true;
	}
	else if (strcmp(value, "false") == 0)
	{
		return false;
	}
	return default_value;
}

int XMLDeserializer::deserializeInteger(const char * field_name)
{
	int return_integer = -1;
	if(!xml_element->Attribute(field_name, &return_integer))
	{
		std::string exception_str = std::string("ERROR: [XMLDeserializer::deserializeInteger(const char * field_name)] Attribute ") + std::string(field_name) + std::string(" was not found in XML element.");
		throw exception_str;
	}
	return return_integer;
}

int XMLDeserializer::deserializeInteger(const char * field_name, int default_value)
{
	int return_integer = default_value;
	xml_element->Attribute(field_name, &return_integer);
	return return_integer;
}

double XMLDeserializer::deserializeDouble(const char * field_name)
{
	double return_double = -1.0;
	if(!xml_element->Attribute(field_name, &return_double))
	{
		std::string exception_str = std::string("ERROR: [XMLDeserializer::deserializeDouble(const char * field_name)] Attribute ") + std::string(field_name) + std::string(" was not found in XML element.");
		throw exception_str;
	}
	return return_double;
}

double XMLDeserializer::deserializeDouble(const char * field_name, double default_value)
{
	double return_double = default_value;
	xml_element->Attribute(field_name, &return_double);
	return return_double;
}

std::string XMLDeserializer::deserializeString(const char * field_name)
{
	std::string return_string = std::string(xml_element->Attribute(field_name));
	if(return_string == std::string(""))
	{
		std::string exception_str = std::string("ERROR: [XMLDeserializer::deserializeString(const char * field_name)] Attribute ") + std::string(field_name) + std::string(" was not found in XML element.");
		throw exception_str;
	}
	return return_string;
}

ViaPointBase * XMLDeserializer::deserializeViaPoint(ControllerType type_of_controller, int controller_dimension)
{
	ViaPointBase * return_value;
	double via_point_time = this->deserializeDouble("time");
	int via_point_type = this->deserializeInteger("type");
	bool via_point_reuse_bool = this->deserializeBoolean("reuse");
	switch( type_of_controller.first )
	{
	case rxController::eControlType_Joint:	
		{
		std::stringstream via_point_ss = std::stringstream(this->deserializeString("dVector"));
		double via_point_value = -1.0;
		dVector via_point_dVector;
		std::string value_str;
		while(getline(via_point_ss, value_str, ',')){
			via_point_dVector.expand(1,atof(value_str.c_str()));
		}
		// NOTE (Roberto): This older version works when the values are separated with white spaces instead of commas
		//for(int i=0; i<controller_dimension; i++)
		//{
		//	via_point_ss >> via_point_value;
		//	via_point_dVector.expand(1,via_point_value);
		//}
		return_value = new ViaPointdVector(via_point_time, via_point_type, via_point_reuse_bool, via_point_dVector);
		break;
		}
	case rxController::eControlType_Displacement:	
		{
		std::stringstream via_point_ss = std::stringstream(this->deserializeString("Vector3D"));
		double via_point_value = -1.0;
		Vector3D via_point_Vector3D;
		for(int i=0; i<controller_dimension; i++)
		{
			via_point_ss >> via_point_value;
			via_point_Vector3D.expand(1,via_point_value);
		}
		return_value = new ViaPointVector3D(via_point_time, via_point_type, via_point_reuse_bool, via_point_Vector3D);
		break;
		}
	case rxController::eControlType_Orientation:	
		{
		std::stringstream via_point_ss = std::stringstream(this->deserializeString("R"));
		double via_point_value = -1.0;
		std::vector<double> via_point_values;
		for(int i=0; i<9; i++)
		{
			via_point_ss >> via_point_value;
			via_point_values.push_back(via_point_value);
		}
		Rotation via_point_Rotation(via_point_values[0], via_point_values[1],via_point_values[2],via_point_values[3],via_point_values[4],via_point_values[5],via_point_values[6],via_point_values[7],via_point_values[8]);
		return_value = new ViaPointRotation(via_point_time, via_point_type, via_point_reuse_bool, via_point_Rotation);
		break;
		}
	case rxController::eControlType_HTransform:	
		{
		std::stringstream via_point_ss = std::stringstream(this->deserializeString("R"));
		double via_point_value = -1.0;
		std::vector<double> via_point_values;
		for(int i=0; i<9; i++)
		{
			via_point_ss >> via_point_value;
			via_point_values.push_back(via_point_value);
		}
		Rotation via_point_Rotation(via_point_values[0], via_point_values[1],via_point_values[2],via_point_values[3],via_point_values[4],via_point_values[5],via_point_values[6],via_point_values[7],via_point_values[8]);

		std::stringstream via_point_ss2 = std::stringstream(this->deserializeString("r"));
		via_point_values.clear();
		for(int i=0; i<9; i++)
		{
			via_point_ss2 >> via_point_value;
			via_point_values.push_back(via_point_value);
		}
		Displacement via_point_Displacement(via_point_values[0], via_point_values[1],via_point_values[2]);		

		HTransform via_point_HTransform(via_point_Rotation, via_point_Displacement);

		return_value = new ViaPointHTransform(via_point_time, via_point_type, via_point_reuse_bool, via_point_HTransform);
		break;
		}
	default:
		std::string exception_str = std::string("ERROR: [XMLDeserializer::deserializeViaPoint(ControllerType type_of_controller, int controller_dimension)] Unknown type of Controller group.");
		throw exception_str;
		break;
	}
	return return_value;
}

std::vector<double> XMLDeserializer::deserializeVectorDouble(const char * field_name)
{
	std::stringstream vector_ss = std::stringstream(xml_element->Attribute(field_name));
	double vector_value = -1.0;
	std::vector<double> ret_vector;
	while ((vector_ss >> vector_value))
	{
		ret_vector.push_back(vector_value);
	}
	return ret_vector;
}