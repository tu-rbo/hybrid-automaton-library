#include "XMLDeserializer.h"

#include "CSpaceMilestone.h"
#include "OpSpaceMilestone.h"
#include "PostureMilestone.h"

#include "rControlalgorithm\rControlalgorithm.h"
#include "rxControlSDK\rxControlSDK.h"

#include "FeatureAttractorController.h"
#include "SubdisplacementController.h"
#include "SubdisplacementSimpleController.h"
#include "ObstacleAvoidanceController.h"
#include "JointBlackBoardController.h"
#include "SingularityAvoidanceController.h"
#include "JointLimitAvoidanceControllerOnDemand.h"
#include "OpSpaceSingularityAvoidanceController.h"

#include "NakamuraControlSet.h"
#include "InterpolatedSetPointDisplacementController.h"
#include "InterpolatedSetPointJointController.h"
#include "InterpolatedSetPointOrientationController.h"
#include "PressureDisplacementController.h"
#include "PressureOrientationController.h"

#include "TPImpedanceControlSet.h"

std::map<std::string, ControllerType> XMLDeserializer::controller_map_ = XMLDeserializer::createControllerMapping();

std::string XMLDeserializer::wstring2string(const std::wstring& wstr)
{
	return std::string(wstr.begin(), wstr.end());
}

std::wstring XMLDeserializer::string2wstring(const std::string& str)
{
	return std::wstring(str.begin(), str.end());
}

/**
* Deserializing Functions
*/

template<class T>
T deserializeElement(TiXmlElement * xml_element, const char * field_name)
{
	T return_value;

	if(!xml_element->Attribute(field_name, &return_value))
	{
		std::string exception_str = std::string("[deserializeElement] ERROR: Attribute ") + std::string(field_name) + std::string(" was not found in XML element.");
		throw exception_str;
	}

	return return_value;
}

template<class T>
T deserializeElement(TiXmlElement * xml_element, const char * field_name, T default_value)
{
	T return_value = default_value;
	xml_element->Attribute(field_name, &return_value);
	return return_value;
}

bool deserializeBoolean(TiXmlElement * xml_element, const char * field_name, bool default_value)
{
	const char* value = xml_element->Attribute(field_name);

	if (value)
	{
		if (strcmp(value, "true") == 0)
		{
			return true;
		}
		else if (strcmp(value, "false") == 0)
		{
			return false;
		}
	}
	return default_value;
}

std::string deserializeString(TiXmlElement * xml_element, const char * field_name, const std::string& default_value)
{
	const char* ret_char_ptr = xml_element->Attribute(field_name);

	if(ret_char_ptr)
		return std::string(ret_char_ptr);

	return default_value;
}

std::string deserializeString(TiXmlElement * xml_element, const char * field_name, bool error_if_not_found)
{
	const char* ret_char_ptr = xml_element->Attribute(field_name);
	if(ret_char_ptr)
	{
		return std::string(ret_char_ptr);
	}else{
		if(error_if_not_found)
		{
			std::string exception_str = std::string("[deserializeString] ERROR: Attribute ") 
				+ std::string(field_name) + std::string(" was not found in XML element.");
			throw exception_str;
		}
	}
	return std::string();
}

dVector deserializeDVector(TiXmlElement * xml_element, const char * field_name)
{
	std::stringstream ss(deserializeString(xml_element, field_name, false));
	dVector v;
	v.all(-1.0);
	double value = -1.0;
	while(ss >> value)
	{	
		v.expand(1, value);
	}
	return v;
}

Rotation deserializeRotation(TiXmlElement * xml_element, const char * field_name, const Rotation& default_value)
{
	std::string rotation = deserializeString(xml_element, field_name, std::string(""));

	if (rotation.empty())
		return default_value;

	return XMLDeserializer::string2rotation(rotation, default_value);
}

Displacement deserializeDisplacement(TiXmlElement * xml_element, const char * field_name, const Displacement& default_value)
{
	dVector v = deserializeDVector(xml_element, field_name);
	if (v.size() != 3)
		return default_value;

	return v;
}

rxBody* deserializeBody(rxSystem* robot, TiXmlElement * xml_element, const char * field_name, rxBody* default_value)
{
	std::string name = deserializeString(xml_element, field_name, std::string(""));
	if (name.empty())
		return default_value;

	//First we query for user defined bodies
	rxBody* body = robot->getUCSBody(XMLDeserializer::string2wstring(name), HTransform());
	if (body != NULL)
		return body;

	//Now query for bodies in the model
	body = robot->findBody(XMLDeserializer::string2wstring(name));
	if(body != NULL)
		return body;

	std::string exception_str = std::string("[deserializeBody] ERROR: Body ") 
		+ name + std::string(" was not found in aml");
	throw exception_str;
	return NULL;


}

template<class T>
std::vector<T> deserializeStdVector(TiXmlElement * xml_element, const char * field_name)
{
	std::vector<T> ret_vector;
	std::stringstream vector_ss = std::stringstream(deserializeString(xml_element, field_name, false));
	double vector_value = -1.0;
	while ((vector_ss >> vector_value))
	{
		ret_vector.push_back((T)vector_value);
	}
	return ret_vector;
}

ViaPointBase * deserializeViaPoint(TiXmlElement * xml_element, ControllerType type_of_controller, int controller_dimension)
{
	ViaPointBase * return_value;
	double via_point_time = deserializeElement<double>(xml_element, "time");
	int via_point_type = deserializeElement<int>(xml_element, "type");
	bool via_point_reuse_bool = deserializeBoolean(xml_element, "reuse");
	switch( type_of_controller.first )
	{
	case rxController::eControlType_Joint:	
		{
			std::stringstream via_point_ss = std::stringstream(deserializeString(xml_element, "dVector"));
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
			std::stringstream via_point_ss = std::stringstream(deserializeString(xml_element, "Vector3D"));
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
			std::stringstream via_point_ss = std::stringstream(deserializeString(xml_element, "R"));
			double via_point_value = -1.0;
			std::vector<double> via_point_values;
			for(int i=0; i<9; i++)
			{
				via_point_ss >> via_point_value;
				via_point_values.push_back(via_point_value);
			}
			Rotation via_point_Rotation(via_point_values[0], via_point_values[1],via_point_values[2],via_point_values[3],
				via_point_values[4],via_point_values[5],via_point_values[6],via_point_values[7],via_point_values[8]);
			return_value = new ViaPointRotation(via_point_time, via_point_type, via_point_reuse_bool, via_point_Rotation);
			break;
		}
	case rxController::eControlType_HTransform:	
		{
			std::stringstream via_point_ss = std::stringstream(deserializeString(xml_element, "R"));
			double via_point_value = -1.0;
			std::vector<double> via_point_values;
			for(int i=0; i<9; i++)
			{
				via_point_ss >> via_point_value;
				via_point_values.push_back(via_point_value);
			}
			Rotation via_point_Rotation(via_point_values[0], via_point_values[1],via_point_values[2],via_point_values[3],
				via_point_values[4],via_point_values[5],via_point_values[6],via_point_values[7],via_point_values[8]);

			std::stringstream via_point_ss2 = std::stringstream(deserializeString(xml_element, "r"));
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
		std::string exception_str = std::string("[deserializeViaPoint] ERROR: Unknown type of Controller group.");
		throw exception_str;
		break;
	}
	return return_value;
}

Rotation XMLDeserializer::string2rotation(const std::string& str, const Rotation& default_value)
{
	std::stringstream ss = std::stringstream(str);
	double value = -1.0;
	std::vector<double> values;
	while(ss >> value)
	{
		values.push_back(value);
	}
	assert(values.size() == 3 || values.size() == 4 || values.size() == 9);

	// decide whether euler, quaternion, or matrix
	switch (values.size())
	{
	case 3: return Rotation(values[0], values[1], values[2]);
	case 4: return Rotation(values[0], values[1], values[2], values[3]); // attention: w, x, y, z
	case 9: return Rotation(values[0], values[1], values[2],
				values[3], values[4], values[5],
				values[6], values[7], values[8]);
	}

	return default_value;
}

std::string colon2space(std::string text)
{
	for(unsigned int i = 0; i < text.length(); i++)
	{
		if( text[i] == ',' )
			text[i] = ' ';
	}
	return text;
}




/**
* The XMLDeserializer is the object that reads all the information from the xml string and deserializes it, creating
* all required objects (Milestones, MotionBehaviours and the HybridAutomaton).
* The idea is to deserialize all the data here (thus, read it from the xml string and convert it into the corresponding type)
* and then call the suitable constructor.
*/

XMLDeserializer::XMLDeserializer()
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
	const char* ret_val = document.Parse(xml_string.c_str());
	if(ret_val == NULL && document.Error()) 	{
		std::cout << "ERROR CODE: " <<  document.ErrorId() << std::endl;
		std::cout << document.ErrorDesc() << std::endl;
		throw std::string("[XMLDeserializer::createHybridAutomaton] ERROR: Parsing the string.");
		delete automaton;
		return NULL;
	}
	TiXmlHandle docHandle(&document);

	// Find the first (there should be the only one) HybridAutomaton element in the base document
	TiXmlElement* ha_element = docHandle.FirstChild("HybridAutomaton").Element();

	// Check if the HybridAutomaton element was found
	if (ha_element == NULL) {
		throw std::string("[XMLDeserializer::createHybridAutomaton] ERROR: Child Element \"HybridAutomaton\" not found in XML element docHandle.");
		delete automaton;
		return NULL;
	}

	std::string start_node = std::string(ha_element->Attribute("InitialMilestone"));
	if(start_node == std::string("")) {
		std::cout << "[XMLDeserializer::createHybridAutomaton] ERROR: Attribute \"InitialMilestone\" not found in XML element hsElement." << std::endl;
		delete automaton;
		return NULL;
	}

	std::string ha_name(ha_element->Attribute("Name"));
	// Print out a message with the general properties of the new HybridAutomaton.
	std::cout << "[XMLDeserializer::createHybridAutomaton] INFO: Creating hybrid system '" << ha_name << "'. Initial Milestone: " << start_node << std::endl;

	ha_name.append(".txt");
	ofstream file;
	file.open(ha_name.c_str());
	file << xml_string;
	file.close();

	// Read the data of the nodes-milestones and create them
	for (TiXmlElement* mst_element = ha_element->FirstChildElement("Milestone"); mst_element != 0; mst_element = mst_element->NextSiblingElement("Milestone")) {
		Milestone* mst;
		std::string mst_type(mst_element->Attribute("type"));
		if(mst_type == "CSpace")
		{
			mst = XMLDeserializer::createCSpaceMilestone(mst_element, robot, dT);
		}
		else if(mst_type == "OpSpace")
		{
			mst = XMLDeserializer::createOpSpaceMilestone(mst_element, robot, dT);
		}
		else if(mst_type == "CSpaceBlackBoard")
		{
			mst = XMLDeserializer::createCSpaceBlackBoardMilestone(mst_element, robot, dT);
		}
		else if(mst_type == "Posture")
		{
			mst = XMLDeserializer::createPostureMilestone(mst_element, robot, dT);
		}
		else
		{
			throw (std::string("[XMLDeserializer::createHybridAutomaton] ERROR: Unknown type of milestone: ") + mst_type);
			delete automaton;
			return NULL;
		}
		automaton->addNode(mst);
	}

	// Set the start node-milestone
	automaton->setStartNode(automaton->getMilestoneByName(start_node));

	if (!automaton->getStartNode()) {
		throw ("[XMLDeserializer::createHybridAutomaton] ERROR: The name of the initial node '" + start_node + "' does not match with the name of any milestone.");
		delete automaton;
		return NULL;
	}

	// Read the data of the edges-motionbehaviours and create them
	for (TiXmlElement* mb_element = ha_element->FirstChildElement("MotionBehaviour"); mb_element != 0; mb_element = mb_element->NextSiblingElement("MotionBehaviour")) {
		std::string ms_parent(mb_element->Attribute("Parent"));
		std::string ms_child(mb_element->Attribute("Child"));
		if(ms_parent == std::string(""))
		{
			throw std::string("[XMLDeserializer::createHybridAutomaton] ERROR: Attribute \"Parent\" not found in XML element edgeElement.");
			delete automaton;
			return NULL;
		}

		Milestone* ms_parent_ptr = automaton->getMilestoneByName(ms_parent);

		if(ms_parent_ptr == NULL) {
			throw std::string("[XMLDeserializer::createHybridAutomaton] ERROR: The name of the parent node does not match with the name of any milestone.");
			delete automaton;
			return NULL;
		}

		if(ms_child == std::string(""))
		{
			throw std::string("[XMLDeserializer::createHybridAutomaton] ERROR: Attribute \"Child\" not found in XML element edgeElement.");
			delete automaton;
			return NULL;
		}

		Milestone* ms_child_ptr = automaton->getMilestoneByName(ms_child);

		if(ms_child_ptr == NULL)
		{
			throw ("[XMLDeserializer::createHybridAutomaton] ERROR: The name of the child node '" + ms_child + "' does not match the name of any milestone.");
			delete automaton;
			return NULL;
		}

		MotionBehaviour* mb = XMLDeserializer::createMotionBehaviour(mb_element, ms_parent_ptr, ms_child_ptr, robot, dT);
		automaton->addEdge(mb);
	}

	return automaton;
}

CSpaceMilestone* XMLDeserializer::createCSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot, double dT)
{
	Milestone::Status mst_status = (Milestone::Status)deserializeElement<int>(milestone_xml, "status");
	std::string mst_name = deserializeString(milestone_xml, "name");
	std::vector<double> mst_configuration = deserializeStdVector<double>(milestone_xml, "value");
	std::vector<double> mst_epsilon = deserializeStdVector<double>(milestone_xml, "epsilon");
	if(mst_configuration.size()==0 || mst_epsilon.size()==0)
	{
		throw std::string("[XMLDeserializer::createCSpaceMilestone] ERROR: The milestone configuration or region of convergence (\"value\" or \"epsilon\") is not defined in the XML string.");
	}


	TiXmlElement* handle_point_set_element = milestone_xml->FirstChildElement("HandlePoints");
	std::vector<Point> mst_handle_points;
	if(handle_point_set_element != NULL)
	{
		for(TiXmlElement* handle_point_element = handle_point_set_element->FirstChildElement("HandlePoint"); handle_point_element; handle_point_element = handle_point_element->NextSiblingElement())
		{
			Point handle_point_value(-1.f,-1.f,-1.f);
			handle_point_value.x = deserializeElement<double>(handle_point_element, "x");
			handle_point_value.y = deserializeElement<double>(handle_point_element, "y");
			handle_point_value.z = deserializeElement<double>(handle_point_element, "z");
			mst_handle_points.push_back(handle_point_value);
		}
	}

	CSpaceMilestone* mst = new CSpaceMilestone(mst_name, mst_configuration, NULL, mst_epsilon, mst_status, mst_handle_points);

	TiXmlElement* mb_element = milestone_xml->FirstChildElement("MotionBehaviour");
	MotionBehaviour* mst_mb = NULL;
	if(mb_element)
	{
		mst_mb = XMLDeserializer::createMotionBehaviour(mb_element, mst, mst, robot, dT);
		mst->setMotionBehaviour(mst_mb);
	}

	double expLength = deserializeElement<double>(milestone_xml, "expectedLength", -1.0);
	mst->setExpectedLength(expLength);

	return mst;
}

CSpaceBlackBoardMilestone* XMLDeserializer::createCSpaceBlackBoardMilestone(TiXmlElement* milestone_xml, rxSystem* robot, double dT)
{
	Milestone::Status mst_status = (Milestone::Status)deserializeElement<int>(milestone_xml, "status");
	std::string mst_name = deserializeString(milestone_xml, "name");
	std::vector<double> mst_configuration = deserializeStdVector<double>(milestone_xml, "value");
	std::vector<double> mst_epsilon = deserializeStdVector<double>(milestone_xml, "epsilon");
	if(mst_configuration.size()==0 || mst_epsilon.size()==0)
	{
		throw std::string("[XMLDeserializer::createCSpaceMilestone] ERROR: The milestone configuration or region of convergence (\"value\" or \"epsilon\") is not defined in the XML string.");
	}

	TiXmlElement* handle_point_set_element = milestone_xml->FirstChildElement("HandlePoints");
	std::vector<Point> mst_handle_points;
	if(handle_point_set_element != NULL)
	{
		for(TiXmlElement* handle_point_element = handle_point_set_element->FirstChildElement("HandlePoint"); handle_point_element; handle_point_element = handle_point_element->NextSiblingElement())
		{
			Point handle_point_value(-1.f,-1.f,-1.f);
			handle_point_value.x = deserializeElement<double>(handle_point_element, "x");
			handle_point_value.y = deserializeElement<double>(handle_point_element, "y");
			handle_point_value.z = deserializeElement<double>(handle_point_element, "z");
			mst_handle_points.push_back(handle_point_value);
		}
	}

	CSpaceBlackBoardMilestone* mst = new CSpaceBlackBoardMilestone(mst_name, mst_configuration, NULL, mst_epsilon, mst_status, mst_handle_points);

	mst->setBlackBoardVariableName(mst->getBlackBoardVariableName());

	double expLength = deserializeElement<double>(milestone_xml, "expectedLength", -1.0);
	mst->setExpectedLength(expLength);

	return mst;
}

OpSpaceMilestone* XMLDeserializer::createOpSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot, double dT)
{
	Milestone::Status mst_status = (Milestone::Status)deserializeElement<int>(milestone_xml, "status");
	std::string mst_name = deserializeString(milestone_xml, "name");
	PosiOriSelector mst_pos = (PosiOriSelector)deserializeElement<int>(milestone_xml, "PosiOriSelector", POS_AND_ORI_SELECTION);
	Displacement position = deserializeDisplacement(milestone_xml, "position", Displacement());
	Rotation orientation = deserializeRotation(milestone_xml, "orientation", Rotation());
	std::vector<double> mst_epsilon = deserializeStdVector<double>(milestone_xml, "epsilon");
	if(/*mst_configuration.size()==0 ||*/ mst_epsilon.size()==0)
	{
		throw std::string("[XMLDeserializer::createOpSpaceMilestone] ERROR: The milestone configuration or region of convergence (\"value\" or \"epsilon\") is not defined in the XML string.");
	}

	TiXmlElement* handle_point_set_element = milestone_xml->FirstChildElement("HandlePoints");
	std::vector<Point> mst_handle_points;
	if(handle_point_set_element != NULL)
	{
		for(TiXmlElement* handle_point_element = handle_point_set_element->FirstChildElement("HandlePoint"); handle_point_element; handle_point_element = handle_point_element->NextSiblingElement())
		{
			Point handle_point_value(-1.f,-1.f,-1.f);
			handle_point_value.x = deserializeElement<double>(handle_point_element, "x");
			handle_point_value.y = deserializeElement<double>(handle_point_element, "y");
			handle_point_value.z = deserializeElement<double>(handle_point_element, "z");
			mst_handle_points.push_back(handle_point_value);
		}
	}

	OpSpaceMilestone* mst = new OpSpaceMilestone(mst_name, position, orientation, mst_pos, NULL, mst_epsilon, mst_status, mst_handle_points);

	TiXmlElement* mb_element = milestone_xml->FirstChildElement("MotionBehaviour");
	MotionBehaviour* mst_mb = NULL;
	if(mb_element)
	{
		mst_mb = XMLDeserializer::createMotionBehaviour(mb_element, mst, mst, robot, dT);
		mst->setMotionBehaviour(mst_mb);
	}

	double expLength = deserializeElement<double>(milestone_xml, "expectedLength", -1.0);
	mst->setExpectedLength(expLength);

	return mst;
}

PostureMilestone* XMLDeserializer::createPostureMilestone(TiXmlElement* milestone_xml, rxSystem* robot, double dT)
{
	Milestone::Status mst_status = (Milestone::Status)deserializeElement<int>(milestone_xml, "status");
	std::string mst_name = deserializeString(milestone_xml, "name");
	PosiOriSelector mst_pos = (PosiOriSelector)deserializeElement<int>(milestone_xml, "PosiOriSelector", POS_AND_ORI_SELECTION);
	Displacement position = deserializeDisplacement(milestone_xml, "position", Displacement());
	Rotation orientation = deserializeRotation(milestone_xml, "orientation", Rotation());
	dVector configuration = deserializeDVector(milestone_xml, "configuration");
	std::vector<double> mst_epsilon = deserializeStdVector<double>(milestone_xml, "epsilon");
	if(configuration.size()==0 || mst_epsilon.size()==0)
	{
		throw std::string("[XMLDeserializer::createPostureMilestone] ERROR: The milestone configuration or region of convergence (\"value\" or \"epsilon\") is not defined in the XML string.");
	}

	TiXmlElement* handle_point_set_element = milestone_xml->FirstChildElement("HandlePoints");
	std::vector<Point> mst_handle_points;
	if(handle_point_set_element != NULL)
	{
		for(TiXmlElement* handle_point_element = handle_point_set_element->FirstChildElement("HandlePoint"); handle_point_element; handle_point_element = handle_point_element->NextSiblingElement())
		{
			Point handle_point_value(-1.f,-1.f,-1.f);
			handle_point_value.x = deserializeElement<double>(handle_point_element, "x");
			handle_point_value.y = deserializeElement<double>(handle_point_element, "y");
			handle_point_value.z = deserializeElement<double>(handle_point_element, "z");
			mst_handle_points.push_back(handle_point_value);
		}
	}

	PostureMilestone* mst = new PostureMilestone(mst_name, position, orientation, configuration,mst_pos, NULL, mst_epsilon, mst_status, mst_handle_points, robot);

	TiXmlElement* mb_element = milestone_xml->FirstChildElement("MotionBehaviour");
	MotionBehaviour* mst_mb = NULL;
	if(mb_element)
	{
		mst_mb = XMLDeserializer::createMotionBehaviour(mb_element, mst, mst, robot, dT);
		mst->setMotionBehaviour(mst_mb);
	}

	double expLength = deserializeElement<double>(milestone_xml, "expectedLength", -1.0);
	mst->setExpectedLength(expLength);

	return mst;
} 

MotionBehaviour* XMLDeserializer::createMotionBehaviour(TiXmlElement* motion_behaviour_xml , Milestone *dad, Milestone *son , rxSystem* robot, double dT )
{
	TiXmlElement* control_set_element = motion_behaviour_xml->FirstChildElement("ControlSet");
	if(control_set_element == NULL)
	{
		throw std::string("[XMLDeserializer::createMotionBehaviour] ERROR: ControlSet element was not found.");
	}

	std::string control_set_type = std::string(control_set_element->Attribute("type"));
	rxControlSetBase* mb_control_set = NULL;
	if(robot)
	{
		if(control_set_type == std::string("rxControlSet"))
		{		
			mb_control_set = new rxControlSet(robot, dT);
			mb_control_set->setGravity(0, 0, -GRAV_ACC);
			mb_control_set->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(robot));
			//mb_control_set->nullMotionController()->setGain(0.02,0.0,0.01);
			mb_control_set->nullMotionController()->setGain(0.0, 0.0, 0.0);
		}
		else if(control_set_type == std::string("rxTPOperationalSpaceControlSet"))
		{		
			mb_control_set = new rxTPOperationalSpaceControlSet(robot, dT);
			mb_control_set->setGravity(0, 0, -GRAV_ACC);
			mb_control_set->nullMotionController()->setGain(10.0,1.0); // taken from ERM values...
		}
		else if(control_set_type == "NakamuraControlSet")
		{
			mb_control_set = new NakamuraControlSet(robot, dT);
			mb_control_set->setGravity(0, 0, -GRAV_ACC);
			mb_control_set->nullMotionController()->setGain(0.0, 0.0, 0.0);
		}
		else if(control_set_type == "TPImpedanceControlSet")
		{
			mb_control_set = new TPImpedanceControlSet(robot, dT);
			mb_control_set->setGravity(0, 0, -GRAV_ACC);
			// TPImpedanceControlSet does not implement extra nullmotion behaviour - 
			// This has to be done at lowest priority!
			mb_control_set->nullMotionController()->setGain(0.0, 0.0, 0.0);
		}
		else
		{
			std::cout << control_set_type << std::endl;
			throw std::string("[XMLDeserializer::createMotionBehaviour] ERROR: Unknown type of rxControlSet.");
		}	
	}

	MotionBehaviour* mb = new MotionBehaviour(dad, son , mb_control_set);

	mb->setMinTimeForInterpolation(deserializeElement<double>(motion_behaviour_xml, "minTime",-1.));
	mb->setMaxVelocityForInterpolation(deserializeElement<double>(motion_behaviour_xml, "maxVelocity",-1.));

	int mb_controller_counter = 0;
	for(TiXmlElement* rxController_xml = control_set_element->FirstChildElement("Controller"); rxController_xml; rxController_xml = rxController_xml->NextSiblingElement())
	{	
		bool mb_goal_controller = deserializeBoolean(rxController_xml, "goalController", true);		
		rxController* mb_controller = XMLDeserializer::createController(rxController_xml, dad, son, robot, dT, mb_goal_controller, mb_controller_counter);
		mb->addController(mb_controller, mb_goal_controller);
		mb_controller_counter++;
	}

	double length = deserializeElement<double>(motion_behaviour_xml, "length", -1.0);
	double prob   = deserializeElement<double>(motion_behaviour_xml, "probability", -1.0);
	mb->setLength(length);
	mb->setProbability(prob);

	bool update   = deserializeBoolean(motion_behaviour_xml, "updateAllowed", false);
	mb->setUpdateAllowed(update);

	return mb;
}

rxController* XMLDeserializer::createController(TiXmlElement *rxController_xml, const Milestone *dad, const Milestone *son, rxSystem *robot, double dT, bool goal_controller, int controller_counter)
{
	//CAUTION!!! impedance values are hardcoded in this method. This will change if simlab provides us with getters
	//for controller parameters.

	// iterate through all attributes (vs. iterate through parameter set)
	// and fill parameter set (check for default values)
	ControllerParameters params;
	params.type = deserializeString(rxController_xml, "type");
	params.ik = deserializeBoolean(rxController_xml, "ik");
	params.kp = deserializeDVector(rxController_xml, "kp");
	params.kv = deserializeDVector(rxController_xml, "kv");
	params.stiffness_b = deserializeDVector(rxController_xml, "stiffness_b");
	params.stiffness_k = deserializeDVector(rxController_xml, "stiffness_k");
	//params.impedance_m = deserializeElement<double>(rxController_xml, "impedance_m");
	//params.impedance_b = deserializeElement<double>(rxController_xml, "impedance_b");
	//params.impedance_k = deserializeElement<double> (rxController_xml, "impedance_k");
	params.invL2sqr = deserializeDVector(rxController_xml, "invL2sqr");
	params.priority = deserializeElement<int>(rxController_xml, "priority", 1);
	params.timeGoal = deserializeElement<double>(rxController_xml, "timeGoal", -1.0);
	params.reuseGoal = deserializeBoolean(rxController_xml, "reuseGoal", false);
	params.typeGoal = deserializeElement<int>(rxController_xml, "typeGoal", 0);
	params.dVectorGoal = deserializeDVector(rxController_xml, "dVectorGoal");
	params.Vector3DGoal = deserializeDVector(rxController_xml, "Vector3DGoal");
	params.RGoal = deserializeRotation(rxController_xml, "RGoal", Rotation());
	params.rGoal = deserializeDVector(rxController_xml, "rGoal");
	params.desired_distance = deserializeElement<double>(rxController_xml, "desiredDistance", 1.);
	params.max_force = deserializeElement<double>(rxController_xml, "maxForce", 1.);
	params.max_vel = deserializeElement<double>(rxController_xml, "maxVel", 1.);
	params.limit_body = deserializeString(rxController_xml, "limitBody", false);
	params.distance_limit = deserializeElement<double>(rxController_xml, "distanceLimit", 0.);
	params.distance_threshold = deserializeElement<double>(rxController_xml, "distanceThreshold", 1.);
	params.deactivation_threshold = deserializeElement<double>(rxController_xml, "deactivationThreshold", 1.);
	params.index = deserializeStdVector<long>(rxController_xml, "index");
	params.safetyThresh = deserializeElement<double>(rxController_xml, "safetyThresh", 1.0);
	params.tc = deserializeStdVector<double>(rxController_xml, "taskConstraints");
	params.alpha = deserializeBody(robot, rxController_xml, "alpha", NULL);
	params.alpha_displacement = deserializeDisplacement(rxController_xml, "alphaDisplacement", Displacement());
	params.alpha_rotation_matrix = deserializeRotation(rxController_xml, "alphaRotation", Rotation());	
	params.beta = deserializeBody(robot, rxController_xml, "beta", NULL);
	if(params.beta != NULL)
	{
		//Use values from xml
		params.beta_displacement = deserializeDisplacement(rxController_xml, "betaDisplacement", Displacement());
		params.beta_rotation_matrix = deserializeRotation(rxController_xml, "betaRotation", Rotation());
	}
	else
	{
		//Set EE as default body
		HTransform transform;
		params.beta = robot->getUCSBody(XMLDeserializer::string2wstring(deserializeString(rxController_xml, "beta", std::string("EE"))), transform);
		params.beta_displacement = transform.r;
		params.beta_rotation_matrix = transform.R;
	}

	//params.beta_displacement = deserializeDisplacement(rxController_xml, "betaDisplacement", Displacement());
	//params.beta_rotation_matrix = deserializeRotation(rxController_xml, "betaRotation", Rotation());
	// viapoints ?

	// call the appropriate factory method which takes the parameter set as input
	rxController* controller = NULL;
	if (params.type == "rxJointController")
	{
		rxJointController* special_controller = new rxJointController(robot, dT);
		if(!goal_controller)
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Cubic);
		special_controller->setGain(params.kv, params.kp, params.invL2sqr);	
		controller = special_controller;		
	}
	else if (params.type == "rxJointComplianceController")
	{
		rxJointComplianceController* special_controller = new rxJointComplianceController(robot, dT);
		if(!goal_controller)
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Cubic);
		special_controller->setStiffness(params.stiffness_b, params.stiffness_k);
		controller = special_controller;
	}
	else if (params.type == "rxJointImpedanceController")
	{
		rxJointImpedanceController* special_controller = new rxJointImpedanceController(robot, dT);
		if(!goal_controller)
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Linear);
		//special_controller->setImpedance(params.impedance_m, params.impedance_b, params.impedance_k);
		//values for 7DOF demo!!!
		special_controller->setImpedance(1.0,2.0/sqrt(2.0), 0.5);
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedJointController")
	{
		rxInterpolatedJointController* special_controller = new rxInterpolatedJointController(robot, dT);
		if(!goal_controller)
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Cubic);
		special_controller->setGain(params.kv, params.kp, params.invL2sqr);	
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedJointComplianceController")
	{
		rxInterpolatedJointComplianceController* special_controller = new rxInterpolatedJointComplianceController(robot, dT);
		if(!goal_controller)		
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Cubic);
		special_controller->setStiffness(params.stiffness_b, params.stiffness_k);
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedJointImpedanceController")
	{
		rxInterpolatedJointImpedanceController* special_controller = new rxInterpolatedJointImpedanceController(robot, dT);
		if(!goal_controller)		
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Linear);
		special_controller->setImpedance(0.1,1.0,2.0);
		//special_controller->setImpedance(params.impedance_m, params.impedance_b, params.impedance_k);
		controller = special_controller;
	}
	else if (params.type == "JointBlackBoardController")
	{
		JointBlackBoardController* special_controller = new JointBlackBoardController(robot, dT);
		special_controller->setBlackBoardVariableName(params.blackboard_variable_name);
		special_controller->setGain(params.kv, params.kp, params.invL2sqr);	
		controller = special_controller;
	}
	else if (params.type == "SingularityAvoidanceController")
	{
		HTransform ht;
		rxBody* EE = robot->getUCSBody(_T("EE"), ht);
		controller = new SingularityAvoidanceController(robot, EE, dT, params.maxVel);
		controller->setGain(params.kv, params.kp, params.invL2sqr);	
	}
	else if (params.type == "rxDisplacementController")
	{
		rxDisplacementController* special_controller = new rxDisplacementController(robot, params.beta, Displacement(params.beta_displacement), params.alpha, Displacement(params.alpha_displacement), dT);
		if(!goal_controller)
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Cubic);
		special_controller->setGain(params.kv, params.kp, params.invL2sqr);	
		controller = special_controller;
	}
	else if (params.type == "rxDisplacementComplianceController")
	{
		rxDisplacementComplianceController* special_controller = new rxDisplacementComplianceController(robot, params.beta, Displacement(params.beta_displacement), params.alpha, Displacement(params.alpha_displacement), dT);
		if(!goal_controller)
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Cubic);		
		special_controller->setStiffness(params.stiffness_b, params.stiffness_k);
		controller = special_controller;
	}
	else if (params.type == "rxDisplacementImpedanceController")
	{
		rxDisplacementImpedanceController* special_controller = new rxDisplacementImpedanceController(robot, params.beta, Displacement(params.beta_displacement), params.alpha, Displacement(params.alpha_displacement), dT);
		if(!goal_controller)
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Cubic);
		special_controller->setImpedance(0.2, 8.0, 20.0);
		//special_controller->setImpedance(params.impedance_m, params.impedance_b, params.impedance_k);
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedDisplacementController")
	{
		rxInterpolatedDisplacementController* special_controller = new rxInterpolatedDisplacementController(robot, params.beta, Displacement(params.beta_displacement), params.alpha, Displacement(params.alpha_displacement), dT);
		if(!goal_controller)
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Cubic);
		special_controller->setGain(params.kv, params.kp, params.invL2sqr);	
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedDisplacementComplianceController")
	{
		rxInterpolatedDisplacementComplianceController* special_controller = new rxInterpolatedDisplacementComplianceController(robot, params.beta, Displacement(params.beta_displacement), params.alpha, Displacement(params.alpha_displacement), dT);
		if(!goal_controller)
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Cubic);
		special_controller->setStiffness(params.stiffness_b, params.stiffness_k);
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedDisplacementImpedanceController")
	{
		rxInterpolatedDisplacementImpedanceController* special_controller = new rxInterpolatedDisplacementImpedanceController(robot, params.beta, Displacement(params.beta_displacement), params.alpha, Displacement(params.alpha_displacement), dT);
		if(!goal_controller)
			special_controller->addPoint(params.dVectorGoal, params.timeGoal, params.reuseGoal, eInterpolatorType_Cubic);
		special_controller->setImpedance(0.2, 8.0, 20.0);
		//special_controller->setImpedance(params.impedance_m, params.impedance_b, params.impedance_k);
		controller = special_controller;
	}
	else if (params.type == "FeatureAttractorController")
	{
		FeatureAttractorController* special_controller = new FeatureAttractorController(robot, params.beta, Displacement(params.beta_displacement), dT, params.desired_distance, params.max_force);
		special_controller->setImpedance(0.2, 8.0, 20.0);
		controller = special_controller;
	}
	else if (params.type == "OpSpaceSingularityAvoidanceController")
	{
		controller = new OpSpaceSingularityAvoidanceController(robot, params.beta, params.alpha, dT, params.max_vel);
		controller->setGain(params.kv, params.kp, params.invL2sqr);	
	}
	else if (params.type == "rxOrientationController")
	{
		rxOrientationController* special_controller = new rxOrientationController(robot, params.beta, params.beta_rotation_matrix, params.alpha, params.alpha_rotation_matrix, dT);
		if(!goal_controller)
			special_controller->addPoint(params.RGoal, params.timeGoal, params.reuseGoal);
		special_controller->setGain(params.kv, params.kp, params.invL2sqr);	
		controller = special_controller;
	}
	else if (params.type == "rxOrientationComplianceController")
	{
		rxOrientationComplianceController* special_controller = new rxOrientationComplianceController(robot, params.beta, params.beta_rotation_matrix, params.alpha, params.alpha_rotation_matrix, dT);
		special_controller->setStiffness(params.stiffness_b, params.stiffness_k);
		if(!goal_controller)
			special_controller->addPoint(params.RGoal, params.timeGoal, params.reuseGoal);
		controller = special_controller;
	}
	else if (params.type == "rxOrientationImpedanceController")
	{
		rxOrientationImpedanceController* special_controller = new rxOrientationImpedanceController(robot, params.beta, params.beta_rotation_matrix, params.alpha, params.alpha_rotation_matrix, dT);
		if(!goal_controller)
			special_controller->addPoint(params.RGoal, params.timeGoal, params.reuseGoal);
		special_controller->setImpedance(0.2, 8.0, 20.0);
		//special_controller->setImpedance(params.impedance_m, params.impedance_b, params.impedance_k);
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedOrientationController")
	{
		rxInterpolatedOrientationController* special_controller = new rxInterpolatedOrientationController(robot, params.beta, params.beta_rotation_matrix, params.alpha, params.alpha_rotation_matrix, dT);
		if(!goal_controller)
			special_controller->addPoint(params.RGoal, params.timeGoal, params.reuseGoal);
		special_controller->setGain(params.kv, params.kp, params.invL2sqr);	
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedOrientationComplianceController")
	{
		rxInterpolatedOrientationComplianceController* special_controller = new rxInterpolatedOrientationComplianceController(robot, params.beta, params.beta_rotation_matrix, params.alpha, params.alpha_rotation_matrix, dT);
		special_controller->setStiffness(params.stiffness_b, params.stiffness_k);
		if(!goal_controller)
			special_controller->addPoint(params.RGoal, params.timeGoal, params.reuseGoal);
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedOrientationImpedanceController")
	{
		rxInterpolatedOrientationImpedanceController* special_controller = new rxInterpolatedOrientationImpedanceController(robot, params.beta, params.beta_rotation_matrix, params.alpha, params.alpha_rotation_matrix, dT);
		if(!goal_controller)
			special_controller->addPoint(params.RGoal, params.timeGoal, params.reuseGoal);
		special_controller->setImpedance(0.2, 8.0, 20.0);
		//special_controller->setImpedance(params.impedance_m, params.impedance_b, params.impedance_k);
		controller = special_controller;
	}
	else if (params.type == "rxHTransformController")
	{
		rxHTransformController* special_controller = new rxHTransformController(robot, params.beta, HTransform(params.beta_rotation_matrix, params.beta_displacement), params.alpha, HTransform(params.alpha_rotation_matrix, params.alpha_displacement), dT);
		HTransform goal_HTransform(params.RGoal, params.rGoal);
		if(!goal_controller)
			special_controller->addPoint(goal_HTransform, params.timeGoal, params.reuseGoal);
		special_controller->setGain(params.kv, params.kp, params.invL2sqr);	
		controller = special_controller;
	}
	else if (params.type == "rxHTransformComplianceController")
	{
		rxHTransformComplianceController* special_controller = new rxHTransformComplianceController(robot, params.beta, HTransform(params.beta_rotation_matrix, params.beta_displacement), params.alpha, HTransform(params.alpha_rotation_matrix, params.alpha_displacement), dT);
		special_controller->setStiffness(params.stiffness_b, params.stiffness_k);
		HTransform goal_HTransform(params.RGoal, params.rGoal);
		if(!goal_controller)
			special_controller->addPoint(goal_HTransform, params.timeGoal, params.reuseGoal);
		controller = special_controller;
	}
	else if (params.type == "rxHTransformImpedanceController")
	{
		rxHTransformImpedanceController* special_controller = new rxHTransformImpedanceController(robot, params.beta, HTransform(params.beta_rotation_matrix, params.beta_displacement), params.alpha, HTransform(params.alpha_rotation_matrix, params.alpha_displacement), dT);
		HTransform goal_HTransform(params.RGoal, params.rGoal);
		if(!goal_controller)
			special_controller->addPoint(goal_HTransform, params.timeGoal, params.reuseGoal);
		special_controller->setImpedance(0.2, 8.0, 20.0);
		//special_controller->setImpedance(params.impedance_m, params.impedance_b, params.impedance_k);
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedHTransformController")
	{
		rxInterpolatedHTransformController* special_controller = new rxInterpolatedHTransformController(robot, params.beta, HTransform(params.beta_rotation_matrix, params.beta_displacement), params.alpha, HTransform(params.alpha_rotation_matrix, params.alpha_displacement), dT);
		HTransform goal_HTransform(params.RGoal, params.rGoal);
		if(!goal_controller)
			special_controller->addPoint(goal_HTransform, params.timeGoal, params.reuseGoal);
		special_controller->setGain(params.kv, params.kp, params.invL2sqr);	
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedHTransformComplianceController")
	{
		rxInterpolatedHTransformComplianceController* special_controller = new rxInterpolatedHTransformComplianceController(robot, params.beta, HTransform(params.beta_rotation_matrix, params.beta_displacement), params.alpha, HTransform(params.alpha_rotation_matrix, params.alpha_displacement), dT);
		special_controller->setStiffness(params.stiffness_b, params.stiffness_k);
		HTransform goal_HTransform(params.RGoal, params.rGoal);
		if(!goal_controller)
			special_controller->addPoint(goal_HTransform, params.timeGoal, params.reuseGoal);
		controller = special_controller;
	}
	else if (params.type == "rxInterpolatedHTransformImpedanceController")
	{
		rxInterpolatedHTransformImpedanceController* special_controller = new rxInterpolatedHTransformImpedanceController(robot, params.beta, HTransform(params.beta_rotation_matrix, params.beta_displacement), params.alpha, HTransform(params.alpha_rotation_matrix, params.alpha_displacement), dT);
		HTransform goal_HTransform(params.RGoal, params.rGoal);
		if(!goal_controller)
			special_controller->addPoint(goal_HTransform, params.timeGoal, params.reuseGoal);
		special_controller->setImpedance(0.2, 8.0, 20.0);
		//special_controller->setImpedance(params.impedance_m, params.impedance_b, params.impedance_k);
		controller = special_controller;
	}
	else if (params.type == "rxNullMotionController")
	{
		controller = new rxNullMotionController(robot, dT);
	}
	else if (params.type == "rxNullMotionComplianceController")
	{
		controller = new rxNullMotionComplianceController(robot,  dT);
	}
	else if (params.type == "SubdisplacementController")
	{
		SubdisplacementController* special_controller = new SubdisplacementController(robot, params.beta, params.beta_displacement, params.alpha, params.alpha_displacement, dT, params.index, params.max_force, robot->findBody(string_type(string2wstring(params.limit_body))), params.distance_limit );
		if(params.index.size() == 1)
		{
			special_controller->setTaskConstraints(params.tc[0]);
		}
		else if (params.index.size() == 2)
		{
			special_controller->setTaskConstraints(params.tc[0], params.tc[1]);
		}
		special_controller->setImpedance(0.2, 8.0, 20.0);
		//special_controller->setImpedance(params.impedance_m, params.impedance_b, params.impedance_k);
		controller = special_controller;
	}
	else if (params.type == "SubdisplacementSimpleController")
    {
		SubdisplacementSimpleController* special_controller = new SubdisplacementSimpleController(robot, params.beta, params.beta_displacement, params.alpha, params.alpha_displacement, dT, params.index, params.max_force, robot->findBody(string_type(string2wstring(params.limit_body))), params.distance_limit );
		if(params.index.size() == 1)
		{
			special_controller->setTaskConstraints(params.tc[0]);
		}
		else if (params.index.size() == 2)
		{
			special_controller->setTaskConstraints(params.tc[0], params.tc[1]);
		}
		special_controller->setGain(params.kv, params.kp, params.invL2sqr);
		controller = special_controller;
	}
	else if (params.type == "ObstacleAvoidanceController")
	{
		if(CollisionInterface::instance)
		{
			controller = new ObstacleAvoidanceController(robot, params.alpha, params.alpha_displacement, params.distance_threshold, CollisionInterface::instance, dT, params.deactivation_threshold);
			controller->setGain(params.kv, params.kp, params.invL2sqr);
		}
		else
			throw std::string("[XMLDeserializer::createController] ERROR: No collision interface used for obstacleAvoidanceController.");

	}
	else if (params.type == "JointLimitAvoidanceControllerOnDemand")
	{
		controller = new JointLimitAvoidanceControllerOnDemand(robot, params.index[0], params.safetyThresh, dT); 
		controller->setGain(params.kv, params.kp, params.invL2sqr);	
	}
	else if (params.type == "InterpolatedSetPointDisplacementController")
	{
		controller = new InterpolatedSetPointDisplacementController(robot, params.beta, params.beta_displacement, params.alpha, params.alpha_displacement, dT);
		controller->setGain(params.kv, params.kp, params.invL2sqr);
	}
	else if (params.type == "InterpolatedSetPointJointController")
	{
		controller = new InterpolatedSetPointJointController(robot, dT);
		controller->setGain(params.kv, params.kp, params.invL2sqr);
	}
	else if (params.type == "InterpolatedSetPointOrientationController")
	{
		controller = new InterpolatedSetPointOrientationController(robot, params.beta, params.beta_rotation_matrix, params.alpha, params.alpha_rotation_matrix, dT);
		controller->setGain(params.kv, params.kp, params.invL2sqr);
	}
	else if (params.type == "PressureDisplacementController")
	{
		controller = new PressureDisplacementController(robot, params.beta, params.beta_displacement, params.beta_rotation_matrix, params.alpha, params.alpha_displacement, dT);
		controller->setGain(params.kv, params.kp, params.invL2sqr);
	}
	else if (params.type == "PressureOrientationController")
	{
		controller = new PressureOrientationController(robot, params.beta, params.beta_rotation_matrix, params.alpha, params.alpha_rotation_matrix, dT);
		controller->setGain(params.kv, params.kp, params.invL2sqr);
	}
	else
	{
		// unknown type
		throw std::string("[XMLDeserializer::createController] ERROR: Unknown Controller type.");
	}

	// quasicoord <-- missing
	// functional <-- missing (and more!)

	// we need unique names for all controllers in a single control set, otherwise RLAB will complain
	std::wstringstream wss;
	wss << "controller_" << controller_counter;
	controller->setName(wss.str());

	controller->setPriority(params.priority);
	controller->setIKMode(params.ik);

	//do not call setGain here - it will overwrite impedance and stiffness values!

	return controller;
}


std::map<std::string, ControllerType> XMLDeserializer::createControllerMapping()
{
	std::map<std::string, ControllerType> mapping;
	mapping["rxJointController"]							= ControllerType(rxController::eControlType_Joint, NONE);
	mapping["rxJointComplianceController"]					= ControllerType(rxController::eControlType_Joint, WITH_COMPLIANCE);
	mapping["rxJointImpedanceController"]					= ControllerType(rxController::eControlType_Joint, WITH_IMPEDANCE);
	mapping["rxInterpolatedJointController"]				= ControllerType(rxController::eControlType_Joint, WITH_INTERPOLATION);
	mapping["rxInterpolatedJointComplianceController"]		= ControllerType(rxController::eControlType_Joint, WITH_INTERPOLATION | WITH_COMPLIANCE);
	mapping["rxInterpolatedJointImpedanceController"]		= ControllerType(rxController::eControlType_Joint, WITH_INTERPOLATION | WITH_IMPEDANCE);

	mapping["rxFunctionController"]							= ControllerType(rxController::eControlType_Functional, NONE);
	mapping["rxFunctionComplianceController"]				= ControllerType(rxController::eControlType_Functional, WITH_COMPLIANCE);
	mapping["rxFunctionImpedanceController"]				= ControllerType(rxController::eControlType_Functional, WITH_IMPEDANCE);
	mapping["rxInterpolatedFunctionController"]				= ControllerType(rxController::eControlType_Functional, WITH_INTERPOLATION);
	mapping["rxInterpolatedFunctionComplianceController"]	= ControllerType(rxController::eControlType_Functional, WITH_INTERPOLATION | WITH_COMPLIANCE);
	mapping["rxInterpolatedFunctionImpedanceController"]	= ControllerType(rxController::eControlType_Functional, WITH_INTERPOLATION | WITH_IMPEDANCE);

	mapping["rxDisplacementController"]						= ControllerType(rxController::eControlType_Displacement, NONE);
	mapping["rxDisplacementFunctionController"]				= ControllerType(rxController::eControlType_Displacement, WITH_FUNCTION);
	mapping["rxDisplacementFunctionComplianceController"]	= ControllerType(rxController::eControlType_Displacement, WITH_FUNCTION | WITH_COMPLIANCE);
	mapping["rxDisplacementFunctionImpedanceController"]	= ControllerType(rxController::eControlType_Displacement, WITH_FUNCTION | WITH_IMPEDANCE);
	mapping["rxDisplacementComplianceController"]			= ControllerType(rxController::eControlType_Displacement, WITH_COMPLIANCE);
	mapping["rxDisplacementImpedanceController"]			= ControllerType(rxController::eControlType_Displacement, WITH_IMPEDANCE);
	mapping["rxInterpolatedDisplacementController"]			= ControllerType(rxController::eControlType_Displacement, WITH_INTERPOLATION);
	mapping["rxInterpolatedDisplacementComplianceController"] = ControllerType(rxController::eControlType_Displacement, WITH_INTERPOLATION | WITH_COMPLIANCE);
	mapping["rxInterpolatedDisplacementImpedanceController"] = ControllerType(rxController::eControlType_Displacement, WITH_INTERPOLATION | WITH_IMPEDANCE);

	mapping["rxOrientationController"]						= ControllerType(rxController::eControlType_Orientation, NONE);
	mapping["rxOrientationFunctionController"]				= ControllerType(rxController::eControlType_Orientation, WITH_FUNCTION);
	mapping["rxOrientationFunctionComplianceController"]	= ControllerType(rxController::eControlType_Orientation, WITH_FUNCTION | WITH_COMPLIANCE);
	mapping["rxOrientationFunctionImpedanceController"]		= ControllerType(rxController::eControlType_Orientation, WITH_FUNCTION | WITH_IMPEDANCE);
	mapping["rxOrientationComplianceController"]			= ControllerType(rxController::eControlType_Orientation, WITH_COMPLIANCE);
	mapping["rxOrientationImpedanceController"]				= ControllerType(rxController::eControlType_Orientation, WITH_IMPEDANCE);
	mapping["rxInterpolatedOrientationController"]			= ControllerType(rxController::eControlType_Orientation, WITH_INTERPOLATION);
	mapping["rxInterpolatedOrientationComplianceController"] = ControllerType(rxController::eControlType_Orientation, WITH_INTERPOLATION | WITH_COMPLIANCE);
	mapping["rxInterpolatedOrientationImpedanceController"] = ControllerType(rxController::eControlType_Orientation, WITH_INTERPOLATION | WITH_IMPEDANCE);

	mapping["rxHTransformController"]						= ControllerType(rxController::eControlType_HTransform, NONE);
	mapping["rxHTransformFunctionController"]				= ControllerType(rxController::eControlType_HTransform, WITH_FUNCTION);
	mapping["rxHTransformFunctionComplianceController"]		= ControllerType(rxController::eControlType_HTransform, WITH_FUNCTION | WITH_COMPLIANCE);
	mapping["rxHTransformFunctionImpedanceController"]		= ControllerType(rxController::eControlType_HTransform, WITH_FUNCTION | WITH_IMPEDANCE);
	mapping["rxHTransformComplianceController"]				= ControllerType(rxController::eControlType_HTransform, WITH_COMPLIANCE);
	mapping["rxHTransformImpedanceController"]				= ControllerType(rxController::eControlType_HTransform, WITH_IMPEDANCE);
	mapping["rxInterpolatedHTransformController"]			= ControllerType(rxController::eControlType_HTransform, WITH_INTERPOLATION);
	mapping["rxInterpolatedHTransformComplianceController"] = ControllerType(rxController::eControlType_HTransform, WITH_INTERPOLATION | WITH_COMPLIANCE);
	mapping["rxInterpolatedHTransformImpedanceController"]	= ControllerType(rxController::eControlType_HTransform, WITH_INTERPOLATION | WITH_IMPEDANCE);

	mapping["rxQuasiCoordController"]						= ControllerType(rxController::eControlType_QuasiCoord, NONE);
	mapping["rxQuasiCoordComplianceController"]				= ControllerType(rxController::eControlType_QuasiCoord, WITH_COMPLIANCE);

	mapping["rxNullMotionController"]						= ControllerType(rxController::eControlType_NullMotion, NONE);
	mapping["rxNullMotionComplianceController"]				= ControllerType(rxController::eControlType_NullMotion, WITH_COMPLIANCE);
	mapping["rxGradientNullMotionController"]				= ControllerType(rxController::eControlType_NullMotion, WITH_GRADIENT);

	mapping["FeatureAttractorController"]					= ControllerType(rxController::eControlType_Displacement, WITH_IMPEDANCE | ATTRACTOR);
	mapping["SubdisplacementController"]					= ControllerType(rxController::eControlType_Functional, WITH_IMPEDANCE | SUBDISPLACEMENT);
	mapping["SubdisplacementSimpleController"]				= ControllerType(rxController::eControlType_Displacement, SUBDISPLACEMENT);
	mapping["ObstacleAvoidanceController"]					= ControllerType(rxController::eControlType_Functional, OBSTACLE_AVOIDANCE);
	mapping["JointBlackBoardController"]					= ControllerType(rxController::eControlType_Joint, BLACKBOARD_ACCESS);

	mapping["SingularityAvoidanceController"]				= ControllerType(rxController::eControlType_Joint, SINGULARITY_AVOIDANCE);
	mapping["OpSpaceSingularityAvoidanceController"]    	= ControllerType(rxController::eControlType_Displacement, SINGULARITY_AVOIDANCE);
	mapping["JointLimitAvoidanceControllerOnDemand"]		= ControllerType(rxController::eControlType_Functional, JOINT_LIMIT_AVOIDANCE);

	mapping["InterpolatedSetPointDisplacementController"]	= ControllerType(rxController::eControlType_Displacement, WITH_INTERPOLATION);
	mapping["InterpolatedSetPointJointController"]			= ControllerType(rxController::eControlType_Joint, WITH_INTERPOLATION);
	mapping["InterpolatedSetPointOrientationController"]	= ControllerType(rxController::eControlType_Orientation, WITH_INTERPOLATION);

	return mapping;
}