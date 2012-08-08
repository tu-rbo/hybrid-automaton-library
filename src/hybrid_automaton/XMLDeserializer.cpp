#include "XMLDeserializer.h"

#include "CSpaceMilestone.h"
#include "OpSpaceMilestone.h"

#include "rControlalgorithm\rControlalgorithm.h"
#include "rxControlSDK\rxControlSDK.h"


#include "FeatureAttractorController.h"
#include "SubdisplacementController.h"
#include "ObstacleAvoidanceController.h"
#include "JointBlackBoardController.h"

std::map<std::string, ControllerType> XMLDeserializer::controller_map_ = XMLDeserializer::createControllerMapping();

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

std::vector<double> deserializeVectorDouble(TiXmlElement * xml_element, const char * field_name)
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

std::string XMLDeserializer::wstring2string(const std::wstring& wstr)
{
	/*std::string str(wstr.length(),' ');
	copy(wstr.begin(),wstr.end(),str.begin());
	return str;*/
	return std::string(wstr.begin(), wstr.end());
}

std::wstring XMLDeserializer::string2wstring(const std::string& str)
{
	/*std::wstring wstr(str.length(),L' ');
	copy(str.begin(),str.end(),wstr.begin());
	return wstr;*/
	return std::wstring(str.begin(), str.end());
}

Rotation XMLDeserializer::string2rotation(const std::string& str)
{
	std::stringstream ss = std::stringstream(str);
	double value = -1.0;
	Rotation result;
	int i = 0;
	int j = 0;
	while(ss >> value)
	{
		result.set(i,j,value);
		j++;
		if(j > 2)
		{
			i++;
			j=0;
		}
	}
	assert(i == j == 3);
	return result;
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

	// Print out a message with the general properties of the new HybridAutomaton.
	std::cout << "[XMLDeserializer::createHybridAutomaton] INFO: Creating hybrid system '" << ha_element->Attribute("Name") << "'. Initial Milestone: " << start_node << std::endl;

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
	std::vector<double> mst_configuration = deserializeVectorDouble(milestone_xml, "value");
	std::vector<double> mst_epsilon = deserializeVectorDouble(milestone_xml, "epsilon");
	if(mst_configuration.size()==0 || mst_epsilon.size()==0)
	{
		throw std::string("[XMLDeserializer::createCSpaceMilestone] ERROR: The milestone configuration or region of convergence (\"value\" or \"epsilon\") is not defined in the XML string.");
	}

    double expLength = deserializeElement<double>(milestone_xml, "expectedLength");

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
    
    mst->setExpectedLength(expLength);

	return mst;
}

CSpaceBlackBoardMilestone* XMLDeserializer::createCSpaceBlackBoardMilestone(TiXmlElement* milestone_xml, rxSystem* robot, double dT)
{
	Milestone::Status mst_status = (Milestone::Status)deserializeElement<int>(milestone_xml, "status");
	std::string mst_name = deserializeString(milestone_xml, "name");
	std::vector<double> mst_configuration = deserializeVectorDouble(milestone_xml, "value");
	std::vector<double> mst_epsilon = deserializeVectorDouble(milestone_xml, "epsilon");
	if(mst_configuration.size()==0 || mst_epsilon.size()==0)
	{
		throw std::string("[XMLDeserializer::createCSpaceMilestone] ERROR: The milestone configuration or region of convergence (\"value\" or \"epsilon\") is not defined in the XML string.");
	}

    double expLength = deserializeElement<double>(milestone_xml, "expectedLength");

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

    mst->setExpectedLength(expLength);
	
	return mst;
}

OpSpaceMilestone* XMLDeserializer::createOpSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot, double dT)
{
	Milestone::Status mst_status = (Milestone::Status)deserializeElement<int>(milestone_xml, "status");
	std::string mst_name = deserializeString(milestone_xml, "name");
	PosiOriSelector mst_pos = (PosiOriSelector)deserializeElement<int>(milestone_xml, "PosiOriSelector");
	std::vector<double> mst_configuration = deserializeVectorDouble(milestone_xml, "value");
	std::vector<double> mst_epsilon = deserializeVectorDouble(milestone_xml, "epsilon");
	if(mst_configuration.size()==0 || mst_epsilon.size()==0)
	{
		throw std::string("[XMLDeserializer::createOpSpaceMilestone] ERROR: The milestone configuration or region of convergence (\"value\" or \"epsilon\") is not defined in the XML string.");
	}

    double expLength = deserializeElement<double>(milestone_xml, "expectedLength");

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

	OpSpaceMilestone* mst = new OpSpaceMilestone(mst_name, mst_configuration, mst_pos, NULL, mst_epsilon, mst_status, mst_handle_points);

	TiXmlElement* mb_element = milestone_xml->FirstChildElement("MotionBehaviour");
	MotionBehaviour* mst_mb = NULL;
	if(mb_element)
	{
		mst_mb = XMLDeserializer::createMotionBehaviour(mb_element, mst, mst, robot, dT);
		mst->setMotionBehaviour(mst_mb);
	}

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
		if(control_set_type==std::string("rxControlSet"))
		{		
			mb_control_set = new rxControlSet(robot, dT);
			mb_control_set->setGravity(0,0,-GRAV_ACC);
			mb_control_set->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(robot));
			mb_control_set->nullMotionController()->setGain(0.02,0.0,0.01);
		}
		else if(control_set_type==std::string("rxTPOperationalSpaceControlSet"))
		{		
			mb_control_set = new rxTPOperationalSpaceControlSet(robot, dT);
			mb_control_set->setGravity(0,0,-GRAV_ACC);
			//mb_control_set->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(robot));
			//mb_control_set->nullMotionController()->setGain(0.02,0.0,0.01);
			mb_control_set->nullMotionController()->setGain(10.0,1.0); // taken from ERM values...
		}
		else
		{
			std::cout << control_set_type << std::endl;
			throw std::string("[XMLDeserializer::createMotionBehaviour] ERROR: Unknown type of rxControlSet.");
		}	
	}

	MotionBehaviour* mb = new MotionBehaviour(dad, son , mb_control_set);

	int mb_controller_counter = 0;
	for(TiXmlElement* rxController_xml = control_set_element->FirstChildElement("Controller"); rxController_xml; rxController_xml = rxController_xml->NextSiblingElement())
	{	
		bool mb_goal_controller = deserializeBoolean(rxController_xml, "goalController", true);		
		rxController* mb_controller = XMLDeserializer::createController(rxController_xml, dad, son, robot, dT, mb_goal_controller, mb_controller_counter);
		mb->addController(mb_controller, mb_goal_controller);
		mb_controller_counter++;
	}

    double length = deserializeElement<double>(motion_behaviour_xml, "length");
    double prob   = deserializeElement<double>(motion_behaviour_xml, "probability");
    mb->setLength(length);
    mb->setProbability(prob);

	return mb;
}


rxController* XMLDeserializer::createController(TiXmlElement *rxController_xml, const Milestone *dad, const Milestone *son, 
												rxSystem *robot, double dT, bool goal_controller, int controller_counter)
{
	std::string controller_class_name = deserializeString(rxController_xml, "type");
	ControllerType type_of_controller = XMLDeserializer::controller_map_[controller_class_name];
	bool controller_ik_bool = deserializeBoolean(rxController_xml, "ik");
	std::stringstream kp_vector_ss = std::stringstream(deserializeString(rxController_xml,"kp"));
	std::stringstream kv_vector_ss = std::stringstream(deserializeString(rxController_xml,"kv"));
	std::stringstream invL2sqr_vector_ss = std::stringstream(deserializeString(rxController_xml,"invL2sqr"));
	int priority = deserializeElement<int>(rxController_xml, "priority", 1);

	dVector kp_vector;
	dVector kv_vector;
	dVector invL2sqr_vector;
	kp_vector.all(-1.0);
	double kp_value = -1.0;
	double kv_value = -1.0;
	double invL2sqr_value = -1.0;
	while(kp_vector_ss >> kp_value && kv_vector_ss >> kv_value && invL2sqr_vector_ss >> invL2sqr_value)
	{	
		kp_vector.expand(1,kp_value);		
		kv_vector.expand(1,kv_value);		
		invL2sqr_vector.expand(1,invL2sqr_value);
	}

	double ctrl_total_time = 0.0;
	std::vector<ViaPointBase*> via_points_ptr;

	// Create the Controller
	rxController* controller = NULL;

	switch(type_of_controller.first)
	{
	case rxController::eControlType_Joint:
		controller = XMLDeserializer::createJointController(type_of_controller.second, dT, via_points_ptr, rxController_xml, robot);
		break;
	case rxController::eControlType_Displacement:
		controller = XMLDeserializer::createDisplacementController(type_of_controller.second, dT, via_points_ptr, rxController_xml, robot);
		break;
	case rxController::eControlType_Orientation:
		controller = XMLDeserializer::createOrientationController(type_of_controller.second, dT, via_points_ptr, rxController_xml, robot);
		break;
	case rxController::eControlType_HTransform:
		controller = XMLDeserializer::createHTransformController(type_of_controller.second, dT, via_points_ptr, rxController_xml, robot);
		break;
	case rxController::eControlType_QuasiCoord:
		controller = XMLDeserializer::createQuasiCoordController(type_of_controller.second, dT, robot);
		break;
	case rxController::eControlType_NullMotion:
		controller = XMLDeserializer::createNullMotionController(type_of_controller.second, dT, robot);
		break;
	case rxController::eControlType_Functional:
		{
		controller = XMLDeserializer::createFunctionalController(type_of_controller.second, dT, rxController_xml, robot);
		break;
		}
	default:
		throw std::string("[XMLDeserializer::createController] ERROR: Unexpected Controller group.");
		break;
	}

	// we need unique names for all controllers in a single control set, otherwise RLAB will complain
	std::wstringstream wss;
	wss << "controller_" << controller_counter;
	controller->setName(wss.str());

	if(!goal_controller && !(type_of_controller.second & OBSTACLE_AVOIDANCE) &&!(type_of_controller.second & SUBDISPLACEMENT) )
	{

		double time_goal = deserializeElement<double>(rxController_xml, "timeGoal");
		bool reuse_goal = deserializeBoolean(rxController_xml, "reuseGoal");
		int type_goal = deserializeElement<int>(rxController_xml, "typeGoal");

		switch(controller->type()) {
	case rxController::eControlType_Joint:
		{
			std::stringstream goal_ss = std::stringstream(deserializeString(rxController_xml,"dVectorGoal"));
			dVector goal_vector;
			double goal_value = -1.0;
			while(goal_ss >> goal_value)
			{	
				goal_vector.expand(1,goal_value);		
			}
			dynamic_cast<rxJointController*>(controller)->addPoint(goal_vector, time_goal, reuse_goal, eInterpolatorType_Cubic);
			break;
		}
	case rxController::eControlType_Displacement:
		{
			std::stringstream goal_ss = std::stringstream(deserializeString(rxController_xml,"Vector3DGoal"));
			Vector3D goal_vector;
			double goal_value = -1.0;
			while(goal_ss >> goal_value)
			{	
				goal_vector.expand(1,goal_value);		
			}
			dynamic_cast<rxDisplacementController*>(controller)->addPoint(goal_vector, time_goal, reuse_goal, eInterpolatorType_Cubic);
			break;
		}
	case rxController::eControlType_Orientation:
		{
			std::stringstream goal_ss = std::stringstream(deserializeString(rxController_xml, "RGoal"));
			double goal_value = -1.0;
			std::vector<double> goal_values;
			for(int i=0; i<9; i++)
			{
				goal_ss >> goal_value;
				goal_values.push_back(goal_value);
			}
			Rotation goal_Rotation(goal_values[0], goal_values[1],goal_values[2],goal_values[3],
				goal_values[4],goal_values[5],goal_values[6],goal_values[7],goal_values[8]);
			dynamic_cast<rxOrientationController*>(controller)->addPoint(goal_Rotation, time_goal, reuse_goal);
			break;
		}
	case rxController::eControlType_HTransform:
		{
			std::stringstream goal_ss = std::stringstream(deserializeString(rxController_xml, "RGoal"));
			double goal_value = -1.0;
			std::vector<double> goal_values;
			for(int i=0; i<9; i++)
			{
				goal_ss >> goal_value;
				goal_values.push_back(goal_value);
			}
			Rotation goal_Rotation(goal_values[0], goal_values[1],goal_values[2],goal_values[3],
				goal_values[4],goal_values[5],goal_values[6],goal_values[7],goal_values[8]);

			std::stringstream goal_ss2 = std::stringstream(deserializeString(rxController_xml, "rGoal"));
			goal_values.clear();
			for(int i=0; i<9; i++)
			{
				goal_ss2 >> goal_value;
				goal_values.push_back(goal_value);
			}
			Displacement goal_Displacement(goal_values[0], goal_values[1],goal_values[2]);		

			HTransform goal_HTransform(goal_Rotation, goal_Displacement);
			dynamic_cast<rxHTransformController*>(controller)->addPoint(goal_HTransform, time_goal, reuse_goal);
			break;
		}
		}
	}

	controller->setPriority(priority);
	controller->setIKMode(controller_ik_bool);
	controller->setGain(kv_vector, kp_vector, invL2sqr_vector);	
	return controller;
}

rxController* XMLDeserializer::createJointController(int joint_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, 
													 TiXmlElement* rxController_xml, rxSystem* robot)
{
	rxController* controller = NULL;
	switch(joint_subtype)
	{
	case NONE:
		controller = new rxJointController(robot, controller_duration);
		break;
	case WITH_COMPLIANCE:
		controller = new rxJointComplianceController(robot, controller_duration);	
		break;
	case WITH_IMPEDANCE:
		controller = new rxJointImpedanceController(robot, controller_duration);
		break;
	case WITH_INTERPOLATION:
		controller = new rxInterpolatedJointController(robot, controller_duration);
		break;
	case WITH_INTERPOLATION | WITH_COMPLIANCE:
		controller = new rxInterpolatedJointComplianceController(robot, controller_duration);
		break;
	case WITH_INTERPOLATION | WITH_IMPEDANCE:
		controller = new rxInterpolatedJointImpedanceController(robot, controller_duration);
		dynamic_cast<rxInterpolatedJointImpedanceController*>(controller)->setImpedance(0.5,3.0,2.0);
		break;
	case BLACKBOARD_ACCESS:
		controller = new JointBlackBoardController(robot, controller_duration);
		break;
	default:
		throw std::string("[XMLDeserializer::createJointController] ERROR: Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointdVector* vpdv_ptr = (ViaPointdVector*)((*vp_it));
		dynamic_cast<rxJointController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}

	if(joint_subtype & WITH_COMPLIANCE)
	{
		std::stringstream stiffness_b_ss = std::stringstream(deserializeString(rxController_xml,"stiffness_b"));
		std::stringstream stiffness_k_ss = std::stringstream(deserializeString(rxController_xml,"stiffness_k"));
		double stiffness_b_value = -1.0;
		double stiffness_k_value = -1.0;
		dVector stiffness_b_displacement;
		dVector stiffness_k_displacement;
		while ((stiffness_b_ss >> stiffness_b_value))
		{
			stiffness_b_displacement.expand(1,stiffness_b_value);
		}
		while ((stiffness_k_ss >> stiffness_k_value))
		{
			stiffness_k_displacement.expand(1,stiffness_k_value);
		}
		dynamic_cast<rxJointComplianceController*>(controller)->setStiffness(stiffness_b_displacement, stiffness_k_displacement);
	}

	if (joint_subtype & BLACKBOARD_ACCESS)
	{
		// parse the name of the blackboard variable to be accessed
		// if not given --- use default
		JointBlackBoardController* bb_controller = dynamic_cast<JointBlackBoardController*>(controller);
		bb_controller->setBlackBoardVariableName(bb_controller->getBlackBoardVariableName());
	}

	return controller;
}

rxController* XMLDeserializer::createDisplacementController(int displacement_subtype, double controller_duration, 
															std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml, rxSystem* robot)
{
	rxBody*   alpha = NULL;
	rxBody*   beta = NULL;
	rxController* controller = NULL;
	dVector alpha_displacement;
	std::string alpha_str = deserializeString(rxController_xml,"alpha", false);
	alpha = robot->findBody(string2wstring(alpha_str));
	if(alpha)
	{
		std::stringstream alpha_ss = std::stringstream(deserializeString(rxController_xml,"alphaDisplacement"));
		double alpha_value = -1.0;
		while ((alpha_ss >> alpha_value))
		{
			alpha_displacement.expand(1,alpha_value);
		}
	}else{
		alpha_displacement = Displacement();
	}

	dVector beta_displacement;
	std::string beta_str = deserializeString(rxController_xml,"beta", false);
	beta = robot->findBody(string2wstring(beta_str));
	if(beta)
	{
		std::stringstream beta_ss = std::stringstream(deserializeString(rxController_xml,"betaDisplacement"));
		double beta_value = -1.0;	
		while((beta_ss >> beta_value))
		{
			beta_displacement.expand(1,beta_value);
		}
	}else{
		beta_displacement = Displacement();
	}

	switch(displacement_subtype)
	{
	case NONE:
		controller = new rxDisplacementController(robot, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration);
		break;
	case WITH_COMPLIANCE:
		{
			controller = new rxDisplacementComplianceController(robot, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
			break;
		}
	case WITH_IMPEDANCE:
		controller = new rxDisplacementImpedanceController(robot, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
		break;
	case WITH_INTERPOLATION:
		controller = new rxInterpolatedDisplacementController(robot, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
		break;
	case WITH_INTERPOLATION | WITH_COMPLIANCE:
		{
			controller = new rxInterpolatedDisplacementComplianceController(robot, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
			break;
		}
	case WITH_INTERPOLATION | WITH_IMPEDANCE:
		controller = new rxInterpolatedDisplacementImpedanceController(robot, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
		break;
	case WITH_IMPEDANCE | ATTRACTOR:
		{
			double desired_distance = deserializeElement<double>(rxController_xml,"desiredDistance", 1.);
			double max_force = deserializeElement<double>(rxController_xml,"maxForce", 1.);
			controller = new FeatureAttractorController(robot, beta, Displacement(beta_displacement), controller_duration, desired_distance, max_force );
			dynamic_cast<FeatureAttractorController*>(controller)->setImpedance(0.2,8.0,20.0);
			break;
		}
	default:
		throw std::string("[XMLDeserializer::createDisplacementController] ERROR: Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointVector3D* vpdv_ptr = (ViaPointVector3D*)((*vp_it));
		dynamic_cast<rxDisplacementController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}

	if(displacement_subtype & WITH_COMPLIANCE)
	{
		std::stringstream stiffness_b_ss = std::stringstream(deserializeString(rxController_xml,"stiffness_b"));
		std::stringstream stiffness_k_ss = std::stringstream(deserializeString(rxController_xml,"stiffness_k"));
		double stiffness_b_value = -1.0;
		double stiffness_k_value = -1.0;
		dVector stiffness_b_displacement;
		dVector stiffness_k_displacement;
		while ((stiffness_b_ss >> stiffness_b_value))
		{
			stiffness_b_displacement.expand(1,stiffness_b_value);
		}
		while ((stiffness_k_ss >> stiffness_k_value))
		{
			stiffness_k_displacement.expand(1,stiffness_k_value);
		}
		dynamic_cast<rxDisplacementComplianceController*>(controller)->setStiffness(stiffness_b_displacement, stiffness_k_displacement);
	}
	return controller;
}

rxController* XMLDeserializer::createOrientationController(int orientation_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr,
														   TiXmlElement* rxController_xml, rxSystem* robot)
{
	rxBody*   alpha = NULL;
	rxBody*   beta = NULL;
	rxController* controller = NULL;
	Rotation alpha_rotation_matrix;
	Rotation beta_rotation_matrix;
	std::string alpha_str = deserializeString(rxController_xml,"alpha",false);
	alpha = robot->findBody(string2wstring(alpha_str));
	if(alpha)
	{
		alpha_rotation_matrix = string2rotation(deserializeString(rxController_xml,"alphaRotation"));
	}
	else
	{
		alpha_rotation_matrix = Rotation();
	}
	std::string beta_str = deserializeString(rxController_xml,"beta",false);
	beta = robot->findBody(string2wstring(beta_str));
	if(beta)
	{
		beta_rotation_matrix = string2rotation(deserializeString(rxController_xml,"betaRotation"));
	}
	else
	{
		beta_rotation_matrix = Rotation();
	}

	switch(orientation_subtype)
	{
	case NONE:
		controller = new rxOrientationController(robot, beta, beta_rotation_matrix, alpha, alpha_rotation_matrix, controller_duration );
		break;
	case WITH_COMPLIANCE:
		{
			controller = new rxOrientationComplianceController(robot, beta, beta_rotation_matrix, alpha,alpha_rotation_matrix, controller_duration );
			break;
		}
	case WITH_IMPEDANCE:
		controller = new rxOrientationImpedanceController(robot, beta, beta_rotation_matrix, alpha,alpha_rotation_matrix, controller_duration );
		break;
	case WITH_INTERPOLATION:
		controller = new rxInterpolatedOrientationController(robot, beta, beta_rotation_matrix, alpha,alpha_rotation_matrix, controller_duration );
		break;
	case WITH_INTERPOLATION | WITH_COMPLIANCE:
		{
			controller = new rxInterpolatedOrientationComplianceController(robot, beta, beta_rotation_matrix, alpha,alpha_rotation_matrix, controller_duration );
			break;
		}
	case WITH_INTERPOLATION | WITH_IMPEDANCE:
		controller = new rxInterpolatedOrientationImpedanceController(robot, beta, beta_rotation_matrix, alpha,alpha_rotation_matrix, controller_duration );
		break;
	default:
		throw std::string("[XMLDeserializer::createOrientationController] ERROR:  Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointRotation* vpdv_ptr = (ViaPointRotation*)((*vp_it));
		dynamic_cast<rxOrientationController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}
	if(orientation_subtype & WITH_COMPLIANCE)
	{
		std::stringstream stiffness_b_ss(deserializeString(rxController_xml,"stiffness_b"));
		std::stringstream stiffness_k_ss(deserializeString(rxController_xml,"stiffness_k"));
		double stiffness_b_value = -1.0;
		double stiffness_k_value = -1.0;
		dVector stiffness_b_displacement;
		dVector stiffness_k_displacement;
		while ((stiffness_b_ss >> stiffness_b_value))
		{
			stiffness_b_displacement.expand(1,stiffness_b_value);
		}
		while ((stiffness_k_ss >> stiffness_k_value))
		{
			stiffness_k_displacement.expand(1,stiffness_k_value);
		}
		dynamic_cast<rxOrientationComplianceController*>(controller)->setStiffness(stiffness_b_displacement, stiffness_k_displacement);
	}
	return controller;
}

rxController* XMLDeserializer::createHTransformController(int htransform_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr,
														  TiXmlElement* rxController_xml, rxSystem* robot)
{
    rxBody*   alpha = NULL;
	rxBody*   beta = NULL;
	rxController* controller = NULL;
	Rotation alpha_rotation_matrix;
    Displacement alpha_displacement;
	Rotation beta_rotation_matrix;
    Displacement beta_displacement;
	std::string alpha_str = deserializeString(rxController_xml,"alpha",false);
	alpha = robot->findBody(string2wstring(alpha_str));
	if(alpha)
	{
		alpha_rotation_matrix = string2rotation(deserializeString(rxController_xml,"alphaRotation"));
        
        std::stringstream alpha_ss = std::stringstream(deserializeString(rxController_xml,"alphaDisplacement"));
		double alpha_value = -1.0;
		while ((alpha_ss >> alpha_value))
		{
			alpha_displacement.expand(1,alpha_value);
		}
	}
	else
	{
		alpha_rotation_matrix = Rotation();
        alpha_displacement = Displacement();
	}

	std::string beta_str = deserializeString(rxController_xml,"beta",false);
	beta = robot->findBody(string2wstring(beta_str));
	if(beta)
	{
		beta_rotation_matrix = string2rotation(deserializeString(rxController_xml,"betaRotation"));
        
        std::stringstream beta_ss = std::stringstream(deserializeString(rxController_xml,"betaDisplacement"));
		double beta_value = -1.0;
		while ((beta_ss >> beta_value))
		{
			beta_displacement.expand(1,beta_value);
		}
	}
	else
	{
		beta_rotation_matrix = Rotation();
        beta_displacement = Displacement();
	}

	switch(htransform_subtype)
	{
	case NONE:
		controller = new rxHTransformController(robot, beta, HTransform(beta_rotation_matrix, beta_displacement), alpha, HTransform(alpha_rotation_matrix, alpha_displacement), controller_duration );
		break;
	case WITH_COMPLIANCE:
		controller = new rxHTransformComplianceController(robot, beta, HTransform(beta_rotation_matrix, beta_displacement), alpha, HTransform(alpha_rotation_matrix, alpha_displacement), controller_duration );
		break;
	case WITH_IMPEDANCE:
		controller = new rxHTransformImpedanceController(robot, beta, HTransform(beta_rotation_matrix, beta_displacement), alpha, HTransform(alpha_rotation_matrix, alpha_displacement), controller_duration );
		break;
	case WITH_INTERPOLATION:
		controller = new rxInterpolatedHTransformController(robot, beta, HTransform(beta_rotation_matrix, beta_displacement), alpha, HTransform(alpha_rotation_matrix, alpha_displacement), controller_duration );
		break;
	case WITH_INTERPOLATION | WITH_COMPLIANCE:
		controller = new rxInterpolatedHTransformComplianceController(robot, beta, HTransform(beta_rotation_matrix, beta_displacement), alpha, HTransform(alpha_rotation_matrix, alpha_displacement), controller_duration );
		break;
	case WITH_INTERPOLATION | WITH_IMPEDANCE:
		controller = new rxInterpolatedHTransformImpedanceController(robot, beta, HTransform(beta_rotation_matrix, beta_displacement), alpha, HTransform(alpha_rotation_matrix, alpha_displacement), controller_duration );
		break;
	default:
		throw std::string("[XMLDeserializer::createHTransformController] ERROR: Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointHTransform* vpdv_ptr = (ViaPointHTransform*)((*vp_it));
		dynamic_cast<rxHTransformController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}

	if(htransform_subtype & WITH_COMPLIANCE)
	{
		std::stringstream stiffness_b_ss = std::stringstream(deserializeString(rxController_xml,"stiffness_b"));
		std::stringstream stiffness_k_ss = std::stringstream(deserializeString(rxController_xml,"stiffness_k"));
		double stiffness_b_value = -1.0;
		double stiffness_k_value = -1.0;
		dVector stiffness_b_displacement;
		dVector stiffness_k_displacement;
		while ((stiffness_b_ss >> stiffness_b_value))
		{
			stiffness_b_displacement.expand(1,stiffness_b_value);
		}
		while ((stiffness_k_ss >> stiffness_k_value))
		{
			stiffness_k_displacement.expand(1,stiffness_k_value);
		}
		dynamic_cast<rxHTransformComplianceController*>(controller)->setStiffness(stiffness_b_displacement, stiffness_k_displacement);
	}
	return controller;
}

rxController* XMLDeserializer::createQuasiCoordController(int quasi_coord_subtype, double controller_duration, rxSystem* robot)
{
	rxController* controller = NULL;
	switch(quasi_coord_subtype)
	{
	case NONE:
		//	controller = new rxQuasiCoordController(robot, , controller_duration);
		break;
	case WITH_COMPLIANCE:
		//	controller = new rxQuasiCoordComplianceController(robot, ,controller_duration);
		break;
	default:
		throw std::string("[XMLDeserializer::createQuasiCoordController] ERROR: Unexpected Controller subgroup.");
		break;
	}
	return controller;
}

rxController* XMLDeserializer::createNullMotionController(int null_motion_subtype, double controller_duration, rxSystem * robot)
{
	rxController* controller = NULL;
	switch(null_motion_subtype)
	{
	case NONE:
		controller = new rxNullMotionController(robot, controller_duration);
		break;
	case WITH_COMPLIANCE:
		controller = new rxNullMotionComplianceController(robot,  controller_duration);
		break;
	case WITH_GRADIENT:
		//	controller = new rxGradientNullMotionController(robot, controller_duration);
		break;
	default:
		throw std::string("[XMLDeserializer::createNullMotionController] ERROR: Unexpected Controller subgroup.");
		break;
	}
	return controller;
}

rxController* XMLDeserializer::createFunctionalController(int functional_subtype, double controller_duration, TiXmlElement* rxController_xml, rxSystem* robot)
{
	rxController* controller = NULL;
	switch(functional_subtype)
	{
	case WITH_IMPEDANCE | SUBDISPLACEMENT:
		{
			rxBody*   alpha = NULL;
			rxBody*   beta = NULL;
			dVector alpha_displacement;
			std::string alpha_str = deserializeString(rxController_xml,"alpha", false);

			alpha = robot->findBody(string2wstring(alpha_str));

			if(alpha)
			{
				std::stringstream alpha_ss = std::stringstream(deserializeString(rxController_xml,"alphaDisplacement"));
				double alpha_value = -1.0;
				while ((alpha_ss >> alpha_value))
				{
					alpha_displacement.expand(1,alpha_value);
				}
			}else{
				alpha_displacement = dVector(3,0.);
			}

			dVector beta_displacement;
			std::string beta_str = deserializeString(rxController_xml,"beta", false);
			beta = robot->findBody(string2wstring(beta_str));
			if(beta)
			{
				std::stringstream beta_ss = std::stringstream(deserializeString(rxController_xml,"betaDisplacement"));
				double beta_value = -1.0;	
				while((beta_ss >> beta_value))
				{
					beta_displacement.expand(1,beta_value);
				}
			}else{
				beta_displacement = dVector(3,0.);
			}
			std::stringstream index_ss = std::stringstream(deserializeString(rxController_xml,"index"));
			std::vector<long> index;
			long index_value;
			while((index_ss >> index_value))
			{
				index.push_back(index_value);
			}

			std::stringstream tc_ss = std::stringstream(deserializeString(rxController_xml,"taskConstraints"));
			std::vector<double> tc;
			double tc_value;
			while((tc_ss >> tc_value))
			{
				tc.push_back(tc_value);
			}

			double max_force = deserializeElement<double>(rxController_xml,"maxForce", 1.);
			std::string limit_body = deserializeString(rxController_xml,"limitBody");
			double distance_limit = deserializeElement<double>(rxController_xml,"distanceLimit", 0.);
			controller = new SubdisplacementController(robot,beta, beta_displacement, alpha, alpha_displacement,
				controller_duration, index, max_force, robot->findBody(string_type(string2wstring(limit_body))), distance_limit );
			if(index.size() == 1)
			{
				dynamic_cast<SubdisplacementController*>(controller)->setTaskConstraints(tc[0]);
			}else if (index.size() ==2)
			{
				dynamic_cast<SubdisplacementController*>(controller)->setTaskConstraints(tc[0], tc[1]);
			}
			dynamic_cast<SubdisplacementController*>(controller)->setImpedance(0.2,8.0,20.0);
		}
		break;
	case OBSTACLE_AVOIDANCE:
		{
			rxBody*   alpha = NULL;
			std::string alpha_str = deserializeString(rxController_xml,"alpha");
			alpha = robot->findBody(string2wstring(alpha_str));
			std::stringstream alpha_disp_ss = std::stringstream(deserializeString(rxController_xml,"alphaDisplacement"));
			dVector alpha_displacement;
			double alpha_value = -1.0;
			while((alpha_disp_ss >> alpha_value))
			{
				alpha_displacement.expand(1,alpha_value);
			}

			double distance_threshold = deserializeElement<double>(rxController_xml,"distanceThreshold", 1.);
			double deactivation_threshold = deserializeElement<double>(rxController_xml,"deactivationThreshold", 1.);
			if(CollisionInterface::instance)
			{
				controller = new ObstacleAvoidanceController(robot, alpha, alpha_displacement, distance_threshold, 
					CollisionInterface::instance, controller_duration, deactivation_threshold);
			}else{
				throw std::string("[XMLDeserializer::createFunctionalController] ERROR Obstacle Avoidance needs Collision interface.");
			}
		}
		break;
	default:
		throw std::string("[XMLDeserializer::createFunctionalController] ERROR: Unexpected Controller subgroup.");
		break;
	}
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
	mapping["ObstacleAvoidanceController"]					= ControllerType(rxController::eControlType_Functional, OBSTACLE_AVOIDANCE);
	mapping["JointBlackBoardController"]					= ControllerType(rxController::eControlType_Joint, BLACKBOARD_ACCESS);

	return mapping;
}