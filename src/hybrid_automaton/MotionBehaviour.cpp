#include "MotionBehaviour.h"
#include <iostream>
#include <string>

#include "XMLDeserializer.h"

#define NOT_IN_RT

using namespace std;
std::map<std::string, ControllerType> MotionBehaviour::controller_map_ = MotionBehaviour::createControllerMapping_();

MotionBehaviour::MotionBehaviour() :
Edge<Milestone>(NULL, NULL, -1)
, control_set_(NULL)
, robot_(NULL)
, time_(-1)
, dT_(-1)
, time_to_converge_(-1)
{
}

MotionBehaviour::MotionBehaviour(const Milestone * dad, const Milestone * son, rxSystem* robot, double weight, double dt ):
Edge<Milestone>(dad, son, weight)
, robot_(robot)
, time_(0)
, dT_(dt)
, time_to_converge_(-1)
{
	if(robot_)
	{
		control_set_ = new rxControlSet(robot_, dT_);
		control_set_->setGravity(0,0,-GRAV_ACC);
		control_set_->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(this->robot_));
		control_set_->nullMotionController()->setGain(0.02,0.0,0.01);
	}
	else
	{
		control_set_= NULL;
	}
}

MotionBehaviour::MotionBehaviour(TiXmlElement* motion_behaviour_xml, const Milestone * dad, const Milestone * son, rxSystem* robot):
Edge(dad, son),
robot_(robot),
time_to_converge_(0)
{
	XMLDeserializer xml_deserializer_(motion_behaviour_xml);
	this->weight = xml_deserializer_.deserializeDouble("Weight");
	//NOTE (Roberto): We ignore this value. The executing time of a new MotionBehaviour should be always 0.
	//this->time_ = xml_deserializer_.deserializeDouble("Time");
	this->time_ = 0;
	this->dT_ = xml_deserializer_.deserializeDouble("dT");

	std::string control_set_type = xml_deserializer_.deserializeString("ControlSetType");
	if(control_set_type==std::string("class rxControlSet *"))
	{
		if(robot_)
		{
			control_set_ = new rxControlSet(robot, dT_);
			control_set_->setGravity(0,0,-GRAV_ACC);
			control_set_->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(this->robot_));
			control_set_->nullMotionController()->setGain(0.02,0.0,0.01);
		}
		else
		{
			control_set_ = NULL;
		}
	}
	else
	{
		throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Unknown type of rxControlSet.");
	}	

	for(TiXmlElement* rxController_xml = motion_behaviour_xml->FirstChildElement("rxController"); rxController_xml; rxController_xml = rxController_xml->NextSiblingElement())
	{	
		this->addController_(rxController_xml);
	}
}

MotionBehaviour::MotionBehaviour(const MotionBehaviour & motion_behaviour_copy) :
Edge(motion_behaviour_copy),
robot_(motion_behaviour_copy.robot_),
time_(motion_behaviour_copy.time_),
dT_(motion_behaviour_copy.dT_),
time_to_converge_(motion_behaviour_copy.time_to_converge_)
{
	// NOTE (Roberto): Should we avoid copying the value of time_? We are creating a new MB, maybe time_ should be 0
	if(motion_behaviour_copy.control_set_)
	{
		this->control_set_ = new rxControlSet((*motion_behaviour_copy.control_set_));
	}
	else
	{
		this->control_set_ = NULL;
	}
}

MotionBehaviour::~MotionBehaviour() 
{
	// Do not delete any object that was not directly created in the constructor!!
	if(control_set_)
	{
		//TODO: Do not delete it! It was already deleted somehow -> Where?
		//delete control_set_;
		//control_set_ = NULL;
	}
}

void MotionBehaviour::addController_(TiXmlElement * rxController_xml)
{
	XMLDeserializer xml_deserializer_(rxController_xml);
	std::string controller_class_name = xml_deserializer_.deserializeString("className");
	ControllerType type_of_controller = controller_map_[controller_class_name];
	std::string controller_name = xml_deserializer_.deserializeString("name");
	int controller_dimension = xml_deserializer_.deserializeInteger("dim");
	double controller_duration = xml_deserializer_.deserializeDouble("dt");
	bool controller_activated_bool = xml_deserializer_.deserializeBoolean("activated");
	bool controller_ik_bool = xml_deserializer_.deserializeBoolean("ik");
	std::stringstream kp_vector_ss = std::stringstream(xml_deserializer_.deserializeString("kp"));
	std::stringstream kv_vector_ss = std::stringstream(xml_deserializer_.deserializeString("kv"));
	std::stringstream invL2sqr_vector_ss = std::stringstream(xml_deserializer_.deserializeString("invL2sqr"));

	dVector kp_vector;
	dVector kv_vector;
	dVector invL2sqr_vector;
	kp_vector.all(-1.0);
	for(int i =0; i<controller_dimension; i++)
	{
		double kp_value = -1.0;
		double kv_value = -1.0;
		double invL2sqr_value = -1.0;

		kp_vector_ss >> kp_value;
		kp_vector.expand(1,kp_value);

		kv_vector_ss >> kv_value;
		kv_vector.expand(1,kv_value);

		invL2sqr_vector_ss >> invL2sqr_value;
		invL2sqr_vector.expand(1,invL2sqr_value);
	}

	double ctrl_total_time = 0.0;
	std::vector<ViaPointBase*> via_points_ptr;
	int ctrl_num_via_points = xml_deserializer_.deserializeInteger("num_of_viaPoints");
	//if(ctrl_num_via_points){
	//	std::cout << "WARNING [MotionBehaviour::addController_(TiXmlElement * rxController_xml)]: The xml string of the controller contains a via point, but they are not allowed. The via point will be ignored." << std::endl;	
	//}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//for(TiXmlElement* via_point_xml = rxController_xml->FirstChildElement("ViaPoint"); via_point_xml; via_point_xml = via_point_xml->NextSiblingElement())
	//{
	//	XMLDeserializer xml_via_point_deserializer_(via_point_xml);
	//	ViaPointBase* via_point_ptr = xml_via_point_deserializer_.deserializeViaPoint(type_of_controller, controller_dimension);
	//	ctrl_total_time += via_point_ptr->time_;
	//	via_points_ptr.push_back(via_point_ptr);
	//}

	// Create the Controller
	rxController* controller = NULL;
	// HACK (Roberto): The total time to complete the motion between both nodes must be bigger than the sum of the 
	// partial timings between via points. BUT, HOW MUCH BIGGER?
	//ctrl_total_time += 1;	//1 seconds to complete the motion!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//if(time_to_converge_ < ctrl_total_time)
#ifdef NOT_IN_RT
	time_to_converge_ = 4;
#else
	time_to_converge_ = 15;
#endif

	switch(type_of_controller.first)
	{
	case JOINT_SPACE_CONTROLLER:
		controller = this->createJointController_(type_of_controller.second, controller_duration, via_points_ptr, rxController_xml);
		dynamic_cast<rxJointController*>(controller)->addPoint(child->getConfiguration(), time_to_converge_, true);
		break;
	case DISPLACEMENT_CONTROLLER:
		{
		controller = this->createDisplacementController_(type_of_controller.second, controller_duration, via_points_ptr, rxController_xml);
		dVector goal = child->getConfiguration();
		Vector3D goal_3D(goal[0],goal[1],goal[2]);
		dynamic_cast<rxDisplacementController*>(controller)->addPoint(goal_3D, time_to_converge_, true);
		break;
		}
	case ORIENTATION_CONTROLLER:
		{
		controller = this->createOrientationController_(type_of_controller.second, controller_duration, via_points_ptr, rxController_xml);
		dVector goal = child->getConfiguration();
		Rotation goal_rot(goal[3], goal[4], goal[5]);		//NOTE: Suposses that the orientation is defined with 
															// goal[3] = roll
															// goal[4] = pitch
															// goal[5] = yaw
		dynamic_cast<rxOrientationController*>(controller)->addPoint(goal_rot, time_to_converge_, true);
		break;
		}
	case H_TRANSFORM_CONTROLLER:
		{
		controller = this->createHTransformController_(type_of_controller.second, controller_duration, via_points_ptr, rxController_xml);
		dVector goal = child->getConfiguration();
		Vector3D goal_3D(goal[0],goal[1],goal[2]);
		Rotation goal_rot(goal[3], goal[4], goal[5]);		//NOTE: Suposses that the orientation is defined with 
															// goal[3] = roll
															// goal[4] = pitch
															// goal[5] = yaw
		dynamic_cast<rxHTransformController*>(controller)->addPoint(HTransform(goal_rot, goal_3D), time_to_converge_, true);
		break;
		}
	case QUASICOORD_CONTROLLER:
		controller = this->createQuasiCoordController_(type_of_controller.second, controller_duration);
		break;
	case NULL_MOTION_CONTROLLER:
		controller = this->createNullMotionController_(type_of_controller.second, controller_duration);
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::addController_(TiXmlElement * rxController_xml)] Unexpected Controller group.");
		break;
	}
	this->addController(controller);	// The controller is deactivated when added
	controller->setIKMode(controller_ik_bool);
	controller->setGain(kv_vector, kp_vector, invL2sqr_vector);	
	if(controller_activated_bool)
	{
		controller->activate();
	}
}

void MotionBehaviour::addController(rxController* ctrl) 
{
	if(ctrl){
		if(ctrl->dt() != this->dT_)
		{
			throw std::string("ERROR: [MotionBehaviour::addController(rxController* ctrl)] Time intervals of MotionBehaviour and all its controllers must be the same.");
		}
		ctrl->deactivate();
		control_set_->addController(ctrl);
	}
	else
	{
		throw std::string("ERROR: [MotionBehaviour::addController(rxController* ctrl)] The pointer to rxController to be added is NULL.");
	}				
}

void MotionBehaviour::activate() 
{
	if(control_set_)
	{
		std::list<rxController*> controllers = control_set_->getControllers();
		for(std::list< rxController* >::const_iterator controllers_it = controllers.begin();
			controllers_it != controllers.end() ; controllers_it ++)
		{
			if(*controllers_it)
			{
				//ControllerType type_of_controller = controller_map_[(*controllers_it)->type()];
				if(!((*controllers_it)->activated()))
				{
					//(*controllers_it)->_reset();
					(*controllers_it)->activate();
				}
				/*else{
				(*controllers_it)->deactivate();
				(*controllers_it)->activate();
				}*/
			}
		}
	}
}

void MotionBehaviour::deactivate() 
{
	if(control_set_)
	{
		control_set_->deactivateAllControllers();
	}
}

bool MotionBehaviour::hasConverged() 
{
	dVector error(this->control_set_->e()) ;

	//NOTE (Roberto) : How could we check if it has converged? There is no way to check automatically all the controllers through the control set.
	//We wait until the time to converge is exceeded and then we check if the error is small.
	if(time_ > time_to_converge_)
	{
		for(int i = 0; i < error.size(); ++i)
		{
			//HACK (George) : This is because I couldn't get the error between the current position and the desired position at the END of the trajectory
			if (::std::abs(error[i]) > 0.01 )
			{
#ifdef NOT_IN_RT
				std::cout << "Error in " << i << " is to large = " << ::std::abs(error[i]) << std::endl;
#endif
				return false;
			}
		}
		//std::cout << "CONVERGED!!!" << std::endl<< std::endl<< std::endl;
		return true;
	}
	return false;
}

dVector MotionBehaviour::update(double t)
{
	dVector torque;
	control_set_->compute(t,torque);
	time_ += dT_;
	return torque;
}

std::string MotionBehaviour::toStringXML() const
{
	TiXmlDocument document_xml;
	TiXmlElement * motion_behaviour_element = new TiXmlElement("MotionBehaviour");
	this->toElementXML(motion_behaviour_element);
	document_xml.LinkEndChild(motion_behaviour_element);

	// Declare a printer
	TiXmlPrinter printer_xml;
	// Attach it to the document you want to convert in to a std::string
	document_xml.Accept(&printer_xml);
	// Create a std::string and copy your document data in to the string
	std::string return_value = printer_xml.CStr();

	//TODO: Memory leaks????????????????
	return return_value;
}

void MotionBehaviour::toElementXML(TiXmlElement* motion_behaviour_xml) const
{
	motion_behaviour_xml->SetDoubleAttribute("Weight", weight);
	motion_behaviour_xml->SetDoubleAttribute("Time", time_);
	motion_behaviour_xml->SetDoubleAttribute("dT", dT_);
	motion_behaviour_xml->SetAttribute("ControlSetType", typeid(control_set_).name());

	std::list<rxController*> controllers = control_set_->getControllers();
	string_type controllers_to_string;
	for(std::list< rxController* >::const_iterator controllers_it = controllers.begin();
		controllers_it != controllers.end() ; controllers_it ++){
			string_type controller_to_string;
			(*controllers_it)->toString(controller_to_string);
			TiXmlElement * rxController_xml;
			rxController_xml = new TiXmlElement("rxController");
			motion_behaviour_xml->LinkEndChild(rxController_xml);
			this->RLabInfoString2ElementXML_(controller_to_string, rxController_xml);
	}
}

void MotionBehaviour::RLabInfoString2ElementXML_(string_type string_data, TiXmlElement* out_xml_element) const
{
#ifdef NOT_IN_RT
	std::wcout << string_data.c_str() << std::endl;
#endif
	std::wstringstream data_ss(string_data);
	string_type temp_st;
	std::getline(data_ss, temp_st);	
	out_xml_element->SetAttribute("className", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());

	ControllerType type_of_controller = controller_map_[wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))];

	std::getline(data_ss, temp_st);	
	out_xml_element->SetAttribute("name", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());

	std::getline(data_ss, temp_st);		// Discard field "system"
	std::getline(data_ss, temp_st);		// Discard field "type"
	std::getline(data_ss, temp_st);	
	out_xml_element->SetAttribute("dim", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
	int dimension_int = -1;
	std::wstringstream dimension_ss(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")));
	dimension_ss >> dimension_int;

	std::getline(data_ss, temp_st);	
	out_xml_element->SetAttribute("dt", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());

	std::getline(data_ss, temp_st);	
	out_xml_element->SetAttribute("activated", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());

	std::getline(data_ss, temp_st);	
	out_xml_element->SetAttribute("ik", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());

	string_type kp_st;
	string_type kv_st;
	string_type invL2sqr_st;
	for(int i=0; i<dimension_int ; i++)
	{
		std::getline(data_ss, temp_st);	
		kp_st += temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"));
		if(i!=dimension_int-1)
			kp_st += L" ";

		std::getline(data_ss, temp_st);	
		kv_st += temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"));
		if(i!=dimension_int-1)
			kv_st += L" ";

		std::getline(data_ss, temp_st);	
		invL2sqr_st += temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"));
		if(i!=dimension_int-1)
			invL2sqr_st += L" ";
	}
	out_xml_element->SetAttribute("kp",wstring2string_(kp_st).c_str());
	out_xml_element->SetAttribute("kv",wstring2string_(kv_st).c_str());
	out_xml_element->SetAttribute("invL2sqr",wstring2string_(invL2sqr_st).c_str());

	std::getline(data_ss, temp_st);	
	out_xml_element->SetAttribute("num_of_viaPoints", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
	int num_via_points_int = -1;
	std::wstringstream num_via_points_ss(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")));
	num_via_points_ss >> num_via_points_int;

	for(int i=0; i< num_via_points_int; i++)
	{
		TiXmlElement * via_point_xml;
		via_point_xml = new TiXmlElement("ViaPoint");
		out_xml_element->LinkEndChild(via_point_xml);

		std::getline(data_ss, temp_st);	
		via_point_xml->SetAttribute("time", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());

		std::getline(data_ss, temp_st);	
		via_point_xml->SetAttribute("type", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());

		std::getline(data_ss, temp_st);
		via_point_xml->SetAttribute("reuse", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());

		switch( type_of_controller.first ){
	case JOINT_SPACE_CONTROLLER:
		std::getline(data_ss, temp_st);	
		via_point_xml->SetAttribute("dVector", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
		break;
	case DISPLACEMENT_CONTROLLER:
		std::getline(data_ss, temp_st);	
		via_point_xml->SetAttribute("Vector3D", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
		break;
	case ORIENTATION_CONTROLLER:
		std::getline(data_ss, temp_st);	
		via_point_xml->SetAttribute("R", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
		break;
	case H_TRANSFORM_CONTROLLER:
		std::getline(data_ss, temp_st);	
		via_point_xml->SetAttribute("R", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
		std::getline(data_ss, temp_st);	
		via_point_xml->SetAttribute("r", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
		break;
	default:
		break;
		}
	}

	switch(type_of_controller.first)
	{
	case DISPLACEMENT_CONTROLLER:
		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("alpha", colon2space_( wstring2string_( temp_st.substr( temp_st.find(L"=") + 1, temp_st.find(L"\n") ) ) ).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("alphaDisplacement", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("beta", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("betaDisplacement",colon2space_( wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
		break;
	case ORIENTATION_CONTROLLER:
		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("alpha", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("alphaRotation", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("beta", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("betaRotation", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
		break;
	case H_TRANSFORM_CONTROLLER:
		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("alpha", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("alphaRotation", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );


		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("alphaDisplacement", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("beta", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("betaRotation", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("betaDisplacement", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
		break;
	default:
		break;
	}

	if(type_of_controller.second & WITH_COMPLIANCE)
	{
		std::string stiffness_b;
		std::string stiffness_k;
		for(int i = 0; i<dimension_int; i++)
		{
			stiffness_b += std::string("15");
			stiffness_k += std::string("100");
			if(i!=dimension_int -1)
			{
				stiffness_b += std::string(" ");
				stiffness_k += std::string(" ");
			}
		}
		out_xml_element->SetAttribute("stiffness_b", stiffness_b.c_str() );
		out_xml_element->SetAttribute("stiffness_k", stiffness_k.c_str() );
	}
}

MotionBehaviour* MotionBehaviour::clone() const 
{
	MotionBehaviour* newMotionBehaviour = new MotionBehaviour(*this);
	return newMotionBehaviour;
}

MotionBehaviour& MotionBehaviour::operator=(const MotionBehaviour & motion_behaviour_assignment)
{
	Edge::operator=(motion_behaviour_assignment);
	this->robot_ = motion_behaviour_assignment.robot_;
	this->time_ = motion_behaviour_assignment.time_;
	this->dT_ = motion_behaviour_assignment.dT_;

	// Delete the current stored control set
	if(this->control_set_)
	{
		delete control_set_;
		control_set_ = NULL;
	}

	// Copy the new control set
	if(motion_behaviour_assignment.control_set_)
	{
		this->control_set_ = new rxControlSet(*(motion_behaviour_assignment.control_set_));
		this->control_set_->setGravity(0,0,-GRAV_ACC);
		//this->control_set_->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(this->robot_));
	}
	return *this;
}

std::map<std::string, ControllerType> MotionBehaviour::createControllerMapping_()
{
	std::map<std::string, ControllerType> mapping;
	mapping[std::string("rxJointController")]							= ControllerType(JOINT_SPACE_CONTROLLER, 0);
	mapping[std::string("rxJointComplianceController")]					= ControllerType(JOINT_SPACE_CONTROLLER, WITH_COMPLIANCE);
	mapping[std::string("rxJointImpedanceController")]					= ControllerType(JOINT_SPACE_CONTROLLER, WITH_IMPEDANCE);
	mapping[std::string("rxInterpolatedJointController")]				= ControllerType(JOINT_SPACE_CONTROLLER, WITH_INTERPOLATION);
	mapping[std::string("rxInterpolatedJointComplianceController")]		= ControllerType(JOINT_SPACE_CONTROLLER, WITH_INTERPOLATION + WITH_COMPLIANCE);
	mapping[std::string("rxInterpolatedJointImpedanceController")]		= ControllerType(JOINT_SPACE_CONTROLLER, WITH_INTERPOLATION + WITH_IMPEDANCE);

	mapping[std::string("rxFunctionController")]						= ControllerType(FUNCTION_CONTROLLER, 0);
	mapping[std::string("rxFunctionComplianceController")]				= ControllerType(FUNCTION_CONTROLLER, WITH_COMPLIANCE);
	mapping[std::string("rxFunctionImpedanceController")]				= ControllerType(FUNCTION_CONTROLLER, WITH_IMPEDANCE);
	mapping[std::string("rxInterpolatedFunctionController")]			= ControllerType(FUNCTION_CONTROLLER, WITH_INTERPOLATION);
	mapping[std::string("rxInterpolatedFunctionComplianceController")]	= ControllerType(FUNCTION_CONTROLLER, WITH_INTERPOLATION + WITH_COMPLIANCE);
	mapping[std::string("rxInterpolatedFunctionImpedanceController")]	= ControllerType(FUNCTION_CONTROLLER, WITH_INTERPOLATION + WITH_IMPEDANCE);

	mapping[std::string("rxDisplacementFunctionController")]			= ControllerType(DISPLACEMENT_CONTROLLER, WITH_FUNCTION);
	mapping[std::string("rxDisplacementFunctionComplianceController")]	= ControllerType(DISPLACEMENT_CONTROLLER, WITH_FUNCTION + WITH_COMPLIANCE);
	mapping[std::string("rxDisplacementFunctionImpedanceController")]	= ControllerType(DISPLACEMENT_CONTROLLER, WITH_FUNCTION + WITH_IMPEDANCE);
	mapping[std::string("rxDisplacementController")]					= ControllerType(DISPLACEMENT_CONTROLLER, 0);
	mapping[std::string("rxDisplacementComplianceController")]			= ControllerType(DISPLACEMENT_CONTROLLER, WITH_COMPLIANCE);
	mapping[std::string("rxDisplacementImpedanceController")]			= ControllerType(DISPLACEMENT_CONTROLLER, WITH_IMPEDANCE);
	mapping[std::string("rxInterpolatedDisplacementController")]		= ControllerType(DISPLACEMENT_CONTROLLER, WITH_INTERPOLATION);
	mapping[std::string("rxInterpolatedDisplacementComplianceController")] = ControllerType(DISPLACEMENT_CONTROLLER, WITH_INTERPOLATION + WITH_COMPLIANCE);
	mapping[std::string("rxInterpolatedDisplacementImpedanceController")] = ControllerType(DISPLACEMENT_CONTROLLER, WITH_INTERPOLATION + WITH_IMPEDANCE);

	mapping[std::string("rxOrientationFunctionController")]				= ControllerType(ORIENTATION_CONTROLLER, WITH_FUNCTION);
	mapping[std::string("rxOrientationFunctionComplianceController")]	= ControllerType(ORIENTATION_CONTROLLER, WITH_FUNCTION + WITH_COMPLIANCE);
	mapping[std::string("rxOrientationFunctionImpedanceController")]	= ControllerType(ORIENTATION_CONTROLLER, WITH_FUNCTION + WITH_IMPEDANCE);
	mapping[std::string("rxOrientationController")]						= ControllerType(ORIENTATION_CONTROLLER, 0);
	mapping[std::string("rxOrientationComplianceController")]			= ControllerType(ORIENTATION_CONTROLLER, WITH_COMPLIANCE);
	mapping[std::string("rxOrientationImpedanceController")]			= ControllerType(ORIENTATION_CONTROLLER, WITH_IMPEDANCE);
	mapping[std::string("rxInterpolatedOrientationController")]			= ControllerType(ORIENTATION_CONTROLLER, WITH_INTERPOLATION);
	mapping[std::string("rxInterpolatedOrientationComplianceController")] = ControllerType(ORIENTATION_CONTROLLER, WITH_INTERPOLATION + WITH_COMPLIANCE);
	mapping[std::string("rxInterpolatedOrientationImpedanceController")] = ControllerType(ORIENTATION_CONTROLLER, WITH_INTERPOLATION + WITH_IMPEDANCE);

	mapping[std::string("rxHTransformFunctionController")]				= ControllerType(H_TRANSFORM_CONTROLLER, WITH_FUNCTION);
	mapping[std::string("rxHTransformFunctionComplianceController")]	= ControllerType(H_TRANSFORM_CONTROLLER, WITH_FUNCTION + WITH_COMPLIANCE);
	mapping[std::string("rxHTransformFunctionImpedanceController")]		= ControllerType(H_TRANSFORM_CONTROLLER, WITH_FUNCTION + WITH_IMPEDANCE);
	mapping[std::string("rxHTransformController")]						= ControllerType(H_TRANSFORM_CONTROLLER, 0);
	mapping[std::string("rxHTransformComplianceController")]			= ControllerType(H_TRANSFORM_CONTROLLER, WITH_COMPLIANCE);
	mapping[std::string("rxHTransformImpedanceController")]				= ControllerType(H_TRANSFORM_CONTROLLER, WITH_IMPEDANCE);
	mapping[std::string("rxInterpolatedHTransformController")]			= ControllerType(H_TRANSFORM_CONTROLLER, WITH_INTERPOLATION);
	mapping[std::string("rxInterpolatedHTransformComplianceController")] = ControllerType(H_TRANSFORM_CONTROLLER, WITH_INTERPOLATION + WITH_COMPLIANCE);
	mapping[std::string("rxInterpolatedHTransformImpedanceController")] = ControllerType(H_TRANSFORM_CONTROLLER, WITH_INTERPOLATION + WITH_IMPEDANCE);

	mapping[std::string("rxQuasiCoordController")]						= ControllerType(QUASICOORD_CONTROLLER, 0);
	mapping[std::string("rxQuasiCoordComplianceController")]			= ControllerType(QUASICOORD_CONTROLLER, WITH_COMPLIANCE);

	mapping[std::string("rxNullMotionController")]						= ControllerType(NULL_MOTION_CONTROLLER, 0);
	mapping[std::string("rxNullMotionComplianceController")]			= ControllerType(NULL_MOTION_CONTROLLER, WITH_COMPLIANCE);
	mapping[std::string("rxGradientNullMotionController")]				= ControllerType(NULL_MOTION_CONTROLLER, WITH_GRADIENT);

	return mapping;
}

std::string MotionBehaviour::wstring2string_(const std::wstring& wstr) const
{
	std::string str(wstr.length(),' ');
	copy(wstr.begin(),wstr.end(),str.begin());
	return str;
}

std::wstring MotionBehaviour::string2wstring_(const std::string& str) const
{
	std::wstring wstr(str.length(),L' ');
	copy(str.begin(),str.end(),wstr.begin());
	return wstr;
}

std::string MotionBehaviour::colon2space_(std::string text) const
{
	for(int i = 0; i < text.length(); i++)
	{
		if( text[i] == ',' )
			text[i] = ' ';
	}
	return text;
}

rxController* MotionBehaviour::createJointController_(int joint_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const
{
	rxController* controller = NULL;
	switch(joint_subgroup)
	{
	case 0:
		controller = new rxJointController(this->robot_,controller_duration);
		break;
	case WITH_COMPLIANCE:
		{
		controller = new rxJointComplianceController(this->robot_,controller_duration);	
		break;
		}
	case WITH_IMPEDANCE:
		controller = new rxJointImpedanceController(this->robot_,controller_duration);
		break;
	case WITH_INTERPOLATION:
		controller = new rxInterpolatedJointController(this->robot_,controller_duration);
		break;
	case WITH_INTERPOLATION + WITH_COMPLIANCE:
		{
		controller = new rxInterpolatedJointComplianceController(this->robot_,controller_duration);
		break;
		}
	case WITH_INTERPOLATION + WITH_IMPEDANCE:
		controller = new rxInterpolatedJointImpedanceController(this->robot_,controller_duration);
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::createJointController_(ControllerSubgroup joint_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr)] Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointdVector* vpdv_ptr = (ViaPointdVector*)((*vp_it));
		dynamic_cast<rxJointController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}

	if(joint_subgroup & WITH_COMPLIANCE)
	{
		XMLDeserializer xml_deserializer_(rxController_xml);
		std::stringstream stiffness_b_ss = std::stringstream(xml_deserializer_.deserializeString("stiffness_b"));
		std::stringstream stiffness_k_ss = std::stringstream(xml_deserializer_.deserializeString("stiffness_k"));
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

	return controller;
}

rxController* MotionBehaviour::createDisplacementController_(int displacement_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const
{
	rxBody*   alpha = NULL;
	rxBody*   beta = NULL;
	rxController* controller = NULL;
	XMLDeserializer xml_deserializer_(rxController_xml);
	std::string alpha_str = xml_deserializer_.deserializeString("alpha");
	alpha = robot_->findBody(string2wstring_(alpha_str));
	std::stringstream alpha_ss = std::stringstream(xml_deserializer_.deserializeString("alphaDisplacement"));
	double alpha_value = -1.0;
	dVector alpha_displacement;
	while ((alpha_ss >> alpha_value))
	{
		alpha_displacement.expand(1,alpha_value);
	}

	std::string beta_str = xml_deserializer_.deserializeString("beta");
	beta = robot_->findBody(string2wstring_(beta_str));
	std::stringstream beta_ss = std::stringstream(xml_deserializer_.deserializeString("betaDisplacement"));
	double beta_value = -1.0;
	dVector beta_displacement;
	while((beta_ss >> beta_value))
	{
		beta_displacement.expand(1,beta_value);
	}

	switch(displacement_subgroup)
	{
	case 0:
		controller = new rxDisplacementController(this->robot_, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration);
		break;
	case WITH_COMPLIANCE:
		{
		controller = new rxDisplacementComplianceController(this->robot_, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
		break;
		}
	case WITH_IMPEDANCE:
		controller = new rxDisplacementImpedanceController(this->robot_, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
		break;
	case WITH_INTERPOLATION:
		controller = new rxInterpolatedDisplacementController(this->robot_, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
		break;
	case WITH_INTERPOLATION + WITH_COMPLIANCE:
		{
		controller = new rxInterpolatedDisplacementComplianceController(this->robot_, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
		break;
		}
	case WITH_INTERPOLATION + WITH_IMPEDANCE:
		controller = new rxInterpolatedDisplacementImpedanceController(this->robot_, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::createDisplacementController_(ControllerSubgroup displacement_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml)] Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointVector3D* vpdv_ptr = (ViaPointVector3D*)((*vp_it));
		dynamic_cast<rxDisplacementController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}

	if(displacement_subgroup & WITH_COMPLIANCE)
	{
		XMLDeserializer xml_deserializer_(rxController_xml);
		std::stringstream stiffness_b_ss = std::stringstream(xml_deserializer_.deserializeString("stiffness_b"));
		std::stringstream stiffness_k_ss = std::stringstream(xml_deserializer_.deserializeString("stiffness_k"));
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

rxController* MotionBehaviour::createOrientationController_(int orientation_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const
{
	rxBody*   alpha = NULL;
	rxBody*   beta = NULL;
	rxController* controller = NULL;
	XMLDeserializer xml_deserializer_(rxController_xml);
	std::string alpha_str = xml_deserializer_.deserializeString("alpha");
	alpha = robot_->findBody(string2wstring_(alpha_str));
	std::stringstream alpha_ss = std::stringstream(xml_deserializer_.deserializeString("alphaRotation"));
	double alpha_value = -1.0;
	dVector alpha_rotation;
	while((alpha_ss >> alpha_value))
	{
		alpha_rotation.expand(1,alpha_value);
	}
	std::string beta_str = xml_deserializer_.deserializeString("beta");
	beta = robot_->findBody(string2wstring_(beta_str));
	std::stringstream beta_ss = std::stringstream(xml_deserializer_.deserializeString("betaRotation"));
	double beta_value = -1.0;
	dVector beta_rotation;
	while((beta_ss >> beta_value))
	{
		beta_rotation.expand(1,beta_value);
	}

	switch(orientation_subgroup)
	{
	case 0:
		controller = new rxOrientationController(this->robot_, beta, Rotation(beta_rotation), alpha, Rotation(alpha_rotation), controller_duration );
		break;
	case WITH_COMPLIANCE:
		{
		controller = new rxOrientationComplianceController(this->robot_, beta, Rotation(beta_rotation), alpha, Rotation(alpha_rotation), controller_duration );
		break;
		}
	case WITH_IMPEDANCE:
		controller = new rxOrientationImpedanceController(this->robot_, beta, Rotation(beta_rotation), alpha, Rotation(alpha_rotation), controller_duration );
		break;
	case WITH_INTERPOLATION:
		controller = new rxInterpolatedOrientationController(this->robot_, beta, Rotation(beta_rotation), alpha, Rotation(alpha_rotation), controller_duration );
		break;
	case WITH_INTERPOLATION + WITH_COMPLIANCE:
		{
		controller = new rxInterpolatedOrientationComplianceController(this->robot_, beta, Rotation(beta_rotation), alpha, Rotation(alpha_rotation), controller_duration );
		break;
		}
	case WITH_INTERPOLATION + WITH_IMPEDANCE:
		controller = new rxInterpolatedOrientationImpedanceController(this->robot_, beta, Rotation(beta_rotation), alpha, Rotation(alpha_rotation), controller_duration );
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::createOrientationController_(ControllerSubgroup orientation_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml)] Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointRotation* vpdv_ptr = (ViaPointRotation*)((*vp_it));
		dynamic_cast<rxOrientationController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}
	if(orientation_subgroup & WITH_COMPLIANCE)
	{
		XMLDeserializer xml_deserializer_(rxController_xml);
		std::stringstream stiffness_b_ss = std::stringstream(xml_deserializer_.deserializeString("stiffness_b"));
		std::stringstream stiffness_k_ss = std::stringstream(xml_deserializer_.deserializeString("stiffness_k"));
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

rxController* MotionBehaviour::createHTransformController_(int htransform_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const
{
	rxBody*   alpha = NULL;
	rxBody*   beta = NULL;
	rxController* controller = NULL;
	XMLDeserializer xml_deserializer_(rxController_xml);
	std::string alpha_str = xml_deserializer_.deserializeString("alpha");
	alpha = robot_->findBody(string2wstring_(alpha_str));
	std::stringstream alpha_rot_ss = std::stringstream(xml_deserializer_.deserializeString("alphaRotation"));
	double alpha_value = -1.0;
	dVector alpha_rotation;
	while((alpha_rot_ss >> alpha_value))
	{
		alpha_rotation.expand(1,alpha_value);
	}
	std::stringstream alpha_disp_ss = std::stringstream(xml_deserializer_.deserializeString("alphaDisplacement"));
	dVector alpha_displacement;
	while((alpha_disp_ss >> alpha_value))
	{
		alpha_displacement.expand(1,alpha_value);
	}

	std::string beta_str = xml_deserializer_.deserializeString("beta");
	beta = robot_->findBody(string2wstring_(beta_str));
	std::stringstream beta_rot_ss = std::stringstream(xml_deserializer_.deserializeString("betaRotation"));
	double beta_value = -1.0;
	dVector beta_rotation;
	while((beta_rot_ss >> beta_value))
	{
		beta_rotation.expand(1,beta_value);
	}			
	std::stringstream beta_disp_ss = std::stringstream(xml_deserializer_.deserializeString("betaDisplacement"));
	dVector beta_displacement;
	while((beta_disp_ss >> beta_value))
	{
		beta_displacement.expand(1,beta_value);
	}

	switch(htransform_subgroup)
	{
	case 0:
		controller = new rxHTransformController(this->robot_, beta, HTransform(beta_rotation, beta_displacement), alpha, HTransform(alpha_rotation, alpha_displacement), controller_duration );
		break;
	case WITH_COMPLIANCE:
		controller = new rxHTransformComplianceController(this->robot_, beta, HTransform(beta_rotation, beta_displacement), alpha, HTransform(alpha_rotation, alpha_displacement), controller_duration );
		break;
	case WITH_IMPEDANCE:
		controller = new rxHTransformImpedanceController(this->robot_, beta, HTransform(beta_rotation, beta_displacement), alpha, HTransform(alpha_rotation, alpha_displacement), controller_duration );
		break;
	case WITH_INTERPOLATION:
		controller = new rxInterpolatedHTransformController(this->robot_, beta, HTransform(beta_rotation, beta_displacement), alpha, HTransform(alpha_rotation, alpha_displacement), controller_duration );
		break;
	case WITH_INTERPOLATION + WITH_COMPLIANCE:
		controller = new rxInterpolatedHTransformComplianceController(this->robot_, beta, HTransform(beta_rotation, beta_displacement), alpha, HTransform(alpha_rotation, alpha_displacement), controller_duration );
		break;
	case WITH_INTERPOLATION + WITH_IMPEDANCE:
		controller = new rxInterpolatedHTransformImpedanceController(this->robot_, beta, HTransform(beta_rotation, beta_displacement), alpha, HTransform(alpha_rotation, alpha_displacement), controller_duration );
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::createHTransformController_(ControllerSubgroup htransform_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml)] Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointHTransform* vpdv_ptr = (ViaPointHTransform*)((*vp_it));
		dynamic_cast<rxHTransformController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}

	if(htransform_subgroup & WITH_COMPLIANCE)
	{
		XMLDeserializer xml_deserializer_(rxController_xml);
		std::stringstream stiffness_b_ss = std::stringstream(xml_deserializer_.deserializeString("stiffness_b"));
		std::stringstream stiffness_k_ss = std::stringstream(xml_deserializer_.deserializeString("stiffness_k"));
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

rxController* MotionBehaviour::createQuasiCoordController_(int quasi_coord_subgroup, double controller_duration) const
{
	rxController* controller = NULL;
	switch(quasi_coord_subgroup)
	{
	case 0:
	//	controller = new rxQuasiCoordController(this->robot_, , controller_duration);
		break;
	case WITH_COMPLIANCE:
	//	controller = new rxQuasiCoordComplianceController(this->robot_, ,controller_duration);
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::createQuasiCoordController_(ControllerSubgroup quasi_coord_subgroup, double controller_duration)] Unexpected Controller subgroup.");
		break;
	}
	return controller;
}

rxController* MotionBehaviour::createNullMotionController_(int null_motion_subgroup, double controller_duration) const
{
	rxController* controller = NULL;
	switch(null_motion_subgroup)
	{
	case 0:
		controller = new rxNullMotionController(this->robot_, controller_duration);
		break;
	case WITH_COMPLIANCE:
		controller = new rxNullMotionComplianceController(this->robot_,  controller_duration);
		break;
	case	WITH_GRADIENT:
	//	controller = new rxGradientNullMotionController(this->robot_, controller_duration);
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::createNullMotionController_(ControllerSubgroup null_motion_subgroup, double controller_duration)] Unexpected Controller subgroup.");
		break;
	}
	return controller;
}