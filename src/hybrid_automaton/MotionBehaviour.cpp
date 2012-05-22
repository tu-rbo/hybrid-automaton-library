#include "MotionBehaviour.h"
#include <iostream>
#include <string>

#include "XMLDeserializer.h"

#include "FeatureAttractorController.h"
#include "SubdisplacementController.h"
#include "ObstacleAvoidanceController.h"

//#define NOT_IN_RT

using namespace std;
std::map<std::string, ControllerType> MotionBehaviour::controller_map_ = MotionBehaviour::createControllerMapping_();

MotionBehaviour::MotionBehaviour() :
Edge<Milestone>(NULL, NULL, -1)
, control_set_(NULL)
, robot_(NULL)
, time_(-1)
, dT_(-1)
, max_velocity_(-1)
, min_time_(-1)
, time_to_converge_(0.0)
{
}

MotionBehaviour::MotionBehaviour(const Milestone * dad, const Milestone * son, rxSystem* robot, double weight, double dt ):
Edge<Milestone>(dad, son, weight)
, robot_(robot)
, time_(0)
, dT_(dt)
, max_velocity_(-1)
, min_time_(-1)
, time_to_converge_(0.0)
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

MotionBehaviour::MotionBehaviour(const Milestone * dad, const Milestone * son, rxControlSet* control_set, double weight ):
Edge<Milestone>(dad, son, weight)
, time_(0)
, max_velocity_(-1)
, min_time_(-1)
, time_to_converge_(0.0)
, control_set_(control_set)
{
	if(control_set_)
	{
		dT_ = control_set->dt();
		robot_ = const_cast<rxSystem*>(control_set->sys());

		// TODO: should this be done by the caller?
		control_set_->setGravity(0,0,-GRAV_ACC);
		control_set_->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(this->robot_));
		control_set_->nullMotionController()->setGain(0.02,0.0,0.01);
	}
	else
	{
		dT_ = 0.002;
		robot_ = NULL;
	}
}


MotionBehaviour::MotionBehaviour(TiXmlElement* motion_behaviour_xml, const Milestone * dad, const Milestone * son, rxSystem* robot, double dt ): Edge(dad, son)
, robot_(robot)
, dT_(dt)
, max_velocity_(-1)
, min_time_(-1)
, time_to_converge_(0.0)
{
	XMLDeserializer xml_deserializer_(motion_behaviour_xml);
	TiXmlElement* control_set_element = motion_behaviour_xml->FirstChildElement("ControlSet");
	if(control_set_element == NULL)
	{
		throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] ControlSet element was not found.");
	}

	std::string control_set_type = std::string(control_set_element->Attribute("type"));
	if(control_set_type==std::string("rxControlSet"))
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
	else if(control_set_type==std::string("rxTPOperationalSpaceControlSet"))
	{
		if(robot_)
		{
			control_set_ = new rxTPOperationalSpaceControlSet(robot, dT_);
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
		std::cout << control_set_type << std::endl;
		throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Unknown type of rxControlSet.");
	}	

	for(TiXmlElement* rxController_xml = control_set_element->FirstChildElement("Controller"); rxController_xml; rxController_xml = rxController_xml->NextSiblingElement())
	{	
		this->addController_(rxController_xml);
	}
}

MotionBehaviour::MotionBehaviour(const MotionBehaviour & motion_behaviour_copy) :
Edge(motion_behaviour_copy),
robot_(motion_behaviour_copy.robot_),
time_(motion_behaviour_copy.time_),
dT_(motion_behaviour_copy.dT_),
min_time_(motion_behaviour_copy.min_time_),
max_velocity_(motion_behaviour_copy.max_velocity_),
goal_controllers_(motion_behaviour_copy.goal_controllers_),
time_to_converge_(motion_behaviour_copy.time_to_converge_)
{
	// NOTE (Roberto): Should we avoid copying the value of time_? We are creating a new MB, maybe time_ should be 0
	if(motion_behaviour_copy.control_set_)
	{
		// NOTE (Roberto): Two ugly options:
		// Use typeid(control_set_).name() as we do when serializing
		// Drawback: typeid has no standard behaviour
		// Use dynamic_cast to different types and check if the resulting pointer is NULL (not possible to cast->
		// it is not of this type) or not NULL (possible to cast -> of this type)
		// Drawback: we can always cast to base classes, we should check only if we can to derived ones
		// Implemented: mixture of both :(
		// CHANGE: Use dynamic_cast tree, because now control_set_ is a rxControlSetBase*, and that is what we 
		// get if we use typeid
		//
		//std::string control_set_name = std::string(typeid(motion_behaviour_copy.control_set_).name());
		//std::string control_set_name2 = control_set_name.substr(6);		//Ignore the 'class ' part
		//control_set_name2.resize(control_set_name2.size()-2);	//Ignore the ' *' part
		//if(control_set_name2 == std::string("rxControlSet"))
		//{
		//	this->control_set_ = new rxControlSet((*dynamic_cast<rxControlSet*>(motion_behaviour_copy.control_set_)));
		//}
		//else if(control_set_name2==std::string("rxTPOperationalSpaceControlSet"))
		//{
		//	this->control_set_ = new rxTPOperationalSpaceControlSet((*dynamic_cast<rxTPOperationalSpaceControlSet*>(motion_behaviour_copy.control_set_)));
		//}
		//else // Use the base class if there is something wrong
		//{
		//	this->control_set_ = new rxControlSet((*dynamic_cast<rxControlSet*>(motion_behaviour_copy.control_set_)));
		//}
		if(dynamic_cast<rxTPOperationalSpaceControlSet*>(motion_behaviour_copy.control_set_))
		{
			this->control_set_ = new rxTPOperationalSpaceControlSet((*dynamic_cast<rxTPOperationalSpaceControlSet*>(motion_behaviour_copy.control_set_)));
		}
		else if(dynamic_cast<rxControlSet*>(motion_behaviour_copy.control_set_))
		{
			this->control_set_ = new rxControlSet((*dynamic_cast<rxControlSet*>(motion_behaviour_copy.control_set_)));
		}
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
	std::string controller_class_name = xml_deserializer_.deserializeString("type");
	ControllerType type_of_controller = controller_map_[controller_class_name];
	bool controller_ik_bool = xml_deserializer_.deserializeBoolean("ik");
	std::stringstream kp_vector_ss = std::stringstream(xml_deserializer_.deserializeString("kp"));
	std::stringstream kv_vector_ss = std::stringstream(xml_deserializer_.deserializeString("kv"));
	std::stringstream invL2sqr_vector_ss = std::stringstream(xml_deserializer_.deserializeString("invL2sqr"));
	bool is_goal_controller = xml_deserializer_.deserializeBoolean("goalController", true);
	dVector controller_goal;
	double controller_goal_value;
	if(!is_goal_controller)
	{
		std::stringstream controller_goal_ss = std::stringstream(xml_deserializer_.deserializeString("controllerGoal"));
		while(controller_goal_ss >> controller_goal_value)
		{	
			controller_goal.expand(1,controller_goal_value);		
		}
	}
	int priority = xml_deserializer_.deserializeInteger("priority", 1);

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
		controller = this->createJointController_(type_of_controller.second, this->dT_, via_points_ptr, rxController_xml);
		break;
	case rxController::eControlType_Displacement:
		controller = this->createDisplacementController_(type_of_controller.second, this->dT_, via_points_ptr, rxController_xml);
		break;
	case rxController::eControlType_Orientation:
		controller = this->createOrientationController_(type_of_controller.second, this->dT_, via_points_ptr, rxController_xml);
		break;
	case rxController::eControlType_HTransform:
		controller = this->createHTransformController_(type_of_controller.second, this->dT_, via_points_ptr, rxController_xml);
		break;
	case rxController::eControlType_QuasiCoord:
		controller = this->createQuasiCoordController_(type_of_controller.second, this->dT_);
		break;
	case rxController::eControlType_NullMotion:
		controller = this->createNullMotionController_(type_of_controller.second, this->dT_);
		break;
	case rxController::eControlType_Functional:
		{
			int dimension = xml_deserializer_.deserializeInteger("dimension", 0);
			controller = this->createFunctionalController_(type_of_controller.second, dimension, this->dT_, rxController_xml);
			break;
		}
	default:
		throw std::string("ERROR: [MotionBehaviour::addController_(TiXmlElement * rxController_xml)] Unexpected Controller group.");
		break;
	}

	// we need unique names for all controllers in a single control set, otherwise RLAB will complain
	std::wstringstream wss;
	wss << "controller_" << control_set_->getControllers().size();
	controller->setName(wss.str());

	switch(controller->type()) {
		case rxController::eControlType_Joint:
			{
				dynamic_cast<rxJointController*>(controller)->addPoint(controller_goal, time_, false, eInterpolatorType_Cubic);
				break;
			}
		case rxController::eControlType_Displacement:
			{
				dynamic_cast<rxDisplacementController*>(controller)->addPoint(controller_goal, time_, false, eInterpolatorType_Cubic);
				break;
			}
		case rxController::eControlType_Orientation:
			{
				dynamic_cast<rxOrientationController*>(controller)->addPoint(controller_goal, time_, false);
				break;
			}
		case rxController::eControlType_HTransform:
			{
				Vector3D goal_3D(controller_goal[0],controller_goal[1],controller_goal[2]);
				Rotation goal_rot(controller_goal[3], controller_goal[4], controller_goal[5]);		//NOTE: Suposses that the orientation is defined with 
				dynamic_cast<rxHTransformController*>(controller)->addPoint(HTransform(goal_rot, goal_3D), time_, false);
				break;
			}
	}

	controller->setPriority(priority);
	this->addController(controller, is_goal_controller);	// The controller is deactivated when added
	controller->setIKMode(controller_ik_bool);
	controller->setGain(kv_vector, kp_vector, invL2sqr_vector);	
}

void MotionBehaviour::addController(rxController* ctrl, bool is_goal_controller) 
{
	if(ctrl){
		if(ctrl->dt() != this->dT_)
		{
			throw std::string("ERROR: [MotionBehaviour::addController(rxController* ctrl)] Time intervals of MotionBehaviour and all its controllers must be the same.");
		}
		ctrl->deactivate();
		goal_controllers_[ctrl->name()] = is_goal_controller;
		control_set_->addController(ctrl, ctrl->name(), ctrl->priority());
	}
	else
	{
		throw std::string("ERROR: [MotionBehaviour::addController(rxController* ctrl)] The pointer to rxController to be added is NULL.");
	}				
}

void MotionBehaviour::activate() 
{
	if (control_set_)
	{
		std::list<rxController*> controllers = control_set_->getControllers();
		for(std::list<rxController*>::const_iterator it = controllers.begin(); it != controllers.end(); ++it)
		{
			if (*it)
			{
				if (((*it)->activated()))
				{
					(*it)->deactivate();
				}

				(*it)->activate();

				// add the goal of the child milestone
				switch((*it)->type()) {
					case rxController::eControlType_Joint:
						{
							double default_max_velocity = 0.30;// in rad/s
							double default_min_time = 5.;
							double time = min_time_;

							// q1 and q2 to interpolate between
							dVector current_q = robot_->q();//parent->getConfiguration();
							dVector desired_q = child->getConfiguration();

							// qd1 and qd2
							dVector current_qd(current_q.size());
							current_qd = robot_->qdot();
							dVector desired_qd(current_q.size());
							desired_qd.zero();

							if (max_velocity_ > 0 || min_time_ <= 0 ) {
								double velocity = max_velocity_;
								if (max_velocity_ <= 0) {
									velocity = default_max_velocity;
									time = default_min_time;
								}
								for (unsigned int i = 0; i < current_q.size(); i++) {
									time = max( fabs((6 * desired_q[i] - 6 * current_q[i]) / (current_qd[i] + desired_qd[i] + 4 * velocity)), time);
								}
							}

							std::cout << "time of trajectory: " << time << std::endl;
							time_to_converge_=time;
							dynamic_cast<rxJointController*>(*it)->addPoint(desired_q, time, false, eInterpolatorType_Cubic);
							break;
						}
					case rxController::eControlType_Displacement:
						{
							double default_max_velocity = 0.20;// in m/s
							double default_min_time = 5.;
							double time = min_time_;

							// q1 and q2 to interpolate between
							std::cout << robot_ << std::endl;
							std::cout << robot_->findBody(_T("EE")) << std::endl;
							dVector current_r = robot_->findBody(_T("EE"))->T().r; //parent->getConfiguration();
							dVector desired_r = child->getConfiguration();

							// qd1 and qd2
							dVector current_rd(current_r.size());
							current_rd.zero();
							dVector desired_rd(current_r.size());
							desired_rd.zero();

							if (max_velocity_ > 0 || min_time_ <= 0 ) {
								double velocity = max_velocity_;
								if (max_velocity_ <= 0) {
									velocity = default_max_velocity;
									time = default_min_time;
								}
								for (unsigned int i = 0; i < current_r.size(); i++) {
									time = max( fabs((6 * desired_r[i] - 6 * current_r[i]) / (current_rd[i] + desired_rd[i] + 4 * velocity)), time);
								}
							}

							std::cout << "time of trajectory: " << time << std::endl;
							time_to_converge_=time;
							dynamic_cast<rxDisplacementController*>(*it)->addPoint(desired_r, time, false, eInterpolatorType_Cubic);
							break;
						}
					case rxController::eControlType_Orientation:
						{
							double time = (min_time_ > 0) ? min_time_ : 10.0;
							dVector goal = child->getConfiguration();
							Rotation goal_rot(goal[3], goal[4], goal[5]);		
							//NOTE: Suposses that the orientation is defined with 
							// goal[3] = roll
							// goal[4] = pitch
							// goal[5] = yaw
							dynamic_cast<rxOrientationController*>(*it)->addPoint(goal_rot, time, false);
							break;
						}
					case rxController::eControlType_HTransform:
						{
							double time = (min_time_ > 0) ? min_time_ : 10.0;
							dVector goal = child->getConfiguration();
							Vector3D goal_3D(goal[0],goal[1],goal[2]);
							Rotation goal_rot(goal[3], goal[4], goal[5]);		//NOTE: Suposses that the orientation is defined with 
							// goal[3] = roll
							// goal[4] = pitch
							// goal[5] = yaw
							dynamic_cast<rxHTransformController*>(*it)->addPoint(HTransform(goal_rot, goal_3D), time, false);
							break;
						}
				}
			}
		}
	}
}

void MotionBehaviour::deactivate()
{
	if (control_set_)
	{
		control_set_->deactivateAllControllers();
	}
}

bool MotionBehaviour::hasConverged() 
{
	//NOTE (Roberto) : How could we check if it has converged? There is no way to check automatically all the controllers through the control set.
	//We wait until the time to converge is exceeded and then we check if the error is small.
	if(time_ > time_to_converge_)
	{

		//change this!!!!!!!!!!!!!!!!
		dVector error = this->control_set_->e();



		//this->robot_->findBody(_T("EE"))->T().r
		//std::list<rxController*> controllers = this->control_set_->getControllers();
		//std::list<rxController*>::iterator it = controllers.begin();
		//dynamic_cast<rxInterpolatedDisplacementComplianceController*>(*it)->
		//dynamic_cast<rxInterpolatedDisplacementComplianceController*>(*it)->r_beta_target()

		//controllers.front()

		for(int i = 0; i < error.size(); ++i)
		{
			//HACK (George) : This is because I couldn't get the error between the current position and the desired position at the END of the trajectory
			if (::std::abs(error[i]) > 0.1 )
			{
				//#ifdef NOT_IN_RT
				std::cout << "Error in " << i << " is to large = " << ::std::abs(error[i]) << std::endl;
				//#endif
				return false;
			}
		}
		//std::cout << "CONVERGED!!!" << std::endl;
		return true;
	}
	return false;
}

dVector MotionBehaviour::getError() const
{
	return control_set_->e();
}

dVector MotionBehaviour::getDesired() const
{
	dVector desired;
	std::list<rxController*> controllers = control_set_->getControllers();
	for(std::list<rxController*>::const_iterator it = controllers.begin(); it != controllers.end(); ++it)
	{
		if (*it)
		{
			if ((*it)->type() == rxController::eControlType_Joint)
			{
				dVector qd = dynamic_cast<rxJointController*>(*it)->qd();
				desired.expand(qd.size(), qd);
			}
		}
	}
	return desired;
}

dVector MotionBehaviour::getDesiredDot() const
{
	dVector desired_dot;
	std::list<rxController*> controllers = control_set_->getControllers();
	for(std::list<rxController*>::const_iterator it = controllers.begin(); it != controllers.end(); ++it)
	{
		if (*it)
		{
			if ((*it)->type() == rxController::eControlType_Joint)
			{
				dVector qdotd = dynamic_cast<rxJointController*>(*it)->qdotd();
				desired_dot.expand(qdotd.size(), qdotd);
			}
		}
	}
	return desired_dot;
}

dVector MotionBehaviour::getErrorDot() const
{
	dVector error_dot;
	std::list<rxController*> controllers = control_set_->getControllers();
	for(std::list<rxController*>::const_iterator it = controllers.begin(); it != controllers.end(); ++it)
	{
		if (*it)
		{
			if ((*it)->type() == rxController::eControlType_Joint)
			{
				dVector edot = dynamic_cast<rxJointController*>(*it)->edot();
				error_dot.expand(edot.size(), edot);
			}
		}
	}
	return error_dot;
}

dVector MotionBehaviour::getCurrentDotReference() const
{
	dVector current_dot_ref;
	std::list<rxController*> controllers = control_set_->getControllers();
	for(std::list<rxController*>::const_iterator it = controllers.begin(); it != controllers.end(); ++it)
	{
		if (*it)
		{
			if ((*it)->type() == rxController::eControlType_Joint)
			{
				dVector qdot_ref = dynamic_cast<rxJointController*>(*it)->qdot_ref();
				current_dot_ref.expand(qdot_ref.size(), qdot_ref);
			}
		}
	}
	return current_dot_ref;
}

dVector MotionBehaviour::getCurrentDotDotReference() const
{
	dVector current_dotdot_ref;
	std::list<rxController*> controllers = control_set_->getControllers();
	for(std::list<rxController*>::const_iterator it = controllers.begin(); it != controllers.end(); ++it)
	{
		if (*it)
		{
			if ((*it)->type() == rxController::eControlType_Joint)
			{
				dVector qddot_ref = dynamic_cast<rxJointController*>(*it)->qddot_ref();
				current_dotdot_ref.expand(qddot_ref.size(), qddot_ref);
			}
		}
	}
	return current_dotdot_ref;
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
	TiXmlElement * mb_element = this->toElementXML();	
	document_xml.LinkEndChild(mb_element);

	// Declare a printer
	TiXmlPrinter printer_xml;
	// Attach it to the document you want to convert in to a std::string
	document_xml.Accept(&printer_xml);
	// Create a std::string and copy your document data in to the string
	std::string return_value = printer_xml.CStr();

	//TODO: Memory leaks????????????????
	//delete mb_element;
	return return_value;
}

TiXmlElement* MotionBehaviour::toElementXML() const
{
	TiXmlElement * mb_element = new TiXmlElement("MotionBehaviour");
	TiXmlElement * control_set_element = new TiXmlElement("ControlSet");
	mb_element->LinkEndChild(control_set_element);
	// NOTE: This doesn't work anymore since control_set_ is now rxControlSetBase*
	//std::string control_set_name = std::string(typeid(control_set_).name());
	//std::string control_set_name2 = control_set_name.substr(6);		//Ignore the 'class ' part
	//control_set_name2.resize(control_set_name2.size()-2);	//Ignore the ' *' part
	//control_set_element->SetAttribute("type", control_set_name2.c_str());

	if(dynamic_cast<rxTPOperationalSpaceControlSet*>(control_set_))
	{
		control_set_element->SetAttribute("type", "rxTPOperationalSpaceControlSet");
	}
	else if(dynamic_cast<rxControlSet*>(control_set_))
	{
		control_set_element->SetAttribute("type", "rxControlSet");
	}

	std::list<rxController*> controllers = control_set_->getControllers();
	string_type controllers_to_string;
	for(std::list< rxController* >::const_iterator controllers_it = controllers.begin();
		controllers_it != controllers.end() ; controllers_it ++){
			string_type controller_to_string;
			(*controllers_it)->toString(controller_to_string);
			TiXmlElement * rxController_xml = new TiXmlElement("Controller");
			control_set_element->LinkEndChild(rxController_xml);
			this->RLabInfoString2ElementXML_(controller_to_string, rxController_xml);
			rxController_xml->SetAttribute("goalController", (goal_controllers_.find((*controllers_it)->name())->second ? "true" : "false"));
			dVector controller_goal;
	double controller_goal_value;
	if(!is_goal_controller)
	{
		std::stringstream controller_goal_ss = std::stringstream(xml_deserializer_.deserializeString("controllerGoal"));
		while(controller_goal_ss >> controller_goal_value)
		{	
			controller_goal.expand(1,controller_goal_value);		
		}
	}
			rxController_xml->SetAttribute("priority", (*controllers_it)->priority());
	}
	return mb_element;
}

void MotionBehaviour::RLabInfoString2ElementXML_(string_type string_data, TiXmlElement* out_xml_element) const
{
#ifdef NOT_IN_RT
	//std::wcout << string_data.c_str() << std::endl;
#endif
	std::wstringstream data_ss(string_data);
	string_type temp_st;
	std::getline(data_ss, temp_st);	
	out_xml_element->SetAttribute("type", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());

	ControllerType type_of_controller = controller_map_[wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))];

	std::getline(data_ss, temp_st);		// Discard field "name"
	std::getline(data_ss, temp_st);		// Discard field "system"
	std::getline(data_ss, temp_st);		// Discard field "type"
	std::getline(data_ss, temp_st);		// Field "dim"
	int dimension_int = -1;
	std::wstringstream dimension_ss(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")));
	dimension_ss >> dimension_int;
	std::getline(data_ss, temp_st);		// Discard field "dt"
	std::getline(data_ss, temp_st);		// Discard field "activated"
	std::getline(data_ss, temp_st);		// Field "ik"
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

	std::getline(data_ss, temp_st);			// Field "number of via points"
	int num_via_points_int = -1;
	std::wstringstream num_via_points_ss(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")));
	num_via_points_ss >> num_via_points_int;

	for(int i=0; i< num_via_points_int; i++)
	{
		std::getline(data_ss, temp_st);		// Discard field "time" of via points
		std::getline(data_ss, temp_st);		// Discard field "type" of via points
		std::getline(data_ss, temp_st);		// Discard field "reuse" of via points 
		switch( type_of_controller.first )
		{
		case rxController::eControlType_Joint:
			std::getline(data_ss, temp_st);		// Discard field "dVector" of via points 
			break;
		case rxController::eControlType_Displacement:
			std::getline(data_ss, temp_st);		// Discard field "Vector3D" of via points 
			break;
		case rxController::eControlType_Orientation:
			std::getline(data_ss, temp_st);	
			//via_point_xml->SetAttribute("R", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
			break;
		case rxController::eControlType_HTransform:
			std::getline(data_ss, temp_st);	
			//via_point_xml->SetAttribute("R", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
			std::getline(data_ss, temp_st);	
			//via_point_xml->SetAttribute("r", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
			break;
		default:
			break;
		}
	}

	switch(type_of_controller.first)
	{
	case rxController::eControlType_Displacement:
		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("alpha", colon2space_( wstring2string_( temp_st.substr( temp_st.find(L"=") + 1, temp_st.find(L"\n") ) ) ).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("alphaDisplacement", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("beta", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("betaDisplacement",colon2space_( wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
		break;
	case rxController::eControlType_Orientation:
		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("alpha", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("alphaRotation", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("beta", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );

		std::getline(data_ss, temp_st);	
		out_xml_element->SetAttribute("betaRotation", colon2space_(wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
		break;
	case rxController::eControlType_HTransform:
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
	this->goal_controllers_ = motion_behaviour_assignment.goal_controllers_;

	// Delete the current stored control set
	if(this->control_set_)
	{
		delete control_set_;
		control_set_ = NULL;
	}

	// Copy the new control set
	if(motion_behaviour_assignment.control_set_)
	{
		// Now are different types of control set allowed
		if(dynamic_cast<rxTPOperationalSpaceControlSet*>(motion_behaviour_assignment.control_set_))
		{
			this->control_set_ = new rxTPOperationalSpaceControlSet((*dynamic_cast<rxTPOperationalSpaceControlSet*>(motion_behaviour_assignment.control_set_)));
		}
		else if(dynamic_cast<rxControlSet*>(motion_behaviour_assignment.control_set_))
		{
			this->control_set_ = new rxControlSet((*dynamic_cast<rxControlSet*>(motion_behaviour_assignment.control_set_)));
		}
		//this->control_set_ = new rxControlSet(*(motion_behaviour_assignment.control_set_));
		this->control_set_->setGravity(0,0,-GRAV_ACC);
		//this->control_set_->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(this->robot_));
	}
	return *this;
}

std::map<std::string, ControllerType> MotionBehaviour::createControllerMapping_()
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

rxController* MotionBehaviour::createJointController_(int joint_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const
{
	rxController* controller = NULL;
	//std::cout << this->robot_->jdof() << std::endl;
	switch(joint_subtype)
	{
	case NONE:
		controller = new rxJointController(this->robot_, controller_duration);
		break;
	case WITH_COMPLIANCE:
		controller = new rxJointComplianceController(this->robot_, controller_duration);	
		break;
	case WITH_IMPEDANCE:
		controller = new rxJointImpedanceController(this->robot_, controller_duration);
		break;
	case WITH_INTERPOLATION:
		controller = new rxInterpolatedJointController(this->robot_, controller_duration);
		break;
	case WITH_INTERPOLATION | WITH_COMPLIANCE:
		controller = new rxInterpolatedJointComplianceController(this->robot_, controller_duration);
		break;
	case WITH_INTERPOLATION | WITH_IMPEDANCE:
		controller = new rxInterpolatedJointImpedanceController(this->robot_, controller_duration);
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::createJointController_(ControllerSubgroup joint_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr)] Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointdVector* vpdv_ptr = (ViaPointdVector*)((*vp_it));
		dynamic_cast<rxJointController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}

	if(joint_subtype & WITH_COMPLIANCE)
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

rxController* MotionBehaviour::createDisplacementController_(int displacement_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const
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

	switch(displacement_subtype)
	{
	case NONE:
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
	case WITH_INTERPOLATION | WITH_COMPLIANCE:
		{
			controller = new rxInterpolatedDisplacementComplianceController(this->robot_, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
			break;
		}
	case WITH_INTERPOLATION | WITH_IMPEDANCE:
		controller = new rxInterpolatedDisplacementImpedanceController(this->robot_, beta, Displacement(beta_displacement), alpha, Displacement(alpha_displacement), controller_duration );
		break;
	case WITH_IMPEDANCE | ATTRACTOR:
		{
			double desired_distance = xml_deserializer_.deserializeDouble("desiredDistance", 1.);
			double max_force = xml_deserializer_.deserializeDouble("maxForce", 1.);
			controller = new FeatureAttractorController(this->robot_, beta, Displacement(beta_displacement), controller_duration, desired_distance, max_force );
			break;
		}
	default:
		throw std::string("ERROR: [MotionBehaviour::createDisplacementController_(ControllerSubgroup displacement_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml)] Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointVector3D* vpdv_ptr = (ViaPointVector3D*)((*vp_it));
		dynamic_cast<rxDisplacementController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}

	if(displacement_subtype & WITH_COMPLIANCE)
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

rxController* MotionBehaviour::createOrientationController_(int orientation_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const
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

	switch(orientation_subtype)
	{
	case NONE:
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
	case WITH_INTERPOLATION | WITH_COMPLIANCE:
		{
			controller = new rxInterpolatedOrientationComplianceController(this->robot_, beta, Rotation(beta_rotation), alpha, Rotation(alpha_rotation), controller_duration );
			break;
		}
	case WITH_INTERPOLATION | WITH_IMPEDANCE:
		controller = new rxInterpolatedOrientationImpedanceController(this->robot_, beta, Rotation(beta_rotation), alpha, Rotation(alpha_rotation), controller_duration );
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::createOrientationController_(ControllerSubgroup orientation_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml)] Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointRotation* vpdv_ptr = (ViaPointRotation*)((*vp_it));
		dynamic_cast<rxOrientationController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}
	if(orientation_subtype & WITH_COMPLIANCE)
	{
		XMLDeserializer xml_deserializer_(rxController_xml);
		std::stringstream stiffness_b_ss(xml_deserializer_.deserializeString("stiffness_b"));
		std::stringstream stiffness_k_ss(xml_deserializer_.deserializeString("stiffness_k"));
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

rxController* MotionBehaviour::createHTransformController_(int htransform_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const
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

	switch(htransform_subtype)
	{
	case NONE:
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
	case WITH_INTERPOLATION | WITH_COMPLIANCE:
		controller = new rxInterpolatedHTransformComplianceController(this->robot_, beta, HTransform(beta_rotation, beta_displacement), alpha, HTransform(alpha_rotation, alpha_displacement), controller_duration );
		break;
	case WITH_INTERPOLATION | WITH_IMPEDANCE:
		controller = new rxInterpolatedHTransformImpedanceController(this->robot_, beta, HTransform(beta_rotation, beta_displacement), alpha, HTransform(alpha_rotation, alpha_displacement), controller_duration );
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::createHTransformController_(ControllerSubgroup htransform_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml)] Unexpected Controller subgroup.");
		break;
	}
	for(std::vector<ViaPointBase*>::iterator vp_it = via_points_ptr.begin(); vp_it != via_points_ptr.end(); vp_it++)
	{
		ViaPointHTransform* vpdv_ptr = (ViaPointHTransform*)((*vp_it));
		dynamic_cast<rxHTransformController*>(controller)->addPoint(vpdv_ptr->point_, vpdv_ptr->time_, vpdv_ptr->reuse_, (eInterpolatorType)(vpdv_ptr->type_));
	}

	if(htransform_subtype & WITH_COMPLIANCE)
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

rxController* MotionBehaviour::createQuasiCoordController_(int quasi_coord_subtype, double controller_duration) const
{
	rxController* controller = NULL;
	switch(quasi_coord_subtype)
	{
	case NONE:
		//	controller = new rxQuasiCoordController(this->robot_, , controller_duration);
		break;
	case WITH_COMPLIANCE:
		//	controller = new rxQuasiCoordComplianceController(this->robot_, ,controller_duration);
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::createQuasiCoordController_(ControllerSubgroup quasi_coord_subtype, double controller_duration)] Unexpected Controller subgroup.");
		break;
	}
	return controller;
}

rxController* MotionBehaviour::createNullMotionController_(int null_motion_subtype, double controller_duration) const
{
	rxController* controller = NULL;
	switch(null_motion_subtype)
	{
	case NONE:
		controller = new rxNullMotionController(this->robot_, controller_duration);
		break;
	case WITH_COMPLIANCE:
		controller = new rxNullMotionComplianceController(this->robot_,  controller_duration);
		break;
	case WITH_GRADIENT:
		//	controller = new rxGradientNullMotionController(this->robot_, controller_duration);
		break;
	default:
		throw std::string("ERROR: [MotionBehaviour::createNullMotionController_(ControllerSubgroup null_motion_subtype, double controller_duration)] Unexpected Controller subgroup.");
		break;
	}
	return controller;
}

rxController* MotionBehaviour::createFunctionalController_(int functional_subtype, int dimension, double controller_duration, TiXmlElement* rxController_xml) const
{
	rxController* controller = NULL;
	switch(functional_subtype)
	{
	case NONE:
		//controller = new rxFunctionController(this->robot_, dimension, controller_duration);
		break;
	case WITH_IMPEDANCE | SUBDISPLACEMENT:
	case OBSTACLE_AVOIDANCE:{
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
		std::stringstream index_ss = std::stringstream(xml_deserializer_.deserializeString("index"));
		std::vector<long> index;
		long index_value;
		while((index_ss >> index_value))
		{
			index.push_back(index_value);
		}
		if(functional_subtype == (WITH_IMPEDANCE | SUBDISPLACEMENT))
		{
			double max_force = xml_deserializer_.deserializeDouble("maxForce", 1.);
			std::string limit_body = xml_deserializer_.deserializeString("limitBody");
			double distance_limit = xml_deserializer_.deserializeDouble("distanceLimit", 0.);
			controller = new SubdisplacementController(this->robot_,beta, beta_displacement, alpha, alpha_displacement,
				controller_duration, index, max_force, this->robot_->findBody(string_type(string2wstring_(limit_body))), distance_limit );
			if(index.size() == 1 && index.at(0) ==1)
			{
				dynamic_cast<SubdisplacementController*>(controller)->setTaskConstraints(0.);
			}else if (index.size() ==2 && index.at(0) ==1 && index.at(1) == 1)
			{
				dynamic_cast<SubdisplacementController*>(controller)->setTaskConstraints(0., 0.);
			}
			break;
		}
		if(functional_subtype == OBSTACLE_AVOIDANCE)
		{
			double distance_threshold = xml_deserializer_.deserializeDouble("distanceThreshold", 1.);
			double deactivation_threshold = xml_deserializer_.deserializeDouble("deactivationThreshold", 1.);
			if(CollisionInterface::instance)
			{
				controller = new ObstacleAvoidanceController(this->robot_, beta, beta_displacement, distance_threshold, 
					CollisionInterface::instance, controller_duration, deactivation_threshold);
			}else{
				throw std::string("ERROR: [MotionBehaviour::createFunctionalController]: Obstacle Avoidance needs Collision interface.");
			}
			break;
		}
							}
	default:
		throw std::string("ERROR: [MotionBehaviour::createFunctionalController_(ControllerSubgroup functional_subtype, double controller_duration)] Unexpected Controller subgroup.");
		break;
	}
	return controller;
}

void MotionBehaviour::print()
{
	std::list<rxController*> controllers = control_set_->getControllers();
	string_type controllers_to_string;
	for(std::list< rxController* >::const_iterator controllers_it = controllers.begin();
		controllers_it != controllers.end() ; controllers_it ++){
			string_type controller_to_string;
			(*controllers_it)->toString(controller_to_string);

			std::wcout << controller_to_string << std::endl;

			//		int num_via_points_int = -1;
			//		std::wstringstream num_via_points_ss(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")));
			//		num_via_points_ss >> num_via_points_int;
			//		for(int i=0; i< num_via_points_int; i++)
			//		{
			//			std::getline(data_ss, temp_st);		// Discard field "time" of via points
			//			std::getline(data_ss, temp_st);		// Discard field "type" of via points
			//			std::getline(data_ss, temp_st);		// Discard field "reuse" of via points 
			//			switch( type_of_controller.first ){
			//case JOINT_SPACE_CONTROLLER:
			//	std::getline(data_ss, temp_st);		// Discard field "dVector" of via points 
			//	break;
			//case rxController::eControlType_Displacement:
			//	std::getline(data_ss, temp_st);		// Discard field "Vector3D" of via points 
			//	break;
			//case rxController::eControlType_Orientation:
			//	std::getline(data_ss, temp_st);	
			//	//via_point_xml->SetAttribute("R", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
			//	break;
			//case rxController::eControlType_HTransform:
			//	std::getline(data_ss, temp_st);	
			//	//via_point_xml->SetAttribute("R", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
			//	std::getline(data_ss, temp_st);	
			//	//via_point_xml->SetAttribute("r", wstring2string_(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
			//	break;
			//default:
			//	break;
			//			}
			//		}

	}
}

void MotionBehaviour::setMaxVelocityForInterpolation(double max_velocity) {
	max_velocity_ = max_velocity;
}

void MotionBehaviour::setMinTimeForInterpolation(double min_time) {
	min_time_ = min_time;
}
