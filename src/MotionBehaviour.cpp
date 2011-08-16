#include "elasticroadmap\include\MotionBehaviour.h"
#include <iostream>
#include <string>

using namespace std;

MotionBehaviour::MotionBehaviour() :
Edge<Milestone>(NULL, NULL, -1)
, control_set_(NULL)
, robot_(NULL)
, time_(-1)
, dT_(-1)
{
}

MotionBehaviour::MotionBehaviour(const Milestone * dad, const Milestone * son, rxSystem* robot, double weight, double dt ):
Edge<Milestone>(dad, son, weight)
, robot_(robot)
, time_(0)
, dT_(dt)
{
	if(robot_)
	{
		control_set_ = new rxControlSet(robot_, dT_);
		control_set_->setGravity(0,0,-GRAV_ACC);
		control_set_->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(this->robot_));
	}
	else
	{
		control_set_= NULL;
	}
}

MotionBehaviour::MotionBehaviour(TiXmlElement* motion_behaviour_xml, const Milestone * dad, const Milestone * son, rxSystem* robot):
Edge(dad, son),
robot_(robot)
{
	double weight_xml;
	if(!motion_behaviour_xml->Attribute( "Weight", &weight_xml))
	{
		throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"Weight\" not found in XML element motion_behaviour_xml.");
	}
	this->setWeight(weight_xml);

	if(!motion_behaviour_xml->Attribute( "Time", &this->time_))
	{
		throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"Time\" not found in XML element motion_behaviour_xml.");
	}

	if(!motion_behaviour_xml->Attribute( "dT", &this->dT_))
	{
		throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"dT\" not found in XML element motion_behaviour_xml.");
	}

	std::string control_set_type = std::string(motion_behaviour_xml->Attribute("ControlSetType"));
	if(control_set_type == std::string(""))
	{
		throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"ControlSetType\" not found in XML element motion_behaviour_xml.");
	}
	else if(control_set_type==std::string("class rxControlSet *"))
	{
		if(robot_)
		{
			control_set_ = new rxControlSet(robot, dT_);
			control_set_->setGravity(0,0,-GRAV_ACC);
			control_set_->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(this->robot_));
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
		std::string motion_behaviour_name = std::string(rxController_xml->Attribute("Name"));
		if(motion_behaviour_name == std::string(""))
		{
			throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"Name\" not found in XML element rxController_xml.");
		}

		int motion_behaviour_activated = -1, motion_behaviour_priority = -1, motion_behaviour_type = -1, interpolator_type_int = -1;

		if(!rxController_xml->Attribute("Activated", &motion_behaviour_activated))
		{
			throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"Activated\" not found in XML element rxController_xml.");
		}
		if(!rxController_xml->Attribute("Priority", &motion_behaviour_priority))
		{
			throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"Priority\" not found in XML element rxController_xml.");
		}
		if(!rxController_xml->Attribute("Type", &motion_behaviour_type))
		{
			throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"Type\" not found in XML element rxController_xml.");
		}

		double motion_behaviour_duration = -1.0;
		if(!rxController_xml->Attribute("Duration", &motion_behaviour_duration))
		{
			//TODO: Restore the exception throwing when the parser writes the interpolator type (now is done manually in the xml)
			//throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"Duration\" not found in XML element rxController_xml.");
			std::cout << "WARNING: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Field \"Duration\" not found!" << std::endl;	
		}

		if(!rxController_xml->Attribute( "InterpolatorType", &interpolator_type_int))
		{
			//TODO: Restore the exception throwing when the parser writes the interpolator type (now is done manually in the xml)
			//throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"InterpolatorType\" not found in XML element rxController_xml.");
			std::cout << "WARNING: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Field \"InterpolatorType\" not found!" << std::endl;
		}		

		dVector kp_vector;
		kp_vector.all(-1.0);
		TiXmlElement* kp_xml = rxController_xml->FirstChildElement("Kp");
		if(!kp_xml){
			//TODO: Restore the exception throwing when the parser writes the interpolator type (now is done manually in the xml)
			//throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Element \"Kp\" not found in XML element rxController_xml.");
			std::cout << "WARNING: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Element \"Kp\" not found!" << std::endl;
		}
		else
		{
			for(TiXmlElement* kp_joint_xml = kp_xml->FirstChildElement("Joint") ; kp_joint_xml; kp_joint_xml=kp_joint_xml->NextSiblingElement())
			{
				double kp_joint_value = -1.0;
				if(!kp_joint_xml->Attribute("value", &kp_joint_value))
				{
					throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"value\" not found in XML element kp_joint_xml.");
				}
				kp_vector.expand(1,kp_joint_value);
			}
		}

		dVector kd_vector;
		kd_vector.all(-1.0);
		TiXmlElement* kd_xml = rxController_xml->FirstChildElement("Kd");
		if(!kd_xml){
			//TODO: Restore the exception throwing when the parser writes the interpolator type (now is done manually in the xml)
			//throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Element \"Kd\" not found in XML element rxController_xml.");
			std::cout << "WARNING: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Element \"Kd\" not found!" << std::endl;
		}
		else
		{
			for(TiXmlElement* kd_joint_xml = kd_xml->FirstChildElement("Joint") ; kd_joint_xml; kd_joint_xml=kd_joint_xml->NextSiblingElement())
			{
				double kd_joint_value = -1.0;
				if(!kd_joint_xml->Attribute("value", &kd_joint_value))
				{
					throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"value\" not found in XML element kd_joint_xml.");
				}
				kd_vector.expand(1,kd_joint_value);
			}
		}

		dVector goal_vector;
		goal_vector.all(-1.0);
		TiXmlElement* goal_xml = rxController_xml->FirstChildElement("Goal");
		if(goal_xml){
			for(TiXmlElement* goal_joint_xml = goal_xml->FirstChildElement("Joint") ; goal_joint_xml; goal_joint_xml=goal_joint_xml->NextSiblingElement())
			{
				double goal_joint_value = -1.0;
				if(!goal_joint_xml->Attribute("value", &goal_joint_value))
				{
					throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Attribute \"value\" not found in XML element goal_joint_xml.");
				}
				goal_vector.expand(1,goal_joint_value);
			}
		}

		double dT = 0;
		rxController* controller = NULL;
		eInterpolatorType interpolator_type = (eInterpolatorType)interpolator_type_int;
		switch(motion_behaviour_type){
			case rxController::eControlType_Joint:

				rxController_xml->Attribute("ControlPeriod", &dT);
				controller = new rxInterpolatedJointController(robot,dT);
				if(goal_xml)
				{
					dynamic_cast< rxJointController* >(controller)->addPoint(goal_vector, motion_behaviour_duration, false, interpolator_type);
					controller->setGain(kd_vector, kp_vector, dVector(kd_vector.size()));
				}
				controller->deactivate();
				this->addController(controller);
				break;
			default:
				//::std::cout << 
				throw std::string("ERROR: [MotionBehaviour::MotionBehaviour(TiXmlElement* motion_xml, const Milestone * dad, const Milestone * son, rxSystem* robot)] Unknown type of rxController.");
				break;

		}
	}
}

MotionBehaviour::MotionBehaviour(const MotionBehaviour & motion_behaviour_copy) :
Edge(motion_behaviour_copy),
robot_(motion_behaviour_copy.robot_),
time_(motion_behaviour_copy.time_),
dT_(motion_behaviour_copy.dT_)
{
	if(motion_behaviour_copy.control_set_)
	{
		this->control_set_ = new rxControlSet((*motion_behaviour_copy.control_set_));
		this->control_set_->setGravity(0,0,-GRAV_ACC);
		//this->control_set_->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(motion_behaviour_copy.robot_));
	}
	else
	{
		this->control_set_ = NULL;
	}
	
	// TODO: HACK!!!!! The pointers are pointing to the controllers of motion_behaviour_assignment, they should point to the copies contained in the control set!!!!!
	for(::std::vector< rxController* >::const_iterator controllers_set__it = motion_behaviour_copy.controllers_set_.begin();
		controllers_set__it != motion_behaviour_copy.controllers_set_.end() ; controllers_set__it ++){
			this->controllers_set_.push_back(*controllers_set__it);
	}
}

MotionBehaviour::~MotionBehaviour() 
{
	// Do not delete any object that was not directly created in the constructor!!
	if(control_set_)
	{
		delete control_set_;
		control_set_ = NULL;
	}
}

void MotionBehaviour::addController(rxController* ctrl) 
{
	if(ctrl){
		ctrl->deactivate();
		controllers_set_.push_back(ctrl);
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
		for(::std::vector< rxController* >::iterator it = controllers_set_.begin(); it != controllers_set_.end(); ++it)
		{
			(*it)->activate();
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

	for(int i = 0; i < error.size(); ++i)
	{
		if (::std::abs(error[i]) > 0.01 || time_ < 2.0)
		{
			return false;
		}
	}
	return true;
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
	//TODO: Is not the same as ControlPeriod????
	motion_behaviour_xml->SetDoubleAttribute("dT", dT_);
	motion_behaviour_xml->SetAttribute("ControlSetType", typeid(control_set_).name());

	//TODO: What about all the parameters to be passed to RLab?
	for(::std::vector< rxController* >::const_iterator controllers_set__it = controllers_set_.begin();
		controllers_set__it != controllers_set_.end() ; controllers_set__it ++){
			TiXmlElement * rxController_xml;
			rxController_xml = new TiXmlElement("rxController");
			motion_behaviour_xml->LinkEndChild(rxController_xml);
			string_type name_str = (*controllers_set__it)->name();
			char name_char[256];
			wcstombs( &name_char[0], name_str.c_str(), 256);
			rxController_xml->SetAttribute("Name", name_char );
			rxController_xml->SetAttribute("Activated",(*controllers_set__it)->activated());
			rxController_xml->SetAttribute("Priority",(*controllers_set__it)->priority());
			rxController_xml->SetAttribute("Type",(*controllers_set__it)->type());
			rxController_xml->SetAttribute("Dimension",(*controllers_set__it)->dim());
			rxController_xml->SetDoubleAttribute("ControlPeriod", (*controllers_set__it)->dt());

			rMath::dVector vect_vel = (*controllers_set__it)->v();
			std::ostringstream oss_vel;
			for (int i =0; i< vect_vel.size(); i++) {
				if(i != 0)
					oss_vel << " ";
				oss_vel << vect_vel(i);
			}
			std::string string_vel = oss_vel.str();
			rxController_xml->SetAttribute("TaskReferenceVelocity",string_vel.c_str());

			rMath::dVector vect_aref = (*controllers_set__it)->aref();
			std::ostringstream oss_aref;
			for (int i =0; i< vect_aref.size(); i++) {
				if(i != 0)
					oss_aref << " ";
				oss_aref << vect_aref(i);
			}
			std::string string_aref = oss_aref.str();
			rxController_xml->SetAttribute("TaskReferenceAcceleration",string_aref.c_str());

			rMath::dVector vect_error = (*controllers_set__it)->e();
			std::ostringstream oss_error;
			for (int i =0; i< vect_error.size(); i++) {
				if(i != 0)
					oss_error << " ";
				oss_error << vect_error(i);
			}
			std::string string_error = oss_error.str();
			rxController_xml->SetAttribute("Error",string_error.c_str());

			rMath::dVector vect_velerror = (*controllers_set__it)->edot();
			std::ostringstream oss_velerror;
			for (int i =0; i< vect_velerror.size(); i++) {
				if(i != 0)
					oss_velerror << " ";
				oss_velerror << vect_velerror(i);
			}
			std::string string_velerror = oss_velerror.str();
			rxController_xml->SetAttribute("ErrorVelocity",string_velerror.c_str());
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

	// TODO: HACK!!!!! The pointers are pointing to the controllers of motion_behaviour_assignment, they should point to the copies contained in the control set!!!!!
	this->controllers_set_.clear();
	for(::std::vector< rxController* >::const_iterator controllers_set__it = motion_behaviour_assignment.controllers_set_.begin();
		controllers_set__it != motion_behaviour_assignment.controllers_set_.end() ; controllers_set__it ++){
			this->controllers_set_.push_back(*controllers_set__it);
	}
	return *this;
}
