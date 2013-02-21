#include "MotionBehaviour.h"
#include <iostream>
#include <string>

#include "XMLDeserializer.h"


//#define NOT_IN_RT

using namespace std;

MotionBehaviour::MotionBehaviour() :
MDPEdge(NULL, NULL, -1)
, control_set_(NULL)
, robot_(NULL)
, time_(-1)
, dT_(-1)
, max_velocity_(-1)
, min_time_(-1)
, time_to_converge_(0.0)
{
}

MotionBehaviour::MotionBehaviour(Milestone * dad, Milestone * son, rxSystem* robot, double weight, double dt ):
MDPEdge(dad, son, weight)
, control_set_(NULL)
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
		//control_set_->nullMotionController()->setGain(0.02,0.0,0.01);
		control_set_->nullMotionController()->setGain(0.0, 0.0, 0.0);
	}
	else
	{
		control_set_= NULL;
	}
}

MotionBehaviour::MotionBehaviour(Milestone * dad, Milestone * son, rxControlSetBase* control_set, double weight):
MDPEdge(dad, son, weight)
, control_set_(control_set)
, robot_(NULL)
, time_(0)
, dT_(-1)
, max_velocity_(-1)
, min_time_(-1)
, time_to_converge_(0.0)
{
	if(control_set_)
	{
		dT_ = control_set->dt();
		robot_ = const_cast<rxSystem*>(control_set->sys());

		// TODO: should this be done by the caller?
		/*control_set_->setGravity(0,0,-GRAV_ACC);
		control_set_->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(this->robot_));
		control_set_->nullMotionController()->setGain(0.02,0.0,0.01);*/
	}
	else
	{
		dT_ = 0.002;
	}
}

MotionBehaviour::MotionBehaviour(const MotionBehaviour & motion_behaviour_copy) :
MDPEdge(motion_behaviour_copy),
robot_(motion_behaviour_copy.robot_),
time_(motion_behaviour_copy.time_),
dT_(motion_behaviour_copy.dT_),
min_time_(motion_behaviour_copy.min_time_),
max_velocity_(motion_behaviour_copy.max_velocity_),
goal_controllers_(motion_behaviour_copy.goal_controllers_),
time_to_converge_(motion_behaviour_copy.time_to_converge_),
_onDemand_controllers(motion_behaviour_copy._onDemand_controllers)
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

void MotionBehaviour::addController(rxController* ctrl, bool is_goal_controller) 
{
	if(ctrl){
		if(ctrl->dt() != this->dT_)
		{
			throw std::string("[MotionBehaviour::addController] ERROR: Time intervals of MotionBehaviour and all its controllers must be the same.");
		}
		ctrl->deactivate();
		goal_controllers_[ctrl->name()] = is_goal_controller;
		control_set_->addController(ctrl, ctrl->name(), ctrl->priority());
	}
	else
	{
		throw std::string("[MotionBehaviour::addController] ERROR: The pointer to rxController to be added is NULL.");
	}
	OnDemandController* is_on_demand_controller = dynamic_cast<OnDemandController*>(ctrl);
	if(is_on_demand_controller)
	{
		this->_onDemand_controllers.push_back(is_on_demand_controller);
	}
}

double MotionBehaviour::calculateTimeToConverge(double default_min_time, double default_max_velocity, const dVector& error_x, const dVector& xd, const dVector& xd_desired)
{
	/**
	 The user can specify a minimum time to converge or a maximum velocity.
	 
	 If none is given, default values will be used.
	 If only time constraint is given, use this.
	 If only velocity constraint is given, use this.
	 If both are given, comply with both constraints.
	**/
	double time_to_converge = min_time_;
	
	if (max_velocity_ > 0 || min_time_ <= 0 ) {
		double velocity = max_velocity_;
		if (max_velocity_ <= 0) {
			velocity = default_max_velocity;
			time_to_converge = default_min_time;
		}
		for (int i = 0; i < error_x.size(); i++) {
			// the cubic case
			// TODO: Check for quintic, linear, etc.
			time_to_converge = max( fabs((6.0 * error_x[i]) / (xd[i] + xd_desired[i] + 4.0 * velocity)), time_to_converge);
		}
	}
	
	return time_to_converge;
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
				if ((*it)->activated())
				{
					(*it)->deactivate();
				}

				(*it)->activate();

				// We only add the goal defined by the child milestone to the controller if it is a goal controller
				// If it is not a goal controller, it contains already its special goal
				if(this->goal_controllers_.find((*it)->name())->second)
				{
					// add the goal of the child milestone
					switch((*it)->type()) {
					case rxController::eControlType_Joint:
						{
							// q1 and q2 to interpolate between
							dVector current_q = robot_->q();//parent->getConfiguration();
                            dVector desired_q = ((Milestone*)child)->getConfiguration();

							// qd1 and qd2
							dVector current_qd(current_q.size());
							current_qd = robot_->qdot();
							dVector desired_qd(current_q.size());
							desired_qd.zero();
							
							// at least 5.0 seconds or maximally 0.3 rad/s if nothing is given
							time_to_converge_ = calculateTimeToConverge(5.0, 0.30, desired_q - current_q, current_qd, desired_qd);
							std::cout << "[MotionBehaviour::activate] INFO: Time of joint trajectory: " << time_to_converge_ << std::endl;
							
							dynamic_cast<rxJointController*>(*it)->addPoint(desired_q, time_to_converge_, false, eInterpolatorType_Cubic);
							break;
						}
					case rxController::eControlType_Displacement:
						{
							// r1 and r2 to interpolate between
							HTransform ht;
							//std::cout << robot_ << std::endl;
							//std::cout << robot_->getUCSBody(_T("EE"),ht) << std::endl;
							rxBody* EE = robot_->getUCSBody(_T("EE"),ht);
							dVector current_r = ht.r + EE->T().r; //parent->getConfiguration();
							dVector desired_r = ((OpSpaceMilestone*)child)->getPosition(); //->getConfiguration()
							
							Rotation current_S = ht.R * EE->T().R;
							dVector current_R;
							current_S.GetQuaternion(current_R);
							/*
							std::cout << "current position: " << current_r[0] << " " << current_r[1] << " " << current_r[2] << std::endl;
							std::cout << "current orientation: " << current_R[0] << " " << current_R[1] << " " << current_R[2] << " " << current_R[3] << std::endl;
							std::cout << "desired position: " << desired_r[0] << " " << desired_r[1] << " " << desired_r[2] << std::endl;
							*/

							// rd1 and rd2
							dVector current_rd(current_r.size());
							current_rd.zero();
							dVector desired_rd(current_r.size());
							desired_rd.zero();

							// at least 5.0 seconds or maximally 0.2 m/s if nothing is given
							time_to_converge_ = calculateTimeToConverge(5.0, 0.2, desired_r - current_r, current_rd, desired_rd);
							std::cout << "[MotionBehaviour::activate] INFO: Time of displacement trajectory: " << time_to_converge_ << std::endl;

							FeatureAttractorController* fac = dynamic_cast<FeatureAttractorController*>(*it);
							if(fac)
							{
								fac->updateFeaturePosition(desired_r[0], desired_r[1], desired_r[2]);
							}
							else
							{
								dynamic_cast<rxDisplacementController*>(*it)->addPoint(desired_r, time_to_converge_, false, eInterpolatorType_Cubic);
							}
							break;
						}
					case rxController::eControlType_Orientation:
						{
							//dVector goal = ((Milestone*)child)->getConfiguration();
							//Rotation goal_rot(goal[3], goal[4], goal[5]);		
							//NOTE: Suposses that the orientation is defined with 
							// goal[3] = roll
							// goal[4] = pitch
							// goal[5] = 
							Rotation goal_rot = ((OpSpaceMilestone*)child)->getOrientation();
							dVector desired_quat;
							goal_rot.GetQuaternion(desired_quat);

							// get current orientation
							HTransform ht;
							rxBody* EE = robot_->getUCSBody(_T("EE"), ht);
							Rotation current_rot = ht.R * EE->T().R;
							dVector current_quat;
							current_rot.GetQuaternion(current_quat);

							// get angular error
							double theta = acos(current_quat.inner(desired_quat));
							if (theta > M_PI/2.0)
								theta -= M_PI/2.0;

							dVector error_theta(1, theta);
							dVector current_thetad(1, 0.0);
							dVector desired_thetad(1, 0.0);
							/*
							std::cout << "desired orientation: " << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << std::endl;
							*/
							
							// at least 10.0 seconds or maximally 0.1 rad/s if nothing is given
							time_to_converge_ = calculateTimeToConverge(10.0, 0.1, error_theta, current_thetad, desired_thetad);
							std::cout << "[MotionBehaviour::activate] INFO: Time of orientation trajectory: " << time_to_converge_ << std::endl;

							dynamic_cast<rxOrientationController*>(*it)->addPoint(goal_rot, time_to_converge_, false);
							break;
						}
					case rxController::eControlType_HTransform:
						{
							double time = (min_time_ > 0) ? min_time_ : 10.0;
							dVector goal = ((Milestone*)child)->getConfiguration();
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
	std::cout << "[MotionBehaviour::hasConverged] ERROR: This function is deprecated and should not be called!" << std::endl;
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

dVector MotionBehaviour::getOriTaskError() const
{
	dVector error;
	std::list<rxController*> controllers = control_set_->getControllers();
	for(std::list<rxController*>::const_iterator it = controllers.begin(); it != controllers.end(); ++it)
	{
		if (*it)
		{
			if ((*it)->type() == rxController::eControlType_Orientation)
			{
				dVector e = (*it)->e();
				error.expand(e.size(), e);
			}
		}
	}
	return error;
}

pair<double,double> MotionBehaviour::getDistanceToNearestObst() const
{
	double distance_base = 999.0;
	double distance_ee = 999.0;
	std::list<rxController*> controllers = control_set_->getControllers();
	for(std::list<rxController*>::const_iterator it = controllers.begin(); it != controllers.end(); ++it)
	{
		if (*it)
		{
			ObstacleAvoidanceController* oa = dynamic_cast<ObstacleAvoidanceController*>(*it);
			if (oa)
			{
				if(oa->isBase())
				{
					if(oa->getDistanceToObst() < distance_base)
					{
						distance_base = oa->getDistanceToObst();
					}
				}
				else
				{				
					if(oa->getDistanceToObst() < distance_ee)
					{
						distance_ee = oa->getDistanceToObst();
					}
				}
			}
		}
	}
	return pair<double,double>(distance_base,distance_ee);
}


dVector MotionBehaviour::getLineTaskError() const
{
	dVector error;
	std::list<rxController*> controllers = control_set_->getControllers();
	for(std::list<rxController*>::const_iterator it = controllers.begin(); it != controllers.end(); ++it)
	{
		if (*it)
		{
			SubdisplacementController* task = dynamic_cast<SubdisplacementController*>(*it);
			if (task)
			{
				dVector e = task->e();
				error.expand(e.size(), e);
			}
		}
	}
	return error;
}

dVector MotionBehaviour::getOuterJointPositionError() const
{
	dVector error;

	NakamuraControlSet* ctrl_set = dynamic_cast<NakamuraControlSet*>(control_set_);
	if (ctrl_set)
	{
		error = ctrl_set->position_error();
	}

	return error;
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
	for(unsigned int i = 0; i < _onDemand_controllers.size(); i++)
	{
		// before performing the update, activate/deactivate the onDemand controllers
		// depending on the measurements they make.
		_onDemand_controllers[i]->updateMeasurement();
	}

	//dVector torque(robot_->jointDOF()); // This does not work for XR4000+WAM
	/*
	int dof = 0;
	list<rxJoint*>::iterator it;
	for(it = robot_->joints().begin(); it !=  robot_->joints().end(); it++)
	{
		dVector ll, ul;
		(*it)->getLimits(ll,ul);
		dof += ll.size();
	}
	dVector torque(dof);
	*/
	// TODO: This might? not work with XR+WAM ?
	dVector torque(robot_->jointDOF() + robot_->earthDOF() + robot_->constraintDOF());
	control_set_->compute(t,torque);
	time_ += dT_;
	return torque;
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
	this->_onDemand_controllers = motion_behaviour_assignment._onDemand_controllers;

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

dVector MotionBehaviour::getGoalConfiguration()
{
	std::list<rxController*> controllers = control_set_->getControllers();
	string_type controllers_to_string;
	for(std::list< rxController* >::const_iterator controllers_it = controllers.begin();
		controllers_it != controllers.end() ; controllers_it ++){
			ReInterpolatedJointImpedanceController* jcontrol = dynamic_cast<ReInterpolatedJointImpedanceController*>(*controllers_it);
			if(jcontrol)
			{
				return jcontrol->getViaPointConfiguration();
			}
	}
	return dVector();
}



void MotionBehaviour::waitMode()
{
	std::list<rxController*> controllers = control_set_->getControllers();
	string_type controllers_to_string;
	for(std::list< rxController* >::const_iterator controllers_it = controllers.begin();
		controllers_it != controllers.end() ; controllers_it ++){
			FeatureAttractorController* feature_control = dynamic_cast<FeatureAttractorController*>(*controllers_it);
			if(feature_control)
			{
				feature_control->getFeaturePosition().print(_T("current_feature"));
				HTransform ht;
				rxBody* EE = robot_->getUCSBody(_T("EE"),ht);
				dVector current_r = ht.r + EE->T().r;
				current_r[0] += 0.2;
				cout << "setting ee goal to " << endl;
				current_r.print(_T("EE"));
				feature_control->updateFeaturePosition(current_r[0],current_r[1],current_r[2]);
				cout << "feature is: " << feature_control->activated() << endl;
				feature_control->deactivate();
			}
	}
}


void MotionBehaviour::setMaxVelocityForInterpolation(double max_velocity) {
	max_velocity_ = max_velocity;
}

void MotionBehaviour::setMinTimeForInterpolation(double min_time) {
	min_time_ = min_time;
}