#include "CSpaceBlackBoardMilestone.h"

#include <iostream>
#include <string>

#include "tinyxml.h"
#include "XMLDeserializer.h"

#include "msgs/Float64MultiArray.h"

using namespace std;

#define RADIUS 0.4

//TODO: (When creating milestone or setting the MB) Is ok to force the MB to have this Milestone as parent and child?

CSpaceBlackBoardMilestone::CSpaceBlackBoardMilestone() :
Milestone("Default"),
motion_behaviour_(NULL),
black_board_(RTBlackBoard::getInstance()),
black_board_variable_name_("cspace_milestone")
{
}

CSpaceBlackBoardMilestone::CSpaceBlackBoardMilestone(const std::string& name) : 
Milestone(name),
motion_behaviour_(NULL),
black_board_(RTBlackBoard::getInstance()),
black_board_variable_name_("cspace_milestone")
{
}

CSpaceBlackBoardMilestone::CSpaceBlackBoardMilestone(const std::string& name, const std::vector<double>& configuration, MotionBehaviour * motion_behaviour, const std::vector<double>& region_convergence_radius) :
Milestone(name),
configuration_(configuration),
region_convergence_radius_(region_convergence_radius),
black_board_(RTBlackBoard::getInstance()),
black_board_variable_name_("cspace_milestone")
{
	if(motion_behaviour)
	{
		this->motion_behaviour_ = motion_behaviour->clone();
		// Force motion_behaviour to have this milestone as parent and child
		this->motion_behaviour_->setChild(this);
		this->motion_behaviour_->setParent(this);
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}

	Point tmp(configuration[0],configuration[1],0.0);
	tmp.x += RADIUS;
	handle_points_.push_back(tmp);
	tmp.x -= 2 * RADIUS;
	handle_points_.push_back(tmp);
	tmp.x += RADIUS;
	tmp.y += RADIUS;
	handle_points_.push_back(tmp);
	tmp.y -= 2 * RADIUS;
	handle_points_.push_back(tmp);
}

CSpaceBlackBoardMilestone::CSpaceBlackBoardMilestone(const std::string& name, const std::vector<double>& configuration, MotionBehaviour * motion_behaviour,
								 const std::vector<double>& region_convergence_radius, Milestone::Status status, const std::vector<Point>& handle_points):
Milestone(name),
configuration_(configuration),
region_convergence_radius_(region_convergence_radius),
handle_points_(handle_points),
black_board_(RTBlackBoard::getInstance()),
black_board_variable_name_("cspace_milestone")
{
	this->status_ = status;
	if(motion_behaviour)
	{
		this->motion_behaviour_ = motion_behaviour->clone();
		// Force motion_behaviour to have this milestone as parent and child
		this->motion_behaviour_->setChild(this);
		this->motion_behaviour_->setParent(this);
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
}

CSpaceBlackBoardMilestone::CSpaceBlackBoardMilestone(const CSpaceBlackBoardMilestone & cmilestone_cpy) :
Milestone(cmilestone_cpy),
configuration_(cmilestone_cpy.configuration_),
region_convergence_radius_(cmilestone_cpy.region_convergence_radius_),
handle_points_(cmilestone_cpy.handle_points_),
black_board_(RTBlackBoard::getInstance()),
black_board_variable_name_("cspace_milestone")
{
	if(cmilestone_cpy.motion_behaviour_)
	{
		this->motion_behaviour_ = cmilestone_cpy.motion_behaviour_->clone();
		this->motion_behaviour_->setChild(this);
		this->motion_behaviour_->setParent(this);
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
}

CSpaceBlackBoardMilestone::~CSpaceBlackBoardMilestone()
{	
	// Do not delete anything but the MotionBehaviour, it is the only thing created/copied in the constructor
	delete motion_behaviour_;
	motion_behaviour_ = NULL;
}

void CSpaceBlackBoardMilestone::setMotionBehaviour(MotionBehaviour * motion_behaviour)
{
	if(motion_behaviour)
	{
		this->motion_behaviour_ = motion_behaviour->clone();
		this->motion_behaviour_->setChild(this);
		this->motion_behaviour_->setParent(this);
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
}

std::vector<double> CSpaceBlackBoardMilestone::getConfigurationSTDVector() const
{
	return this->configuration_;	
}

dVector CSpaceBlackBoardMilestone::getConfiguration() const
{
	dVector ret_value;
	for(unsigned int i = 0; i<configuration_.size(); i++)
	{
		ret_value.expand(1, configuration_[i]);
	}
	return ret_value;
}

void CSpaceBlackBoardMilestone::update()
{
	//std::cout << "Trying to update milestone." << std::endl;

	// get the value from the BlackBoard
	/* This always evaluates to false, so we avoid it.
	if (black_board_->isUpdated(black_board_variable_name_))
	{	
	*/
		rlab::Float64MultiArray* set_point = new rlab::Float64MultiArray;
		if (black_board_->getData<rlab::Float64MultiArray>(black_board_variable_name_, set_point))
		{
			// convert it into the configuration
			configuration_ = set_point->get();

			//std::cout << "Successful! " << configuration_[0] << " " << configuration_[1] << std::endl;
		}
		//else
		//	std::cout << "Could not get data from BlackBoard!" << std::endl;

		delete set_point;
	//}
	//else
	//	std::cout << "No update of " << black_board_variable_name_ << " in BlackBoard!" << std::endl;
}

CSpaceBlackBoardMilestone* CSpaceBlackBoardMilestone::clone() const
{
	CSpaceBlackBoardMilestone* new_c_space_milestone = new CSpaceBlackBoardMilestone(*this);
	return new_c_space_milestone;
}

CSpaceBlackBoardMilestone& CSpaceBlackBoardMilestone::operator=(const CSpaceBlackBoardMilestone & cmilestone_assignment)
{
	Milestone::operator=(cmilestone_assignment);
	this->configuration_ = cmilestone_assignment.configuration_;
	if(cmilestone_assignment.motion_behaviour_)
	{
		this->motion_behaviour_ = cmilestone_assignment.motion_behaviour_->clone();
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}


	this->region_convergence_radius_ = cmilestone_assignment.region_convergence_radius_;
	this->handle_points_ = cmilestone_assignment.handle_points_;
	return *this;
}

void CSpaceBlackBoardMilestone::setBlackBoardVariableName(const std::string& name)
{
	black_board_variable_name_ = name;
	black_board_->subscribeToROSMessage(name);
}

std::string CSpaceBlackBoardMilestone::getBlackBoardVariableName() {
	return black_board_variable_name_;
}

bool CSpaceBlackBoardMilestone::hasConverged(rxSystem* sys) 
{
	assert (sys != NULL);

	update();

	const dVector& q = sys->q();
	for (int i = 0; i < sys->jdof(); i++) {
		double e = ::std::abs(q[i] - configuration_[i]);
		if (e > region_convergence_radius_[i])
		{
#ifdef NOT_IN_RT
			std::cout << "Error in " << i << " is too large = " << e << std::endl;
#endif
			return false;
		}
	}
	return true;
}