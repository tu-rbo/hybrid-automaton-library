#include "OpSpaceMilestone.h"

#include <iostream>
#include <string>

#include "tinyxml.h"
#include "XMLDeserializer.h"
#include "Quaternion.h"

OpSpaceMilestone::OpSpaceMilestone() :
Milestone("Default"),
motion_behaviour_(NULL),
posi_ori_selection_(POSITION_SELECTION),	// Default value in the default constructor
frame_id_("")
{
}

OpSpaceMilestone::OpSpaceMilestone(std::string osm_name) :
Milestone(osm_name),
motion_behaviour_(NULL),
posi_ori_selection_(POSITION_SELECTION),	// Default value in the default constructor
frame_id_("")
{
}

OpSpaceMilestone::OpSpaceMilestone(std::string osm_name, const Displacement& position, const Rotation& orientation, PosiOriSelector posi_ori_selection, MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius) :
Milestone(osm_name),
posi_ori_selection_(posi_ori_selection),
position_(position),
orientation_(orientation),
region_convergence_radius_(region_convergence_radius),
frame_id_(""),
position_relative_(position),
orientation_relative_(orientation)
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
}

OpSpaceMilestone::OpSpaceMilestone(std::string osm_name, const Displacement& position, const Rotation& orientation, PosiOriSelector posi_ori_selection, 
								   MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
								   std::vector<Point> handle_points):
Milestone(osm_name),
posi_ori_selection_(posi_ori_selection),
position_(position),
orientation_(orientation),
region_convergence_radius_(region_convergence_radius),
handle_points_(handle_points),
frame_id_(""),
position_relative_(position),
orientation_relative_(orientation)
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

OpSpaceMilestone::OpSpaceMilestone(std::string osm_name, const Displacement& position, const Rotation& orientation, const std::string& frame_id, PosiOriSelector posi_ori_selection, 
		MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
		std::vector<Point> handle_points):
Milestone(osm_name),
posi_ori_selection_(posi_ori_selection),
position_(position),
orientation_(orientation),
region_convergence_radius_(region_convergence_radius),
handle_points_(handle_points),
frame_id_(frame_id),
position_relative_(position),
orientation_relative_(orientation)
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

OpSpaceMilestone::OpSpaceMilestone(const OpSpaceMilestone & op_milestone_cpy) :
Milestone(op_milestone_cpy),
region_convergence_radius_(op_milestone_cpy.region_convergence_radius_),
handle_points_(op_milestone_cpy.handle_points_),
posi_ori_selection_(op_milestone_cpy.posi_ori_selection_),
position_(op_milestone_cpy.position_),
orientation_(op_milestone_cpy.orientation_),
position_relative_(op_milestone_cpy.position_relative_),
orientation_relative_(op_milestone_cpy.orientation_relative_),
frame_id_(op_milestone_cpy.frame_id_)
{
	if(op_milestone_cpy.motion_behaviour_)
	{
		this->motion_behaviour_ = op_milestone_cpy.motion_behaviour_->clone();
		this->motion_behaviour_->setChild(this);
		this->motion_behaviour_->setParent(this);
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
}

OpSpaceMilestone::~OpSpaceMilestone()
{	
	// Do not delete anything but the MotionBehaviour, it is the only thing created/copied in the constructor
	delete motion_behaviour_;
	motion_behaviour_ = NULL;
}

void OpSpaceMilestone::setMotionBehaviour(MotionBehaviour * motion_behaviour)
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

Displacement OpSpaceMilestone::getPosition() const
{
	return this->position_;
}

Rotation OpSpaceMilestone::getOrientation() const
{
	return this->orientation_;
}

std::string OpSpaceMilestone::getFrame() const
{
	return this->frame_id_;
}

void OpSpaceMilestone::activate(rxSystem* sys)
{
	this->activated_ = true;

	if (!frame_id_.empty())
	{
		// transform pose
		HTransform transform;
		std::wstring wide(this->frame_id_.begin(), this->frame_id_.end());
		rxBody* frame = sys->getUCSBody(wide, transform);

		if (!frame)
		{
			std::cout << "Frame: " << frame_id_ << " doesn't exist! Using global frame." << std::endl;
		}
		else
		{
			HTransform goal_relative(this->orientation_relative_, this->position_relative_);
			HTransform absolute_transform = frame->T() * transform * goal_relative;

			absolute_transform.Print(_T("old "));
			
			/*
			HTransform absolute_transform2 = goal_relative * (frame->T() * transform);
			absolute_transform2.Print(_T("new2 "));
			*/
			
			this->position_ = absolute_transform.r;
			this->orientation_ = absolute_transform.R;
		}
	}
}

void OpSpaceMilestone::update(const rTime& t)
{
}

void OpSpaceMilestone::deactivate()
{
	this->activated_ = false;
}

OpSpaceMilestone* OpSpaceMilestone::clone() const
{
	OpSpaceMilestone* new_op_space_milestone = new OpSpaceMilestone(*this);
	return new_op_space_milestone;
}

OpSpaceMilestone& OpSpaceMilestone::operator=(const OpSpaceMilestone & op_milestone_assignment)
{
	Milestone::operator=(op_milestone_assignment);
	if(op_milestone_assignment.motion_behaviour_)
	{
		this->motion_behaviour_ = op_milestone_assignment.motion_behaviour_->clone();
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
	this->region_convergence_radius_ = op_milestone_assignment.region_convergence_radius_;
	//this->object_id_ = op_milestone_assignment.getObjectId();
	this->handle_points_ = op_milestone_assignment.handle_points_;
	this->posi_ori_selection_ = op_milestone_assignment.getPosiOriSelector();
	this->position_ = op_milestone_assignment.position_;
	this->orientation_ = op_milestone_assignment.orientation_;
	this->position_relative_ = op_milestone_assignment.position_relative_;
	this->orientation_relative_ = op_milestone_assignment.orientation_relative_;
	this->frame_id_ = op_milestone_assignment.frame_id_;
	return *this;
}

Displacement OpSpaceMilestone::getHandlePoint(int i) const
{
	return Displacement(this->handle_points_[i].x, this->handle_points_[i].y, this->handle_points_[i].z);
}

int OpSpaceMilestone::getHandlePointNumber() const
{
	return handle_points_.size();
}

PosiOriSelector OpSpaceMilestone::getPosiOriSelector() const
{
	return posi_ori_selection_;
}

bool OpSpaceMilestone::hasConverged(rxSystem* sys) 
{
	assert(sys != NULL);
	
	HTransform ht;
	rxBody* EE = sys->getUCSBody(_T("EE"),ht);
	HTransform ee_pose = EE->T() * ht;

	if (posi_ori_selection_ == POSITION_SELECTION
		|| posi_ori_selection_ == POS_AND_ORI_SELECTION)
	{
		Displacement current_r = ee_pose.r - this->position_; //parent->getConfiguration();
		
/*		static int dbg_counter = 0;
		if (dbg_counter++ % 300 == 0)
			current_r.Print(_T("Error in displacement"));*/

		if ( ::std::fabs(current_r[0]) > region_convergence_radius_[0] ||
			 ::std::fabs(current_r[1]) > region_convergence_radius_[1] ||
			 ::std::fabs(current_r[2]) > region_convergence_radius_[2] )
			 return false;
	}

	if (posi_ori_selection_ == ORIENTATION_SELECTION
		|| posi_ori_selection_ == POS_AND_ORI_SELECTION)
	{
		// desired orientation
		//Rotation Rd(configuration_[3], configuration_[4], configuration_[5]);

		dVector quatd;
		orientation_.GetQuaternion(quatd);
		Quaternion qd(quatd);

		// current orientation
		dVector quat;
		ee_pose.R.GetQuaternion(quat);
		Quaternion q(quat);

		q.invert();
		Quaternion qres = q*qd;
		dVector axis;
		double angle;
		qres.to_axis_angle(axis, angle);

		double e = ::std::abs(angle);
		if (e > region_convergence_radius_[3]) 
		{
			return false;
		}
	}

	return true;
}