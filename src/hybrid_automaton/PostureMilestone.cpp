#include "PostureMilestone.h"

PostureMilestone::PostureMilestone() :
OpSpaceMilestone()
{
}

PostureMilestone::PostureMilestone(std::string osm_name) :
OpSpaceMilestone(osm_name)
{
}

PostureMilestone::PostureMilestone(std::string osm_name, const dVector configuration, PosiOriSelector posi_ori_selection, 
								   MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
								   std::vector<Point> handle_points, rxSystem* sys):
OpSpaceMilestone(osm_name, Displacement(), Rotation(), posi_ori_selection, motion_behaviour, region_convergence_radius, status, handle_points),
_sys(sys)
{
	this->setConfiguration(configuration);
}

PostureMilestone::PostureMilestone(std::string osm_name,  Displacement position, Rotation orientation, const dVector configuration, PosiOriSelector posi_ori_selection, 
								   MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
								   std::vector<Point> handle_points, rxSystem* sys):
OpSpaceMilestone(osm_name, position, orientation, posi_ori_selection, motion_behaviour, region_convergence_radius, status, handle_points),
_sys(sys),
_configuration(configuration)
{

}

PostureMilestone::PostureMilestone(const PostureMilestone& op_milestone_cpy):
OpSpaceMilestone(op_milestone_cpy),
_configuration(op_milestone_cpy._configuration),
_sys(op_milestone_cpy._sys)
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

PostureMilestone::~PostureMilestone()
{	
	// Do not delete anything but the MotionBehaviour, it is the only thing created/copied in the constructor
	delete motion_behaviour_;
	motion_behaviour_ = NULL;
}

PostureMilestone* PostureMilestone::clone() const
{
	PostureMilestone* new_op_space_milestone = new PostureMilestone(*this);
	return new_op_space_milestone;
}

PostureMilestone& PostureMilestone::operator=(const PostureMilestone & pos_milestone_assignment)
{
	Milestone::operator=(pos_milestone_assignment);
	if(pos_milestone_assignment.motion_behaviour_)
	{
		this->motion_behaviour_ = pos_milestone_assignment.motion_behaviour_->clone();
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
	this->region_convergence_radius_ = pos_milestone_assignment.region_convergence_radius_;
	//this->object_id_ = pos_milestone_assignment.getObjectId();
	this->handle_points_ = pos_milestone_assignment.handle_points_;
	this->posi_ori_selection_ = pos_milestone_assignment.getPosiOriSelector();
	this->position_ = pos_milestone_assignment.position_;
	this->orientation_ = pos_milestone_assignment.orientation_;
	this->_configuration=pos_milestone_assignment._configuration;
	this->_sys=pos_milestone_assignment._sys;
	return *this;
}


dVector PostureMilestone::getConfiguration() const
{
	return _configuration;
}

void PostureMilestone::setConfiguration(dVector configuration)
{
	_configuration = configuration;

	//Forward kinematics
	HTransform ht;

	rxBody* EE = _sys->getUCSBody(_T("EE"),ht);
	dVector current_r = (EE->T()*ht).r;
	this->setPosition(current_r);
	this->setPosiOriSelector(POSITION_SELECTION);

}
