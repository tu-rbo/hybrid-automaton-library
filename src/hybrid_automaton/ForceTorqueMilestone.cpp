#include "ForceTorqueMilestone.h"

ForceTorqueMilestone::ForceTorqueMilestone() :
OpSpaceMilestone()
{
}

ForceTorqueMilestone::ForceTorqueMilestone(const std::string& osm_name) :
OpSpaceMilestone(osm_name)
{
}

ForceTorqueMilestone::ForceTorqueMilestone(const std::string& osm_name,
										   const dVector& minForceTorque, const dVector& maxForceTorque,
										   PosiOriSelector posi_ori_selection, MotionBehaviour * motion_behaviour,
										   std::vector<double>& region_convergence_radius, Milestone::Status status, 
										   std::vector<Point> handle_points, rxSystem* sys):
OpSpaceMilestone(osm_name, Displacement(), Rotation(), posi_ori_selection, motion_behaviour, region_convergence_radius, status, handle_points),
minForceTorque_(minForceTorque), maxForceTorque_(maxForceTorque)
{
	forceSensor_ = sys->findDevice(_T("FT_SENSOR"));
	if (!forceSensor_)
		std::cout << "Force sensor missing!" << std::endl;
}

ForceTorqueMilestone::ForceTorqueMilestone(const std::string& osm_name, Displacement position, Rotation orientation, const std::string& frame_id,
										   const dVector& minForceTorque, const dVector& maxForceTorque,
										   PosiOriSelector posi_ori_selection, MotionBehaviour * motion_behaviour,
										   std::vector<double>& region_convergence_radius, Milestone::Status status,
										   std::vector<Point> handle_points, rxSystem* sys):
OpSpaceMilestone(osm_name, position, orientation, frame_id, posi_ori_selection, motion_behaviour, region_convergence_radius, status, handle_points),
minForceTorque_(minForceTorque), maxForceTorque_(maxForceTorque)
{
	forceSensor_ = sys->findDevice(_T("FT_SENSOR"));
	if (!forceSensor_)
		std::cout << "Force sensor missing!" << std::endl;
}

ForceTorqueMilestone::ForceTorqueMilestone(const ForceTorqueMilestone& ft_milestone_cpy):
OpSpaceMilestone(ft_milestone_cpy),
minForceTorque_(ft_milestone_cpy.minForceTorque_),
maxForceTorque_(ft_milestone_cpy.maxForceTorque_),
forceSensor_(ft_milestone_cpy.forceSensor_)
{
	if(ft_milestone_cpy.motion_behaviour_)
	{
		this->motion_behaviour_ = ft_milestone_cpy.motion_behaviour_->clone();
		this->motion_behaviour_->setChild(this);
		this->motion_behaviour_->setParent(this);
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
}

ForceTorqueMilestone::~ForceTorqueMilestone()
{	
	// Do not delete anything but the MotionBehaviour, it is the only thing created/copied in the constructor
	delete motion_behaviour_;
	motion_behaviour_ = NULL;
}

ForceTorqueMilestone* ForceTorqueMilestone::clone() const
{
	ForceTorqueMilestone* new_op_space_milestone = new ForceTorqueMilestone(*this);
	return new_op_space_milestone;
}

ForceTorqueMilestone& ForceTorqueMilestone::operator=(const ForceTorqueMilestone & ft_milestone_assignment)
{
	Milestone::operator=(ft_milestone_assignment);
	if(ft_milestone_assignment.motion_behaviour_)
	{
		this->motion_behaviour_ = ft_milestone_assignment.motion_behaviour_->clone();
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
	this->region_convergence_radius_ = ft_milestone_assignment.region_convergence_radius_;
	//this->object_id_ = ft_milestone_assignment.getObjectId();
	this->handle_points_ = ft_milestone_assignment.handle_points_;
	this->posi_ori_selection_ = ft_milestone_assignment.getPosiOriSelector();
	this->position_ = ft_milestone_assignment.position_;
	this->orientation_ = ft_milestone_assignment.orientation_;
	this->frame_id_ = ft_milestone_assignment.frame_id_;
	this->minForceTorque_ = ft_milestone_assignment.minForceTorque_;
	this->maxForceTorque_ = ft_milestone_assignment.maxForceTorque_;
	this->forceSensor_ = ft_milestone_assignment.forceSensor_;
	return *this;
}


dVector ForceTorqueMilestone::getMinForceTorque() const
{
	return minForceTorque_;
}

dVector ForceTorqueMilestone::getMaxForceTorque() const
{
	return maxForceTorque_;
}

void ForceTorqueMilestone::setMinForceTorque(const dVector& minForceTorque)
{
	minForceTorque_ = minForceTorque;
}

void ForceTorqueMilestone::setMaxForceTorque(const dVector& maxForceTorque)
{
	maxForceTorque_ = maxForceTorque;
}

bool ForceTorqueMilestone::hasConverged(rxSystem* sys) 
{
	//if(!OpSpaceMilestone::hasConverged(sys))
	//	return false;
	
	dVector currentFTMeasurement(6);
	forceSensor_->readDeviceValue(&currentFTMeasurement[0], sizeof(double)*6);

	static int dbg_cntr = 0;
	if (dbg_cntr++ % 250 == 0) {
		currentFTMeasurement.print(_T("FTMilestone, current f/t:"));
	}
	

	for(int i = 0; i < 6; ++i)
	{
		if (currentFTMeasurement[i] < minForceTorque_[i] || currentFTMeasurement[i] > maxForceTorque_[i])
			return false;
	}

	currentFTMeasurement.print(_T("FTMilestone CONVERGED!, current f/t:"));
	return true;
}
