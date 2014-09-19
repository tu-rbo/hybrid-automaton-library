#include "ForceTorqueMilestone.h"

ForceTorqueMilestone::ForceTorqueMilestone() :
OpSpaceMilestone()
{
}

ForceTorqueMilestone::ForceTorqueMilestone(const std::string& osm_name) :
OpSpaceMilestone(osm_name)
{
}

ForceTorqueMilestone::ForceTorqueMilestone(const std::string& osm_name, const dVector& forceTorque, PosiOriSelector posi_ori_selection, 
								   MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
								   std::vector<Point> handle_points, rxSystem* sys):
OpSpaceMilestone(osm_name, Displacement(), Rotation(), posi_ori_selection, motion_behaviour, region_convergence_radius, status, handle_points),
forceTorque_(forceTorque)
{
	forceSensor_ = sys->findDevice(_T("FT_SENSOR"));
	if (!forceSensor_)
		std::cout << "Force sensor missing!" << std::endl;
}

ForceTorqueMilestone::ForceTorqueMilestone(const std::string& osm_name, Displacement position, Rotation orientation, const std::string& frame_id, const dVector& forceTorque, PosiOriSelector posi_ori_selection, 
								   MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
								   std::vector<Point> handle_points, rxSystem* sys):
OpSpaceMilestone(osm_name, position, orientation, frame_id, posi_ori_selection, motion_behaviour, region_convergence_radius, status, handle_points),
forceTorque_(forceTorque)
{
	forceSensor_ = sys->findDevice(_T("FT_SENSOR"));
	if (!forceSensor_)
		std::cout << "Force sensor missing!" << std::endl;
}

ForceTorqueMilestone::ForceTorqueMilestone(const ForceTorqueMilestone& ft_milestone_cpy):
OpSpaceMilestone(ft_milestone_cpy),
forceTorque_(ft_milestone_cpy.forceTorque_),
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
	this->forceTorque_=ft_milestone_assignment.forceTorque_;
	this->forceSensor_=ft_milestone_assignment.forceSensor_;
	return *this;
}


dVector ForceTorqueMilestone::getForceTorque() const
{
	return forceTorque_;
}

void ForceTorqueMilestone::setForceTorque(const dVector& forceTorque)
{
	forceTorque_ = forceTorque;
}

bool ForceTorqueMilestone::hasConverged(rxSystem* sys) 
{
	//if(!OpSpaceMilestone::hasConverged(sys))
	//	return false;
	
	dVector currentFTMeasurement(6);
	forceSensor_->readDeviceValue(&currentFTMeasurement[0], sizeof(double)*6);
	
	bool converged = false;

	static int dbg_cntr = 0;
	for(int i = 0; i < 3; ++i)
	{
		double e = ::std::abs(currentFTMeasurement[i] - forceTorque_[i]);
		
		if (dbg_cntr % 250 == 0) {
			std::cout << currentFTMeasurement[i] << " ";
			if (i == 2) {
				std::cout << std::endl;
			}
		}
		if (currentFTMeasurement[2] < forceTorque_[2])
		{
			converged = true;
			std::cout << "Converged: " << currentFTMeasurement[0] << " " << currentFTMeasurement[1] << " " << currentFTMeasurement[2] << std::endl;
		}
	}

	dbg_cntr++;

	return converged;
}
