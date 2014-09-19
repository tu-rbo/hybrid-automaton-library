
#ifndef FORCETORQUE_MILESTONE_
#define FORCETORQUE_MILESTONE_

#include "rMath/rMath.h"

#include "OpSpaceMilestone.h"
#include "MotionBehaviour.h"

#include <vector>

/**
* This is an OpSpaceMilestone with an attached goal posture.
* This is needed for the Elastic Roadmap.
*/
class ForceTorqueMilestone : public OpSpaceMilestone
{

public:
	ForceTorqueMilestone();
	
	ForceTorqueMilestone(const std::string& osm_name);

	ForceTorqueMilestone(const std::string& osm_name, const dVector& forceTorque, PosiOriSelector posi_ori_selection, 
		MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
		std::vector<Point> handle_points, rxSystem* _sys);

	ForceTorqueMilestone(const std::string& osm_name,  Displacement position, Rotation orientation, const std::string& frame_id, const dVector& forceTorque, PosiOriSelector posi_ori_selection, 
								   MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
								   std::vector<Point> handle_points, rxSystem* sys);

	virtual ~ForceTorqueMilestone();

	ForceTorqueMilestone(const ForceTorqueMilestone&  op_milestone_cpy);
	virtual ForceTorqueMilestone& operator=(const ForceTorqueMilestone & ft_milestone_assignment);
	ForceTorqueMilestone* ForceTorqueMilestone::clone() const;

	//Like opSpaceMilestone::hasConverged but checks also for base position in 10 dof chain
	virtual bool hasConverged(rxSystem* sys); 

	// gets the force-torque
	dVector getForceTorque() const;

	// Sets the force-torque
	void setForceTorque(const dVector& forceTorque);
	
	virtual std::string toStringXML() const;
	virtual TiXmlElement* toElementXML() const ;

protected:
	rxDevice*   forceSensor_;
	dVector		forceTorque_;
};

#endif // FORCETORQUE_MILESTONE_