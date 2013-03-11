
#ifndef POSTURE_MILESTONE_
#define POSTURE_MILESTONE_

#include "rMath/rMath.h"

#include "OpSpaceMilestone.h"
#include "MotionBehaviour.h"

#include <vector>

/**
* This is an OpSpaceMilestone with an attached goal posture.
* This is needed for the Elastic Roadmap.
*/
class PostureMilestone : public OpSpaceMilestone
{

public:
	PostureMilestone();
	
	PostureMilestone(std::string osm_name);

	PostureMilestone(std::string osm_name, dVector configuration, PosiOriSelector posi_ori_selection, 
		MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
		std::vector<Point> handle_points, rxSystem* _sys);

	PostureMilestone(std::string osm_name,  Displacement position, Rotation orientation, const dVector configuration, PosiOriSelector posi_ori_selection, 
								   MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
								   std::vector<Point> handle_points, rxSystem* sys);

	virtual ~PostureMilestone();

	PostureMilestone(const PostureMilestone&  op_milestone_cpy);
	virtual PostureMilestone& operator=(const PostureMilestone & op_milestone_assignment);
	PostureMilestone* PostureMilestone::clone() const;

	// gets the c-space configuration
	dVector getCConfiguration() const;

	// Sets the c-space configuration (and the corresponding opspace configuration)
	void setCConfiguration(dVector configuration);
	
	virtual std::string toStringXML() const;
	virtual TiXmlElement* toElementXML() const ;

protected:

	rxSystem*	_sys;				//neds a system pointer
	dVector		_configuration;

};

#endif // POSTURE_MILESTONE