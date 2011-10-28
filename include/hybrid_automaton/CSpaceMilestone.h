
#ifndef C_SPACE_MILESTONE_
#define C_SPACE_MILESTONE_

#include "Milestone.h"
#include "MotionBehaviour.h"

#include <vector>

class CSpaceMilestone : public Milestone
{

public:

	CSpaceMilestone();  

	CSpaceMilestone(int dofs);  

	CSpaceMilestone(int dofs, std::vector<double>& configuration, MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius,int object_id);

	CSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot);

	CSpaceMilestone(const CSpaceMilestone & cmilestone_cpy);

	virtual ~CSpaceMilestone();

	void setMotionBehaviour(MotionBehaviour * motion_behaviour);

	std::vector<double> getConfigurationSTDVector() const;

	int getObjectId() const;

	int getDofs() const;

	void update();

	virtual std::string toStringXML() const;

	virtual void toElementXML(TiXmlElement* cspace_milestone_xml) const ;

	virtual CSpaceMilestone* clone() const;

	void addHandlePoint( const Point & point_to_add );

	virtual bool operator ==(const CSpaceMilestone & n) const ;

	virtual bool operator ==(const Milestone & n) const ;

	virtual bool operator !=(const CSpaceMilestone & n) const ;

	virtual bool operator !=(const Milestone & n) const ;

	virtual CSpaceMilestone& operator=(const CSpaceMilestone & cmilestone_assignment);

	virtual dVector getConfiguration() const;

protected:

	int						dofs_;
	std::vector<double>		configuration_;
	MotionBehaviour*		motion_behaviour_;
	std::vector<double>		region_convergence_radius_;
	int						object_id_;
	std::vector<Point>		handle_points_;

};

#endif // C_SPACE_MILESTONE_

//virtual bool operator ==(const Milestone * n) const ;
//virtual bool operator ==(const CSpaceMilestone * n) const ;
//virtual bool operator ==(CSpaceMilestone * n) const ;