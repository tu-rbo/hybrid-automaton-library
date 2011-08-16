
#ifndef C_SPACE_MILESTONE_
#define C_SPACE_MILESTONE_

#include "elasticroadmap\include\Milestone.h"
#include "elasticroadmap\include\MotionBehaviour.h"

#include <vector>

typedef struct Point
{
	double x;
	double y;
	double z;

	Point(double X, double Y, double Z)
	{
		x = X;
		y = Y;
		z = Z;
	}

	bool operator ==(const Point & p) const 
	{
		return (this->x==p.x && this->y==p.y && this->z==p.z );
	}

	bool operator ==(const Point & p) 
	{
		return (this->x==p.x && this->y==p.y && this->z==p.z );
	}


}point;

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

	std::vector<double> getConfiguration() const;

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