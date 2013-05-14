
#ifndef OP_SPACE_MILESTONE_
#define OP_SPACE_MILESTONE_

#include "rMath/rMath.h"

#include "Milestone.h"
#include "MotionBehaviour.h"

#include <vector>

typedef enum PosiOriSelector{
	POSITION_SELECTION = 0,
	RELATIVE_POSITION_SELECTION = 1,
	ORIENTATION_SELECTION = 2,
	RELATIVE_ORIENTATION_SELECTION = 3,
	POS_AND_ORI_SELECTION = 4,
	RELATIVE_POS_AND_ORI_SELECTION = 5
};

class OpSpaceMilestone : public Milestone
{

public:
	OpSpaceMilestone();
	
	OpSpaceMilestone(std::string osm_name);

	OpSpaceMilestone(std::string osm_name, const Displacement& position, const Rotation& orientation, PosiOriSelector posi_ori_selection, 
		MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
		std::vector<Point> handle_points);

	OpSpaceMilestone(std::string osm_name, const Displacement& position, const Rotation& orientation, PosiOriSelector posi_ori_selection, 
		MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius);

	virtual ~OpSpaceMilestone();

	OpSpaceMilestone(const OpSpaceMilestone & op_milestone_cpy);
	virtual OpSpaceMilestone& operator=(const OpSpaceMilestone & op_milestone_assignment);

	void setMotionBehaviour(MotionBehaviour * motion_behaviour);

	Displacement getPosition() const;
	Rotation getOrientation() const;

	void setPosition(const Displacement pos){position_=pos;};
	void setOrientation(const Rotation ori){orientation_=ori;};

	virtual PosiOriSelector getPosiOriSelector() const;
	void setPosiOriSelector(const PosiOriSelector sel){posi_ori_selection_=sel;};

	void update();

	virtual std::string toStringXML() const;
	virtual TiXmlElement* toElementXML() const ;

	virtual OpSpaceMilestone* clone() const;

	void addHandlePoint( const Point & point_to_add );
	virtual Displacement getHandlePoint(int i) const;
	virtual int getHandlePointNumber() const;

	virtual bool hasConverged(rxSystem* sys);

protected:

	MotionBehaviour*		motion_behaviour_;
	std::vector<double>		region_convergence_radius_;			// Position = 3 first values
																// Orientation = 1 second value
																// Total size = 4
	std::vector<Point>		handle_points_;
	PosiOriSelector			posi_ori_selection_;				// Position/Orientation/Both values defined in this Milestone

	Displacement			position_;							// Milestone goal position
	Rotation				orientation_;						// Milestone goal orientation
};

#endif // OP_SPACE_MILESTONE_

//virtual bool operator ==(const Milestone * n) const ;
//virtual bool operator ==(const OpSpaceMilestone * n) const ;
//virtual bool operator ==(OpSpaceMilestone * n) const ;