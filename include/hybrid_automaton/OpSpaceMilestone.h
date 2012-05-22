
#ifndef OP_SPACE_MILESTONE_
#define OP_SPACE_MILESTONE_

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

	OpSpaceMilestone(std::string osm_name, std::vector<double>& posi_ori_value, PosiOriSelector posi_ori_selection, MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius);

	OpSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot, double dt);

	OpSpaceMilestone(const OpSpaceMilestone & op_milestone_cpy);

	virtual ~OpSpaceMilestone();

	void setMotionBehaviour(MotionBehaviour * motion_behaviour);

	void setConfigurationSTDVector(std::vector<double> configuration_in, PosiOriSelector posi_ori_selector);

	std::vector<double> getConfigurationSTDVector() const;

	void setConfiguration(dVector configuration_in, PosiOriSelector posi_ori_selector);

	virtual dVector getConfiguration() const;

	//int getObjectId() const;

	//int getDofs() const;

	void update();

	virtual std::string toStringXML() const;

	virtual TiXmlElement* toElementXML() const ;

	virtual OpSpaceMilestone* clone() const;

	void addHandlePoint( const Point & point_to_add );

	//virtual bool operator ==(const OpSpaceMilestone & n) const ;

	//virtual bool operator ==(const Milestone & n) const ;

	//virtual bool operator !=(const OpSpaceMilestone & n) const ;

	//virtual bool operator !=(const Milestone & n) const ;

	virtual OpSpaceMilestone& operator=(const OpSpaceMilestone & op_milestone_assignment);

	virtual PosiOriSelector getPosiOriSelector() const;

	virtual bool hasConverged(rxSystem* sys);

protected:

	std::vector<double>		configuration_;						// Position = 3 first values
																// Orientation = 3 second values 
																//		(Euler angles ZYX)
																// Total size = 6
	MotionBehaviour*		motion_behaviour_;
	std::vector<double>		region_convergence_radius_;			// Position = 3 first values
																// Orientation = 1 second value
																// Total size = 4
	//int						object_id_;
	std::vector<Point>		handle_points_;
	PosiOriSelector			posi_ori_selection_;				// Position/Orientation/Both values defined in this Milestone

};

#endif // OP_SPACE_MILESTONE_

//virtual bool operator ==(const Milestone * n) const ;
//virtual bool operator ==(const OpSpaceMilestone * n) const ;
//virtual bool operator ==(OpSpaceMilestone * n) const ;