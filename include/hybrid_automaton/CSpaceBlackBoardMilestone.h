#ifndef C_SPACE_BLACKBOARD_MILESTONE_
#define C_SPACE_BLACKBOARD_MILESTONE_

#include "Milestone.h"
#include "MotionBehaviour.h"


#include "ros_bridge/include/RTBlackBoard.h"
#include <vector>

/**
* A configuration-space milestone is one type of milestone that defines a goal in the robot's joint space.
* It is a node in the graph represented by the hybrid automaton.
*/
class CSpaceBlackBoardMilestone : public Milestone
{

public:

	/**
	* Creates an empty milestone with name "default"
	*/
	CSpaceBlackBoardMilestone();  

	/**
	* Creates an empty milestone with a name.
	* \param name the name of the milestone.
	*/
	CSpaceBlackBoardMilestone(const std::string& name);

	/**
	* Creates a milestone with a goal configuration and ball of convergence.
	* \param name the name of the milestone.
	* \param configuration the goal configuration of the milestone.
	* \param motion_behaviour the motion_behaviour (edge) that is associated with this milestone (node).
	* \param region_convergence_radius the radius of the ball of convergence around the goal; same dimensionality as configuration.
	*/
	CSpaceBlackBoardMilestone(const std::string& name,
		const std::vector<double>& configuration,
		MotionBehaviour * motion_behaviour,
		const std::vector<double>& region_convergence_radius);

	/**
	* Creates a milestone with a goal configuration and ball of convergence.
	* \param name the name of the milestone.
	* \param configuration the goal configuration of the milestone.
	* \param motion_behaviour the motion_behaviour (edge) that is associated with this milestone (node).
	* \param region_convergence_radius the radius of the ball of convergence around the goal; same dimensionality as configuration.
	* \param status
	* \param handle_points
	*/
	CSpaceBlackBoardMilestone(const std::string& name,
		const std::vector<double>& configuration,
		MotionBehaviour * motion_behaviour,
		const std::vector<double>& region_convergence_radius,
		Milestone::Status status,
		const std::vector<Point>& handle_points);

	/**
	* Copy constructor.
	*/
	CSpaceBlackBoardMilestone(const CSpaceBlackBoardMilestone& cmilestone_cpy);

	virtual CSpaceBlackBoardMilestone* clone() const;

	virtual CSpaceBlackBoardMilestone& operator=(const CSpaceBlackBoardMilestone & cmilestone_assignment);

	virtual ~CSpaceBlackBoardMilestone();

	void setMotionBehaviour(MotionBehaviour * motion_behaviour);

	/**
	* Returns the goal configuration of this milestone.
	* \return the desired configuration.
	*/
	std::vector<double> getConfigurationSTDVector() const;
	
	/**
	* Returns the goal configuration of this milestone.
	* \return the desired configuration.
	*/
	virtual dVector getConfiguration() const;

	/**
	* Checks if the robot fulfills the convergence criterion; i.e. its configuration is inside the region of convergence of this milestone.
	* \param sys the robot whose configuration will be used.
	* \return true, if |sys->q() - configuration| < region_convergence_radius, false otherwise
	*/
	virtual bool hasConverged(rxSystem* sys);

	/**
	* Milestones can change in real time. This should be considered here.
	* Does nothing up to now.
	*/
	virtual void update(const rTime& t);

	void setBlackBoardVariableName(const std::string& name);
	std::string getBlackBoardVariableName();

	void addHandlePoint( const Point & point_to_add );
	
	virtual std::string toStringXML() const;
	virtual TiXmlElement* toElementXML() const;

protected:

	std::vector<double>		configuration_;
	MotionBehaviour*		motion_behaviour_;
	std::vector<double>		region_convergence_radius_;
	std::vector<Point>		handle_points_;

	
	RTBlackBoard*			black_board_;
	std::string				black_board_variable_name_;
};

#endif // C_SPACE_BLACKBOARD_MILESTONE_