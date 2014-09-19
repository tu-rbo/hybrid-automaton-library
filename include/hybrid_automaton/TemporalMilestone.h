#ifndef TEMPORAL_MILESTONE_
#define TEMPORAL_MILESTONE_

#include "Milestone.h"

/**
* A configuration-space milestone is one type of milestone that defines a goal in the robot's joint space.
* It is a node in the graph represented by the hybrid automaton.
*/
class TemporalMilestone : public Milestone
{

public:

	/**
	* Creates an empty milestone with name "default"
	*/
	TemporalMilestone();  

	/**
	* Creates an empty milestone with a name.
	* \param name the name of the milestone.
	*/
	TemporalMilestone(const std::string& name);

	/**
	* Creates an empty milestone with a name.
	* \param name the name of the milestone.
	*/
	TemporalMilestone(const std::string& name, const rTime& duration);

	/**
	* Copy constructor.
	*/
	TemporalMilestone(const TemporalMilestone& cmilestone_cpy);
	virtual TemporalMilestone* clone() const;
	virtual TemporalMilestone& operator=(const TemporalMilestone & cmilestone_assignment);
	virtual ~TemporalMilestone();

	/**
	* Returns the duration of this milestone.
	* \return the desired duration.
	*/
	rTime getDuration() const;
	rTime getDesiredTime() const;

	/**
	* Checks if the robot fulfills the convergence criterion; i.e. its configuration is inside the region of convergence of this milestone.
	* \param sys the robot whose configuration will be used.
	* \return true, if |sys->q() - configuration| < region_convergence_radius, false otherwise
	*/
	virtual bool hasConverged(rxSystem* sys);

	/**
	* Updates the timing.
	*/
	virtual void activate(rxSystem* sys);
	virtual void update(const rTime& t);
	virtual void deactivate();
	
	virtual std::string toStringXML() const;
	virtual TiXmlElement* toElementXML() const;

protected:
	rTime					t_;
	rTime					duration_;
	rTime					desiredT_;
	bool					firstUpdateSinceActivated_;
};

#endif // TEMPORAL_MILESTONE_