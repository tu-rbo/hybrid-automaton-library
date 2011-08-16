#ifndef MOTION_BEHAVIOUR
#define MOTION_BEHAVIOUR

#include "Edge.h"
#include "Milestone.h"
#include "rControlalgorithm\rControlalgorithm.h"
#include "rxControlSDK\rxControlSDK.h"

#include "tinyxml.h"

class MotionBehaviour: public Edge<Milestone> 
{

public:

	/**
	* Constructor
	*/
	MotionBehaviour();

	/**
	* Constructor
	* @param dad Pointer to the parent milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param son Pointer to the child milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param robot Pointer to the RLab system object.
	* @param weight Weight of the edge in the graph
	* @param dt Control interval
	*/
	MotionBehaviour(const Milestone * dad, const Milestone * son, rxSystem* robot, double weight = 1.0, double dt = 0.002);

	/**
	* Constructor
	* @param motion_behaviour_xml TinyXML element containing the information to (re)construct a new MotionBehaviour - usually transmitted over a network.
	* @param dad Pointer to the parent milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param son Pointer to the child milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param robot Pointer to the RLab system object.
	*/
	MotionBehaviour(TiXmlElement* motion_behaviour_xml , const Milestone *dad, const Milestone *son , rxSystem* robot );

	/**
	* Copy constructor - Create a new MotionBehaviour that is a copy of the MotionBehaviour given as parameter
	* @param motion_behaviour_copy MotionBehaviour object that will be copied
	*/
	MotionBehaviour(const MotionBehaviour & motion_behaviour_copy);

	/**
	* Destructor
	*/
	virtual ~MotionBehaviour();

	/**
	* Add a pointer to a rxController to the internal set of controllers. 
	* It will be added to both the vector of pointers to rxController's and the rxControlSet.
	* @param ctrl Pointer to the rxController object to be added.
	*/
	void addController(rxController* ctrl);

	/**
	* Activate the controllers stored in the rxControlSet through the vector of rxController's (NOT WORKING!!!!)
	*/
	void activate();

	/**
	* Deactivate the controllers stored in the rxControlSet
	*/
	void deactivate();

	/**
	* Check if the motion has been succesfully executed until the goal point
	*/
	bool hasConverged();

	/**
	* Activate the controllers stored in the rxControlSet through the vector of rxController's (NOT WORKING!!!!)
	*/
	dVector update(double t);

	/**
	* Create a string containing all the information necessary to construct a copy of this MotionBehaviour
	*/
	virtual std::string toStringXML() const;

	/**
	* Write in a TinyXML element all the information necessary to construct a copy of this MotionBehaviour
	* @param motion_behaviour_xml TinyXML element that will contain the information of this MotionBehaviour
	*/
	virtual void toElementXML(TiXmlElement* motion_behaviour_xml) const;

	/**
	* Create a copy of this MotionBehaviour and return a pointer to it
	*/
	virtual MotionBehaviour* clone() const;

	/**
	* Assignement operator - Assign the values of the MotionBehaviour given as parameter. The same as the copy constructor
	* but on a previously created MotionBehaviour object.
	* @param motion_behaviour_assignment MotionBehaviour object that will be copied
	*/
	virtual MotionBehaviour& operator=(const MotionBehaviour & motion_behaviour_assignment);

private:

	rxControlSet*					control_set_;		// Stores the set of rxController's defining this MotionBehaviour
	rxSystem*						robot_;	

	// TODO: only to be used until KC provides us a way to get the controllers of a rxControlSet
	// and only used to activate them (we cannot activate all as we deactivate them) -> Not working!!! (when copying the vector cannot be updated with the copies of the rxController's)
	std::vector< rxController* >	controllers_set_;

	double							time_;				// Counts the time that the MotionBehaviour is active (for convergence check including time)
	double							dT_;				// Control interval
};

#endif

