#ifndef MOTION_BEHAVIOUR
#define MOTION_BEHAVIOUR

#include "MDPEdge.h"
#include "Milestone.h"
#include "rControlalgorithm\rControlalgorithm.h"
#include "rxControlSDK\rxControlSDK.h"
#include "collision_detection\collision_interface\include\CollisionInterface.h"

#include "controllers\include\OnDemandController.h"
#include "controllers\include\FeatureAttractorController.h"
#include "controllers\include\SubdisplacementController.h"

#include "tinyxml.h"

#include <map>
#include <math.h>

typedef enum {
	NONE				    = 0,
	WITH_FUNCTION		    = 1,
	WITH_COMPLIANCE		    = 2,
	WITH_IMPEDANCE	    	= 4,
	WITH_INTERPOLATION	    = 8,
	WITH_GRADIENT		    = 16,
	ATTRACTOR			    = 32, 
	SUBDISPLACEMENT		    = 64,
	OBSTACLE_AVOIDANCE	    = 128,
	BLACKBOARD_ACCESS	    = 256,
    SINGULARITY_AVOIDANCE   = 512,
    JOINT_LIMIT_AVOIDANCE   = 1024
} ControllerSubtype;

// impossible to replace int by ControllerSubtype because then no combinations of subtypes are allowed!
typedef std::pair<rxController::eControlType, int> ControllerType;

struct ViaPointBase {
	double					time_;
	int						type_;
	bool					reuse_;
};

struct ViaPointdVector : ViaPointBase{
	dVector					point_;

	ViaPointdVector(double time, int type, bool reuse, dVector point)
	{
		time_ = time;
		type_ = type;
		reuse_ = reuse;
		point_ = point;
	};
};

struct ViaPointVector3D : ViaPointBase{
	Vector3D				point_;
	ViaPointVector3D(double time, int type, bool reuse, Vector3D point) 
	{
		time_ = time;
		type_ = type;
		reuse_ = reuse;
		point_ = point;
	};
};

struct ViaPointRotation : ViaPointBase{
	Rotation				point_;
	ViaPointRotation(double time, int type, bool reuse, Rotation point)
	{
				time_ = time;
		type_ = type;
		reuse_ = reuse;
		point_ = point;
	};
};

struct ViaPointHTransform : ViaPointBase{
	HTransform				point_;
	ViaPointHTransform(double time, int type, bool reuse, HTransform point)
	{
		time_ = time;
		type_ = type;
		reuse_ = reuse;
		point_ = point;
	};
};


class MotionBehaviour: public MDPEdge
{

private:
	/**
	* Constructor
	*/
	MotionBehaviour();

public:
	/**
	* Constructor
	* @param dad Pointer to the parent milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param son Pointer to the child milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param robot Pointer to the RLab system object.
	* @param weight Weight of the edge in the graph
	* @param dt Control interval
	*/
	MotionBehaviour(Milestone * dad, Milestone * son, rxSystem* robot, double weight = 1.0, double dt = 0.002);

	/**
	* Constructor
	* @param dad Pointer to the parent milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param son Pointer to the child milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param control_set Pointer to the control set, which is used to get the pointer to the RLab system object and the control interval of the controllers in this MB.
	* @param weight Weight of the edge in the graph
	*/
	MotionBehaviour(Milestone * dad, Milestone * son, rxControlSetBase* control_set, double weight = 1.0);

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
	* Add an rxController to the control set. 
	* @param ctrl Pointer to the rxController object to be added.
	* @param is_goal_controller Flag to mark if this controller must be checked for convergence of the MB
	*/
	void addController(rxController* ctrl, bool is_goal_controller);

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
	*
	* @deprecated
	*/
	bool hasConverged();

	/**
	* Get the current error from the control set
	*/
	dVector getError() const;

	dVector getDesired() const;
	dVector getDesiredDot() const;
	dVector getErrorDot() const;
	dVector getCurrentDotReference() const;
	dVector getCurrentDotDotReference() const;
	dVector getOriTaskError() const;
	dVector getLineTaskError() const;

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
	*/
	virtual TiXmlElement* toElementXML() const;

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

	virtual void print();

	// USE WITH CARE!!! - mainly for ERM internal speed-up!
	virtual void setControlSetByPointerOnly(rxControlSetBase* control_set){this->control_set_ = control_set;};


	void setMaxVelocityForInterpolation(double max_velocity);
	void setMinTimeForInterpolation(double min_time);


private:

	/**
	* Extract the information to regenerate an rxController from the string provided by RLab and convert it into an XML Element for tinyXML.
	* @param string_data String (wstring on Windows) containing the information of the rxController.
	* @param out_xml_element TinyXML Element where the information of the rxController is written.
	* @param is_goal_controller If it is not a goal controller it could contain its goal as the first via point. If it is a goal controller we ignore the via points.
	*/
	void RLabInfoString2ElementXML_(string_type string_data, TiXmlElement* out_xml_element, bool is_goal_controller) const;

	rxControlSetBase*								control_set_;		// Stores the set of rxController's defining this MotionBehaviour
	rxSystem*										robot_;	
	double											time_;				// Execution time. Counts the time that the MotionBehaviour is active 
																		// (for convergence check including time)
	double											time_to_converge_;	// duration of the interpolation
	double											dT_;				// Control interval

	double											max_velocity_;		// maximum desired velocity at joint or tip (depending on controller); used for calculating the interpolation time
	double											min_time_;			// minimum time that is used for interpolation (if max_velocity constraint is not set)
	
	std::map<string_type, bool>						goal_controllers_;	// tells us if a controller is goal controller or not (if its convergence must be checked or not) 

		// obstacle avoidance, etc...
	std::vector<OnDemandController*>				_onDemand_controllers;
};

#endif

