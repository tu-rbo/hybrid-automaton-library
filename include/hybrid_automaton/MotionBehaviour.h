#ifndef MOTION_BEHAVIOUR
#define MOTION_BEHAVIOUR

#include "Edge.h"
#include "Milestone.h"
#include "rControlalgorithm\rControlalgorithm.h"
#include "rxControlSDK\rxControlSDK.h"
#include "collision_detection\collision_interface\include\CollisionInterface.h"

#include "tinyxml.h"

#include <map>
#include <math.h>

typedef enum {
	NONE				= 0,
	WITH_FUNCTION		= 1,
	WITH_COMPLIANCE		= 2,
	WITH_IMPEDANCE		= 4,
	WITH_INTERPOLATION	= 8,
	WITH_GRADIENT		= 16,
	ATTRACTOR			= 32,
	SUBDISPLACEMENT		= 64,
	OBSTACLE_AVOIDANCE	= 128
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


class MotionBehaviour: public Edge<Milestone> 
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
	MotionBehaviour(const Milestone * dad, const Milestone * son, rxSystem* robot, double weight = 1.0, double dt = 0.002);

	/**
	* Constructor
	* @param motion_behaviour_xml TinyXML element containing the information to (re)construct a new MotionBehaviour - usually transmitted over a network.
	* @param dad Pointer to the parent milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param son Pointer to the child milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param robot Pointer to the RLab system object.
	* @param dt Control interval of the controllers in this MB.
	*/
	MotionBehaviour(TiXmlElement* motion_behaviour_xml , const Milestone *dad, const Milestone *son , rxSystem* robot, double dt );

	/**
	* Constructor
	* @param dad Pointer to the parent milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param son Pointer to the child milestone (MotionBehaviour-Edge stores directly the pointer, no internal copy!).
	* @param control_set Pointer to the control set, which is used to get the pointer to the RLab system object and the control interval of the controllers in this MB.
	* @param weight Weight of the edge in the graph
	*/
	MotionBehaviour(const Milestone * dad, const Milestone * son, rxControlSet* control_set, double weight = 1.0);

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
	* Create the mapping to translate class names (string) to controller type (pair of integers: group and subgroup)
	*/
	static std::map<std::string, ControllerType> createControllerMapping_();

	/**
	* Extract the information to regenerate an rxController from the string provided by RLab and convert it into an XML Element for tinyXML.
	* @param string_data String (wstring on Windows) containing the information of the rxController.
	* @param out_xml_element TinyXML Element where the information of the rxController is written.
	*/
	void RLabInfoString2ElementXML_(string_type string_data, TiXmlElement* out_xml_element) const;

	/**
	* Add an rxController to the control set. First the information to generate the controller is extracted from an XML element.
	* Then, the controller is created and added.
	* @param rxController_xml Pointer to tinyXML Element with the information to generate the controller.
	*/
	void addController_(TiXmlElement * rxController_xml);

	/**
	* Convert a wstring into a string.
	* @param wstr Wide string to be converted.
	*/
	std::string wstring2string_(const std::wstring& wstr) const;

	/**
	* Convert a string into a wstring.
	* @param str String to be converted.
	*/
	std::wstring string2wstring_(const std::string& str) const;

	/**
	* Replace the colons of a string with white spaces.
	* @param text String to be processed.
	*/
	std::string colon2space_(std::string text) const;

	/**
	* Recreate a Joint controller
	* @param joint_subgroup Subgroup of the controller to be created within the Joint Controller group.
	* @param controller_duration Time interval of the controller to be created.
	* @param via_points_ptr Via points of the controller to be created.
	*/
	rxController* createJointController_(int joint_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const;

	/**
	* Recreate a Displacement controller
	* @param displacement_subgroup Subgroup of the controller to be created within the Displacement Controller group.
	* @param controller_duration Time interval of the controller to be created.
	* @param via_points_ptr Via points of the controller to be created.
	* @param rxController_xml TinyXML Element with some other required information (alpha, alpha displacement, beta, beta displacement).
	*/
	rxController* createDisplacementController_(int displacement_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const;

	/**
	* Recreate an Orientation controller
	* @param orientation_subgroup Subgroup of the controller to be created within the Orientation Controller group.
	* @param controller_duration Time interval of the controller to be created.
	* @param via_points_ptr Via points of the controller to be created.
	* @param rxController_xml TinyXML Element with some other required information (alpha, alpha orientation, beta, beta orientation).
	*/
	rxController* createOrientationController_(int orientation_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const;

	/**
	* Recreate a HTransform controller
	* @param htransform_subgroup Subgroup of the controller to be created within the HTransform Controller group.
	* @param controller_duration Time interval of the controller to be created.
	* @param via_points_ptr Via points of the controller to be created.
	* @param rxController_xml TinyXML Element with some other required information (alpha, alpha htransform, beta, beta htransform).
	*/
	rxController* createHTransformController_(int htransform_subtype, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const;

	/**
	* Recreate a QuasiCoord controller
	* @param quasi_coord_subgroup Subgroup of the controller to be created within the Joint Controller group.
	* @param controller_duration Time interval of the controller to be created.
	*/
	rxController* createQuasiCoordController_(int quasi_coord_subtype, double controller_duration) const;

	/**
	* Recreate a NullMotion controller
	* @param null_motion_coord_subgroup Subgroup of the controller to be created within the Null Motion Controller group.
	* @param controller_duration Time interval of the controller to be created.
	*/
	rxController* createNullMotionController_(int null_motion_subtype, double controller_duration) const;

	/**
	* Recreate a Functional controller
	* @param functional_subtype Subgroup of the controller to be created within the Null Motion Controller group.
	* @param dimension Dimension to be controlled
	* @param controller_duration Time interval of the controller to be created.
	*/
	rxController* createFunctionalController_(int functional_subtype, int dimension, double controller_duration, TiXmlElement* rxController_xml) const;


	rxControlSetBase*								control_set_;		// Stores the set of rxController's defining this MotionBehaviour
	rxSystem*										robot_;	
	double											time_;				// Execution time. Counts the time that the MotionBehaviour is active 
																		// (for convergence check including time)
	double											time_to_converge_;	// duration of the interpolation
	double											dT_;				// Control interval
	static std::map<std::string, ControllerType>	controller_map_;	// Translation between class name (string) of the controllers and their type

	double											max_velocity_;		// maximum desired velocity at joint or tip (depending on controller); used for calculating the interpolation time
	double											min_time_;			// minimum time that is used for interpolation (if max_velocity constraint is not set)
	
	std::map<string_type, bool>						goal_controllers_;	// tells us if a controller is goal controller or not (if its convergence must be checked or not) 
};

#endif

