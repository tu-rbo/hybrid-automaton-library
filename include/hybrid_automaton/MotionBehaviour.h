#ifndef MOTION_BEHAVIOUR
#define MOTION_BEHAVIOUR

#include "Edge.h"
#include "Milestone.h"
#include "rControlalgorithm\rControlalgorithm.h"
#include "rxControlSDK\rxControlSDK.h"

#include "tinyxml.h"

#include <map>
#include <math.h>

typedef enum{
	JOINT_SPACE_GROUP				= 0,
	FUNCTION_GROUP					= JOINT_SPACE_GROUP+1,
	DISPLACEMENT_GROUP				= FUNCTION_GROUP+1,
	ORIENTATION_GROUP				= DISPLACEMENT_GROUP+1,
	H_TRANSFORM_GROUP				= ORIENTATION_GROUP+1,
	QUASICOORD_GROUP				= H_TRANSFORM_GROUP+1
} ControllerGroup;

typedef enum{
	FUNCTION_CONTROLLER					= 0,
	FUNCTION_COMPLIANCE_CONTROLLER		= FUNCTION_CONTROLLER+1,
	FUNCTION_IMPEDANCE_CONTROLLER		= FUNCTION_COMPLIANCE_CONTROLLER+1,
	STANDARD_CONTROLLER					= FUNCTION_IMPEDANCE_CONTROLLER+1,
	COMPLIANCE_CONTROLLER				= STANDARD_CONTROLLER+1,
	IMPEDANCE_CONTROLLER				= COMPLIANCE_CONTROLLER+1,
	INTERPOLATED_CONTROLLER				= IMPEDANCE_CONTROLLER+1,
	INTERPOLATED_COMPLIANCE_CONTROLLER	= INTERPOLATED_CONTROLLER+1,
	INTERPOLATED_IMPEDANCE_CONTROLLER	= INTERPOLATED_COMPLIANCE_CONTROLLER+1,
	NULL_MOTION_CONTROLLER				= INTERPOLATED_IMPEDANCE_CONTROLLER+1,
	NULL_MOTION_COMPLIANCE_CONTROLLER	= NULL_MOTION_CONTROLLER+1,
	GRADIENT_NULL_MOTION_CONTROLLER		= NULL_MOTION_COMPLIANCE_CONTROLLER+1
} ControllerSubgroup;

typedef std::pair<ControllerGroup, ControllerSubgroup> ControllerType;

struct ViaPointBase {};

struct ViaPointdVector : ViaPointBase{
	double					time_;
	int						type_;
	bool					reuse_;
	dVector					point_;

	ViaPointdVector(double time, int type, bool reuse, dVector point) :
	time_(time),
		type_(type),
		reuse_(reuse)
	{
		point_ = point;
	};
};

struct ViaPointVector3D : ViaPointBase{
	double					time_;
	int						type_;
	bool					reuse_;
	Vector3D				point_;
	ViaPointVector3D(double time, int type, bool reuse, Vector3D point) :
	time_(time),
		type_(type),
		reuse_(reuse)
	{
		point_ = point;
	};
};

struct ViaPointRotation : ViaPointBase{
	double					time_;
	int						type_;
	bool					reuse_;
	Rotation				point_;
	ViaPointRotation(double time, int type, bool reuse, Rotation point) :
	time_(time),
		type_(type),
		reuse_(reuse)
	{
		point_ = point;
	};
};

struct ViaPointHTransform : ViaPointBase{
	double					time_;
	int						type_;
	bool					reuse_;
	HTransform				point_;
	ViaPointHTransform(double time, int type, bool reuse, HTransform point) :
	time_(time),
		type_(type),
		reuse_(reuse)
	{
		point_ = point;
	};
};


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
	* Add an rxController to the control set. 
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
	rxController* createJointController_(ControllerSubgroup joint_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr) const;

	/**
	* Recreate a Displacement controller
	* @param displacement_subgroup Subgroup of the controller to be created within the Displacement Controller group.
	* @param controller_duration Time interval of the controller to be created.
	* @param via_points_ptr Via points of the controller to be created.
	* @param rxController_xml TinyXML Element with some other required information (alpha, alpha displacement, beta, beta displacement).
	*/
	rxController* createDisplacementController_(ControllerSubgroup displacement_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const;

	/**
	* Recreate an Orientation controller
	* @param orientation_subgroup Subgroup of the controller to be created within the Orientation Controller group.
	* @param controller_duration Time interval of the controller to be created.
	* @param via_points_ptr Via points of the controller to be created.
	* @param rxController_xml TinyXML Element with some other required information (alpha, alpha orientation, beta, beta orientation).
	*/
	rxController* createOrientationController_(ControllerSubgroup orientation_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const;

	/**
	* Recreate a HTransform controller
	* @param htransform_subgroup Subgroup of the controller to be created within the HTransform Controller group.
	* @param controller_duration Time interval of the controller to be created.
	* @param via_points_ptr Via points of the controller to be created.
	* @param rxController_xml TinyXML Element with some other required information (alpha, alpha htransform, beta, beta htransform).
	*/
	rxController* createHTransformController_(ControllerSubgroup htransform_subgroup, double controller_duration, std::vector<ViaPointBase*> via_points_ptr, TiXmlElement* rxController_xml) const;

	/**
	* Recreate a QuasiCoord controller
	* @param quasi_coord_subgroup Subgroup of the controller to be created within the Joint Controller group.
	* @param controller_duration Time interval of the controller to be created.
	*/
	rxController* createQuasiCoordController_(ControllerSubgroup quasi_coord_subgroup, double controller_duration) const;


	rxControlSet*									control_set_;		// Stores the set of rxController's defining this MotionBehaviour
	rxSystem*										robot_;	
	double											time_;				// Counts the time that the MotionBehaviour is active (for convergence check including time)
	double											dT_;				// Control interval
	static std::map<std::string, ControllerType>	controller_map_;	// Translation between class name (string) of the controllers and their type
};

#endif

