///////////////////////////////////////////////////////////
// Macros for registering controllers with HybridAutomaton

// Required to enable deserialization of this controller.
// Invoke this in the header of your controller.
// Example:
//   HA_RLAB_CONTROLLER_REGISTER_HEADER()
#define HA_RLAB_CONTROLLER_REGISTER_HEADER()\
	class Initializer {\
	public:\
	Initializer();\
	};\
	static Initializer initializer;\

// The first parameter specifies the name that will be used 
// in the serialization, the second is the class name
// of your controller.
// Invoke this in the cpp file of your controller.
// Example:
//   HA_RLAB_CONTROLLER_REGISTER_CPP("JointController", rlabJointController)
#define HA_RLAB_CONTROLLER_REGISTER_CPP(STR, NAME) NAME::Initializer initializer;\
	NAME::Initializer::Initializer() { ha::HybridAutomaton::registerController(STR, &NAME::instance); }


///////////////////////////////////////////////////////////
// Macros for registering control sets with HybridAutomaton

// Required to enable deserialization of this controller.
// Invoke this in the header of your control set.
// Example:
//   HA_RLAB_CONTROLSET_REGISTER_HEADER()
#define HA_RLAB_CONTROLSET_REGISTER_HEADER()\
	class Initializer {\
	public:\
	Initializer();\
	};\
	static Initializer initializer;\

// The first parameter specifies the name that will be used 
// in the serialization, the second is the class name
// of your control set.
// Invoke this in the cpp file of your control set.
// Example:
//   HA_RLAB_CONTROLSET_REGISTER_CPP("MyControlSet", rlabMyControlSet)
#define HA_RLAB_CONTROLSET_REGISTER_CPP(STR, NAME) NAME::Initializer initializer;\
	NAME::Initializer::Initializer() { ha::HybridAutomaton::registerControlSet(STR, &NAME::instance); }