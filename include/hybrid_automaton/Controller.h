#ifndef HYBRID_AUTOMATON_CONTROLLER_H_
#define HYBRID_AUTOMATON_CONTROLLER_H_

namespace ha {

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


class Controller {
public:
  Controller() {
  }
  
  virtual void step() {
    // TODO
    //throw NotImplementedException;
  }
  
};

}

#endif
