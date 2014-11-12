#ifndef HYBRID_AUTOMATON_CONTROLLER_H_
#define HYBRID_AUTOMATON_CONTROLLER_H_

#include "hybrid_automaton/Serializable.h"

#include <boost/shared_ptr.hpp>

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

	class Controller;
	typedef boost::shared_ptr<Controller> ControllerPtr;

	class Controller : public Serializable {

	protected:
		int priority;
		double max_velocity;

	public:
		typedef boost::shared_ptr<Controller> Ptr;

		Controller() {
		}

		virtual void step() {
			throw "not implemented";
		}

		virtual void serialize(DescriptionTreeNode& tree) const;
		virtual void deserialize(const DescriptionTreeNode& tree);

		ControllerPtr clone() const {
			return ControllerPtr(_doClone());
		}

	protected:
		virtual Controller* _doClone() const {
			return new Controller(*this);
		}
	};

}

#endif
