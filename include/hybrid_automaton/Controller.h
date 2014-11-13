#ifndef HYBRID_AUTOMATON_CONTROLLER_H_
#define HYBRID_AUTOMATON_CONTROLLER_H_

#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/System.h"
#include "hybrid_automaton/hybrid_automaton_registration.h"

#include <boost/shared_ptr.hpp>

namespace ha {

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

		virtual void serialize(DescriptionTreeNode::Ptr& tree) const;
		virtual void deserialize(const DescriptionTreeNode::Ptr tree);

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
