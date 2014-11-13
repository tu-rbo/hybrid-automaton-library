#ifndef HYBRID_AUTOMATON_CONTROLSET_H_
#define HYBRID_AUTOMATON_CONTROLSET_H_

#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/Serializable.h"

#include <boost/shared_ptr.hpp>

#include <vector>

namespace ha {

	class ControlSet;
	typedef boost::shared_ptr<ControlSet> ControlSetPtr;
	typedef boost::shared_ptr<const ControlSet> ControlSetConstPtr;

	class ControlSet : public Serializable {

	protected:
		// TODO
		//std::vector<Controller::Ptr> _controllers;

	public:
		typedef boost::shared_ptr<ControlSet> Ptr;
		typedef boost::shared_ptr<const ControlSet> ConstPtr;

		ControlSet() {
		}

		virtual void step() {
			throw "not implemented";
		}

		virtual std::vector<Controller::Ptr> getControllers() {
			throw "not implemented";
		}

		virtual void serialize(const DescriptionTreeNode::Ptr& tree) const;
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree);

		ControlSetPtr clone() const {
			return ControlSetPtr(_doClone());
		}

	protected:
		virtual ControlSet* _doClone() const {
			return new ControlSet(*this);
		}
	};

}

#endif
