#ifndef HYBRID_AUTOMATON_CONTROL_MODE_H_
#define HYBRID_AUTOMATON_CONTROL_MODE_H_

#include "hybrid_automaton/ControlSet.h"
#include "hybrid_automaton/Serializable.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class ControlMode;
	typedef boost::shared_ptr<ControlMode> ControlModePtr;

	class ControlMode : public Serializable {

	protected:
		// TODO
		//ControlSet::Ptr _control_set;

		// unique identifier within one hybrid automaton
		std::string _name;

	public:
		typedef boost::shared_ptr<ControlMode> Ptr;

		ControlMode() {}

		virtual ~ControlMode() {}

		//virtual void setControlSet(ControlSet::Ptr cs) {
		//	throw "not implemented";
		//}

		virtual Eigen::Vector step() {
			throw "not implemented";
		}    

		virtual ControlSet::Ptr getControlSet() {
			throw "not implemented";
		}    

		virtual void serialize(DescriptionTreeNode::Ptr& tree) const;
		virtual void deserialize(const DescriptionTreeNode::Ptr tree);

		ControlModePtr clone() const {
			return ControlModePtr(_doClone());
		}

		void setName(const std::string& name) {
			_name = name;
		}

		const std::string& getName() const {
			return _name;
		}

	protected:
		virtual ControlMode* _doClone() const {
			return new ControlMode(*this);
		}

	};

}

#endif
