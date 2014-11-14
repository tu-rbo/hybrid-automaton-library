#ifndef HYBRID_AUTOMATON_CONTROL_MODE_H_
#define HYBRID_AUTOMATON_CONTROL_MODE_H_

#include "hybrid_automaton/ControlSet.h"
#include "hybrid_automaton/Serializable.h"

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

namespace ha {

	class ControlMode;
	typedef boost::shared_ptr<ControlMode> ControlModePtr;
	typedef boost::shared_ptr<const ControlMode> ControlModeConstPtr;

	class ControlMode : public Serializable {
	public:
		typedef boost::shared_ptr<ControlMode> Ptr;
		typedef boost::shared_ptr<const ControlMode> ConstPtr;

		ControlMode() {}

		virtual ~ControlMode() {}

		//virtual void setControlSet(ControlSet::Ptr cs) {
		//	throw "not implemented";
		//}

		virtual void activate() {
			throw "not implemented";
		}

		virtual void deactivate() {
			throw "not implemented";
		}

		virtual ::Eigen::VectorXd step(const double& t) {
			return _control_set->step(t);
		}    

		virtual ControlSet::Ptr getControlSet() {
			return _control_set;
		}    

		virtual void serialize(const DescriptionTreeNode::Ptr& tree) const;
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree);

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

	protected:
		ControlSet::Ptr _control_set;

		// unique identifier within one hybrid automaton
		std::string _name;

	};

}

#endif
