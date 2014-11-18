#ifndef HYBRID_AUTOMATON_CONTROL_MODE_H_
#define HYBRID_AUTOMATON_CONTROL_MODE_H_

#include "hybrid_automaton/ControlSet.h"
#include "hybrid_automaton/Serializable.h"

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

namespace ha {

	class HybridAutomaton;

	class ControlMode;
	typedef boost::shared_ptr<ControlMode> ControlModePtr;
	typedef boost::shared_ptr<const ControlMode> ControlModeConstPtr;

	class ControlMode : public Serializable {
	public:
		typedef boost::shared_ptr<ControlMode> Ptr;
		typedef boost::shared_ptr<const ControlMode> ConstPtr;

		ControlMode() {}
		ControlMode(const std::string& name);

		virtual ~ControlMode() {}

		virtual void activate() {
			if (_control_set)
				_control_set->activate();
			else
				throw std::string("[ControlMode::activate] No control set defined.");
		}

		virtual void deactivate() {
			if (_control_set)
				_control_set->deactivate();
			else
				throw std::string("[ControlMode::deactivate] No control set defined.");
		}

		virtual ::Eigen::MatrixXd step(const double& t) {
			if (_control_set)
				return _control_set->step(t);
			else
				throw std::string("[ControlMode::step] No control set defined.");
		}

		virtual void setControlSet(const ControlSet::Ptr control_set) {
			_control_set = control_set;
		}

		virtual ControlSet::Ptr getControlSet() const {
			return _control_set;
		}    

		virtual Controller::Ptr getControllerByName(const std::string& name) const {
			return _control_set->getControllerByName(name);
		}    

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system);

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
