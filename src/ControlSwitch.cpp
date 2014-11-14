#include "hybrid_automaton/ControlSwitch.h"

namespace ha {
	bool ControlSwitch::isActive() const {
		for (std::vector<JumpConditionPtr>::const_iterator it = _jump_conditions.begin(); it != _jump_conditions.end(); ++it) {
			if (!(*it)->isActive())
				return false;
		}
		return true;
	}

	void ControlSwitch::activate(const double& t) {
		for (std::vector<JumpConditionPtr>::const_iterator it = _jump_conditions.begin(); it != _jump_conditions.end(); ++it) {
			(*it)->activate(t);
		}
	}

	void ControlSwitch::deactivate() {
		for (std::vector<JumpConditionPtr>::const_iterator it = _jump_conditions.begin(); it != _jump_conditions.end(); ++it) {
			(*it)->deactivate();
		}
	}

	void ControlSwitch::step(const double& t) {
		for (std::vector<JumpConditionPtr>::const_iterator it = _jump_conditions.begin(); it != _jump_conditions.end(); ++it) {
			(*it)->step(t);
		}
	}

	void ControlSwitch::serialize(const DescriptionTreeNode::Ptr& tree) const {
		// TODO
	}
	
	void ControlSwitch::deserialize(const DescriptionTreeNode::ConstPtr& tree) {
		// TODO
	}

}