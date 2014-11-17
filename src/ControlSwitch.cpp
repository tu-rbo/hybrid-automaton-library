#include "hybrid_automaton/ControlSwitch.h"

namespace ha {
	void ControlSwitch::add(const JumpConditionPtr& jump_condition)
	{
		_jump_conditions.push_back(jump_condition);
	}

	const std::vector<JumpConditionPtr>& ControlSwitch::getJumpConditions()
	{
		return _jump_conditions;
	}

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
	
	DescriptionTreeNode::Ptr ControlSwitch::serialize(const DescriptionTree::ConstPtr& factory) const {
		throw "Not implemented";
	}
	
	void ControlSwitch::deserialize(const DescriptionTreeNode::ConstPtr& tree) {
		// TODO
	}

}