#include "hybrid_automaton/ControlMode.h"

namespace ha {
	
	ControlMode::ControlMode(const std::string& name)
		: _name(name)
	{
	}

	DescriptionTreeNode::Ptr ControlMode::serialize(const DescriptionTree::ConstPtr& factory) const {
		// TODO
		return DescriptionTreeNode::Ptr();
	}
	
	void ControlMode::deserialize(const DescriptionTreeNode::ConstPtr& tree) {
		// TODO
	}

}