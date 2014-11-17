#include "hybrid_automaton/ControlMode.h"

namespace ha {
	DescriptionTreeNode::Ptr ControlMode::serialize(const DescriptionTree::ConstPtr& factory) const {
		// TODO
		return factory->createNode("ControlMode");
	}
	
	void ControlMode::deserialize(const DescriptionTreeNode::ConstPtr& tree) {
		// TODO
	}

}