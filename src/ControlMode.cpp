#include "hybrid_automaton/ControlMode.h"

namespace ha {
	ControlMode::ControlMode(const std::string& name)
		: _name(name)
	{
	}
	
	DescriptionTreeNode::Ptr ControlMode::serialize(const DescriptionTree::ConstPtr& factory) const 
	{
		DescriptionTreeNode::Ptr tree_node = factory->createNode("ControlMode");
		tree_node->setAttribute<std::string>(std::string("name"), this->getName());

		tree_node->addChildNode(this->_control_set->serialize(factory));
		return tree_node;
	}
	
	void ControlMode::deserialize(const DescriptionTreeNode::ConstPtr& tree) {
		// TODO
	}

}