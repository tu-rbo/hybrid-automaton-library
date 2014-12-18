#include "hybrid_automaton/ControlMode.h"
#include "hybrid_automaton/System.h"
#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/error_handling.h"

namespace ha {
	ControlMode::ControlMode(const std::string& name)
		: _name(name)
	{
	}
	
	DescriptionTreeNode::Ptr ControlMode::serialize(const DescriptionTree::ConstPtr& factory) const 
	{
		DescriptionTreeNode::Ptr tree_node = factory->createNode("ControlMode");
		tree_node->setAttribute<std::string>(std::string("name"), this->getName());

		if (!this->_control_set) {
			HA_THROW_ERROR("ControlMode.serialize", "Control set is null!");
		}
		tree_node->addChildNode(this->_control_set->serialize(factory));
		return tree_node;
	}
	
	void ControlMode::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha) {
		if (tree->getType() != "ControlMode") {
			HA_THROW_ERROR("ControlMode.deserialize", "DescriptionTreeNode must have type 'ControlMode', not '" << tree->getType() << "'!");
		}

		if(!tree->getAttribute<std::string>("name", _name))
			HA_WARN("ControlMode.deserialize", "No \"name\" parameter given in ControlMode - using default value");

		DescriptionTreeNode::ConstNodeList control_set;
		tree->getChildrenNodes("ControlSet", control_set);

		if (control_set.empty()) {
			HA_THROW_ERROR("ControlMode.deserialize", "No control set found!");
		}

		if (control_set.size() > 1) {
			HA_THROW_ERROR("ControlMode.deserialize", "Too many (>1) control sets found!");
		}

		DescriptionTreeNode::ConstPtr first = * (control_set.begin());
		this->_control_set = HybridAutomaton::createControlSet(first, system, ha);

	}

}