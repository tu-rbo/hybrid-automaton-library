#include "hybrid_automaton/ControlMode.h"
#include "hybrid_automaton/System.h"
#include "hybrid_automaton/HybridAutomaton.h"

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
			throw std::string("[ControlMode::serialize] Control set is null!");
		}
		tree_node->addChildNode(this->_control_set->serialize(factory));
		return tree_node;
	}
	
	void ControlMode::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system) {
		if (tree->getType() != "ControlMode") {
			std::stringstream ss;
			ss << "[ControlMode::deserialize] DescriptionTreeNode must have type 'ControlMode', not '" << tree->getType() << "'!";
			throw ss.str();
		}

		tree->getAttribute<std::string>("name", _name);

		DescriptionTreeNode::ConstNodeList control_set;
		tree->getChildrenNodes("ControlSet", control_set);

		if (control_set.empty()) {
			throw "[ControlMode::deserialize] No control set found!";
		}

		if (control_set.size() > 1) {
			throw "[ControlMode::deserialize] Too many (>1) control sets found!";
		}

		DescriptionTreeNode::Ptr first = * (control_set.begin());
		this->_control_set = HybridAutomaton::createControlSet( first, system);
		this->_control_set->setHybridAutomaton(this->getHybridAutomaton());

	}

}