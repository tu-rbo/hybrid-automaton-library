#include "hybrid_automaton/ControlSet.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {
	DescriptionTreeNode::Ptr ControlSet::serialize(const DescriptionTree::ConstPtr& factory) const {
		DescriptionTreeNode::Ptr tree = factory->createNode("ControlSet");

		tree->setAttribute<std::string>(std::string("type"), this->getType());
		tree->setAttribute<std::string>(std::string("name"), this->getName());

		// TODO more properties

		return tree;
	}

	void ControlSet::deserialize(const DescriptionTreeNode::ConstPtr& tree) {
		if (tree->getType() != "ControlSet") {
			std::stringstream ss;
			ss << "[ControlSet::deserialize] DescriptionTreeNode must have type 'ControlSet', not '" << tree->getType() << "'!";
			throw ss.str();
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isControlSetRegistered(_type)) {
			std::stringstream ss;
			ss << "[ControlSet::deserialize] ControlSet type '" << _type << "' "
			   << "invalid - empty or not registered with HybridAutomaton!";
			throw ss.str();
		}

		tree->getAttribute<std::string>("name", _name, "");

		// TODO more properties
	}

	void ControlSet::setType(const std::string& new_type) {
		this->_type = new_type;
	}

	const std::string& ControlSet::getType() const {
		return this->_type;
	}

	void ControlSet::setName(const std::string& new_name) {
		this->_name = new_name;
	}

	const std::string& ControlSet::getName() const {
		return this->_name;
	}

	void ControlSet::appendController(const Controller::Ptr& controller)
	{
		this->_controllers.push_back(controller);
		_addController(controller);
	}

	const std::vector<Controller::Ptr>& ControlSet::getControllers() const
	{
		return this->_controllers;
	}
}