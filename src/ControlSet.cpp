#include "hybrid_automaton/ControlSet.h"

namespace ha {
	void ControlSet::serialize(const DescriptionTreeNode::Ptr& tree) const {
		// TODO
	}

	void ControlSet::deserialize(const DescriptionTreeNode::ConstPtr& tree) {
		// TODO
	}

	void ControlSet::setType(const std::string& new_type) {
		this->_type = new_type;
	}

	const std::string& ControlSet::getType() const {
		return this->_type;
	}

	void ControlSet::addController(const Controller::Ptr& controller)
	{
		this->_controllers.push_back(controller);
	}

	const std::vector<Controller::Ptr>& ControlSet::getControllers() const
	{
		return this->_controllers;
	}
}