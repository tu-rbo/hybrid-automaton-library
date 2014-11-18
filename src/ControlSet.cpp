#include "hybrid_automaton/ControlSet.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {

	ControlSet::ControlSet()
	{
	}

	ControlSet::ControlSet(const ControlSet& cs)
	{
		this->_type = cs._type;
		this->_controllers = cs._controllers;
	}

	void ControlSet::activate() {
		throw std::string("[ControlSet::activate] Not implemented");
	}

	void ControlSet::deactivate() {
		throw std::string("[ControlSet::deactivate] Not implemented");
	}

	::Eigen::MatrixXd ControlSet::step(const double& t) {
		throw std::string("[ControlSet::step] Not implemented");
	}

	void ControlSet::_addController(const Controller::Ptr& cntrl) {
		// nothing
		// THIS SHOULD BE OVERLOADED!
	}

	DescriptionTreeNode::Ptr ControlSet::serialize(const DescriptionTree::ConstPtr& factory) const {
		DescriptionTreeNode::Ptr tree = factory->createNode("ControlSet");

		tree->setAttribute<std::string>(std::string("type"), this->getType());
		tree->setAttribute<std::string>(std::string("name"), this->getName());

		for(std::vector<Controller::Ptr>::const_iterator ctrl_it = this->_controllers.begin(); ctrl_it != this->_controllers.end(); ++ctrl_it)
		{
			tree->addChildNode((*ctrl_it)->serialize(factory));
		}

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

	const std::string ControlSet::getName() const {
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