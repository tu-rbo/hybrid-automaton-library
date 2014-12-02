#include "hybrid_automaton/ControlSet.h"
#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/error_handling.h"

namespace ha {

	ControlSet::ControlSet()
	{
	}

	ControlSet::ControlSet(const ControlSet& cs)
	{
		this->_type = cs._type;
		this->_controllers = cs._controllers;
	}

	void ControlSet::initialize() {
		HA_THROW_ERROR("ControlSet.initialize", "Not implemented");
	}

	void ControlSet::terminate() {
		HA_THROW_ERROR("ControlSet.terminate", "Not implemented");
	}

	::Eigen::MatrixXd ControlSet::step(const double& t) {
		HA_THROW_ERROR("ControlSet.step", "Not implemented");
	}

	void ControlSet::_addController(const Controller::Ptr&) {
		// This can be implemented if desired
	}

	DescriptionTreeNode::Ptr ControlSet::serialize(const DescriptionTree::ConstPtr& factory) const {
		DescriptionTreeNode::Ptr tree = factory->createNode("ControlSet");

		tree->setAttribute<std::string>(std::string("type"), this->getType());
		tree->setAttribute<std::string>(std::string("name"), this->getName());

		std::map<std::string, Controller::Ptr>::const_iterator ctrl_it;
		for(ctrl_it = this->_controllers.begin(); ctrl_it != this->_controllers.end(); ++ctrl_it)
		{
			tree->addChildNode((ctrl_it->second)->serialize(factory));
		}

		return tree;
	}

	void ControlSet::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha) {
		if (tree->getType() != "ControlSet") {
			HA_THROW_ERROR("ControlSet.deserialize", "DescriptionTreeNode must have type 'ControlSet', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isControlSetRegistered(_type)) {
			HA_THROW_ERROR("ControlSet.deserialize", "ControlSet type '" << _type << "' "
			   << "invalid - empty or not registered with HybridAutomaton!");
		}

		tree->getAttribute<std::string>("name", _name, "");

		// deserialize controllers
		DescriptionTreeNode::ConstNodeList ctrl_nodes;
		tree->getChildrenNodes("Controller", ctrl_nodes);

		DescriptionTreeNode::ConstNodeList::const_iterator it;
		for (it = ctrl_nodes.begin(); it != ctrl_nodes.end(); ++it) {
			Controller::Ptr ctrl =  HybridAutomaton::createController(*it, system, ha);
			this->appendController(ctrl);
		}

		// TODO more properties
	}

	void ControlSet::setType(const std::string& new_type) {
		this->_type = new_type;
	}

	const std::string ControlSet::getType() const {
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
		const std::string& name = controller->getName();

		std::map<std::string, Controller::Ptr>::iterator it = _controllers.find(name);
		if (it != _controllers.end()) {
			HA_THROW_ERROR("ControlSet.appendController", "Controller with same name already registered: " << name);
		}

		this->_controllers[name] = controller;
		this->_addController(controller); // call user defined overloaded method
	}

	/*
	const std::vector<Controller::Ptr>& ControlSet::getControllers() const
	{
		return this->_controllers;
	}
	*/

	Controller::ConstPtr ControlSet::getControllerByName(const std::string& name) const {
		std::map<std::string, Controller::Ptr>::const_iterator it = _controllers.find(name);
		if (it == _controllers.end()) {
			throw std::string("[ControlSet.getControllerByName] cannot find controller ") + name;
		}
		return it->second;
	}
}