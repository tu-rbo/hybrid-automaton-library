#include "hybrid_automaton/ControlSwitch.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {
	void ControlSwitch::add(const JumpConditionPtr& jump_condition)
	{
		_jump_conditions.push_back(jump_condition);
	}

	const std::vector<JumpConditionPtr>& ControlSwitch::getJumpConditions()
	{
		return _jump_conditions;
	}

	bool ControlSwitch::isActive() const 
	{
		for (std::vector<JumpConditionPtr>::const_iterator it = _jump_conditions.begin(); it != _jump_conditions.end(); ++it) {
			if (!(*it)->isActive())
				return false;
		}
		return true;
	}

	void ControlSwitch::activate(const double& t) 
	{
		for (std::vector<JumpConditionPtr>::const_iterator it = _jump_conditions.begin(); it != _jump_conditions.end(); ++it) {
			(*it)->activate(t);
		}
	}

	void ControlSwitch::deactivate() 
	{
		for (std::vector<JumpConditionPtr>::const_iterator it = _jump_conditions.begin(); it != _jump_conditions.end(); ++it) {
			(*it)->deactivate();
		}
	}

	void ControlSwitch::step(const double& t) 
	{
		for (std::vector<JumpConditionPtr>::const_iterator it = _jump_conditions.begin(); it != _jump_conditions.end(); ++it) 
		{
			(*it)->step(t);
		}
	}

	void ControlSwitch::setName(const std::string& name) 
	{
		_name = name;
	}

	const std::string ControlSwitch::getName() const 
	{
		return _name;
	}

	DescriptionTreeNode::Ptr ControlSwitch::serialize(const DescriptionTree::ConstPtr& factory) const 
	{
		DescriptionTreeNode::Ptr tree_node = factory->createNode("ControlSwitch");
		tree_node->setAttribute<std::string>(std::string("name"), this->getName());
		tree_node->setAttribute<std::string>(std::string("source"), _hybrid_automaton->getSourceControlMode(this->_name)->getName());
		tree_node->setAttribute<std::string>(std::string("target"), _hybrid_automaton->getTargetControlMode(this->_name)->getName());
		
		for (std::vector<JumpConditionPtr>::const_iterator it = _jump_conditions.begin(); it != _jump_conditions.end(); ++it) {
			tree_node->addChildNode((*it)->serialize(factory));
		}

		return tree_node;
	}
	
	void ControlSwitch::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha) 
	{
		if (tree->getType() != "ControlSwitch") {
			HA_THROW_ERROR("ControlSwitch.deserialize", "DescriptionTreeNode must have type 'ControlSwitch', not '" << tree->getType() << "'!");
		}

		tree->getAttribute<std::string>("name", _name, "");

		DescriptionTreeNode::ConstNodeList jump_conditions;
		tree->getChildrenNodes("JumpCondition", jump_conditions);

		if (jump_conditions.empty()) {
			HA_THROW_ERROR("ControlSwitch.deserialize", "No jump conditions found!");
		}

		DescriptionTreeNode::ConstNodeList::iterator js_it;
		for (js_it = jump_conditions.begin(); js_it != jump_conditions.end(); ++js_it) {
			JumpCondition::Ptr js(new JumpCondition);

			//We need to set the controller pointer before calling deserialize!
			std::string source_control_mode_name;
			tree->getAttribute<std::string>("source", source_control_mode_name, "");

			if (source_control_mode_name == "")
			{
				HA_THROW_ERROR("ControlSwitch.deserialize", "Source control mode of control switch '" << _name << "' is empty.");
			}

			js->setSourceModeName(source_control_mode_name);
			js->deserialize(*js_it, system, ha);
			this->add(js);
		}
	}

	void ControlSwitch::setHybridAutomaton(const HybridAutomaton* hybrid_automaton)
	{
		_hybrid_automaton = hybrid_automaton;
	}
}