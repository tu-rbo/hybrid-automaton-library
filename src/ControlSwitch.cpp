/*
 * Copyright 2015-2017, Robotics and Biology Lab, TU Berlin
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the 
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
		HA_INFO("ControlSwitch.isActive", "Jump condition: "<<this->getName());
		return true;
	}

	void ControlSwitch::initialize(const double& t) 
	{
		for (std::vector<JumpConditionPtr>::const_iterator it = _jump_conditions.begin(); it != _jump_conditions.end(); ++it) {
			(*it)->initialize(t);
		}
	}

	void ControlSwitch::terminate() 
	{
		for (std::vector<JumpConditionPtr>::const_iterator it = _jump_conditions.begin(); it != _jump_conditions.end(); ++it) {
			(*it)->terminate();
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