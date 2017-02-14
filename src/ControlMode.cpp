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