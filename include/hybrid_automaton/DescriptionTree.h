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
#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_H_

#include <string>
#include <assert.h>

#include "hybrid_automaton/DescriptionTreeNode.h"

namespace ha {

/**
 * @brief A DescriptionTree is an interface for a tree-based description of a HybridAutomaton.
 * This inteface could be implemented by xml, yaml or similar languages.
 */
	class DescriptionTree {

	protected:
	
	private:  

	public:		
		typedef boost::shared_ptr<DescriptionTree> Ptr;
		typedef boost::shared_ptr<const DescriptionTree> ConstPtr;

        /**
         * @brief Return the first tree element
         */
		virtual DescriptionTreeNode::Ptr getRootNode() = 0;

        /**
         * @brief Set the first tree element
         */
		virtual void setRootNode(const DescriptionTreeNode::Ptr& root_node) = 0;

        /**
         * @brief Factory function to create a tree node
         */
		virtual DescriptionTreeNode::Ptr createNode(const std::string& type) const  = 0;
	};

	typedef boost::shared_ptr<DescriptionTree> DescriptionTreePtr;
	typedef boost::shared_ptr<const DescriptionTree> DescriptionTreeConstPtr;
}

#endif
