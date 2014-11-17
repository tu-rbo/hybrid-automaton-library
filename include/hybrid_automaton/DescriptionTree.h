#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_H_

#include <string>
#include <assert.h>

#include "hybrid_automaton/DescriptionTreeNode.h"

// FIXME remove
#include <iostream>

namespace ha {

	class DescriptionTree {

	protected:
	
	private:  

	public:		
		typedef boost::shared_ptr<DescriptionTree> Ptr;
		typedef boost::shared_ptr<const DescriptionTree> ConstPtr;

		//Return first tree element
		virtual DescriptionTreeNode::Ptr getRootNode() = 0;

		virtual void setRootNode(const DescriptionTreeNode::Ptr& root_node) = 0;

		virtual DescriptionTreeNode::Ptr createNode(const std::string& type) const  = 0;
	};

	typedef boost::shared_ptr<DescriptionTree> DescriptionTreePtr;
	typedef boost::shared_ptr<const DescriptionTree> DescriptionTreeConstPtr;
}

#endif
