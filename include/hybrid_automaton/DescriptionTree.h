#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_H_

#include <string>
#include <assert.h>

#include "hybrid_automaton/DescriptionTreeNode.h"

// FIXME remove
#include <iostream>

namespace ha {

	class DescriptionTree{

	protected:
	
	private:  

	public:		
		//Return first tree element
		virtual bool getRootNode(DescriptionTreeNode::Ptr& root_node) = 0;

		//virtual DescriptionTreeNode::Ptr createNode() = 0;
	};
}

#endif
