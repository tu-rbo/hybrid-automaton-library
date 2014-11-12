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
		// Generate / parse Description Tree
		virtual bool initTree(std::istream input) = 0;

		//Return first tree element
		virtual bool getRootNode(DescriptionTreeNode* root_node) = 0;
	};
}

#endif
