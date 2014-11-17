#ifndef HYBRID_AUTOMATON_TESTS_MOCK_DESCRIPTION_TREE_H_
#define HYBRID_AUTOMATON_TESTS_MOCK_DESCRIPTION_TREE_H_

#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"

namespace ha {

	class MockDescriptionTree : public DescriptionTree {

	public:
		MOCK_METHOD0(getRootNode, DescriptionTreeNode::Ptr () );
		MOCK_CONST_METHOD1(createNode, DescriptionTreeNode::Ptr (const std::string& type) );
		MOCK_METHOD1(setRootNode, void (const DescriptionTreeNode::Ptr& root_node) );
	};

}

#endif