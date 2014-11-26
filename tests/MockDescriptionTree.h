#ifndef HYBRID_AUTOMATON_TESTS_MOCK_DESCRIPTION_TREE_H_
#define HYBRID_AUTOMATON_TESTS_MOCK_DESCRIPTION_TREE_H_

#include "hybrid_automaton/DescriptionTree.h"
#include "tests/MockDescriptionTreeNode.h"

namespace ha {

	class MockDescriptionTree : public DescriptionTree {

	public:
		typedef boost::shared_ptr<MockDescriptionTree> Ptr;
		typedef boost::shared_ptr<const MockDescriptionTree> ConstPtr;

		MOCK_METHOD0(getRootNode, DescriptionTreeNode::Ptr () );
		MOCK_METHOD1(setRootNode, void (const DescriptionTreeNode::Ptr& root_node) );

		MOCK_CONST_METHOD1(createNode, DescriptionTreeNode::Ptr (const std::string& type) );
		/*
		DescriptionTreeNode::Ptr createNode(const std::string& type) const {
			DescriptionTreeNode::Ptr node(new MockDescriptionTreeNode);
			return node;
		}
		*/

	};

	typedef boost::shared_ptr<MockDescriptionTree> MockDescriptionTreePtr;
	typedef boost::shared_ptr<const MockDescriptionTree> MockDescriptionTreeConstPtr;

}

#endif