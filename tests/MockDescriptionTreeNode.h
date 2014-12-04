#ifndef HYBRID_AUTOMATON_TESTS_MOCK_DESCRIPTION_TREE_NODE_H_
#define HYBRID_AUTOMATON_TESTS_MOCK_DESCRIPTION_TREE_NODE_H_

#include "hybrid_automaton/DescriptionTreeNode.h"

namespace ha {

	class MockDescriptionTreeNode : public DescriptionTreeNode {
	public:
		typedef boost::shared_ptr<MockDescriptionTreeNode> Ptr;
		typedef boost::shared_ptr<const MockDescriptionTreeNode> ConstPtr;

		MOCK_CONST_METHOD0(getType, const std::string () );
		MOCK_CONST_METHOD2(getAttributeString, bool (const std::string& field_name, std::string& field_value) );
		MOCK_CONST_METHOD2(getChildrenNodes, bool (const std::string& type, ConstNodeList& children) );
		MOCK_CONST_METHOD1(getChildrenNodes, bool (ConstNodeList& children) );
		MOCK_CONST_METHOD1(getAllAttributes, void (std::map<std::string, std::string> & attrs) );

		MOCK_METHOD2(setAttributeString, void (const std::string& field_name, const std::string& field_value) );
		MOCK_METHOD1(addChildNode, void (const DescriptionTreeNode::Ptr& child) );

	protected:
		MOCK_CONST_METHOD1(_doClone, MockDescriptionTreeNode* (const MockDescriptionTreeNode& dtn) );

		MockDescriptionTreeNode* _doClone() const
		{
			return new MockDescriptionTreeNode();
		}
	};
	typedef boost::shared_ptr<MockDescriptionTreeNode> MockDescriptionTreeNodePtr;
	typedef boost::shared_ptr<const MockDescriptionTreeNode> MockDescriptionTreeNodeConstPtr;

}

#endif
