#include "hybrid_automaton/DescriptionTreeNode.h"

namespace ha {

	class MockDescriptionTreeNode : public DescriptionTreeNode {
	public:
		MOCK_CONST_METHOD0(getType, const std::string () );
		MOCK_CONST_METHOD2(getAttribute, bool (const std::string& field_name, std::string& field_value) );
		MOCK_CONST_METHOD2(getChildrenNodes, bool (const std::string& type, ConstNodeList& children) );
		MOCK_CONST_METHOD1(getChildrenNodes, bool (ConstNodeList& children) );

		MOCK_CONST_METHOD3(getAttributeBool, bool (const std::string& field_name, bool& return_value, bool default_value) );
		MOCK_CONST_METHOD3(getAttributeString, bool (const std::string& field_name, std::string& return_value, std::string& default_value) );

		MOCK_METHOD2(setAttribute, void (const std::string& field_name, std::string& field_value) );
		MOCK_METHOD1(addChildNode, void (DescriptionTreeNode::Ptr child) );
	};

}