#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <string>

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/DescriptionTreeNode.h"
#include "hybrid_automaton/tests/MockDescriptionTree.h"
#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::AtLeast;
using ::testing::_;

using namespace ::ha;

namespace ControlModeSerialization1 {
class MockSerializableControlSet : public ha::ControlSet {
	public:
		MOCK_CONST_METHOD1(serialize, DescriptionTreeNode::Ptr (const DescriptionTree::ConstPtr& factory) );
		MOCK_CONST_METHOD1(deserialize, void (const ha::DescriptionTreeNode::ConstPtr& tree) );

		HA_CONTROLSET_INSTANCE(node, system) {
			return ControlSet::Ptr(new MockSerializableControlSet);
		}
};
HA_CONTROLSET_REGISTER("MockSerializableControlSet", MockSerializableControlSet);
}

TEST(ControlMode, Serialization) {
	using namespace ha;
	using namespace std;

	string ctrlSetType("MockSerializableControlSet");

	//-------
	// Serialized and deserialized control set
	ControlModeSerialization1::MockSerializableControlSet* _cs = new ControlModeSerialization1::MockSerializableControlSet;
	ControlSet::Ptr cs(_cs);
	cs->setName("myCS");

	// Mocked description returned by control set
	MockDescriptionTreeNode* _cs_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr cs_node(_cs_node);
	EXPECT_CALL(*_cs_node, getType()).WillRepeatedly(Return("ControlSet"));
	EXPECT_CALL(*_cs_node, getAttributeString(_, _))
		.WillRepeatedly(DoAll(SetArgReferee<1>(""),Return(true)));

	EXPECT_CALL(*_cs, serialize(_))
		.WillRepeatedly(Return(cs_node));

	//-------
	// TESTED ControlMode

	ControlMode* _cm1 = new ControlMode;
	ControlMode::Ptr cm1(_cm1);	
	cm1->setName("myCM1");


	// this will be the node "generated" by the tree
	MockDescriptionTreeNode* _cm1_node = new MockDescriptionTreeNode;
	DescriptionTreeNode::Ptr cm1_node(_cm1_node);

	// Mocked tree factory
	MockDescriptionTree* _tree = new MockDescriptionTree;
	MockDescriptionTree::Ptr tree(_tree);
	EXPECT_CALL(*_tree, createNode("ControlMode"))
		.WillOnce(Return(cm1_node));

	// Expect that exactly one child node = cs will be added as child node
	EXPECT_CALL(*_cm1_node, addChildNode(cs_node))
		.WillOnce(Return());

	// -> some properties (don't care)
	EXPECT_CALL(*_cm1_node, setAttributeString(_,_))
		.Times(AtLeast(0));

	DescriptionTreeNode::Ptr cm_serialized;
	cm_serialized = cm1->serialize(tree);
	

}
