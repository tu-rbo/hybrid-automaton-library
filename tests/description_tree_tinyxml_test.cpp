#include <limits.h>
#include "gtest/gtest.h"

#include "hybrid_automaton/DescriptionTreeXML.h"

using namespace ha;

//This just test the structure of the tree - no XML specific things are tested
TEST(TestDescriptionTreeStructure, Positive) {
	DescriptionTreeXML tree;
	DescriptionTreeNodeXML::Ptr rootNode(new DescriptionTreeNodeXML("root"));
	EXPECT_EQ(rootNode->getType(), "root");

	DescriptionTreeNodeXML::Ptr daughterNode(new DescriptionTreeNodeXML("daughter"));
	DescriptionTreeNodeXML::Ptr sonNode(new DescriptionTreeNodeXML("son"));
	DescriptionTreeNodeXML::Ptr son2Node(new DescriptionTreeNodeXML("son"));
	DescriptionTreeNodeXML::Ptr grandDaughterNode(new DescriptionTreeNodeXML("granddaughter"));

	tree.getRootNode(rootNode);	
	rootNode->addChildNode(daughterNode);
	rootNode->addChildNode(sonNode);
	rootNode->addChildNode(son2Node);
	sonNode->addChildNode(grandDaughterNode);

	
	//test getter for children nodes
	DescriptionTreeNode::ConstNodeList childrenOfRoot;
	EXPECT_TRUE(rootNode->getChildrenNodes(childrenOfRoot));
	EXPECT_EQ(childrenOfRoot.size(), 3);

	DescriptionTreeNode::ConstNodeList childrenOfDaughter;
	EXPECT_FALSE(daughterNode->getChildrenNodes(childrenOfDaughter));
	EXPECT_EQ(childrenOfDaughter.size(), 0);

	//test named getter for children nodes
	DescriptionTreeNode::ConstNodeList sonsOfRoot;
	EXPECT_TRUE(rootNode->getChildrenNodes("son", sonsOfRoot));
	EXPECT_EQ(sonsOfRoot.size(), 2);
	DescriptionTreeNode::ConstNodeListIterator it = sonsOfRoot.begin();
	for(it; it!=sonsOfRoot.end(); it++)
	{
		EXPECT_EQ(it->get()->getType(), "son");
	}

	DescriptionTreeNode::ConstNodeList daughtersOfRoot;
	EXPECT_TRUE(rootNode->getChildrenNodes("daughter", daughtersOfRoot));
	EXPECT_EQ(daughtersOfRoot.size(), 1);

	DescriptionTreeNode::ConstNodeList sonsOfSon;
	EXPECT_FALSE(sonNode->getChildrenNodes("son", sonsOfSon));
	EXPECT_EQ(sonsOfSon.size(), 0);

	//test fields
	daughterNode->setAttribute("name", "lucy");
	std::string daughtersname;
	EXPECT_TRUE(daughterNode->getAttribute("name", daughtersname));
	EXPECT_FALSE(daughterNode->getAttribute("beer", std::string("Lucy is too young too drink!")));

	EXPECT_EQ(daughtersname, "lucy");

	//Oh my god, what happened to Lucy?
	daughterNode->setAttribute("name", "lucifer");
	EXPECT_TRUE(daughterNode->getAttribute("name", daughtersname));
	EXPECT_EQ(daughtersname, "lucifer");
}
