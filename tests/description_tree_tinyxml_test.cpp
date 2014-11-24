#include <limits.h>
#include "gtest/gtest.h"

#include "hybrid_automaton/DescriptionTreeXML.h"

using namespace ha;


//This just tests the structure of the tree - no XML specific things are tested
TEST(TestDescriptionTreeStructure, Positive) {
	DescriptionTreeXML::Ptr tree(new DescriptionTreeXML());
	DescriptionTreeNodeXML::Ptr rootNode(new DescriptionTreeNodeXML("root"));

	EXPECT_EQ(rootNode->getType(), "root");

	DescriptionTreeNodeXML::Ptr daughterNode(new DescriptionTreeNodeXML("daughter"));

	DescriptionTreeNodeXML::Ptr sonNode(new DescriptionTreeNodeXML("son"));
	DescriptionTreeNodeXML::Ptr son2Node(new DescriptionTreeNodeXML("son"));
	DescriptionTreeNodeXML::Ptr grandDaughterNode(new DescriptionTreeNodeXML("granddaughter"));

	DescriptionTreeNode::Ptr raw = tree->getRootNode();

	rootNode = boost::dynamic_pointer_cast<DescriptionTreeNodeXML>(raw);	

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
	daughterNode->setAttribute<std::string>("name", "lucy");
	std::string daughtersname;
	EXPECT_TRUE(daughterNode->getAttribute<std::string>("name", daughtersname));
	std::string beer_str("Lucy is too young too drink!");
	EXPECT_FALSE(daughterNode->getAttribute<std::string>("beer", beer_str));

	EXPECT_EQ(daughtersname, "lucy");

	//Oh my god, what happened to Lucy?
	daughterNode->setAttribute<std::string>("name", "lucifer");
	EXPECT_TRUE(daughterNode->getAttribute<std::string>("name", daughtersname));
	EXPECT_EQ(daughtersname, "lucifer");

	std::map<std::string, std::string> map;
	daughterNode->getAllAttributes(map);
	EXPECT_EQ("lucifer", map["name"]);
	EXPECT_EQ(1, map.size());
}

//This test tests deparsing and reading out an xml string
TEST(TestDescriptionTreeFromXMLString, Positive) 
{
	DescriptionTreeXML::Ptr tree(new DescriptionTreeXML());

	//Test three xml cases - field with parameter a, child field b, and empty field c.
	std::string XMLString("<a par=k><b></b><c/></a>");
	tree->initTree(XMLString);
	DescriptionTreeNode::Ptr root = tree->getRootNode();

	DescriptionTreeNode::ConstNodeList childrenOfRoot;
	root->getChildrenNodes(childrenOfRoot);
	EXPECT_EQ(root->getType(), "a");
	std::string ret;
	EXPECT_TRUE(root->getAttribute<std::string>("par", ret));
	EXPECT_EQ(ret, "k");

	//Case sensitive!!!
	EXPECT_FALSE(root->getAttribute<std::string>("Par", ret)); 
	
	DescriptionTreeNode::ConstNodeListIterator it = childrenOfRoot.begin();
	
	EXPECT_EQ(it->get()->getType(), "b");
	it++;
	EXPECT_EQ(it->get()->getType(), "c");
}

//This test tests out an xml string
TEST(TestDescriptionTreeToXMLString, Positive) 
{
	DescriptionTreeXML::Ptr tree(new DescriptionTreeXML());
	DescriptionTreeNodeXML::Ptr rootNode(new DescriptionTreeNodeXML("root"));

	DescriptionTreeNodeXML::Ptr firstNode(new DescriptionTreeNodeXML("firstchild"));
	DescriptionTreeNodeXML::Ptr secondNode(new DescriptionTreeNodeXML("secondchild"));

	DescriptionTreeNode::Ptr raw = tree->getRootNode();

	rootNode = boost::dynamic_pointer_cast<DescriptionTreeNodeXML>(raw);	

	rootNode->addChildNode(firstNode);
	secondNode->setAttribute<std::string>("param", "value");
	
	::Eigen::MatrixXd eigen_vector(5,1);
	eigen_vector << 1.8673, 2., 3., 4., 5.;
	secondNode->setAttribute< ::Eigen::MatrixXd>("vector", eigen_vector);

	::Eigen::MatrixXd eigen_matrix(5,2);
	eigen_matrix << 1., 2., 3., 4., 5., 6., 7., 8., 9., 10.;
	secondNode->setAttribute< ::Eigen::MatrixXd>("matrix", eigen_matrix);

	rootNode->addChildNode(secondNode);

	//New test: change vlaue after insertion
	secondNode->setAttribute<std::string>("param2", "value2");

	std::string outString = tree->writeTreeXML();
	//std::cout << outString << std::endl;

	//Now parse again and compare 
	DescriptionTreeXML::Ptr tree2(new DescriptionTreeXML());

	//Test three xml cases - field with parameter a, child field b, and empty field c.
	tree->initTree(outString);
	DescriptionTreeNode::Ptr root2 = tree->getRootNode();

	DescriptionTreeNode::ConstNodeList childrenOfRoot;
	root2->getChildrenNodes(childrenOfRoot);

	DescriptionTreeNode::ConstNodeListIterator it = childrenOfRoot.begin();
	
	EXPECT_EQ(it->get()->getType(), "firstchild");
	it++;
	EXPECT_EQ(it->get()->getType(), "secondchild");
	std::string ret;
	EXPECT_TRUE(it->get()->getAttribute<std::string>("param", ret));
	EXPECT_EQ(ret, "value");
	
	EXPECT_TRUE(it->get()->getAttribute<std::string>("param2", ret));
	EXPECT_EQ(ret, "value2");

	::Eigen::MatrixXd result(0,0);
	EXPECT_TRUE(it->get()->getAttribute< ::Eigen::MatrixXd>("vector", result));
	EXPECT_EQ(eigen_vector, result);
	
	EXPECT_TRUE(it->get()->getAttribute< ::Eigen::MatrixXd>("matrix", result));
	EXPECT_EQ(eigen_matrix, result);


}
