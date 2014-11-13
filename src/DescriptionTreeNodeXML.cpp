#include "hybrid_automaton/DescriptionTreeNodeXML.h"
namespace ha {

DescriptionTreeNodeXML::DescriptionTreeNodeXML(TiXmlElement* xmlNode):
node(xmlNode)
{
}

const std::string DescriptionTreeNodeXML::getType() const
{
	return std::string(node->Value());
}

bool DescriptionTreeNodeXML::getAttribute(const std::string& field_name, std::string& field_value) const
{
	return false;
}

bool DescriptionTreeNodeXML::getChildrenNodes(const std::string& type, ConstNodeList& children) const
{
	bool foundChildren = false;
	for (TiXmlElement* mst_element = node->FirstChildElement(type.c_str()); 
		mst_element != NULL; 
		mst_element = mst_element->NextSiblingElement(type.c_str())) 
	{
		DescriptionTreeNode::Ptr nextChild(new DescriptionTreeNodeXML(mst_element));
		children.push_back(nextChild);
		foundChildren = true;
	}

	return foundChildren;
}

bool DescriptionTreeNodeXML::getChildrenNodes(ConstNodeList& children) const
{
	bool foundChildren = false;
	for (TiXmlElement* mst_element = node->FirstChildElement(); 
		mst_element != NULL; 
		mst_element = mst_element->NextSiblingElement() )
	{
		DescriptionTreeNode::Ptr nextChild(new DescriptionTreeNodeXML(mst_element));
		children.push_back(nextChild);
		foundChildren = true;
	}

	return foundChildren;
}
}
