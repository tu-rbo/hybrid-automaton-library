#include "hybrid_automaton/DescriptionTreeNodeXML.h"
namespace ha {

	DescriptionTreeNodeXML::DescriptionTreeNodeXML(const std::string& type )
	{
		_node = boost::shared_ptr<TiXmlElement>(new TiXmlElement(type.c_str()));
	}

	DescriptionTreeNodeXML::DescriptionTreeNodeXML(TiXmlElement* xmlNode):
	_node(xmlNode)
	{
	}

	DescriptionTreeNodeXML::DescriptionTreeNodeXML(const DescriptionTreeNodeXML& dtn)
	{
		this->_node = dtn._node;
	}

	const std::string DescriptionTreeNodeXML::getType() const
	{
		return std::string(_node->Value());
	}

	bool DescriptionTreeNodeXML::getAttributeString(const std::string& field_name, std::string& field_value) const
	{
		const char* value = _node->Attribute(field_name.c_str());
		if(value != NULL)
		{
			field_value = std::string(value);
			return true;
		}
		else
			return false;
	}

	bool DescriptionTreeNodeXML::getChildrenNodes(const std::string& type, ConstNodeList& children) const
	{
		bool foundChildren = false;
		for (TiXmlElement* mst_element = _node->FirstChildElement(type.c_str()); 
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
		for (TiXmlElement* mst_element = _node->FirstChildElement(); 
			mst_element != NULL; 
			mst_element = mst_element->NextSiblingElement() )
		{
			DescriptionTreeNode::Ptr nextChild(new DescriptionTreeNodeXML(mst_element));
			children.push_back(nextChild);
			foundChildren = true;
		}

		return foundChildren;
	}

	void DescriptionTreeNodeXML::setAttributeString(const std::string& field_name, const std::string& field_value)
	{
		_node->SetAttribute(field_name.c_str(), field_value.c_str());
	}

	void DescriptionTreeNodeXML::addChildNode(const DescriptionTreeNode::Ptr& child) 
	{
		//Downcast
		DescriptionTreeNodeXML::Ptr childXMLNode = boost::dynamic_pointer_cast<DescriptionTreeNodeXML>(child);
		_node->InsertEndChild(*(childXMLNode->_node));
	}

}
