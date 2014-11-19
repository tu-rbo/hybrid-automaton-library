#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/DescriptionTreeNodeXML.h"

namespace ha {

	DescriptionTreeNodeXML::DescriptionTreeNodeXML(const std::string& type)
		: _tinyxml_node(new TiXmlElement(type.c_str()))
	{		
	}

	DescriptionTreeNodeXML::DescriptionTreeNodeXML(TiXmlElement* tinyxml_node)
		: _tinyxml_node(tinyxml_node)
	{
	}

	DescriptionTreeNodeXML::DescriptionTreeNodeXML(const DescriptionTreeNodeXML& dtn)
	{
		this->_tinyxml_node = dtn._tinyxml_node;
	}

	const std::string DescriptionTreeNodeXML::getType() const
	{
		return std::string(_tinyxml_node->Value());
	}

	bool DescriptionTreeNodeXML::getAttributeString(const std::string& field_name, std::string& field_value) const
	{
		const char* value = _tinyxml_node->Attribute(field_name.c_str());
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
		for (TiXmlElement* mst_element = _tinyxml_node->FirstChildElement(type.c_str()); 
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
		for (TiXmlElement* mst_element = _tinyxml_node->FirstChildElement(); 
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
		_tinyxml_node->SetAttribute(field_name.c_str(), field_value.c_str());
	}

	void DescriptionTreeNodeXML::addChildNode(const DescriptionTreeNode::Ptr& child) 
	{
		//Downcast
		DescriptionTreeNodeXML::Ptr childXMLNode = boost::dynamic_pointer_cast<DescriptionTreeNodeXML>(child);

		_tinyxml_node->InsertEndChild(*(childXMLNode->_tinyxml_node));
	}

	void DescriptionTreeNodeXML::getAllAttributes(std::map<std::string, std::string> & attrs) const {
		attrs.clear();

		const TiXmlAttribute *ptr = _tinyxml_node->FirstAttribute();
		while(ptr != NULL)
		{
			attrs.insert(std::pair<std::string, std::string>(ptr->Name(), ptr->Value()));
			ptr = ptr->Next();
		}	
	}

	TiXmlElement* DescriptionTreeNodeXML::getXMLNode() const
	{
		return this->_tinyxml_node;
	}

}
