/*
 * Copyright 2015-2017, Robotics and Biology Lab, TU Berlin
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the 
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/DescriptionTreeNodeXML.h"

namespace ha {

	DescriptionTreeNodeXML::DescriptionTreeNodeXML(const std::string& type)
		: _tinyxml_node(new TiXmlElement(type.c_str())),
		  _node_is_in_tree(false)
	{		
	}

	DescriptionTreeNodeXML::DescriptionTreeNodeXML(TiXmlElement* tinyxml_node)
		: _tinyxml_node(tinyxml_node),
		_node_is_in_tree(false)

	{
	}

	DescriptionTreeNodeXML::DescriptionTreeNodeXML(const DescriptionTreeNodeXML& dtn)
	{
		this->_tinyxml_node = dtn._tinyxml_node;
		this->_node_is_in_tree = dtn._node_is_in_tree;
	}

	DescriptionTreeNodeXML::~DescriptionTreeNodeXML()
	{
		//Only delete the TIXML element, if it is not yet assigned to a TiXMLTree!
		//Otherwise TinyXML will clean up
		if(!this->_node_is_in_tree)
			delete this->_tinyxml_node;
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
			DescriptionTreeNodeXML::Ptr nextChild(new DescriptionTreeNodeXML(mst_element));
			nextChild->addNodeToTree();
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
			DescriptionTreeNodeXML::Ptr nextChild(new DescriptionTreeNodeXML(mst_element));
			nextChild->addNodeToTree();
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
	
		childXMLNode->_node_is_in_tree = true;
		_tinyxml_node->LinkEndChild(childXMLNode->_tinyxml_node);

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
