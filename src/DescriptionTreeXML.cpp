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
#include "hybrid_automaton/DescriptionTreeXML.h"
#include "hybrid_automaton/error_handling.h"

namespace ha {

	DescriptionTreeXML::DescriptionTreeXML():
		_tinyxml_document(new TiXmlDocument())
	{
		// Create the first (and only) root element and link it to the base document
		this->_root_node.reset(new DescriptionTreeNodeXML(new TiXmlElement("HybridAutomaton")));
		this->_root_node->addNodeToTree();	
		this->_tinyxml_document->LinkEndChild(this->_root_node->getXMLNode());
	}
		
	DescriptionTreeXML::DescriptionTreeXML(const std::string& input)
		: _tinyxml_document(new TiXmlDocument())
	{
		this->initTree(input);
	}

	DescriptionTreeXML::~DescriptionTreeXML()
	{
		delete _tinyxml_document;
	}

	/**
	 * @brief Factory method for creating DescriptionTreeNodes of this type
	 */
	DescriptionTreeNode::Ptr DescriptionTreeXML::createNode(const std::string& type) const {
		return DescriptionTreeNode::Ptr (new DescriptionTreeNodeXML(type));
	}


	bool DescriptionTreeXML::initTree(const std::string& input)
	{
		_tinyxml_document->Clear();
		const char* ret_val = _tinyxml_document->Parse(input.c_str());
		
		if(ret_val == NULL && _tinyxml_document->Error()) 	
		{
            HA_THROW_ERROR("DescriptionTreeXML.initTree", "Parsing the xml document: " <<
                           "ERROR CODE: " <<  _tinyxml_document->ErrorId() << std::endl <<
                           _tinyxml_document->ErrorDesc() << std::endl);
			return false;
		}
		
		TiXmlHandle docHandle(this->_tinyxml_document);
		this->_root_node.reset(new DescriptionTreeNodeXML(docHandle.FirstChild().ToElement()));
		_root_node->addNodeToTree();
		// Check if the HybridAutomaton element was found
		if (this->_root_node == NULL) {
			HA_THROW_ERROR("DescriptionTreeXML.initTree", "undefined");
			return false;
		}
		return true;
	}

	std::string DescriptionTreeXML::writeTreeXML() const
	{
		TiXmlPrinter printer;
		printer.SetIndent( "\t" );

		_tinyxml_document->Accept( &printer );
		return std::string(printer.CStr());		
	}

	DescriptionTreeNode::Ptr DescriptionTreeXML::getRootNode()
	{
		return this->_root_node;
	}

	void DescriptionTreeXML::setRootNode(const DescriptionTreeNode::Ptr& root_node)
	{
		this->_tinyxml_document->Clear();
		this->_root_node = boost::dynamic_pointer_cast<DescriptionTreeNodeXML>(root_node);
		_root_node->addNodeToTree();
		this->_tinyxml_document->LinkEndChild(this->_root_node->getXMLNode());
	}

}
