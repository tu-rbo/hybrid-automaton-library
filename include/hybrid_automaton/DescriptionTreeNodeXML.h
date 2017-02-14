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
#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_XML_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_XML_H_

#include <string>
#include <cassert>

#include <boost/shared_ptr.hpp>

#include "tinyxml.h"

#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/DescriptionTreeNode.h"

namespace ha {

	class DescriptionTreeNodeXML;
	typedef boost::shared_ptr<DescriptionTreeNodeXML> DescriptionTreeNodeXMLPtr;
	typedef boost::shared_ptr<const DescriptionTreeNodeXML> DescriptionTreeNodeXMLConstPtr;

    /**
    * @brief A xml implementation of the DescriptionTreeNode
    */
	class DescriptionTreeNodeXML: public DescriptionTreeNode{

	protected:
		/** 
		* @brief the underlying TinyXml element
		* This member can have two possible owners - if this node is part of a tree (_node_is_in_tree = true), TinyXML will delete the TiXmlElement
		* If not, it will be deleted in the destructor.
		*/
		TiXmlElement* _tinyxml_node;
		bool	      _node_is_in_tree;

	public:


		typedef boost::shared_ptr<DescriptionTreeNodeXML> Ptr;
		typedef boost::shared_ptr<const DescriptionTreeNodeXML> ConstPtr;

		/** 
		 * @brief Constructor for DescriptionTreeNodeXML
		 */
		DescriptionTreeNodeXML(const std::string& type);

		DescriptionTreeNodeXML(TiXmlElement* tynyxml_node);

		DescriptionTreeNodeXMLPtr clone() const {
			return DescriptionTreeNodeXMLPtr(_doClone());
		}


		/**
         * @brief Copy constructor
		 */
		DescriptionTreeNodeXML(const DescriptionTreeNodeXML& dtn);

		virtual ~DescriptionTreeNodeXML();

		virtual const std::string getType() const;

		/**
		* addNodeToTree
		* This function should only be called by the DescriptionTreeXML to note that ownership of
		* the TIXMLNode now belongs to the TIXMLTree
		*/
		virtual void addNodeToTree() {_node_is_in_tree = true;};

		/**
		* getChildrenNodes
		* returns true, if node has at least one child of type type
		* returns child nodes in children
		*/
		virtual bool getChildrenNodes(const std::string& type, ConstNodeList& children) const;
		
		/**
		* getChildrenNodes
		* returns true, if node has at least one child of any type
		* returns all child nodes
		*/
		virtual bool getChildrenNodes(ConstNodeList& children) const;
		
		/**
		* setAttribute 
		* @param field_name returns string value of field field_name in field_value
		*/
		virtual void addChildNode(const DescriptionTreeNode::Ptr& child);

		virtual void getAllAttributes(std::map<std::string, std::string> & attrs) const;

		TiXmlElement* getXMLNode() const;

	protected:

		virtual bool getAttributeString(const std::string& field_name, std::string& field_value) const;

		/**
		* setAttribute 
		* @param field_name returns string value of field field_name in field_value
		*/
		virtual void setAttributeString(const std::string& field_name, const std::string& field_value);


		virtual DescriptionTreeNodeXML* _doClone() const {
			return new DescriptionTreeNodeXML(*this);
		}

	};

}

#endif
