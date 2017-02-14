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
#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_XML_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_XML_H_

#include <string>
#include <assert.h>

#include <boost/shared_ptr.hpp>

#include "tinyxml.h"

#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/DescriptionTreeNodeXML.h"

namespace ha {

	class DescriptionTreeXML;
	typedef boost::shared_ptr<DescriptionTreeXML> DescriptionTreeXMLPtr;
	typedef boost::shared_ptr<const DescriptionTreeXML> DescriptionTreeXMLConstPtr;

    /**
    * @brief A xml implementation of the DescriptionTree
    */
	class DescriptionTreeXML: public DescriptionTree{
	public:
		typedef boost::shared_ptr<DescriptionTreeXML> Ptr;
		typedef boost::shared_ptr<const DescriptionTreeXML> ConstPtr;

	public:		
		DescriptionTreeXML();
		DescriptionTreeXML(const std::string& input);

		virtual ~DescriptionTreeXML();

		/**
		 * @brief Factory method for creating DescriptionTreeNodes of this type
		 */
		virtual DescriptionTreeNode::Ptr createNode(const std::string& type) const;

        /**
         * @brief Generate / parse description tree
         *
         * @throws Error if there are any xml syntax errors
         */
		virtual bool initTree(const std::string& input);

        /**
         * @brief Deparse xml tree into string
         */
		virtual std::string writeTreeXML() const;

		virtual DescriptionTreeNode::Ptr getRootNode();

		virtual void setRootNode(const DescriptionTreeNode::Ptr& root_node);
	
	protected: 
        /**
         * @brief The underlying Tinyxml Document for parsing/deparsing
         */
		TiXmlDocument* _tinyxml_document;

        /**
         * @brief The root xml node
         */
		DescriptionTreeNodeXML::Ptr _root_node;
	};
}

#endif
