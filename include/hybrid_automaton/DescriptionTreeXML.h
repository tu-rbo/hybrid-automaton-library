#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_XML_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_XML_H_

#include <string>
#include <assert.h>

#include <boost/shared_ptr.hpp>

#include "thirdparty/tinyxml/include/tinyxml.h"

#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/DescriptionTreeNodeXML.h"

// FIXME remove
#include <iostream>

namespace ha {

	class DescriptionTreeXML;
	typedef boost::shared_ptr<DescriptionTreeXML> DescriptionTreeXMLPtr;
	typedef boost::shared_ptr<const DescriptionTreeXML> DescriptionTreeXMLConstPtr;

	class DescriptionTreeXML: public DescriptionTree{
	public:
		typedef boost::shared_ptr<DescriptionTreeXML> Ptr;
		typedef boost::shared_ptr<const DescriptionTreeXML> ConstPtr;

	public:		
		DescriptionTreeXML();

		virtual ~DescriptionTreeXML();

		/**
		 * @brief Factory method for creating DescriptionTreeNodes of this type
		 */
		virtual DescriptionTreeNode::Ptr createNode(const std::string& type) const;

		// Generate / parse Description Tree
		virtual bool initTree(const std::string& input);

		//Return first tree element
		virtual DescriptionTreeNode::Ptr getRootNode();
	
	protected: 
		TiXmlDocument* _tinyxml_document;
		DescriptionTreeNodeXML::Ptr _root_node;
	};
}

#endif
