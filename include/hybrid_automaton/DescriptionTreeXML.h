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

	class DescriptionTreeXML: public DescriptionTree{

	protected:

	private:  
		TiXmlDocument _document;
		DescriptionTreeNode::Ptr _rootNode;

	public:		
		DescriptionTreeXML();

		/**
		 * @brief Factory method for creating DescriptionTreeNodes of this type
		 */
		virtual DescriptionTreeNode::Ptr createNode(const std::string& type) const;

		// Generate / parse Description Tree
		virtual bool initTree(const std::string& input);

		//Return first tree element
		virtual bool getRootNode(DescriptionTreeNode::Ptr root_node);
	};
}

#endif
