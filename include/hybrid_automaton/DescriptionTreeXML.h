#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_XML_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_XML_H_

#include <string>
#include <assert.h>

#include <boost/shared_ptr.hpp>

#include "tinyxml.h"

#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/DescriptionTreeNodeXML.h"

// FIXME remove
#include <iostream>

namespace ha {

	class DescriptionTreeXML: public DescriptionTree{

	protected:

	private:  
		TiXmlDocument _document;
		boost::shared_ptr<TiXmlElement> _rootNode;

	public:		
		// Generate / parse Description Tree
		virtual bool initTree(std::istream input);

		//Return first tree element
		bool getRootNode(DescriptionTreeNode* root_node);
	};
}

#endif
