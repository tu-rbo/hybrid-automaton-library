#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_XML_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_XML_H_

#include <string>
#include <cassert>

#include <boost/shared_ptr.hpp>

#include "tinyxml.h"

#include "hybrid_automaton/DescriptionTreeNode.h"

namespace ha {

	class DescriptionTreeNodeXML: public DescriptionTreeNode{

	protected:

	private:  
		TiXmlElement* node;

	public:
		
		virtual bool getAttribute(const std::string& field_name, std::string& field_value) const;

		/**
		* getChildrenNodes
		* returns true, if node has at least one child of type type
		* returns child nodes in children
		*/
		virtual bool getChildrenNodes(const std::string& type, const ConstNodeList& children) const;
		
		/**
		* getChildrenNodes
		* returns true, if node has at least one child of any type
		* returns all child nodes
		*/
		virtual bool getChildrenNodes(const ConstNodeList& children) const;

	};

}

#endif
