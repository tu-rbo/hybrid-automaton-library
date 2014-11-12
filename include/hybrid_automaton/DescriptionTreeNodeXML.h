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
		TiXmlElement node;

	public:
		
		/**
		* getAttribute
		* returns true, iff field_name exists
		* returns string value of field field_name in field_value
		*/
		virtual bool getAttribute(const std::string& field_name, std::string& field_value);

		/**
		* getChildNode
		* returns true, iff node has child field_name
		* returns child node in child_node
		*/
		virtual bool getChildNode(const std::string& field_name, const DescriptionTreeNode& child_node);

	};

}

#endif
