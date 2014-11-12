#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_

#include <string>
#include <cassert>

// FIXME remove
#include <iostream>

namespace ha {

	class DescriptionTreeNode {

	protected:

	private:  

	public:
		
		// Override these two methods in the implementation class (i.e. DescriptionTreeNodeTinyXML)

		/**
		* getAttribute
		* returns true, if field_name exists
		* returns string value of field field_name in field_value
		*/
		virtual bool getAttribute(const std::string& field_name, std::string& field_value) = 0;

		/**
		* getChilNode
		* returns true, if node has child field_name
		* returns child node in child_node
		*/
		virtual bool getChildNode(const std::string& field_name, const DescriptionTreeNode& child_node) = 0;

		//Implement these helper functions here (internally they call getAttribute)
		bool getAttributeBool(const std::string& field_name, bool& return_value, bool default_value = false);			
		bool getAttributeString(const std::string& field_name, std::string& return_value, std::string& default_value = std::string(""));
	
	};

}

#endif
