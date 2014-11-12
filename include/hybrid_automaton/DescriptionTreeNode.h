#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_

#include <string>
#include <assert.h>
#include <list>

// FIXME remove
#include <iostream>

namespace ha {

	class DescriptionTreeNode {	

	protected:

	private:  

	public:

		typedef std::list<const DescriptionTreeNode> ConstNodeList;
		
		// Override these two methods in the implementation class (i.e. DescriptionTreeNodeTinyXML)

		
		virtual const std::string getType();

		/**
		* getAttribute
		* @return true, if field_name exists
		* @param field_name returns string value of field field_name in field_value
		*/
		virtual bool getAttribute(const std::string& field_name, std::string& field_value) const = 0;

		/**
		* getChildrenNodes
		* returns true, if node has at least one child of type type
		* returns child nodes in children
		*/
		virtual bool getChildrenNodes(const std::string& type, const ConstNodeList& children) const = 0;
		
		/**
		* getChildrenNodes
		* returns true, if node has at least one child of any type
		* returns all child nodes
		*/
		virtual bool getChildrenNodes(const ConstNodeList& children) const = 0;

		//Implement these helper functions here (internally they call getAttribute)
		virtual bool getAttributeBool(const std::string& field_name, bool& return_value, bool default_value = false) const;			
		virtual bool getAttributeString(const std::string& field_name, std::string& return_value, std::string& default_value = std::string("")) const;
	
	};

}

#endif
