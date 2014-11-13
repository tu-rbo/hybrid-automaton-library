#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_

#include <string>
#include <assert.h>
#include <list>

#include <boost/shared_ptr.hpp>

// FIXME remove
#include <iostream>

namespace ha {

	class DescriptionTreeNode;
	typedef boost::shared_ptr<DescriptionTreeNode> DescriptionTreeNodePtr;
	typedef boost::shared_ptr<const DescriptionTreeNode> DescriptionTreeNodeConstPtr;

	class DescriptionTreeNode {	

	protected:

	private:  

	public:
		typedef boost::shared_ptr<DescriptionTreeNode> Ptr;
		typedef boost::shared_ptr<const DescriptionTreeNode> ConstPtr;

		typedef std::list<const DescriptionTreeNode::Ptr> ConstNodeList;
		

		///////////////////////////////////////////////////////////////////////////////////////////////
		// Override all following methods in the implementation class (i.e. DescriptionTreeNodeTinyXML)
		///////////////////////////////////////////////////////////////////////////////////////////////
		virtual const std::string getType() const = 0;
		
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
		virtual bool getChildrenNodes(const std::string& type, ConstNodeList& children) const = 0;

		/**
		* getChildrenNodes
		* returns true, if node has at least one child of any type
		* returns all child nodes
		*/
		virtual bool getChildrenNodes(ConstNodeList& children) const = 0;

		/**
		* setAttribute 
		* @param field_name returns string value of field field_name in field_value
		*/
		virtual void setAttribute(const std::string& field_name, std::string& field_value) = 0;
		
		/**
		* setAttribute 
		* @param field_name returns string value of field field_name in field_value
		*/
		virtual void addChildNode(DescriptionTreeNode::Ptr child) = 0;
		
		
		///////////////////////////////////////////////////////////////////////////////////////////////
		//Implement these helper functions here (internally they will call getAttribute)
		///////////////////////////////////////////////////////////////////////////////////////////////
		virtual bool getAttributeBool(const std::string& field_name, bool& return_value, bool default_value = false) const;			
		virtual bool getAttributeString(const std::string& field_name, std::string& return_value, std::string& default_value = std::string("")) const;
	};

}

#endif
