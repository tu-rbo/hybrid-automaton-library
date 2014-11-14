#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_XML_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_XML_H_

#include <string>
#include <cassert>

#include <boost/shared_ptr.hpp>

#include "thirdparty/tinyxml/include/tinyxml.h"

#include "hybrid_automaton/DescriptionTreeNode.h"

namespace ha {

	class DescriptionTreeNodeXML: public DescriptionTreeNode{

	class DescriptionTreeNodeXML;
	typedef boost::shared_ptr<DescriptionTreeNodeXML> DescriptionTreeNodeXMLPtr;

	protected:

	private:  
		boost::shared_ptr<TiXmlElement> _node;

	public:

		typedef boost::shared_ptr<DescriptionTreeNodeXML> Ptr;

		DescriptionTreeNodeXML(const std::string& type);
		DescriptionTreeNodeXML(TiXmlElement* xmlNode);

		virtual const std::string getType() const;

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

	protected:

		virtual bool getAttributeString(const std::string& field_name, std::string& field_value) const;

		/**
		* setAttribute 
		* @param field_name returns string value of field field_name in field_value
		*/
		virtual void setAttributeString(const std::string& field_name, const std::string& field_value);

	};

}

#endif
