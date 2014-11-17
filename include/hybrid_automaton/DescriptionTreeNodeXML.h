#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_XML_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_XML_H_

#include <string>
#include <cassert>

#include <boost/shared_ptr.hpp>

#include "thirdparty/tinyxml/include/tinyxml.h"

#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/DescriptionTreeNode.h"

namespace ha {

	class DescriptionTreeNodeXML;
	typedef boost::shared_ptr<DescriptionTreeNodeXML> DescriptionTreeNodeXMLPtr;
	typedef boost::shared_ptr<const DescriptionTreeNodeXML> DescriptionTreeNodeXMLConstPtr;

	class DescriptionTreeNodeXML: public DescriptionTreeNode{

	protected:
		TiXmlElement* _tinyxml_node;

	public:

		typedef boost::shared_ptr<DescriptionTreeNodeXML> Ptr;
		typedef boost::shared_ptr<const DescriptionTreeNodeXML> ConstPtr;

		/** 
		 * @brief Constructor for DescriptionTreeNodeXML
		 */
		DescriptionTreeNodeXML(const std::string& type);

		DescriptionTreeNodeXML(TiXmlElement* tynyxml_node);

		DescriptionTreeNodeXMLPtr clone() const {
			return DescriptionTreeNodeXMLPtr(_doClone());
		}


		/*!
		 * \brief
		 * Copy constructor
		 * 
		 * \param dtn
		 * Description of parameter dtn.
		 * 
		 * \throws Nothing
		 * Description of criteria for throwing this exception.
		 * 
		 * Write detailed description for DescriptionTreeNodeXML here.
		 * 
		 * \remarks
		 * Write remarks for DescriptionTreeNodeXML here.
		 * 
		 * \see
		 * Separate items with the '|' character.
		 */
		DescriptionTreeNodeXML(const DescriptionTreeNodeXML& dtn);

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

		TiXmlElement* getXMLNode() const;

	protected:

		virtual bool getAttributeString(const std::string& field_name, std::string& field_value) const;

		/**
		* setAttribute 
		* @param field_name returns string value of field field_name in field_value
		*/
		virtual void setAttributeString(const std::string& field_name, const std::string& field_value);


		virtual DescriptionTreeNodeXML* _doClone() const {
			return new DescriptionTreeNodeXML(*this);
		}

	};

}

#endif
