#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_XML_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_XML_H_

#include <string>
#include <assert.h>

#include <boost/shared_ptr.hpp>

#include "tinyxml.h"

#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/DescriptionTreeNodeXML.h"

namespace ha {

	class DescriptionTreeXML;
	typedef boost::shared_ptr<DescriptionTreeXML> DescriptionTreeXMLPtr;
	typedef boost::shared_ptr<const DescriptionTreeXML> DescriptionTreeXMLConstPtr;

    /**
    * @brief A xml implementation of the DescriptionTree
    */
	class DescriptionTreeXML: public DescriptionTree{
	public:
		typedef boost::shared_ptr<DescriptionTreeXML> Ptr;
		typedef boost::shared_ptr<const DescriptionTreeXML> ConstPtr;

	public:		
		DescriptionTreeXML();
		DescriptionTreeXML(const std::string& input);

		virtual ~DescriptionTreeXML();

		/**
		 * @brief Factory method for creating DescriptionTreeNodes of this type
		 */
		virtual DescriptionTreeNode::Ptr createNode(const std::string& type) const;

        /**
         * @brief Generate / parse description tree
         *
         * @throws Error if there are any xml syntax errors
         */
		virtual bool initTree(const std::string& input);

        /**
         * @brief Deparse xml tree into string
         */
		virtual std::string writeTreeXML() const;

		virtual DescriptionTreeNode::Ptr getRootNode();

		virtual void setRootNode(const DescriptionTreeNode::Ptr& root_node);
	
	protected: 
        /**
         * @brief The underlying Tinyxml Document for parsing/deparsing
         */
		TiXmlDocument* _tinyxml_document;

        /**
         * @brief The root xml node
         */
		DescriptionTreeNodeXML::Ptr _root_node;
	};
}

#endif
