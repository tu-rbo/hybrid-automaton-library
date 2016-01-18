/**
* DescriptionTreeNode.h
* 
* Copyright (c) 2014 by RBO TU Berlin
*/

#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_

#include <string>
#include <assert.h>
#include <list>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <vector>
#include <map>

#include "hybrid_automaton/HybridAutomatonOStringStream.h"

namespace ha {

	class DescriptionTreeNode;
	typedef boost::shared_ptr<DescriptionTreeNode> DescriptionTreeNodePtr;
	typedef boost::shared_ptr<const DescriptionTreeNode> DescriptionTreeNodeConstPtr;

	/**
    * @brief A General interface for a hierarchical, text based description object - part of the DescriptionTree.
	* 
	* Code against this interface to integrate your xml, yaml, whatever - based description of
	* hybrid automata.
	*/
	class DescriptionTreeNode {	

	protected:

	private:  

	public:
		typedef boost::shared_ptr<DescriptionTreeNode> Ptr;
		typedef boost::shared_ptr<const DescriptionTreeNode> ConstPtr;

		typedef std::list<DescriptionTreeNode::ConstPtr> ConstNodeList;
		typedef std::list<DescriptionTreeNode::ConstPtr>::const_iterator ConstNodeListIterator;


		DescriptionTreeNode();

		virtual ~DescriptionTreeNode();

		DescriptionTreeNode(const DescriptionTreeNode& dtn);

		DescriptionTreeNodePtr clone() const {
			return DescriptionTreeNodePtr(_doClone());
		}


		///////////////////////////////////////////////////////////////////////////////////////////////
        // Override all following methods in the implementation class (i.e. DescriptionTreeNodeXML)
		///////////////////////////////////////////////////////////////////////////////////////////////

		virtual const std::string getType() const = 0;

		/**
        * @brief getChildrenNodes returns true, if node has at least one child of type \a type
        *
        * returns child nodes in children
		*/
		virtual bool getChildrenNodes(const std::string& type, ConstNodeList& children) const = 0;

		/**
        * @brief getChildrenNodes returns true, if node has at least one child of any type
		* returns all child nodes
		*/
		virtual bool getChildrenNodes(ConstNodeList& children) const = 0;

		/**
        * @brief setAttribute
        * @param field_name returns string value of field \a field_name in \a field_value
		*/
		virtual void addChildNode(const DescriptionTreeNode::Ptr& child) = 0;

        /**
        * @brief dump all attributesd of this Node
        */
		virtual void getAllAttributes(std::map<std::string, std::string> & attrs) const = 0;

		///////////////////////////////////////////////////////////////////////////////////////////////
		//Implement these helper functions here (internally they will call getAttribute)
		///////////////////////////////////////////////////////////////////////////////////////////////

        /**
        * @brief setAttribute sets the field \a field_name to value \a field_value.
        *
        * Specify type with the template parameter T.
        * Your type T needs to implement the << operator.
        * If you want to use a different format than the << operator, edit ha_ostringstream.
        */
        template <typename T> void setAttribute(const std::string& field_name, const T& field_value)
		{
			ha_ostringstream ha_oss;
			ha_oss << field_value ;
			this->setAttributeString(field_name, ha_oss.str());
		}

        /**
        * @brief getAttribute get the value \a field_value of field \a field_name.
        * @param default_vlaue the value to return if field does not exist.
        * @returns true if field exists, false if it does not.
        */
		template <typename T> bool getAttribute(const std::string& field_name, T& return_value, const T& default_value) const
		{
			std::string val;
			bool ret = this->getAttributeString(field_name, val);
			if (ret)
			{
				ha_ostringstream ss(val);
				ss >> return_value;

				return true;
			}
			else
			{
				return_value = default_value;
				return false;
			}
		}

        /**
        * @brief getAttribute get the value \a field_value of field \a field_name.
        * @returns true if field exists, false if it does not.
        */
		template <typename T> bool getAttribute(const std::string& field_name, T& return_value) const
		{
			std::string val;
			bool ret = this->getAttributeString(field_name, val);
			if (ret)
			{
				ha_ostringstream ss(val);
				ss >> return_value;
				return true;
			}
			else
			{
				return false;
			}
		}

	protected:

		virtual void setAttributeString(const std::string& field_name, const std::string& field_value) = 0;

		virtual bool getAttributeString(const std::string& field_name, std::string& field_value) const = 0;

		virtual DescriptionTreeNode* _doClone() const = 0;
	};

}

#endif
