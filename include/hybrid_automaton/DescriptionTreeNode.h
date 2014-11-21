/*!
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

	// Forward declaration to avoid circular dependencies
	//class ha_ostringstream;

	// forward declaration
	//class DescriptionTree;

	//class ha_istringstream// : public std::istringstream
	//{
	//public:
	//	ha_istringstream(std::string string)
	///*		:
	//	std::istringstream(string)*/
	//	{
	//	}

	//	
	//	ha_istringstream& operator>>(Eigen::MatrixXd& vector);
	//};

	//class ha_ostringstream// : public std::ostringstream
	//{
	//public:
	//	ha_ostringstream& operator<<(const ::Eigen::MatrixXd& vector);
	//};




	class DescriptionTreeNode;
	typedef boost::shared_ptr<DescriptionTreeNode> DescriptionTreeNodePtr;
	typedef boost::shared_ptr<const DescriptionTreeNode> DescriptionTreeNodeConstPtr;

	/*!
	* \brief
	* General interface for a hierarchical, text based description object.
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
		// Override all following methods in the implementation class (i.e. DescriptionTreeNodeTinyXML)
		///////////////////////////////////////////////////////////////////////////////////////////////

		virtual const std::string getType() const = 0;

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
		virtual void addChildNode(const DescriptionTreeNode::Ptr& child) = 0;

		virtual void getAllAttributes(std::map<std::string, std::string> & attrs) const = 0;

		///////////////////////////////////////////////////////////////////////////////////////////////
		//Implement these helper functions here (internally they will call getAttribute)
		///////////////////////////////////////////////////////////////////////////////////////////////
		template <typename T> void setAttribute(const std::string& field_name, const T& field_value)
		{
			//std::ostringstream ss;
			ha_ostringstream ha_oss;
			ha_oss << field_value ;
			//std::cout << "in setAttribute: " << ss.str() << std::endl;
			this->setAttributeString(field_name, ha_oss.str());
		}



		friend std::istringstream& operator>>(std::istringstream& iss, Eigen::MatrixXd& matrix)
		{
			int num_rows = 0;
			int num_cols = 0;
			std::string deparsing_string;

			// First we read the char '['
			getline(iss,deparsing_string,'[');

			// Then we read the number of rows
			getline(iss,deparsing_string,',');
			std::istringstream iss_num_rows(deparsing_string);
			iss_num_rows >> num_rows;
			//std::cout << "Num of rows: " << num_rows << std::endl;

			// Then we read the number of cols
			getline(iss,deparsing_string,']');
			std::istringstream iss_num_cols(deparsing_string);
			iss_num_cols >> num_cols;
			//std::cout << "Num of cols: " << num_cols << std::endl;

			// Resize the output matrix
			matrix.resize(num_rows,num_cols);

			// Read matrix values from the string
			double matrix_element = -1.0;
			for(int i = 0; i<num_rows; ++i)
			{
				getline(iss,deparsing_string,';');
				std::istringstream iss_row(deparsing_string);
				for(int j=0; j<num_cols; ++j)
				{
					getline(iss_row, deparsing_string, ',');
					std::istringstream iss_element(deparsing_string);
					iss_element >> matrix_element;
					matrix(i,j) = matrix_element;
				}
			}

			return iss;
		};

		template <typename T> bool getAttribute(const std::string& field_name, T& return_value, const T& default_value) const
		{
			std::string val;
			bool ret = this->getAttributeString(field_name, val);
			if (ret)
			{
				std::istringstream ss(val);
				ss >> return_value;

				return true;
			}
			else
			{
				return_value = default_value;
				return false;
			}
		}

		template <typename T> bool getAttribute(const std::string& field_name, T& return_value) const
		{
			std::string val;
			bool ret = this->getAttributeString(field_name, val);
			if (ret)
			{
				std::istringstream ss(val);
				ss >> return_value;
				return true;
			}
			else
			{
				return false;
			}
		}

	protected:
		/**
		* setAttribute 
		* @param field_name returns string value of field field_name in field_value
		*/
		virtual void setAttributeString(const std::string& field_name, const std::string& field_value) = 0;

		/**
		* getAttribute
		* @return true, if field_name exists
		* @param field_name returns string value of field field_name in field_value
		*/
		virtual bool getAttributeString(const std::string& field_name, std::string& field_value) const = 0;

		virtual DescriptionTreeNode* _doClone() const = 0;
	};

}

#endif
