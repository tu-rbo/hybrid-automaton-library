#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_

#include <string>
#include <assert.h>
#include <list>

#include <boost/shared_ptr.hpp>

// FIXME remove
#include <iostream>

#include <Eigen/Dense>

namespace ha {


	class ha_istringstream : public std::istringstream
	{
	public:
		ha_istringstream(std::string string)
			:
		std::istringstream(string)
		{
		}
	};



	class ha_ostringstream : public std::ostringstream
	{
	};

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
		virtual void addChildNode(const DescriptionTreeNode::Ptr& child) = 0;


		friend ha_istringstream& operator>>(ha_istringstream& iss, Eigen::VectorXd& vector)
		{
			double value = -1.0;
			while(iss >> value)
			{	
				vector << value;
			}
			return iss;
		};

		friend ha_ostringstream& operator<<(ha_ostringstream& oss, Eigen::VectorXd& vector)
		{
			std::stringstream epsilon_ss;
			for(unsigned int idx=0; idx<vector.size(); ++idx)
			{
				oss << vector(idx);

				if(idx != vector.size() -1)
				{
					oss << " ";
				}
			}
			return oss;
		}

		///////////////////////////////////////////////////////////////////////////////////////////////
		//Implement these helper functions here (internally they will call getAttribute)
		///////////////////////////////////////////////////////////////////////////////////////////////
		template <typename T> void setAttribute(const std::string& field_name, const T& field_value)
		{
			ha_ostringstream ss;
			ss << field_value;
			setAttribute(field_name, ss.str());
		}

		template <typename T> bool getAttribute(const std::string& field_name, T& return_value, const T& default_value) const
		{
			std::string val;
			bool ret = getAttribute(field_name, val);
			if (ret)
			{
				ha_istringstream ss(val);
				if(ss >> return_value)
				{
					return true;
				}else{
					throw std::string("DescriptionTreeNode::getAttribute. There is no conversion from string to type T.");
				}
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
			bool ret = getAttribute(field_name, val);
			if (ret)
			{
				ha_istringstream ss(val);
				if(ss >> return_value)
				{
					return true;
				}else{
					throw std::string("DescriptionTreeNode::getAttribute. There is no conversion from string to type T.");
				}
			}
			else
			{
				return false;
			}
		}
	};

}

#endif
