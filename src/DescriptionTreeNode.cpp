#include "hybrid_automaton/DescriptionTreeNode.h"

#include <algorithm>

namespace ha {


	DescriptionTreeNode::DescriptionTreeNode()
	{

	}

	DescriptionTreeNode::~DescriptionTreeNode()
	{

	}

	DescriptionTreeNode::DescriptionTreeNode(const DescriptionTreeNode& dtn)
	{

	}


	//bool DescriptionTreeNode::
	//	getAttributeBool(const std::string& field_name, bool& return_value, bool default_value) const
	//{
	//	std::string val;
	//	bool ret = getAttribute(field_name, val);
	//	std::transform(val.begin(), val.end(), val.begin(), ::tolower);
	//	if (ret)
	//	{
	//		if (val == "true")
	//		{
	//			return_value = true;
	//		}
	//		else if (val == "false")
	//		{
	//			return_value = false;
	//		}
	//		else
	//		{
	//			//Malformed field value
	//			throw("todo");
	//			assert(0);
	//		}
	//		return true;
	//	}
	//	else
	//	{
	//		return_value = default_value;
	//		return false;
	//	}
	//}

	//bool DescriptionTreeNode::
	//	getAttributeString(const std::string& field_name, std::string& return_value, std::string& default_value) const
	//{
	//	std::string val;
	//	bool ret = getAttribute(field_name, val);
	//	if (ret)
	//	{
	//		return_value = val;
	//		return true;
	//	}
	//	else
	//	{
	//		return_value = default_value;
	//		return false;
	//	}
	//}

}
