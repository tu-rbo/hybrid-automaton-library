#include "hybrid_automaton/DescriptionTreeNode.h"
namespace ha {
	bool DescriptionTreeNode::
		getAttributeBool(const std::string& field_name, bool& return_value, bool default_value)
	{
		std::string val;
		bool ret = getAttribute(field_name, val);
		if (ret)
		{
			if (val == "true")
			{
				return_value = true;
			}
			else if (val == "false")
			{
				return_value = false;
			}
			else
			{
				//Malformed field value
				assert(0);
			}
			return true;
		}
		else
		{
			return_value = default_value;
			return false;
		}
	}

	bool DescriptionTreeNode::
		getAttributeString(const std::string& field_name, std::string& return_value, std::string& default_value)
	{
		std::string val;
		bool ret = getAttribute(field_name, val);
		if (ret)
		{
			return_value = val;
			return true;
		}
		else
		{
			return_value = default_value;
			return false;
		}
	}
	
}
