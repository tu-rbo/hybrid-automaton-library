#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_NODE_H_

#include <string>

// FIXME remove
#include <iostream>

namespace ha {

	class DescriptionTreeNode {

	protected:

	private:  

	public:
		
		// Override these in the implementation class (i.e. DescriptionTreeNodeTinyXML)
		virtual std::string getAttribute(const std::string& field_name) = 0;
		virtual const DescriptionTreeNode& getChildNode(const std::string& field_name) = 0;

		//Implement these helper functions here (internally they call getAttribute)
		virtual bool getAttributeBool(const std::string& field_name, bool default_value)
		{
			std::string val = getAttribute(field_name);
			if (!val.empty())
			{
				if (val == "true")
				{
					return true;
				}
				else if (val == "false")
				{
					return false;
				}
			}
			return default_value;
		}

	};

}

#endif
