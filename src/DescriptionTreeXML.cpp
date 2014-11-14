#include "hybrid_automaton/DescriptionTreeXML.h"
namespace ha {

	DescriptionTreeXML::DescriptionTreeXML()
	{
		TiXmlHandle docHandle(&_document);
		_rootNode = DescriptionTreeNodeXML::Ptr(new DescriptionTreeNodeXML(docHandle.Element()));
		//std::cout<<_rootNode->Value();
	}

	/**
	 * @brief Factory method for creating DescriptionTreeNodes of this type
	 */
	DescriptionTreeNode::Ptr DescriptionTreeXML::createNode(const std::string& type) const {
		return DescriptionTreeNode::Ptr (new DescriptionTreeNodeXML(type));
	}


	bool DescriptionTreeXML::initTree(const std::string& input)
	{
		const char* ret_val = _document.Parse(input.c_str());
		
		if(ret_val == NULL && _document.Error()) 	
		{
			std::cout << "ERROR CODE: " <<  _document.ErrorId() << std::endl;
			std::cout << _document.ErrorDesc() << std::endl;
			throw std::string("[DescriptionTreeXML::initTree] ERROR: Parsing the xml document.");
			return false;
		}
		
		TiXmlHandle docHandle(&_document);
		_rootNode = DescriptionTreeNodeXML::Ptr(new DescriptionTreeNodeXML(docHandle.Element()));
		
		// Check if the HybridAutomaton element was found
		if (_rootNode == NULL) {
			throw std::string("[DescriptionTreeXML::initTree] ERROR: undefined");
			return false;
		}

		return true;
	}

	DescriptionTreeNode::Ptr DescriptionTreeXML::getRootNode()
	{
		return this->_rootNode;
	}

}
