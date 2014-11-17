#include "hybrid_automaton/DescriptionTreeXML.h"
namespace ha {

	DescriptionTreeXML::DescriptionTreeXML():
		_tinyxml_document(new TiXmlDocument())
	{
		// ACHTUNG: This is ugly but is the only way to avoid segmentation fault
		// TinyXML stores a list of pointers to the elements
		// Externally, we also store smart pointers to the same elements
		// When it goes out of scope, the objected pointed by the smart pointers are deleted
		// If tinyxml_document is also a smart pointer, the object will also get deleted, deleting the object pointed by root_node, 
		// that is also cleared by the smart pointer of root_node
		// SOLUTION: tinyxml_document is created with new and never deleted.
		//this->_tinyxml_document = new TiXmlDocument();

		// Create the first (and only) root element and link it to the base document
		this->_root_node.reset(new DescriptionTreeNodeXML(new TiXmlElement("HybridAutomaton")));

		this->_tinyxml_document->LinkEndChild(this->_root_node->getXMLNode());
	}

	DescriptionTreeXML::~DescriptionTreeXML()
	{
		// This tries to delete an element that is also deleted by a smart pointer (read note in the constructor)
		//delete this->_tinyxml_document;
	}

	/**
	 * @brief Factory method for creating DescriptionTreeNodes of this type
	 */
	DescriptionTreeNode::Ptr DescriptionTreeXML::createNode(const std::string& type) const {
		return DescriptionTreeNode::Ptr (new DescriptionTreeNodeXML(type));
	}


	bool DescriptionTreeXML::initTree(const std::string& input)
	{
		const char* ret_val = _tinyxml_document->Parse(input.c_str());
		
		if(ret_val == NULL && _tinyxml_document->Error()) 	
		{
			std::cout << "ERROR CODE: " <<  _tinyxml_document->ErrorId() << std::endl;
			std::cout << _tinyxml_document->ErrorDesc() << std::endl;
			throw std::string("[DescriptionTreeXML::initTree] ERROR: Parsing the xml document.");
			return false;
		}
		
		TiXmlHandle docHandle(this->_tinyxml_document.get());
		this->_root_node.reset(new DescriptionTreeNodeXML(docHandle.Element()));
		
		// Check if the HybridAutomaton element was found
		if (this->_root_node == NULL) {
			throw std::string("[DescriptionTreeXML::initTree] ERROR: undefined");
			return false;
		}
		return true;
	}

	DescriptionTreeNode::Ptr DescriptionTreeXML::getRootNode()
	{
		return this->_root_node;
	}

}
