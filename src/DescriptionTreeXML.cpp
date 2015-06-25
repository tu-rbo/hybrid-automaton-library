#include "hybrid_automaton/DescriptionTreeXML.h"
#include "hybrid_automaton/error_handling.h"

namespace ha {

	DescriptionTreeXML::DescriptionTreeXML():
		_tinyxml_document(new TiXmlDocument())
	{
		// Create the first (and only) root element and link it to the base document
		this->_root_node.reset(new DescriptionTreeNodeXML(new TiXmlElement("HybridAutomaton")));
		this->_root_node->addNodeToTree();	
		this->_tinyxml_document->LinkEndChild(this->_root_node->getXMLNode());
	}
		
	DescriptionTreeXML::DescriptionTreeXML(const std::string& input)
		: _tinyxml_document(new TiXmlDocument())
	{
		this->initTree(input);
	}

	DescriptionTreeXML::~DescriptionTreeXML()
	{
		delete _tinyxml_document;
	}

	/**
	 * @brief Factory method for creating DescriptionTreeNodes of this type
	 */
	DescriptionTreeNode::Ptr DescriptionTreeXML::createNode(const std::string& type) const {
		return DescriptionTreeNode::Ptr (new DescriptionTreeNodeXML(type));
	}


	bool DescriptionTreeXML::initTree(const std::string& input)
	{
		_tinyxml_document->Clear();
		const char* ret_val = _tinyxml_document->Parse(input.c_str());
		
		if(ret_val == NULL && _tinyxml_document->Error()) 	
		{
            HA_THROW_ERROR("DescriptionTreeXML.initTree", "Parsing the xml document: " <<
                           "ERROR CODE: " <<  _tinyxml_document->ErrorId() << std::endl <<
                           _tinyxml_document->ErrorDesc() << std::endl);
			return false;
		}
		
		TiXmlHandle docHandle(this->_tinyxml_document);
		this->_root_node.reset(new DescriptionTreeNodeXML(docHandle.FirstChild().ToElement()));
		_root_node->addNodeToTree();
		// Check if the HybridAutomaton element was found
		if (this->_root_node == NULL) {
			HA_THROW_ERROR("DescriptionTreeXML.initTree", "undefined");
			return false;
		}
		return true;
	}

	std::string DescriptionTreeXML::writeTreeXML() const
	{
		TiXmlPrinter printer;
		printer.SetIndent( "\t" );

		_tinyxml_document->Accept( &printer );
		return std::string(printer.CStr());		
	}

	DescriptionTreeNode::Ptr DescriptionTreeXML::getRootNode()
	{
		return this->_root_node;
	}

	void DescriptionTreeXML::setRootNode(const DescriptionTreeNode::Ptr& root_node)
	{
		this->_tinyxml_document->Clear();
		this->_root_node = boost::dynamic_pointer_cast<DescriptionTreeNodeXML>(root_node);
		_root_node->addNodeToTree();
		this->_tinyxml_document->LinkEndChild(this->_root_node->getXMLNode());
	}

}
