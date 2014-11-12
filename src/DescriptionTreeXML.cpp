#include "hybrid_automaton/DescriptionTreeXML.h"
namespace ha {

	bool DescriptionTreeXML::initTree(std::istream input)
	{
		//istream to std::string
		std::string tmp( (std::istreambuf_iterator<char>( input )),
               (std::istreambuf_iterator<char>()));

		const char* ret_val = _document.Parse(tmp.c_str());
		
		if(ret_val == NULL && _document.Error()) 	
		{
			std::cout << "ERROR CODE: " <<  _document.ErrorId() << std::endl;
			std::cout << _document.ErrorDesc() << std::endl;
			throw std::string("[XMLDeserializer::createHybridAutomaton] ERROR: Parsing the string.");
			return false;
		}
		
		TiXmlHandle docHandle(&_document);

		_rootNode;


		return true;
	}

	bool DescriptionTreeXML::getRootNode(DescriptionTreeNode* root_node)
	{
		root_node = new DescriptionTreeNodeXML();
		return true;
	}

}
