#include "hybrid_automaton/HybridAutomaton.h"

#include "hybrid_automaton/DescriptionTreeNode.h"

#include <sstream>

namespace ha {

	void HybridAutomaton::registerController(const std::string& crtl_name, ControllerCreator cc) {
		std::map<std::string, HybridAutomaton::ControllerCreator>& controller_type_map = getControllerTypeMap();
		assert ( controller_type_map.find(crtl_name) == controller_type_map.end() );
		controller_type_map[crtl_name] = cc;
	}

	void HybridAutomaton::registerControlSet(const std::string& crtl_name, ControlSetCreator cc) {
		std::map<std::string, HybridAutomaton::ControlSetCreator>& controlset_type_map = getControlSetTypeMap();
		assert ( controlset_type_map.find(crtl_name) == controlset_type_map.end() );
		controlset_type_map[crtl_name] = cc;
	}

	Controller::Ptr HybridAutomaton::createController(DescriptionTreeNode::Ptr node, System::Ptr system) {
		std::string crtl_type;
		node->getAttribute("type", crtl_type);

		std::map<std::string, HybridAutomaton::ControllerCreator>& controller_type_map = getControllerTypeMap();
		std::map<std::string, HybridAutomaton::ControllerCreator>::iterator it = controller_type_map.find(crtl_type);
		if ( it == controller_type_map.end() ) {
			std::stringstream ss;
			ss << "[HybridAutomaton::createController] Controller type not registered: " << crtl_type;
			throw ss.str();
		}
		return (*(it->second))(node, system);
	}

	ControlSet::Ptr HybridAutomaton::createControlSet(DescriptionTreeNode::Ptr node, System::Ptr system) {
		std::string crtl_type;
		node->getAttribute("type", crtl_type);

		std::map<std::string, HybridAutomaton::ControlSetCreator>& controlset_type_map = getControlSetTypeMap();
		std::map<std::string, HybridAutomaton::ControlSetCreator>::iterator it = controlset_type_map.find(crtl_type);
		if ( it == controlset_type_map.end() ) {
			std::stringstream ss;
			ss << "[HybridAutomaton::createControlSet] ControlSet type not registered: " << crtl_type;
			throw ss.str();
		}
		return (*(it->second))(node, system);
	}


	void HybridAutomaton::serialize(DescriptionTreeNode::Ptr& tree) const {
		throw "not implemented";
	}

	void HybridAutomaton::deserialize(const DescriptionTreeNode::Ptr tree){
		//tree.getAttribute(std::string("name"));
		//name = tree.getAttribute("name");
	}

}
