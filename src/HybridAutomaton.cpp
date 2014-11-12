#include "hybrid_automaton/HybridAutomaton.h"

#include "hybrid_automaton/DescriptionTreeNode.h"

// FIXME remove
#include <iostream>

namespace ha {

void HybridAutomaton::registerController(const std::string& crtl_name, ControllerCreator cc) {
  std::map<std::string, HybridAutomaton::ControllerCreator> controller_type_map = getControllerTypeMap();
  assert ( controller_type_map.find(crtl_name) == controller_type_map.end() );
  // FIXME
  std::cout << "Registering " << crtl_name.c_str() << std::endl;
  controller_type_map[crtl_name] = cc;
}

void HybridAutomaton::serialize(DescriptionTreeNode& tree) const {
}

void HybridAutomaton::deserialize(const DescriptionTreeNode& tree){
}

}
