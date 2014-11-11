#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {

void HybridAutomaton::registerController(const std::string& crtl_name, ControllerCreator cc) {
  std::map<std::string, HybridAutomaton::ControllerCreator> controller_type_map = getControllerTypeMap();
  assert ( controller_type_map.find(crtl_name) == controller_type_map.end() );
  std::cout << "Registering " << crtl_name << std::endl;
  controller_type_map[crtl_name] = cc;
}

}
