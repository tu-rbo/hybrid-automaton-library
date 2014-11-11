#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {

std::map<std::string, HybridAutomaton::ControllerCreator> HybridAutomaton::_controller_type_map;

}
