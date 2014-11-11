#ifndef HYBRID_AUTOMATON_HYBRID_AUTOMATON_H_
#define HYBRID_AUTOMATON_HYBRID_AUTOMATON_H_

#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/ControlMode.h"

#include <assert.h>

// FIXME remove
#include <iostream>

namespace ha {

class HybridAutomaton {

protected:
  
  // FIXME
  ControlMode* control_mode;

  static std::map<std::string, ControllerCreator> _controller_type_map;
  
public:
  
  typedef Controller* (*ControllerCreator) (void);

  static void registerController(const std::string& crtl_name, ControllerCreator cc) {
    assert ( _controller_type_map.find(crtl_name) == _controller_type_map.end() );
    std::cout << "Registering " << crtl_name << std::endl;
    _controller_type_map[crtl_name] = cc;
  }
  
  void addControlMode(ControlMode* cm) {
    // FIXME
    control_mode = cm;
  }
  
  void step() {
    control_mode->step();
  }
  
};

}

#endif
