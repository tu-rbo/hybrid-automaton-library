#ifndef HYBRID_AUTOMATON_CONTROL_MODE_H_
#define HYBRID_AUTOMATON_CONTROL_MODE_H_

#include "hybrid_automaton/Controller.h"

namespace ha {

class ControlMode {
public:
  ControlMode() {}
  
  virtual void addController(Controller* c) {
    // TODO
    //throw NotImplementedException;
  }
    
  virtual void step() {
    // TODO
    //throw NotImplementedException;
  }    
};

}

#endif
