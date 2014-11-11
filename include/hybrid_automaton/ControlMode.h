#ifndef HYBRID_AUTOMATON_CONTROL_MODE_H_
#define HYBRID_AUTOMATON_CONTROL_MODE_H_

#include "hybrid_automaton/Controller.h"

namespace ha {

class ControlMode {
public:
  ControlMode() {}
  
  virtual void addController(Controller* c) = 0;
  virtual void step() = 0;
};

}

#endif
