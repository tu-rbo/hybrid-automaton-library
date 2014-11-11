#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/ControlMode.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <map>
#include <vector>
#include <string>

#include <assert.h>

#include <iostream>

// DELETE ME
//
// File used for showing basic wiring of classes + their functionality
//
// Compile with g++ -Wall -O3 -g main.cpp src/HybridAutomaton.cpp -Iinclude -o main

using namespace std;

//----------------------------
// RLAB
namespace rlab {

class rxController {
protected:
  int param;
public:
  rxController(int required_ctrtor_param) : param( required_ctrtor_param ) {
    cout << "rlab::rxController - construct with required_ctrtor_param " << required_ctrtor_param << endl;
  }
  
  void setParam(int required_ctrtor_param) {
    cout << "rlab::rxController - resetting param " << required_ctrtor_param << endl;
    param = required_ctrtor_param;
  }
  
  virtual void step() = 0;
};

class rxControlSet {
  std::vector<rxController*> controllers;
  
public:
  rxControlSet(int dt) {
    cout << "rlab::rxControlSet - construct with dt " << dt << endl;
  }
  
  void addController (rxController* c) {
    cout << "rlab::rxControlSet Adding rxController" << endl;
    controllers.push_back(c);
  }

  void step() {
    std::vector<rxController*>::iterator it;
    for (it = controllers.begin(); it != controllers.end(); it++) {
      (*it)->step();
    }
  }
  
};

}


//----------------------------
// ha_rlab

namespace ha_rlab {

//template <class T>
class rlabController : public ha::Controller, public rlab::rxController {
  rlab::rxController* _controller; 
  
public:
  rlabController() : ha::Controller(), rlab::rxController(0) {
    cout << "rlabController construct" << endl;
  }
  
   rlab::rxController* get() const 
   { return _controller; }

   void set(rlab::rxController* c)
   { _controller = c; }

};


class rlabControlMode : public ha::ControlMode {

protected:
  rlab::rxControlSet* cs;
  
public:  
  
  rlabControlMode() : ha::ControlMode() {
    cs = new rlab::rxControlSet(0.1);
  }
  
  virtual void addController(ha::Controller* c) {
    rlabController* cw = static_cast<rlabController*>(c);
    assert (cw != NULL);
    cout << "rlabControlSet Adding rxController" << endl;
    
    cs->addController (cw);
  }
  
  void step() {
    cs->step();
  }
  
};

class rlabJointController : public rlabController {

public:
  //static rlabJointController* instance() {
  rlabJointController() : rlabController() {
    cout << "rlabJointController - construct" << endl;
  }

  void step() {
    //cout << " Stepping rlabJointController!" << endl;
  }

  static ha::Controller* instance() {
    //return new rlabController<T>;
    return new rlabJointController;
  }

  class Initializer {
  public:
    Initializer();
  };
  static Initializer initializer;
};

rlabJointController::Initializer initializer;
rlabJointController::Initializer::Initializer() {
  ha::HybridAutomaton::registerController("rlabJointController", &rlabJointController::instance);
}

}

//----------------------------

/*
int main() {
  
  cout << "Main starts " << endl;
  
  using namespace ha;
  using namespace ha_rlab;
  
  rlabController* cw = new rlabJointController;

  ControlMode* cm = new rlabControlMode;
  cm->addController(cw);
  
  HybridAutomaton ha;
  ha.addControlMode(cm);
  
  ha.step();
  
  return 0;
}
*/
