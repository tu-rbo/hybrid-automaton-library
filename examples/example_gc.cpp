#include <iostream>
#include "hybrid_automaton/HybridAutomatonRBOFactory.h"
#include "hybrid_automaton/HybridAutomatonAbstractFactory.h"

int main(int argc, char* argv[]){
    ha::HybridAutomatonAbstractFactory::Ptr haf(new ha::HybridAutomatonRBOFactory);

    ha::HybridAutomaton::Ptr ha = haf->createEmptyHybridAutomaton();

    ha::ControlMode::Ptr gccm(new ha::ControlMode());
    ha::HybridAutomatonAbstractParams params;
    haf->CreateGCCM(params, gccm,"MyGravityControlMode");
    ha->addControlMode(gccm);

    exit(0);

}
