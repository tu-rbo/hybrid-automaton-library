#include "hybrid_automaton/HybridAutomatonFactory.h"

int main(int argc, char* argv[]){
    ha::HybridAutomatonFactory::Ptr haf(new ha::HybridAutomatonFactory);

    ha::HybridAutomaton::Ptr ha = haf->createEmptyHybridAutomaton();

    ha::ControlMode::Ptr gccm;
    haf->CreateGCCM(gccm,"MyGravityControlMode");

    ha->addControlMode(gccm);

    exit(0);

}
