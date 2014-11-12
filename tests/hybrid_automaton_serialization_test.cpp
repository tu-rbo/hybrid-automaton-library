#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {

class HybridAutomatonSerializable : public HybridAutomaton {
};

}

TEST(HybridAutomatonSerialization, Simple) {
	using namespace ha;
	HybridAutomaton hybaut;
}
