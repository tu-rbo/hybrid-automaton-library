#ifndef HYBRID_AUTOMATON_SERIALIZABLE_H_
#define HYBRID_AUTOMATON_SERIALIZABLE_H_

#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/DescriptionTreeNode.h"
#include "hybrid_automaton/System.h"

namespace ha {

	class HybridAutomaton;

class Serializable {
public:

	virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const = 0;
	virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha) = 0;

};

}

#endif