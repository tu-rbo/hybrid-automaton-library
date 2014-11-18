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

	/**
	 * @brief Deserialization method
	 *
	 * Important things to know when implementing deserialize:
	 * Do not request the hybrid automaton by calling getHybridAutomaton()!
	 * It is not guaranteed to be set before calling deserialize.
	 * 
	 * If you need to access object from the graph structure, store the
	 * names of the entities in a string member and query them in the
	 * activate() method of your entity.
	 *   
	 */
	virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha) = 0;

};

}

#endif