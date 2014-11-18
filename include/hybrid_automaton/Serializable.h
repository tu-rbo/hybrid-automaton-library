#ifndef HYBRID_AUTOMATON_SERIALIZABLE_H_
#define HYBRID_AUTOMATON_SERIALIZABLE_H_

#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/DescriptionTreeNode.h"
#include "hybrid_automaton/System.h"

namespace ha {

	class HybridAutomaton;

class Serializable {
public:
	Serializable() : _ha(NULL) {}

	virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const = 0;
	virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system) = 0;

	// Comment by Sebastian: It would be cleaner to put this into a separate interface
	// but since our Serializable interface is only used in the ha package it is Ok for now
	
	void setHybridAutomaton(const HybridAutomaton* ha) {
		_ha = ha;
	}
	const HybridAutomaton* getHybridAutomaton() const {
		return _ha;
	}

private:
	/**
	 * Only accessible via getter and setter
	 */
	const HybridAutomaton* _ha;
};

}

#endif