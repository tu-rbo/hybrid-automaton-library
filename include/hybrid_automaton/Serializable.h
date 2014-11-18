#ifndef HYBRID_AUTOMATON_SERIALIZABLE_H_
#define HYBRID_AUTOMATON_SERIALIZABLE_H_

#include "hybrid_automaton/DescriptionTree.h"
#include "hybrid_automaton/DescriptionTreeNode.h"
#include "hybrid_automaton/System.h"

namespace ha {

class Serializable {

	virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const = 0;
	virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree) = 0;

};

}

#endif