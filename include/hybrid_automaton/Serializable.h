#ifndef HYBRID_AUTOMATON_SERIALIZABLE_H_
#define HYBRID_AUTOMATON_SERIALIZABLE_H_

#include "hybrid_automaton/DescriptionTreeNode.h"

namespace ha {

class Serializable {

	virtual void serialize(DescriptionTreeNode::Ptr& tree) const = 0;
	virtual void deserialize(const DescriptionTreeNode::Ptr tree) = 0;

};

}

#endif