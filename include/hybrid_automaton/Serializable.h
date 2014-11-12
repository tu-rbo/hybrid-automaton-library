#ifndef HYBRID_AUTOMATON_SERIALIZABLE_H_
#define HYBRID_AUTOMATON_SERIALIZABLE_H_

namespace ha {

class DescriptionTreeNode;

class Serializable {

	virtual void serialize(DescriptionTreeNode& tree) const = 0;
	virtual void deserialize(const DescriptionTreeNode& tree) = 0;

};

}

#endif