#ifndef HYBRID_AUTOMATON_JUMP_CONDITION_H
#define HYBRID_AUTOMATON_JUMP_CONDITION_H

#include "hybrid_automaton/Serializable.h"

#include <boost/shared_ptr.hpp>

namespace ha {

    class JumpCondition;
    typedef boost::shared_ptr<JumpCondition> JumpConditionPtr;

	class JumpCondition : public Serializable
    {
    public:

    typedef boost::shared_ptr<JumpCondition> Ptr;

    JumpCondition() {}
    virtual ~JumpCondition() {}

    JumpConditionPtr clone() const
    {
      return (JumpConditionPtr(_doClone()));
    }

	virtual void activate(const double& t) {
		throw "not implemented";
	}

	virtual void deactivate() {
		throw "not implemented";
	}

	virtual void step(const double& t) {
		throw "not implemented";
	}

    virtual bool isActive() const {
        throw "not implemented";
    }

	virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const { return DescriptionTreeNode::Ptr(); };

	/**
	 * @brief Deserialization for JumpConditions
	 *
	 * Do not request the hybrid automaton by calling getHybridAutomaton()!
	 * It is not guaranteed to be set before calling deserialize.
	 * 
	 * The same holds true for _control_switch
	 * If you need to access object from the graph structure, store the
	 * names of the entities in a string member and query them in the
	 * activate() method of your entity.
	 *   
	 */
	virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha) {};

    protected:

		virtual JumpCondition* _doClone() const
    {
      return (new JumpCondition(*this));
    }

};

}

#endif
