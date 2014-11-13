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

    virtual bool isActive() {
        throw "not implemented";
    }

	virtual void serialize(const DescriptionTreeNode::Ptr& tree) const;
	virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree);

    protected:

    virtual JumpCondition* _doClone() const
    {
      return (new JumpCondition(*this));
    }

};

}

#endif
