#ifndef HYBRID_AUTOMATON_JUMP_CONDITION_H
#define HYBRID_AUTOMATON_JUMP_CONDITION_H

#include <boost/shared_ptr.hpp>

namespace ha {

    class JumpCondition;
    typedef boost::shared_ptr<JumpCondition> JumpConditionPtr;

    class JumpCondition
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

    protected:

    virtual JumpCondition* _doClone() const
    {
      return (new JumpCondition(*this));
    }

};

}

#endif
