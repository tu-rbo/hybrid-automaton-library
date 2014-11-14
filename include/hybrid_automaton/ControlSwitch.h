#ifndef HYBRID_AUTOMATON_CONTROL_SWITCH_H_
#define HYBRID_AUTOMATON_CONTROL_SWITCH_H_

#include <boost/shared_ptr.hpp>

#include "hybrid_automaton/JumpCondition.h" 
#include "hybrid_automaton/Serializable.h"

namespace ha {

  class ControlSwitch;
  typedef boost::shared_ptr<ControlSwitch> ControlSwitchPtr;

  class ControlSwitch : public Serializable
  {
  public:
    typedef boost::shared_ptr<ControlSwitch> Ptr;

    ControlSwitch() {}

    virtual ~ControlSwitch() {}

    ControlSwitchPtr clone() const
    {
      return (ControlSwitchPtr(_doClone()));
    }

    virtual bool isActive() {
        throw "not implemented";
    }

	virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr factory) const;
	virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree);

    protected:

    std::vector<JumpConditionPtr> _jump_conditions;

    virtual ControlSwitch* _doClone() const
    {
      return (new ControlSwitch(*this));
    }

  };

}

#endif
