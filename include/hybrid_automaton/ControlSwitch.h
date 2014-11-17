#ifndef HYBRID_AUTOMATON_CONTROL_SWITCH_H_
#define HYBRID_AUTOMATON_CONTROL_SWITCH_H_

#include <boost/shared_ptr.hpp>

#include <vector>

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

	virtual void activate(const double& t);
	virtual void deactivate();

	virtual void step(const double& t);
    virtual bool isActive() const;

	virtual void add(const JumpConditionPtr& jump_condition);
	virtual const std::vector<JumpConditionPtr>& getJumpConditions();

	virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;
	virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree);

	virtual void setSourceControlMode(const std::string& source);
	virtual const std::string getSourceControlMode() const;

	virtual void setTargetControlMode(const std::string& source);
	virtual const std::string getTargetControlMode() const;

	virtual void setName(const std::string& name);
	virtual const std::string getName() const;

  protected:
    std::vector<JumpConditionPtr> _jump_conditions;

	std::string _source_control_mode;
	std::string _target_control_mode;

	std::string _name;

    virtual ControlSwitch* _doClone() const
    {
      return (new ControlSwitch(*this));
    }
  };

}

#endif
