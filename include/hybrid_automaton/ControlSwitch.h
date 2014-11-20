#ifndef HYBRID_AUTOMATON_CONTROL_SWITCH_H_
#define HYBRID_AUTOMATON_CONTROL_SWITCH_H_

#include <boost/shared_ptr.hpp>

#include <vector>

#include "hybrid_automaton/JumpCondition.h" 
#include "hybrid_automaton/ControlMode.h" 
#include "hybrid_automaton/Serializable.h"

namespace ha {
  //class HybridAutomaton;

  class ControlSwitch;
  typedef boost::shared_ptr<ControlSwitch> ControlSwitchPtr;
  typedef boost::shared_ptr<const ControlSwitch> ControlSwitchConstPtr;

  class ControlSwitch : public Serializable
  {
  public:
    typedef boost::shared_ptr<ControlSwitch> Ptr;
	typedef boost::shared_ptr<const ControlSwitch> ConstPtr;

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
	virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

	virtual void setName(const std::string& name);
	virtual const std::string getName() const;

	//Only call these functions when deserializing - afterwards they might be invalid!!
	//If you want to access the real source mode call _hybrid_automaton->getSourceControlMode(this->getName())
	virtual const std::string getSourceControlModeName() const;
	virtual const std::string getTargetControlModeName() const;
	virtual void setSourceControlModeName(const std::string& source);
	virtual void setTargetControlModeName(const std::string& target);

  protected:
    std::vector<JumpConditionPtr> _jump_conditions;

	std::string _name;

    virtual ControlSwitch* _doClone() const
    {
      return (new ControlSwitch(*this));
    }

	//These variables are just used when deserializing! Do not use them
	//anywhere else. They might be invalid!
	std::string _source_control_mode_name;
	std::string _target_control_mode_name;  

	const HybridAutomaton* _hybrid_automaton;
  };

}

#endif
