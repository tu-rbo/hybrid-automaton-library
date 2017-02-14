/*
 * Copyright 2015-2017, Robotics and Biology Lab, TU Berlin
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the 
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
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

  /**
   * @brief ControlSwitch - A control switch is an edge in the HybridAutomaton.
   *
   * A ControlSwitch describes a discrete transitions between two ControlModes.
   *
   * A ControlSwitch contains one or more JumpConditions. these JumpConditions are conditional statements.
   * If they all evaluate to "true", this ControlSwitch will become active and the ControlMode this switch points to
   * will be executed.
   *
   * @see ControlSet
   * @see Controller
   */
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

    /**
    * @brief Activate the ControlSwitch. Is called automatically when the source ControlMode is activated.
    */
	virtual void initialize(const double& t);

    /**
    * @brief Deactivate the ControlSwitch. Is called automatically when the source ControlMode is deactivated.
    */
	virtual void terminate();

    /**
     * @brief Update the underlying JumpConditions
     *
     * Is called from the HybridAutomaton - once within each control loop
     */
	virtual void step(const double& t);

    /**
     * @brief Check if all contained JumpConditions evaluate to true
     * Is called from the HybridAutomaton once within each control loop
     */
    virtual bool isActive() const;

	virtual void add(const JumpConditionPtr& jump_condition);
	virtual const std::vector<JumpConditionPtr>& getJumpConditions();

	virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;
	virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

	virtual void setName(const std::string& name);
	virtual const std::string getName() const;

	void setHybridAutomaton(const HybridAutomaton* hybrid_automaton);

  protected:
    /**
     * @brief The JumpConditions in this ControlSwitch - all need to evaluate to ture for this switch to become active
     */
    std::vector<JumpConditionPtr> _jump_conditions;

    /**
     * @brief The name of this ControlSwitch - needs to be unique within one HybridAutomaton
     */
	std::string _name;

    virtual ControlSwitch* _doClone() const
    {
      return (new ControlSwitch(*this));
    }

    /**
     * @brief A pointer to the HybridAutomaton - the ControlSet needs this to access the graph structure
     */
	const HybridAutomaton* _hybrid_automaton;
  };

}

#endif
