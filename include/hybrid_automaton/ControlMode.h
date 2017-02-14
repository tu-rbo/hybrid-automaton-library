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
#ifndef HYBRID_AUTOMATON_CONTROL_MODE_H_
#define HYBRID_AUTOMATON_CONTROL_MODE_H_

#include "hybrid_automaton/ControlSet.h"
#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/error_handling.h"

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

namespace ha {

	class HybridAutomaton;

	class ControlMode;
	typedef boost::shared_ptr<ControlMode> ControlModePtr;
	typedef boost::shared_ptr<const ControlMode> ControlModeConstPtr;


    /**
     * @brief Control mode - A control mode is a node in the hybrid automaton graph.
     *
     * It contains a ControlSet which again contains a set of controllers.
     *
     * @see ControlSet
     * @see Controller
     */
    class ControlMode : public Serializable {
	public:
		typedef boost::shared_ptr<ControlMode> Ptr;
		typedef boost::shared_ptr<const ControlMode> ConstPtr;

		ControlMode() {}
		ControlMode(const std::string& name);

		virtual ~ControlMode() {}

        /**
         * @brief Activate the controller for execution.
         *
         * Is called after switching from another mode to this one
         */
		virtual void initialize() {
			if (_control_set)
				_control_set->initialize();
			else
				HA_THROW_ERROR("ControlSet.initialize", "No control set defined.");
		}

        /**
         * @brief Deactivate the controller for execution.
         *
         * Is called when switching from this mode to another
         */
		virtual void terminate() {
			if (_control_set)
				_control_set->terminate();
			else
				HA_THROW_ERROR("ControlSet.terminate", "No control set defined.");
		}

        /**
         * @brief Compute the current control signal
         *
         * Is called from the HybridAutomaton - once within each control loop
         */
		virtual ::Eigen::MatrixXd step(const double& t) {
			if (_control_set)
				return _control_set->step(t);
			else
				HA_THROW_ERROR("ControlSet.step", "No control set defined.");
		}

		virtual void switchControlMode(ControlMode::Ptr otherMode)
		{
			this->getControlSet()->switchControlSet(otherMode->getControlSet());
		}

		virtual void setControlSet(const ControlSet::Ptr control_set) {
			_control_set = control_set;
		}

		virtual ControlSet::Ptr getControlSet() const {
			return _control_set;
		}    

		virtual Controller::ConstPtr getControllerByName(const std::string& name) const {
			return _control_set->getControllerByName(name);
		}    

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		ControlModePtr clone() const {
			return ControlModePtr(_doClone());
		}

		void setName(const std::string& name) {
			_name = name;
		}

		const std::string& getName() const {
			return _name;
		}

	protected:
		virtual ControlMode* _doClone() const {
			return new ControlMode(*this);
		}

	protected:

        /**
         * @brief The ControlSet that generates the commands for this CotrolMode
         */
		ControlSet::Ptr _control_set;

        /**
         * @brief identifier of this ControlMode - must be unique within each HybridAutomaton!
         */
		std::string _name;

	};

}

#endif
