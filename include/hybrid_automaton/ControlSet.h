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
#ifndef HYBRID_AUTOMATON_CONTROLSET_H_
#define HYBRID_AUTOMATON_CONTROLSET_H_

#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/HybridAutomatonRegistration.h"

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <map>

namespace ha {

	class ControlSet;
	typedef boost::shared_ptr<ControlSet> ControlSetPtr;
	typedef boost::shared_ptr<const ControlSet> ControlSetConstPtr;

    /**
     * @brief Interface for a ControlSet
     *
     * A ControlSet combines the output of several Controllers.
     * Some advanced control methods need to combine several subtasks into a control signal.
     *
     * An example for this would be closing a hand and maintaining arm position at the same time.
     * In this case a Controller for the hand and a Controller for the arm would be added to the ControlSet.
     *
     * Another example could be different forces at different operational points of the robot which
     * then are combined in the ControlSet using nullspace projections.
     */
	class ControlSet : public Serializable {

	public:
		typedef boost::shared_ptr<ControlSet> Ptr;
		typedef boost::shared_ptr<const ControlSet> ConstPtr;

		ControlSet(); 

		ControlSet(const ControlSet& cs);


        /**
        * @brief Activate the ControlSet for execution. Is called automatically from the ControlMode
        */
		virtual void initialize();

        /**
        * @brief Deactivate the ControlSet after execution. Is called automatically from the ControlMode
        */
		virtual void terminate();

        /**
        * @brief Compute the control signal from all contained Controllers. Is called in the control loop.
        */
		virtual ::Eigen::MatrixXd step(const double& t);

        /**
          @brief Serialize into DescriptionTreeNode
        */
		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

        /**
        * @brief Deserialize from DescriptionTreeNode
        */
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		ControlSetPtr clone() const {
			return ControlSetPtr(_doClone());
		}

		virtual void setType(const std::string& new_type);

		virtual const std::string getType() const;

		template <typename T> void setArgument(const std::string& field_name, const T& field_value)
		{
			ha_stringstream ha_oss;
			ha_oss << field_value;
			this->setArgumentString(field_name, ha_oss.str());
		}

		template <typename T> bool getArgument(const std::string& field_name, T& return_value) const
		{
			std::string val;
			bool ret = this->getArgumentString(field_name, val);
			if (ret)
			{
				ha_stringstream ss(val);
				ss >> return_value;
				return true;
			}
			else
			{
				return false;
			}
		}

        /**
        * @brief Add a Controller to this ControlSet
        */
		void appendController(const Controller::Ptr& controller);

		/**
		* This function realizes a smooth transfer from this controlSet to otherSet
		*
		* Overload it in your controlSet, i.e. if you want to transfer members from one CS to the other
		*/
	    virtual void switchControlSet(ControlSet::ConstPtr other_set)
		{
			//Default: do nothing
		}
		

		virtual void setName(const std::string& new_name);

		virtual const std::string getName() const;

        /**
        * @brief Return the contained Controller with name \a name
        *
        * @returns Null poionter if it does not exist
        */
		virtual Controller::ConstPtr getControllerByName(const std::string& name) const; 

        /**
         * @brief Get the list of stored controllers (const reference)
         *
         * This method is used by HybridAutomaton to visualize the controllers.
         *
         */
        virtual const std::map<std::string, Controller::Ptr>& getControllers()  {
            return _controllers;
        }

		virtual std::map<std::string, Controller::Ptr> getControllers() const {
            return _controllers;
        }

    protected:
		virtual void setArgumentString(const std::string& name, const std::string& value)
		{
			this->_additional_arguments[name] = value;
		}

		virtual bool getArgumentString(const std::string& name, std::string& value) const 
		{
			std::map<std::string, std::string>::const_iterator it;
			it = this->_additional_arguments.find(name);
			if (it == this->_additional_arguments.end())
				return false;
			value = it->second;
			return true;
		}

		virtual ControlSet* _doClone() const {
			return new ControlSet(*this);
		}

		virtual void _addController(const Controller::Ptr& cntrl);



	protected:
        /**
         * @brief The name of this ControlSet
         */
		std::string						_name;

        /**
         * @brief The type of this ControlSet - your ControlSet must be registered with the HybridAutomaton
         */
		std::string						_type;

        /**
         * @brief The Controllers contained in this ControlSet
         */
		std::map<std::string, Controller::Ptr>	_controllers;
	
        /**
         * @brief A list of optional attributes for specific ControlSet implementations.
         *
         * All entries in this map will be serialized and deserialized
         */
		std::map<std::string, std::string> _additional_arguments;
	};

}

#endif
