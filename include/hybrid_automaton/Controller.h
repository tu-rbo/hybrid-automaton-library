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
#ifndef HYBRID_AUTOMATON_CONTROLLER_H_
#define HYBRID_AUTOMATON_CONTROLLER_H_

#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/System.h"
#include "hybrid_automaton/HybridAutomatonRegistration.h"
#include "hybrid_automaton/error_handling.h"

#include "hybrid_automaton/HybridAutomatonStringStream.h"

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ha {

	class Controller;
	typedef boost::shared_ptr<Controller> ControllerPtr;
	typedef boost::shared_ptr<const Controller> ControllerConstPtr;

    /**
     * @brief Interface for a Controller primitive.
     *
     * Each Controller defines a Atomic control behaviour for a component of the robot
     * This could be a joint space controller, or a controller moving one operational point on the robot towards a goal.
     * A controller could also open and close a gripper.
     *
     * The ControlSet combines the output of all contained Controllers into one output control signal.
     */
	class Controller : public Serializable {

	public:
		typedef boost::shared_ptr<Controller> Ptr;
		typedef boost::shared_ptr<const Controller> ConstPtr;

        /**
		* Constructor
		*/
		Controller();

        /**
		* Destructor
		*/
		virtual ~Controller();

        /**
		* Copy constructor
		*/
		Controller(const Controller& controller);

        /**
        * @brief Activate the controller for execution. Is called automatically from the ControlSet
	    */
		virtual void initialize() {
			// Some more things will be done in the derived classes
			this->updateGoal();
		}

		/**
        * @brief Updates the goal of the controller. To do so, it first converts it from relative to absolute (if it was passed
		* as relative goal). The function is called by initialize() and by the Blackboard decorator to update the goal. 
		* This function is not virtual because it should not be reimplemented in derived classes.
	    */
		void updateGoal() {
			Eigen::MatrixXd abs_goal;
			if(this->_goal_is_relative)
				abs_goal = this->relativeGoalToAbsolute(this->_goal);
			else
				abs_goal = this->_goal;

			this->_updateAbsoluteGoal(abs_goal);
		}

        /**
        * @brief Deactivate the controller for execution. Is called automatically from the ControlSet
        */
		virtual void terminate() {
			HA_THROW_ERROR("Controller.terminate", "not implemented");
		}

        /**
        * @brief Step function - gets called in each control cycle
		*/
		virtual ::Eigen::MatrixXd step(const double& t) {
			HA_THROW_ERROR("Controller.step", "not implemented");
		}

        /**
		* @brief Transform a goal in relative coordinates into absolute coordinates.
        *
        * Controllers should provide an interface to add a "delta" value  \a goalRel to the currents system state.
        * i.e. in joint space, \a goalRel is a configuration that must be added to the current robots position.
        * in task space, goalRel is a frame that must be multiplied.
        *
        * If your Controller does not overload this method, you can not use \a relative goals.
        *
        * @returns the transformed goal , i.e. \a current + \a goalRel
	    */
		virtual ::Eigen::MatrixXd relativeGoalToAbsolute(const Eigen::MatrixXd& goalRel) const{
			HA_THROW_ERROR("Controller.relativeGoalToAbsolute", "not implemented - You can not use a relative goal with this type of controller!: "<<this->_type);
		}

        /**
          @brief Serialize into DescriptionTreeNode
		*/
		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

        /**
        * @brief Deserialize from DescriptionTreeNode
		*/
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

        /**
        * @brief Clone function
		*/
		ControllerPtr clone() const {
			return ControllerPtr(_doClone());
		}

		virtual void setSystem(const System::ConstPtr& system);

		virtual Eigen::MatrixXd getGoal() const;
		virtual void setGoal(const Eigen::MatrixXd& new_goal);

		/**
		* Helper function to return the current via point of an interpolator.
		* Overwrite it for interpolated controllers, ignore it for non-interpolated controllers.
		*/
		virtual ::Eigen::MatrixXd getCurrentInterpolationGoal() const;
	
		virtual bool getGoalIsRelative() const;
		virtual void setGoalIsRelative(bool is_goal_relative);

		virtual Eigen::MatrixXd getKp() const;
		virtual void setKp(const Eigen::MatrixXd& new_kp);

		virtual Eigen::MatrixXd getKv() const;
		virtual void setKv(const Eigen::MatrixXd& new_kv);

		virtual void setCompletionTimes(const Eigen::MatrixXd& new_times);
		virtual void setCompletionTime(const double& t); //for convenience - creates 1x1 matrix	
		virtual Eigen::MatrixXd getCompletionTimes() const;

		virtual void setMaximumVelocity(const Eigen::MatrixXd& max_vel);
		virtual	void setMaximumAcceleration(const Eigen::MatrixXd& max_acc);

		virtual Eigen::MatrixXd	getMaximumVelocity() const;
		virtual Eigen::MatrixXd	getMaximumAcceleration() const;

		virtual void setDoReinterpolation(bool do_reinterpolation);
		virtual bool getDoReinterpolation() const;

		virtual double getPriority() const;
		virtual void setPriority(double priority);
		
		virtual std::string getName() const;
		virtual void setName(const std::string& new_name);

		virtual const std::string getType() const;
		virtual void setType(const std::string& new_type);

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
		 * @brief Computes a default interpolation time for a trajectory from x0 to xf
		 * under velocity and acceleration limits. Currrently assumes linear interpolation!
		 * 
		 * The implementation in Controller computes element-wise linear interpolation.
		 * If you want to have more complex behaviour (i.e. rotational interpolation) overload this function.
		 *
	     * @return minimal interpolation time.
		 */
		virtual double computeInterpolationTime
			(const Eigen::MatrixXd& x0, const Eigen::MatrixXd& xf,
			const Eigen::MatrixXd& xdot0 = Eigen::MatrixXd(), const Eigen::MatrixXd& xdotf = Eigen::MatrixXd()) const;


        const std::map<std::string, std::string>& getAdditionalArgumentsString() {
            return _additional_arguments;
        }

		/**
		* This function realizes a smooth transfer from this controller to controller
		*
		* Overload it in your controller, i.e. if you want to transfer members from one controller to the other
		*/
	    virtual void switchController(Controller::ConstPtr other_ctrl)
		{
			//Default: do nothing
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

        /**
        * @brief
		*/
		virtual Controller* _doClone() const {
			return new Controller(*this);
		}

		
	protected:

		/** 
		 * @brief This method invokes the right call setPoint / addPoint for this controller type
		 *
		 * this method needs the goal in absolute coordinates and thus should only be called
		 * in initialize() AND updateGoal()
		 */
		virtual void _updateAbsoluteGoal(const Eigen::MatrixXd& goal_abs)
		{
			//default: do nothing special - most controllers do rlab specific things here - i.e. call setPoint(..)
		}


        /**
         * @brief A pointer to the active robot system
         */
		System::ConstPtr _system;

        /**
         * @brief The goal of this Controller
         *
         * The Controller will converge to this goal.
         */
		Eigen::MatrixXd		_goal;

        /**
         * @brief The goal of this Controller
         *
         * If set to true, the robot will only move relative to it's current position.
         * Ie. if goal is a null matrix it will maintain the position
         */
		bool				_goal_is_relative;

        /**
         * @brief The proportional gain of this Controller
         *
         * If you have no idea for possible values look into examples for your specific Controller, i.e. in the cookbook
         */
        Eigen::MatrixXd		_kp;

        /**
         * @brief The derivative gain of this Controller
         *
         * If you have no idea for possible values look into examples for your specific Controller, i.e. in the cookbook
         */
		Eigen::MatrixXd		_kv;

        /**
         * @brief The priority of this Controller in the ControlSet.
         *
         * Optional attribute - some ControlSets can handle Controllers at different priorities.
         * i.e. End-effector task is more important than obstacle avoidance
         */
		double				_priority;

        /**
         * @brief The name this Controller. Needs to be unique within a Hybrid Automaton.
         *
         * JumpConditions can refer to this name to check for convergence of a Controller
         */
		std::string			_name;

        /**
         * @brief The type id of this Controller
         *
         * All Controller types need to be registered with the HybridAutomaton so that they can be matched to their typeid at deserialization.
         * @see HybridAutomaton
         */
		std::string			_type;

        /**
         * @brief Completion times to reach goal and intermediate points
         *
         * Optional attribute, e.g. makes sense for interpolated
         * controllers
         */
		Eigen::MatrixXd		_completion_times;

        /**
         * @brief Maximum velocity to reach goal and intermediate points
         *
         * Optional attribute, e.g. makes sense for interpolated
         * controllers
         */
        Eigen::MatrixXd		_v_max;

        /**
         * @brief Maximum acceleration to reach goal and intermediate points
         *
         * Optional attribute, e.g. makes sense for interpolated
         * controllers
         *
         * Caution: currently not implemented for RLab controllers
         */
		Eigen::MatrixXd		_a_max;

        /**
         * @brief For Interpolated controllers: ff set to true, controllers will continuously reinterpolate a smooth trajectory whenever setGoal is called.
         *
         * If set to false, the point in setGoal will just be attached to the back of the trajectory.
         *
         * Optional attribute for interpolated controllers.
         */
		bool				_do_reinterpolation;

        /**
         * @brief A map containing additional, controller specific arguments.
         *
         * You can add any argument as a serialized string to this controller and it will be serialized and
         * deserialized with this controller.
         *
         * @see setArgument()
         * @see getArgument()
         */
		std::map<std::string, std::string> _additional_arguments;
	};

}

#endif
