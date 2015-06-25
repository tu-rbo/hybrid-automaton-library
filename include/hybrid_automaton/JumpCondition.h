#ifndef HYBRID_AUTOMATON_JUMP_CONDITION_H
#define HYBRID_AUTOMATON_JUMP_CONDITION_H

#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/error_handling.h"

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/Controller.h"


#include <boost/shared_ptr.hpp>

namespace ha {

	class JumpCondition;
	typedef boost::shared_ptr<JumpCondition> JumpConditionPtr;

	/**
	 * @brief A JumpCondition is a necessary condition for a ControlSwitch to become active.
	 *
     * Each jump condition contains a Sensor, a goal, and a metric.
	 * A jump condition becomes active, when the distance (in the given metric) between sensor reading
	 * and the goal is smaller than the parameter epsilon.
	 */
	class JumpCondition : public Serializable
	{
	public:

        /**
         * @brief A list of metrics to compare sensor againt goal
         *
         * NORM_L1: sum_i (|goal_i| - |sensor_i|)
         *
         * NORM_L2: sum_i sqrt(|goal_i|^2 - |sensor_i|^2)
         *
         * NORM_LINF: max_i  (|goal_i| - |sensor_i|)
         *
         * NORM_ROTATION: angle between rotation matrices goal and sensor
         *
         * NORM_TRANSFORMATION: weighted sum of angle between rotation parts and euclidian distance of translation parts
         *
         * THRESH_UPPER_BOUND: evaluates to true if sensor_i < goal_i for all i - use with epsilon = 0
         *
         * THRESH_LOWER_BOUND: evaluates to true if sensor_i > goal_i for all i - use with epsilon = 0
         *
         * NUM_CRITERIA must be always the last value of the enum to know the number of norms
         */
		enum JumpCriterion {NORM_L1, NORM_L2, NORM_L_INF, NORM_ROTATION, NORM_TRANSFORM, THRESH_UPPER_BOUND, THRESH_LOWER_BOUND, NUM_CRITERIA}; 

        /**
         * @brief The source of the goal
         *
         * CONSTANT: the constant stored in member \a _goal
         *
         * CONTROLLER: the getGoal() function of the Controller stored in \a _controller
         *
         * ROSTOPIC: the value of the ROS topic stored in member \a _ros_topic_goal_name
         *
         * ROSTOPIC_TF: the transformation from tf frame _ros_tf_goal_parent to _ros_tf_goal_child
         */
		enum GoalSource {CONSTANT, CONTROLLER, ROSTOPIC, ROSTOPIC_TF}; 

		typedef boost::shared_ptr<JumpCondition> Ptr;

		JumpCondition();

		virtual ~JumpCondition();

        /**
		* Copy constructor
		*/
		JumpCondition(const JumpCondition& jc);

		JumpConditionPtr clone() const
		{
			return (JumpConditionPtr(_doClone()));
		};

        /**
        * @brief Activate the JumpCondition. Is called automatically when the ControlSwitch is activated.
        */
		virtual void initialize(const double& t);

        /**
        * @brief Deactivate the JumpCondition. Is called automatically when the ControlSwitch is deactivated.
        */
		virtual void terminate();

        /**
        * @brief Update the JumpCondition. Is called automatically when the ControlSwitch is updated, once in a control cycle.
        */
		virtual void step(const double& t);

        /**
        * @brief Computes ||goal - sensor||_N < epsilon in the given norm N. Is called from ControlSwitch, once in a control cycle.
        *
        * the source of the goal, the used sensor, the used norm, and epsilon all can be specified.
        * @see setSensor
        * @see setConstantGoal, setControllerGoal, setROSTopicGoal, setROSTfGoal
        * @see setNorm
        * @see setEpsilon
        */
		virtual bool isActive() const;

		/**
		 * @brief Set a controller goal
		 * 
		 * This function sets the desired vlaue of this condition to the (possibly non-constant) goal of a controller.
		 * This allows to match the convergence of a controller with the activation of a jump condition.
		 */
		virtual void setControllerGoal(const Controller::ConstPtr& controller);
		
		/**
		 * @brief Set a constant goal
		 * 
		 * This function sets the desired value of this condition to a constant value.
		 */
		virtual void setConstantGoal(const ::Eigen::MatrixXd goal);

        /**
         * @brief Set a constant goal number
         *
         * This function sets the desired value of this condition to a number
         *
         * This ijust creates a 1x1 matrix internally
         */
		virtual void setConstantGoal(double goal);

        /**
         * @brief Set a ROS topic goal
         *
         * This function sets goal to be the vlaue of the ROS topic \topic
         * @param topic_type - an ID for the type of the topic
         * for the RLab infrastructure this needs to be one of Bool/Float64/Float64MultiArray/Transform
         */
		virtual void setROSTopicGoal(const std::string& topic, const std::string& topic_type);

        /**
         * @brief Set a ROS tf topic goal
         *
         * This function sets goal to be the transfromation between parent and child
         */
		virtual void setROSTfGoal(const std::string& child, const std::string& parent);

		virtual void setGoalRelative();

		virtual void setGoalAbsolute();

		virtual bool isGoalRelative() const;
		
		/**
		 * @brief Get the current goal
		 * 
		 * This function returns the current desired value of the jump condition.
         * It can be either a constant goal, the goal value of a given controller, or a value from ROS
		 */
		virtual ::Eigen::MatrixXd getGoal() const;

		/**
		 * @brief Set a sensor
		 * 
		 * This function sets the sensor of this jump condition and has to be called before
		 * starting execution.
		 */
		virtual void setSensor(const Sensor::Ptr sensor);
		virtual Sensor::ConstPtr getSensor() const;

		/**
		 * @brief Set a Criterion to use for comparing current and desired goals
		 * 
		 * The criterion can get active in these cases:
		 *   a) ||current - goal||_N < epsilon (for a given norm N i.e. use NORM_L1 to sum up entries, or NORM_L_INF for maximal error)
		 *   b) current > goal (for THRESH_UPPER)
		 *   c) current < goal (for THRESH_LOWER)
		 * Other are angular distance between two rotation matrices. 
		 *
         * @param weight The entries of the goals can be weighted differently.
		 * i.e. if you do not care about a certain dimension, set the weight for this dimension to zero 
		 */
		virtual void setJumpCriterion(JumpCriterion jump_criterion, ::Eigen::MatrixXd weights = ::Eigen::MatrixXd());
		virtual JumpCriterion getJumpCriterion() const;
		virtual ::Eigen::MatrixXd getNormWeights() const;

		 /**
		 * @brief Set epsilon - this condition is active when:
		 * || current - desired|| < epsilon
		 */
		virtual void setEpsilon(double epsilon);
		virtual double getEpsilon() const;

		/**
		* @brief Set the name of the mode this jumpCondition emanates from. This function needs to be called
		* before deserializing! 
		*/
		virtual void setSourceModeName(const std::string& sourceModeName);

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

        virtual std::string toString(bool use_latex=false);


	protected:

		GoalSource	_goalSource;
		::Eigen::MatrixXd _goal;
		Controller::ConstPtr _controller;

		std::string _ros_topic_goal_name;
		std::string _ros_topic_goal_type;

		std::string _ros_tf_goal_child;
		std::string _ros_tf_goal_parent;

		Sensor::Ptr _sensor;
		System::ConstPtr _system;

		JumpCriterion	_jump_criterion;
		::Eigen::MatrixXd _norm_weights;
		double _epsilon;

		//The name of the mode this JumpCondition's edge emanates from.
		//Needed for deserialization
		std::string _sourceModeName;

		bool	_is_goal_relative;

		double _computeJumpCriterion(const ::Eigen::MatrixXd& x, const ::Eigen::MatrixXd& y) const;

		virtual JumpCondition* _doClone() const
		{
			return (new JumpCondition(*this));
		}

	};

}

#endif
