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
	 * Each jump condition contains a sensor, a goal, and a metric.
	 * A jump condition becomes active, when the distance (in the given metric) between sensor reading
	 * and the goal is smaller than the parameter epsilon.
	 */
	class JumpCondition : public Serializable
	{
	public:

		// NUM_CRITERIA must be always the last value of the enum to know the number of elements it contains
		enum JumpCriterion {NORM_L1, NORM_L2, NORM_L_INF, NORM_ROTATION, NORM_TRANSFORM, THRESH_UPPER_BOUND, THRESH_LOWER_BOUND, NUM_CRITERIA}; 

		enum GoalSource {CONSTANT, CONTROLLER, ROSTOPIC}; 

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

		virtual void initialize(const double& t);

		virtual void terminate();

		virtual void step(const double& t);

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

		virtual void setConstantGoal(double goal);

		virtual void setGoalRelative();

		virtual void setGoalAbsolute();

		virtual bool isGoalRelative() const;
		
		
		// TODO
		//virtual void setROSTopicGoal(std::string rosTopicName);

		/**
		 * @brief Get the current goal
		 * 
		 * This function returns the current desired value of the jump condition.
		 * It can be either a constant goal or the goal value of a given controller.
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
		 * The entries of the goals can be weighted differently. 
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

		Sensor::Ptr _sensor;

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
