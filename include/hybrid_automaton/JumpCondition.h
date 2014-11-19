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

	class JumpCondition : public Serializable
	{
	public:

		enum Norm {L1, L2, L_INF, ROTATION, TRANSFORM}; 

		typedef boost::shared_ptr<JumpCondition> Ptr;

		JumpCondition();

		virtual ~JumpCondition();

		/*!
		* Copy constructor
		*/
		JumpCondition(const JumpCondition& jc);

		JumpConditionPtr clone() const
		{
			return (JumpConditionPtr(_doClone()));
		};

		virtual void activate(const double& t);

		virtual void deactivate();

		virtual void step(const double& t);

		virtual bool isActive() const;

		virtual void setControllerGoal(const Controller* controller);
		virtual void setConstantGoal(const ::Eigen::MatrixXd goal);

		virtual ::Eigen::MatrixXd JumpCondition::getGoal() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

	protected:
		System::ConstPtr _system;
				
		const Controller* _controller;
		boost::shared_ptr<const ::Eigen::MatrixXd> _goal;

		boost::shared_ptr<const ha::Sensor> _sensor;

		Norm	_normType;

		::Eigen::MatrixXd _weights;

		double _epsilon;

		double _computeNorm(::Eigen::MatrixXd x, ::Eigen::MatrixXd y) const;

		virtual JumpCondition* _doClone() const
		{
			return (new JumpCondition(*this));
		}

	};

}

#endif
