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

		enum GoalSource {CONSTANT, CONTROLLER, ROSTOPIC}; 

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
		// TODO
		//virtual void setROSTopicGoal(std::string rosTopicName);

		virtual ::Eigen::MatrixXd JumpCondition::getGoal() const;

		virtual void setSensor(const Sensor::Ptr sensor);
		virtual Sensor::ConstPtr getSensor() const;

		virtual void setNorm(Norm normType, ::Eigen::MatrixXd weights = ::Eigen::MatrixXd());
		virtual Norm getNormType() const;
		virtual ::Eigen::MatrixXd getNormWeights() const;

		virtual void setEpsilon(double epsilon);
		virtual double getEpsilon() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

	protected:

		GoalSource	_goalSource;
		::Eigen::MatrixXd _goal;
		const Controller* _controller;

		Sensor::ConstPtr _sensor;

		Norm	_normType;
		::Eigen::MatrixXd _normWeights;
		double _epsilon;

		double _computeMetric(const ::Eigen::MatrixXd& x, const ::Eigen::MatrixXd& y) const;

		virtual JumpCondition* _doClone() const
		{
			return (new JumpCondition(*this));
		}

	};

}

#endif
