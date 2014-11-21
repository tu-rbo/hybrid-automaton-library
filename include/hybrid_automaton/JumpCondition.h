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

		//Enum for the number of norms - NUM_NORMS is for counting 
		//and should always be kept at last position, even if you add norms.
		enum Norm {L1, L2, L_INF, ROTATION, TRANSFORM, NUM_NORMS}; 

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

		virtual void setControllerGoal(const Controller::ConstPtr& controller);
		virtual void setConstantGoal(const ::Eigen::MatrixXd goal);
		// TODO
		//virtual void setROSTopicGoal(std::string rosTopicName);

		virtual ::Eigen::MatrixXd getGoal() const;

		virtual void setSensor(const Sensor::Ptr sensor);
		virtual Sensor::ConstPtr getSensor() const;

		virtual void setNorm(Norm normType, ::Eigen::MatrixXd weights = ::Eigen::MatrixXd());
		virtual Norm getNormType() const;
		virtual ::Eigen::MatrixXd getNormWeights() const;

		virtual void setEpsilon(double epsilon);
		virtual double getEpsilon() const;

		virtual void setSourceModeName(const std::string& sourceModeName);

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

	protected:

		GoalSource	_goalSource;
		::Eigen::MatrixXd _goal;
		Controller::ConstPtr _controller;

		Sensor::ConstPtr _sensor;

		Norm	_normType;
		::Eigen::MatrixXd _normWeights;
		double _epsilon;

		//The name of the mode this JumpCondition's edge emanates from.
		//Needed for deserialization
		std::string _sourceModeName;

		double _computeMetric(const ::Eigen::MatrixXd& x, const ::Eigen::MatrixXd& y) const;

		virtual JumpCondition* _doClone() const
		{
			return (new JumpCondition(*this));
		}

	};

}

#endif
