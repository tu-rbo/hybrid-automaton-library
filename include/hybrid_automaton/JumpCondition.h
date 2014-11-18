#ifndef HYBRID_AUTOMATON_JUMP_CONDITION_H
#define HYBRID_AUTOMATON_JUMP_CONDITION_H

#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/Controller.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class JumpCondition;
	typedef boost::shared_ptr<JumpCondition> JumpConditionPtr;

	class JumpCondition : public Serializable
	{
	public:

		typedef boost::shared_ptr<JumpCondition> Ptr;

		JumpCondition(){};

		virtual ~JumpCondition() {};

		JumpConditionPtr clone() const
		{
			return (JumpConditionPtr(_doClone()));
		};

		virtual void activate(const double& t) {
			throw "not implemented";
		};

		virtual void deactivate() {
			throw "not implemented";
		};

		virtual void step(const double& t) {
			throw "not implemented";
		};

		virtual bool isActive() const {
			throw "not implemented";
		};

		virtual void setControllerGoal(const Controller* controller){_controller = controller;};
		virtual void setConstantGoal(const ::Eigen::MatrixXd goal){_goal = goal;};

		virtual ::Eigen::MatrixXd getGoal() const	
		{
			if(_controller)
				return _controller->getGoal();
			else
				return _goal;
		};

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const { return DescriptionTreeNode::Ptr(); };
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system) {};

	protected:
		System::ConstPtr _system;

		const Controller* _controller;
		::Eigen::MatrixXd _goal;

		virtual JumpCondition* _doClone() const
		{
			return (new JumpCondition(*this));
		}

	};

}

#endif
