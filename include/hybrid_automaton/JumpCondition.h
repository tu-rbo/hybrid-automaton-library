#ifndef HYBRID_AUTOMATON_JUMP_CONDITION_H
#define HYBRID_AUTOMATON_JUMP_CONDITION_H

#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/error_handling.h"

#include "hybrid_automaton/Controller.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class JumpCondition;
	typedef boost::shared_ptr<JumpCondition> JumpConditionPtr;

	class JumpCondition : public Serializable
	{
	public:

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

		virtual void activate(const double& t) {
			HA_THROW_ERROR("JumpCondition.activate", "not implemented");
		};

		virtual void deactivate() {
			HA_THROW_ERROR("JumpCondition.deactivate", "not implemented");
		};

		virtual void step(const double& t) {
			HA_THROW_ERROR("JumpCondition.step", "not implemented");
		};

		virtual bool isActive() const {
			HA_THROW_ERROR("JumpCondition.isActive", "not implemented");
		};

		virtual void setControllerGoal(const Controller* controller);
		virtual void setConstantGoal(const ::Eigen::MatrixXd goal);

		virtual ::Eigen::MatrixXd JumpCondition::getGoal() const;

		virtual std::string getType() const;

		virtual void setType(const std::string& new_type);

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

	protected:
		System::ConstPtr _system;

		std::string		  _type;
		
		const Controller* _controller;
		boost::shared_ptr<const ::Eigen::MatrixXd> _goal;

		virtual JumpCondition* _doClone() const
		{
			return (new JumpCondition(*this));
		}

	};

}

#endif
