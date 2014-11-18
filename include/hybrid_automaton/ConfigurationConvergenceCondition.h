#ifndef HYBRID_AUTOMATON_CONFIGURATION_CONVERGENCE_CONDITION_H
#define HYBRID_AUTOMATON_CONFIGURATION_CONVERGENCE_CONDITION_H

#include "hybrid_automaton/JumpCondition.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class ConfigurationConvergenceCondition;
	typedef boost::shared_ptr<ConfigurationConvergenceCondition> ConfigurationConvergenceConditionPtr;

	class ConfigurationConvergenceCondition : public JumpCondition
	{
	public:

		typedef boost::shared_ptr<ConfigurationConvergenceCondition> Ptr;

		ConfigurationConvergenceCondition();
		virtual ~ConfigurationConvergenceCondition() {}

		ConfigurationConvergenceConditionPtr clone() const
		{
			return (ConfigurationConvergenceConditionPtr(_doClone()));
		}

		virtual void activate(const double& t);
		virtual void deactivate();

		virtual void step(const double& t);
		virtual bool isActive() const;

		virtual void setEpsilon(::Eigen::MatrixXd epsilon);
		virtual ::Eigen::MatrixXd getEpsilon() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system);

	protected:
		::Eigen::MatrixXd _epsilon;

		virtual ConfigurationConvergenceCondition* _doClone() const
		{
			return (new ConfigurationConvergenceCondition(*this));
		}

	};

}

#endif
