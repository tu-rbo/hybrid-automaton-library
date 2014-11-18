#include "hybrid_automaton/ConfigurationConvergenceCondition.h"

namespace ha {

	ConfigurationConvergenceCondition::ConfigurationConvergenceCondition()
	{
	}

	void ConfigurationConvergenceCondition::activate(const double& t) {
	}

	void ConfigurationConvergenceCondition::deactivate() {
	}

	bool ConfigurationConvergenceCondition::isActive() const {
		::Eigen::MatrixXd goal = this->getGoal();
		for(int i = 0; i < goal.rows(); i++)
		{
			if (fabs(this->getGoal()(i) - this->_system->getConfiguration()(i)) > _epsilon(i))
				return false;
		}

		return true;
	}

	void ConfigurationConvergenceCondition::step(const double& t) {
	}

	void ConfigurationConvergenceCondition::setEpsilon(::Eigen::MatrixXd epsilon)
	{
		if(epsilon.rows() != 1 || epsilon.cols() != _system->getDof())
			HA_THROW_ERROR("ConfigurationConvergenceCondition::setEpsilon", "Wrong dimensions: "); 
		_epsilon = epsilon;
	}

	::Eigen::MatrixXd ConfigurationConvergenceCondition::getEpsilon() const
	{
		return _epsilon;
	}

	DescriptionTreeNode::Ptr ConfigurationConvergenceCondition::serialize(const DescriptionTree::ConstPtr& factory) const {
		// TODO
		return DescriptionTreeNode::Ptr();
	}

	void ConfigurationConvergenceCondition::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system) {
		// TODO
	}

}