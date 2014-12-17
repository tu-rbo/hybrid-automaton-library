#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/error_handling.h"

namespace ha {

	Controller::Controller()
		:_goal(),
		_goal_is_relative(false),
		_kp(),
		_kv(),
		_completion_times(),
		_priority(0.0),
		_name("default")
	{

	}

	Controller::~Controller()
	{

	}

	Controller::Controller(const ha::Controller &controller)
	{
		this->_goal = controller._goal;
		this->_goal_is_relative = controller._goal_is_relative;		
		this->_kp = controller._kp;
		this->_kv = controller._kv;
		this->_completion_times = controller._completion_times;
		this->_priority = controller._priority;
		this->_name = controller._name;
	}

	DescriptionTreeNode::Ptr Controller::serialize(const DescriptionTree::ConstPtr& factory) const 
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Controller");

		tree->setAttribute<std::string>(std::string("type"), this->getType());
		tree->setAttribute<std::string>(std::string("name"), this->getName());

		tree->setAttribute<Eigen::MatrixXd>(std::string("goal"), this->_goal);
		tree->setAttribute<bool>(std::string("goal_is_relative"), this->_goal_is_relative);
		tree->setAttribute<Eigen::MatrixXd>(std::string("kp"), this->_kp);
		tree->setAttribute<Eigen::MatrixXd>(std::string("kv"), this->_kv);
		tree->setAttribute<Eigen::MatrixXd>(std::string("completion_times"), this->_completion_times);
		tree->setAttribute<double>(std::string("priority"), this->_priority);

		std::map<std::string, std::string>::const_iterator it;
		for (it = this->_additional_arguments.begin(); it != this->_additional_arguments.end(); ++it) {
			tree->setAttribute(it->first, it->second);
		}

		return tree;
	}

	void Controller::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha) 
	{
		if (tree->getType() != "Controller") {
			HA_THROW_ERROR("Controller.deserialize", "DescriptionTreeNode must have type 'Controller', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isControllerRegistered(_type)) {
			HA_THROW_ERROR("Controller.deserialize", "Controller type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		if(!tree->getAttribute<std::string>("name", _name, ""))
			HA_WARN("Controller.deserialize", "No \"name\" parameter given in Controller - using default value");
		// TODO register object with HybridAutomaton / check that it is unique!

		if(!tree->getAttribute<Eigen::MatrixXd>(std::string("goal"), this->_goal))
			HA_WARN("Controller.deserialize", "No \"goal\" parameter given in Controller "<<_name<<" - using default value");
		
		if(!tree->getAttribute<bool>(std::string("goal_is_relative"), this->_goal_is_relative, false))
			HA_WARN("Controller.deserialize", "No \"goal_is_relative\" parameter given in Controller "<<_name<<" - using default value");
		
		if(!tree->getAttribute<Eigen::MatrixXd>(std::string("kp"), this->_kp))
			HA_WARN("Controller.deserialize", "No \"kp\" parameter given in Controller "<<_name<<" - using default value");
		
		if(!tree->getAttribute<Eigen::MatrixXd>(std::string("kv"), this->_kv))
			HA_WARN("Controller.deserialize", "No \"kv\" parameter given in Controller "<<_name<<" - using default value");

		if(!tree->getAttribute<Eigen::MatrixXd>(std::string("completion_times"), this->_completion_times))
			HA_WARN("Controller.deserialize", "No \"completion_times\" parameter given in Controller "<<_name<<" - using default value");
		
		if(!tree->getAttribute<double>(std::string("priority"), this->_priority, 0.0))
			HA_WARN("Controller.deserialize", "No \"priority\" parameter given in Controller "<<_name<<" - using default value");

		// write all arguments into "_additional_arguments" field
		tree->getAllAttributes(_additional_arguments);
	}

	int Controller::getDimensionality() const
	{
		return -1;
	}

	Eigen::MatrixXd Controller::getGoal() const
	{
		return this->_goal;
	}

	void Controller::setGoal(const Eigen::MatrixXd& new_goal)
	{
		this->_goal = new_goal;
	}

	bool Controller::getGoalIsRelative() const
	{
		return _goal_is_relative;
	}
	
	void Controller::setGoalIsRelative(bool is_goal_relative)
	{
		_goal_is_relative = is_goal_relative;
	}

	Eigen::MatrixXd Controller::getKp() const
	{
		return this->_kp;
	}

	void Controller::setKp(const Eigen::MatrixXd& new_kp)
	{
		this->_kp = new_kp;
	}

	Eigen::MatrixXd Controller::getKv() const
	{
		return this->_kv;
	}

	void Controller::setKv(const Eigen::MatrixXd& new_kv)
	{
		this->_kv = new_kv;
	}

	void Controller::setCompletionTime(const double& t)
	{
		this->_completion_times.resize(1,1);
		this->_completion_times(0) = t;
	}
	void Controller::setCompletionTimes(const Eigen::MatrixXd& new_times)
	{
		this->_completion_times = new_times;
	}

	Eigen::MatrixXd Controller::getCompletionTimes() const
	{
		return this->_completion_times;
	}

	double Controller::getPriority() const
	{
		return _priority;
	}
	
	void Controller::setPriority(double priority)
	{
		_priority = priority;
	}

	std::string Controller::getName() const
	{
		return this->_name;
	}

	void Controller::setName(const std::string& new_name)
	{
		this->_name = new_name;
	}

	const std::string Controller::getType() const
	{
		return this->_type;
	}

	void Controller::setType(const std::string& new_type)
	{
		this->_type = new_type;
	}
}