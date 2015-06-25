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
		_v_max(),
		_a_max(),
		_do_reinterpolation(false),
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
		this->_v_max = controller._v_max;
		this->_a_max = controller._a_max;
		this->_do_reinterpolation = controller._do_reinterpolation;
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
		tree->setAttribute<Eigen::MatrixXd>(std::string("v_max"), this->_v_max);
		tree->setAttribute<Eigen::MatrixXd>(std::string("a_max"), this->_a_max);
		tree->setAttribute<bool>(std::string("reinterpolation"), this->_do_reinterpolation);
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

        if (!ha->getDeserializeDefaultEntities()) {
            if (_type == "" || !HybridAutomaton::isControllerRegistered(_type)) {
                HA_THROW_ERROR("Controller.deserialize", "Controller type '" << _type << "' "
                               << "invalid - empty or not registered with HybridAutomaton!");
            } else {
                setType(_type);
            }
        }

		if(!tree->getAttribute<std::string>("name", _name, ""))
			HA_WARN("Controller.deserialize", "No \"name\" parameter given in Controller - using default value (" << _name << ")");
		// TODO register object with HybridAutomaton / check that it is unique!

		if(!tree->getAttribute<Eigen::MatrixXd>(std::string("goal"), this->_goal))
			HA_WARN("Controller.deserialize", "No \"goal\" parameter given in Controller "<<_name<<" - using default value (" << _goal << ")");
		
		if(!tree->getAttribute<bool>(std::string("goal_is_relative"), this->_goal_is_relative, false))
			HA_WARN("Controller.deserialize", "No \"goal_is_relative\" parameter given in Controller "<<_name<<" - using default value (" << _goal_is_relative << ")");
		
		if(!tree->getAttribute<Eigen::MatrixXd>(std::string("kp"), this->_kp))
			HA_WARN("Controller.deserialize", "No \"kp\" parameter given in Controller "<<_name<<" - using default value (" << _kp << ")");
		
		if(!tree->getAttribute<Eigen::MatrixXd>(std::string("kv"), this->_kv))
			HA_WARN("Controller.deserialize", "No \"kv\" parameter given in Controller "<<_name<<" - using default value (" << _kv << ")");

		if(!tree->getAttribute<Eigen::MatrixXd>(std::string("completion_times"), this->_completion_times))
			HA_WARN("Controller.deserialize", "No \"completion_times\" parameter given in Controller "<<_name<<" - using default value (" << _completion_times << ")");

		//TODO: Read v_max and a_max from dml file
		if(!tree->getAttribute<Eigen::MatrixXd>(std::string("v_max"), this->_v_max))
			HA_WARN("Controller.deserialize", "No \"v_max\" parameter given in Controller "<<_name<<" - using default value (" << _v_max << ")");

		if(!tree->getAttribute<Eigen::MatrixXd>(std::string("a_max"), this->_a_max))
			HA_WARN("Controller.deserialize", "No \"a_max\" parameter given in Controller "<<_name<<" - using default value (" << _a_max << ")");

		if(!tree->getAttribute<bool>(std::string("reinterpolation"), this->_do_reinterpolation))
			HA_WARN("Controller.deserialize", "No \"reinterpolation\" parameter given in Controller "<<_name<<" - using default value (" << _do_reinterpolation << ")");
		
		if(!tree->getAttribute<double>(std::string("priority"), this->_priority, 0.0))
			HA_WARN("Controller.deserialize", "No \"priority\" parameter given in Controller "<<_name<<" - using default value (" << _priority << ")");

		// write all arguments into "_additional_arguments" field
		tree->getAllAttributes(_additional_arguments);

		_system = system;
	}

	void Controller::setSystem(const System::ConstPtr& system)
	{
		this->_system = system;
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

	void Controller::setMaximumVelocity(const Eigen::MatrixXd& max_vel)
	{
		_v_max = max_vel;
	}

	void Controller::setMaximumAcceleration(const Eigen::MatrixXd& max_acc)
	{
		_a_max = max_acc;
	}

	Eigen::MatrixXd	Controller::getMaximumVelocity() const
	{
		return _v_max;
	}

	Eigen::MatrixXd	Controller::getMaximumAcceleration() const
	{
		return _a_max;
	}

    void Controller::setDoReinterpolation(bool do_reinterpolation)
	{
		_do_reinterpolation = do_reinterpolation;
	}

	bool Controller::getDoReinterpolation() const
	{
		return _do_reinterpolation;
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

	double Controller::computeInterpolationTime(const Eigen::MatrixXd& x0, const Eigen::MatrixXd& xf, const Eigen::MatrixXd& xdot0, const Eigen::MatrixXd& xdotf) const{
		
		//Currently returns only time for linear interpolation!
		//TODO: cubic splines!
		double tf = 0.0;

		if(x0.rows() != xf.rows() || x0.cols() != xf.cols())
		{
			HA_THROW_ERROR("rlabController::computeInterpolationTime", "Dimension mismatch between initial (dim " << x0.rows() << ") and final (dim " << xf.rows() <<") position!!");
		}

		if(x0.rows() != _v_max.rows() || x0.cols() != _v_max.cols())
		{
			HA_THROW_ERROR("rlabController::computeInterpolationTime", "Dimension mismatch between initial (dim " << x0.rows() << ") and max velocity (dim " << _v_max.rows() <<")!!");
		}

		if(_a_max.rows() != 0 || xdot0.rows() != 0 || xdotf.rows() != 0)
		{
			HA_WARN("rlabController::computeInterpolationTime", "maximal acceleration limits currently not supported!");
		}

		if(x0.cols() > 1 || xf.cols() > 1)
			HA_THROW_ERROR("rlabController::computeInterpolationTime", "Initial and/or final position are matrices!!");

		for(int i = 0; i < x0.rows(); i++)
		{
			for(int j = 0; j < x0.cols(); j++)
			{
				if( _v_max(i,j) <= 0.0)
					HA_THROW_ERROR("rlabController::computeInterpolationTime", "You gave zero or negative maximum velocity!");
				tf = std::max(tf,fabs(xf(i,j) - x0(i,j)) / _v_max(i,j));
			}
		}

		return tf;
	}
}
