#include "hybrid_automaton/JumpCondition.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {

	JumpCondition::JumpCondition():
		_goalSource(CONSTANT),
		_jump_criterion(NORM_L1),
		_epsilon(0.0),
		_is_goal_relative(false)
	{

	}

	JumpCondition::~JumpCondition()
	{

	}

	JumpCondition::JumpCondition(const ha::JumpCondition &jc)
	{
		this->_goalSource = jc._goalSource;
		this->_goal = jc._goal;
		this->_controller = jc._controller;
		this->_sensor = jc._sensor;
		this->_jump_criterion = jc._jump_criterion;
		this->_norm_weights = jc._norm_weights;
		this->_epsilon = jc._epsilon;
	}

	void JumpCondition::initialize(const double& t) 
	{
		this->_sensor->initialize(t); 
	}

	void JumpCondition::terminate() 
	{
		this->_sensor->terminate();
	}

	void JumpCondition::step(const double& t) 
	{
		this->_sensor->step(t);
	}

	bool JumpCondition::isActive() const 
	{
		::Eigen::MatrixXd current = this->_sensor->getCurrentValue();
		::Eigen::MatrixXd initial = this->_sensor->getInitialValue();
		::Eigen::MatrixXd desired = this->getGoal();

		if(desired.rows() != current.rows() || desired.cols() != current.cols())
		{
			HA_THROW_ERROR("JumpCondition.isActive", "Dimension mismatch in sensor and goal - sensor: "
			<<current.rows()<<"x"<<current.cols()<<", current: "<<desired.rows()<<"x"<<desired.cols()<<"!"
			<< " Sensor Type: " << this->_sensor->getType());
		}


		 
		if(initial.rows() != current.rows() || initial.cols() != current.cols())
		{
			HA_THROW_ERROR("JumpCondition.isActive", "Dimension mismatch in initial and current sensor values: "
			<<current.rows()<<"x"<<current.cols()<<", current: "<<initial.rows()<<"x"<<initial.cols()<<"!");
		}
		 
		if(this->_is_goal_relative)
		{
			return (this->_computeMetric(this->_sensor->getRelativeCurrentValue(), desired) <= _epsilon);
		}else{
			return (this->_computeMetric( this->_sensor->getCurrentValue(), desired) <= _epsilon);
		}
	}

	double JumpCondition::_computeMetric(const ::Eigen::MatrixXd& x, const ::Eigen::MatrixXd& y) const
	{
		//First check if weights are given - otherwise use default weights (=1.0)
		::Eigen::MatrixXd weights;
		if(_norm_weights.rows() == 0)
		{
			weights.resize(x.rows(), x.cols());
			weights.setConstant(1.0);
		}
		else
		{
			weights = _norm_weights;
		}
		
		double ret = 0;
		switch(_jump_criterion) {
			case NORM_L1: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{ 
						ret += weights(i,j) * fabs(x(i,j) - y(i,j));
					}
				}
				break;
			
			case NORM_L2: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						ret += weights(i,j) * pow(fabs(x(i,j) - y(i,j)),2);
					}
				}
				ret = sqrt(ret);
				break;

			case NORM_L_INF: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						ret = std::max(ret, weights(i,j) * fabs(x(i,j) - y(i,j)));
					}
				}
				break;
			case NORM_ROTATION:
				HA_THROW_ERROR("JumpCondition._computeMetric", "Not Implemented: ROTATION");
				break;
			case NORM_TRANSFORM:
				HA_THROW_ERROR("JumpCondition._computeMetric", "Not Implemented: TRANSFORM");
				break;

			case THRESH_UPPER_BOUND:
				
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						//std::cout << "upper bound" << weights(i,j)*(y(i,j) - x(i,j)) << std::endl;
						ret = std::max(ret, weights(i,j)*(y(i,j) - x(i,j)));
					}
				}
				break;

			case THRESH_LOWER_BOUND:
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						//std::cout << weights(i,j)*(x(i,j) - y(i,j)) << std::endl;
						ret = std::max(ret, weights(i,j)*(x(i,j) - y(i,j)));
					}
				}
				break;
			default:
				HA_THROW_ERROR("JumpCondition._computeMetric", "Not Implemented: unknown jump criterion");
		}

		return ret;
	}

	void JumpCondition::setControllerGoal(const Controller::ConstPtr& controller)
	{
		_goalSource = CONTROLLER;
		_controller = controller;
	}

	void JumpCondition::setConstantGoal(const ::Eigen::MatrixXd goal)
	{
		_goalSource = CONSTANT;
		_goal = goal;
	}

	void JumpCondition::setConstantGoal(double goal)
	{
		_goalSource = CONSTANT;
		_goal.resize(1,1);
		_goal<<goal;
	}

	::Eigen::MatrixXd JumpCondition::getGoal() const	
	{
		switch(_goalSource) {
			case CONSTANT: 
				return _goal;
			
			case CONTROLLER: 
				return _controller->getGoal();

			case ROSTOPIC: 				
				HA_THROW_ERROR("JumpCondition::getGoal", "Not Implemented: ROSTOPIC");
				break;

			default:
				HA_THROW_ERROR("JumpCondition::getGoal", "Not Implemented: unknown goal source");
		}

		return ::Eigen::MatrixXd();
	}
	
	void JumpCondition::setSensor(const Sensor::Ptr sensor) 
	{
		_sensor = sensor;
	}

	Sensor::ConstPtr JumpCondition::getSensor() const 
	{
		return _sensor;
	} 

	void JumpCondition::setJumpCriterion(JumpCriterion jump_criterion, ::Eigen::MatrixXd weights)
	{
		if(weights.rows() == 0)
			HA_INFO("JumpCondition::setJumpCriterion", "No value given for weights. Using default weights of 1.");
		_jump_criterion = jump_criterion;
		_norm_weights = weights;
	}
	
	JumpCondition::JumpCriterion JumpCondition::getJumpCriterion() const
	{
		return _jump_criterion;
	}

	::Eigen::MatrixXd JumpCondition::getNormWeights() const
	{
		return _norm_weights;
	}

	void JumpCondition::setEpsilon(double epsilon)
	{
		_epsilon = epsilon;
	}

	double JumpCondition::getEpsilon() const
	{
		return _epsilon;
	}

	void JumpCondition::setSourceModeName(const std::string& sourceModeName)
	{
		_sourceModeName = sourceModeName;
	}

	DescriptionTreeNode::Ptr JumpCondition::serialize(const DescriptionTree::ConstPtr& factory) const 
	{ 
		DescriptionTreeNode::Ptr tree = factory->createNode("JumpCondition");

		switch(_goalSource) {
			case CONSTANT: 
				tree->setAttribute<Eigen::MatrixXd>(std::string("goal"), _goal);
				break;
			
			case CONTROLLER: 
				tree->setAttribute<std::string>(std::string("controller"), this->_controller->getName());
				break;
			
			case ROSTOPIC: 				
				HA_THROW_ERROR("JumpCondition::serialize", "Not Implemented: ROSTOPIC");
				break;

			default:
				HA_THROW_ERROR("JumpCondition::serialize", "Not Implemented: unknown goal source");
		}

		tree->setAttribute<int>(std::string("jump_criterion"), this->_jump_criterion);
		if(this->_norm_weights.rows() > 0){
			tree->setAttribute< ::Eigen::MatrixXd>(std::string("norm_weights"), this->_norm_weights);
		}

		tree->setAttribute<double>(std::string("epsilon"), this->_epsilon);

		if (!this->_sensor) {
			HA_THROW_ERROR("JumpCondition::serialize", "All JumpConditions need to have a sensor!");
		}


		tree->setAttribute<bool>(std::string("goal_is_relative"), this->_is_goal_relative);

		tree->addChildNode(this->_sensor->serialize(factory));

		return tree;
	}

	void JumpCondition::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha) 
	{
		if (tree->getType() != "JumpCondition") {
			HA_THROW_ERROR("JumpCondition.deserialize", "JumpCondition must have type 'JumpCondition', not '" << tree->getType() << "'!");
		}

		//////////////////////////////
		////SENSORS///////////////////
		//////////////////////////////
		DescriptionTreeNode::ConstNodeList sensorList;
		tree->getChildrenNodes("Sensor", sensorList);

		if (sensorList.empty()) {
			HA_THROW_ERROR("JumpCondition.deserialize", "No Sensor found!");
		}

		if (sensorList.size() > 1) {
			HA_THROW_ERROR("JumpCondition.deserialize", "Too many (>1) sensors found!");
		}

		DescriptionTreeNode::ConstPtr first = * (sensorList.begin());
		this->_sensor = HybridAutomaton::createSensor(first, system, ha);

		//////////////////////////////
		////GOALS/////////////////////
		//////////////////////////////
		Eigen::MatrixXd goal;
		if(tree->getAttribute<Eigen::MatrixXd>(std::string("goal"), goal))
		{
			this->setConstantGoal(goal);
		}

		std::string controllerName;
		if(tree->getAttribute<std::string>(std::string("controller"), controllerName))
		{
			//Now match name to controller
			if(_sourceModeName == "")
				HA_THROW_ERROR("JumpCondition.deserialize", "When deserializing a controller goal, you need to call JumpCondition::setSourceModeName() and pass the name of the mode this Jump Condition's Switch emanates from!!!");

			Controller::ConstPtr ctrl = ha->getControllerByName(_sourceModeName, controllerName);
			//Todo error handling!

			this->setControllerGoal(ctrl);
		}

		//////////////////////////////
		////PARAMETERS////////////////
		//////////////////////////////
		tree->getAttribute<Eigen::MatrixXd>("norm_weights", _norm_weights);
		tree->getAttribute<double>("epsilon", _epsilon, 0.0);

		int jump_criterion = -1;
		if(tree->getAttribute<int>("jump_criterion", jump_criterion))
		{
			if(jump_criterion < 0 || jump_criterion >= NUM_CRITERIA)
				HA_THROW_ERROR("JumpCondition.deserialize", "Unknown jumpCriterion " << jump_criterion);

			//Cast int to enum
			_jump_criterion = static_cast<JumpCriterion> (jump_criterion);	
		};

		tree->getAttribute<bool>("goal_is_relative", _is_goal_relative);

	}

	void JumpCondition::setGoalRelative()
	{
		this->_is_goal_relative = true;
	}

	void JumpCondition::setGoalAbsolute()
	{
		this->_is_goal_relative = false;
	}

	bool JumpCondition::isGoalRelative() const
	{
		return this->_is_goal_relative;
	}
}