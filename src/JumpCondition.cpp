#include "hybrid_automaton/JumpCondition.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {

	JumpCondition::JumpCondition():
		_goalSource(CONSTANT),
		_normType(L1),
		_epsilon(0.0)
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
		this->_normType = jc._normType;
		this->_normWeights = jc._normWeights;
		this->_epsilon = jc._epsilon;
	}

	void JumpCondition::initialize(const double& t) 
	{
	}

	void JumpCondition::terminate() 
	{
	}

	void JumpCondition::step(const double& t) 
	{
		this->_sensor->step(t);
	}

	bool JumpCondition::isActive() const 
	{
		::Eigen::MatrixXd current = this->_sensor->getCurrentValue();
		::Eigen::MatrixXd desired = this->getGoal();

		if(desired.rows() != current.rows() || desired.cols() != current.cols())
		{
			HA_THROW_ERROR("JumpCondition.isActive", "Dimension mismatch in sensor and goal - sensor: "
			<<current.rows()<<"x"<<current.cols()<<", current: "<<desired.rows()<<"x"<<desired.cols()<<"!");
		}
		 
		return (this->_computeMetric(current, desired) <= _epsilon);
	}

	double JumpCondition::_computeMetric(const ::Eigen::MatrixXd& x, const ::Eigen::MatrixXd& y) const
	{
		//First check if weights are given - otherwise use default weights (=1.0)
		::Eigen::MatrixXd weights;
		if(_normWeights.rows() == 0)
		{
			weights.resize(x.rows(), x.cols());
			weights.setConstant(1.0);
		}
		else
		{
			weights = _normWeights;
		}
		
		double ret = 0;
		switch(_normType) {
			case L1: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{ 
						ret += weights(i,j) * fabs(x(i,j) - y(i,j));
					}
				}
				break;
			
			case L2: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						ret += weights(i,j) * pow(fabs(x(i,j) - y(i,j)),2);
					}
				}
				ret = sqrt(ret);
				break;

			case L_INF: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						ret = std::max(ret, weights(i,j) * fabs(x(i,j) - y(i,j)));
					}
				}
				break;
			case ROTATION:
				HA_THROW_ERROR("JumpCondition._computeMetric", "Not Implemented: ROTATION");
				break;
			case TRANSFORM:
				HA_THROW_ERROR("JumpCondition._computeMetric", "Not Implemented: TRANSFORM");
				break;
			default:
				HA_THROW_ERROR("JumpCondition._computeMetric", "Not Implemented: unknown norm");
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

	void JumpCondition::setNorm(Norm normType, ::Eigen::MatrixXd weights)
	{
		_normType = normType;
		_normWeights = weights;
	}
	
	JumpCondition::Norm JumpCondition::getNormType() const
	{
		return _normType;
	}

	::Eigen::MatrixXd JumpCondition::getNormWeights() const
	{
		return _normWeights;
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

		tree->setAttribute<int>(std::string("normType"), this->_normType);
		if(this->_normWeights.rows() > 0){
			tree->setAttribute< ::Eigen::MatrixXd>(std::string("normWeights"), this->_normWeights);
		}
		tree->setAttribute<double>(std::string("epsilon"), this->_epsilon);

		if (!this->_sensor) {
			HA_THROW_ERROR("JumpCondition::serialize", "All JumpConditions need to have a sensor!");
		}
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
		tree->getAttribute<Eigen::MatrixXd>("normWeights", _normWeights);
		tree->getAttribute<double>("epsilon", _epsilon, 0.0);

		int normType = -1;
		if(tree->getAttribute<int>("normType", normType))
		{
			if(normType < 0 || normType >= NUM_NORMS)
				HA_THROW_ERROR("JumpCondition.deserialize", "Unknown normType "<<normType);

			//Cast int to enum
			_normType = static_cast<Norm> (normType);	
		};

	}
}