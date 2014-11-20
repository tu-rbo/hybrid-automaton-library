#include "hybrid_automaton/JumpCondition.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {

	JumpCondition::JumpCondition():
		_goalSource(CONSTANT),
		_controller(NULL),
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

	void JumpCondition::activate(const double& t) 
	{
	}

	void JumpCondition::deactivate() 
	{
	}

	void JumpCondition::step(const double& t) 
	{
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
		 
		return (this->_computeMetric(current, desired) <_epsilon);
	}

	double JumpCondition::_computeMetric(::Eigen::MatrixXd x, ::Eigen::MatrixXd y) const
	{
		double ret = 0;
		switch(_normType) {
			case L1: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{ 
						ret += _normWeights(i,j) * fabs(x(i,j) - y(i,j));
					}
				}
				break;
			
			case L2: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						ret += _normWeights(i,j) * pow(fabs(x(i,j) - y(i,j)),2);
					}
				}
				ret = sqrt(ret);
				break;

			case L_INF: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						ret = std::max(ret, _normWeights(i,j) * fabs(x(i,j) - y(i,j)));
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

	void JumpCondition::setControllerGoal(const Controller* controller)
	{
		_goalSource = CONTROLLER;
		_controller = controller;
	}

	void JumpCondition::setConstantGoal(const ::Eigen::MatrixXd goal)
	{
		_goalSource = CONSTANT;
		_goal = goal;
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

		
		if(weights.rows() == 0)
		{ //Default weights = 1 for every entry
			//query sensor (just to get the dimensions for the weights)
			_normWeights = _sensor->getCurrentValue();
			
			//Default weighting: 1 for every entry
			_normWeights.setConstant(1);
		}
		else
		{
			_normWeights = weights;
		}


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
			tree->setAttribute<::Eigen::MatrixXd>(std::string("normWeights"), this->_normWeights);
		}
		tree->setAttribute<double>(std::string("epsilon"), this->_epsilon);

		if (!this->_sensor) {
			HA_THROW_ERROR("ControlMode.serialize", "Sensor is null!");
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
		//TODO

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
			std::string modeName;

			//Now match name to controller
			Controller::Ptr ctrl = ha->getControllerByName(controllerName, modeName);
		}

		//////////////////////////////
		////PARAMETERS////////////////
		//////////////////////////////

		//query sensor (just to get the dimensions for the weights)
		Eigen::MatrixXd defaultWeights = _sensor->getCurrentValue();
		
		//Default weighting: 1 for every entry
		defaultWeights.setConstant(1);
		tree->getAttribute<Eigen::MatrixXd>("normWeights", _normWeights, defaultWeights);

	}
}