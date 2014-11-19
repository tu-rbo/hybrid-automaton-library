#include "hybrid_automaton/JumpCondition.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {

	JumpCondition::JumpCondition():
		//_goal(),
		_controller(NULL)
	{

	}

	JumpCondition::~JumpCondition()
	{

	}

	JumpCondition::JumpCondition(const ha::JumpCondition &jc)
	{
		this->_goal = jc._goal;
		this->_controller = jc._controller;
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
		 
		return (this->_computeNorm(current, desired) <_epsilon);
	}

	double JumpCondition::_computeNorm(::Eigen::MatrixXd x, ::Eigen::MatrixXd y) const
	{
		double ret = 0;
		switch(_normType) {
			case L1: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{ 
						ret += _weights(i,j) * fabs(x(i,j) - y(i,j));
					}
				}
				break;
			
			case L2: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						ret += _weights(i,j) * pow(fabs(x(i,j) - y(i,j)),2);
					}
				}
				ret = sqrt(ret);
				break;

			case L_INF: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						ret = std::max(ret, _weights(i,j) * fabs(x(i,j) - y(i,j)));
					}
				}
				break;
			case ROTATION:
				HA_THROW_ERROR("JumpCondition._computeNorm", "Not Implemented: ROTATION");
				break;
			case TRANSFORM:
				HA_THROW_ERROR("JumpCondition._computeNorm", "Not Implemented: TRANSFORM");
				break;
			default:
				HA_THROW_ERROR("JumpCondition._computeNorm", "Not Implemented: unknown norm");
		}

		return ret;
	}

	void JumpCondition::setControllerGoal(const Controller* controller)
	{
		_controller = controller;
	}

	void JumpCondition::setConstantGoal(const ::Eigen::MatrixXd goal)
	{
		_goal.reset<const ::Eigen::MatrixXd>(&goal);
	}

	::Eigen::MatrixXd JumpCondition::getGoal() const	
	{
		if(_controller)
			return _controller->getGoal();
		else
			return *(_goal.get());
	}

	DescriptionTreeNode::Ptr JumpCondition::serialize(const DescriptionTree::ConstPtr& factory) const 
	{ 
		DescriptionTreeNode::Ptr tree = factory->createNode("JumpCondition");

		if(this->_controller)
			tree->setAttribute<std::string>(std::string("controller"), this->_controller->getName());
		
		if(this->_goal)
			tree->setAttribute<Eigen::MatrixXd>(std::string("goal"), *(this->_goal.get()));

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
		Eigen::MatrixXd defaultWeights = _sensor->getCurrentValue();
		defaultWeights.setConstant(1);
		tree->getAttribute<Eigen::MatrixXd>("weights", _weights, defaultWeights);

	}
}