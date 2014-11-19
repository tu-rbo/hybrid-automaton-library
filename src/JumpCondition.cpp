#include "hybrid_automaton/JumpCondition.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {

	JumpCondition::JumpCondition():
		//_goal(),
		_controller(NULL),
		_type("")
	{

	}

	JumpCondition::~JumpCondition()
	{

	}

	JumpCondition::JumpCondition(const ha::JumpCondition &jc)
	{
		this->_goal = jc._goal;
		this->_controller = jc._controller;
		this->_type = jc._type;
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

	std::string JumpCondition::getType() const
	{
		return this->_type;
	}

	void JumpCondition::setType(const std::string& new_type)
	{
		this->_type = new_type;
	}

	DescriptionTreeNode::Ptr JumpCondition::serialize(const DescriptionTree::ConstPtr& factory) const 
	{ 
		DescriptionTreeNode::Ptr tree = factory->createNode("JumpCondition");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

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

		tree->getAttribute<std::string>("type", _type, "");

		//TODO!!!
		//if (_type == "" || !HybridAutomaton::isJumpConditionRegistered(_type)) {
		//	HA_THROW_ERROR("JumpCondition.deserialize", "JumpCondition type '" << _type << "' "
		//	   << "invalid - empty or not registered with HybridAutomaton!");
		//}

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

	}
}