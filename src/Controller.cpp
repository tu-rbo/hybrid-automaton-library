#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {

Controller::Controller():
_goal(),
_kp(),
_kv(),
_name("default")
{

}

Controller::~Controller()
{

}

Controller::Controller(const ha::Controller &controller)
{
	this->_goal = controller._goal;
	this->_kp = controller._kp;
	this->_kv = controller._kv;
	this->_name = controller._name;
}

DescriptionTreeNode::Ptr Controller::serialize(const DescriptionTree::ConstPtr& factory) const 
{
	DescriptionTreeNode::Ptr tree = factory->createNode("Controller");

	tree->setAttribute<std::string>(std::string("type"), this->getType());
	tree->setAttribute<std::string>(std::string("name"), this->getName());

	tree->setAttribute<Eigen::MatrixXd>(std::string("goal"), this->_goal);
	tree->setAttribute<Eigen::MatrixXd>(std::string("kp"), this->_kp);
	tree->setAttribute<Eigen::MatrixXd>(std::string("kv"), this->_kv);

	return tree;
}

void Controller::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system) 
{
	if (tree->getType() != "Controller") {
		std::stringstream ss;
		ss << "[Controller::deserialize] DescriptionTreeNode must have type 'Controller', not '" << tree->getType() << "'!";
		throw ss.str();
	}
	tree->getAttribute<std::string>("type", _type, "");

	if (_type == "" || !HybridAutomaton::isControllerRegistered(_type)) {
		std::stringstream ss;
		ss << "[Controller::deserialize] Controller type '" << _type << "' "
		   << "invalid - empty or not registered with HybridAutomaton!";
		throw ss.str();
	}

	tree->getAttribute<std::string>("name", _name, "");
	// TODO register object with HybridAutomaton / check that it is unique!

	// FIXME nicer error handling, sir?
	if(!tree->getAttribute<Eigen::MatrixXd>(std::string("goal"), this->_goal))
	{
		std::cout << "error" <<std::endl;
	}

	if(!tree->getAttribute<Eigen::MatrixXd>(std::string("kp"), this->_kp))
	{
		std::cout << "error" <<std::endl;
	}

	if(!tree->getAttribute<Eigen::MatrixXd>(std::string("kv"), this->_kv))
	{
		std::cout << "error" <<std::endl;
	}
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

std::string Controller::getName() const
{
	return this->_name;
}

void Controller::setName(const std::string& new_name)
{
	this->_name = new_name;
}

std::string Controller::getType() const
{
	return this->_type;
}

void Controller::setType(const std::string& new_type)
{
	this->_type = new_type;
}
}