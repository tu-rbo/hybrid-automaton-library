#include "hybrid_automaton/Controller.h"

namespace ha {

	Controller::Controller():
_dimensionality(-1),
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
	this->_dimensionality = controller._dimensionality;
	this->_goal = controller._goal;
	this->_kp = controller._kp;
	this->_kv = controller._kv;
	this->_name = controller._name;
}

void Controller::serialize(const DescriptionTreeNode::Ptr& tree) const 
{
	tree->setAttribute<int>(std::string("dimensionality"), this->_dimensionality);
	tree->setAttribute<Eigen::VectorXd>(std::string("goal"), this->_goal);
	tree->setAttribute<Eigen::VectorXd>(std::string("kp"), this->_kp);
	tree->setAttribute<Eigen::VectorXd>(std::string("kv"), this->_kv);
}

void Controller::deserialize(const DescriptionTreeNode::ConstPtr& tree) 
{
	if(!tree->getAttribute<int>(std::string("dimensionality"), this->_dimensionality))
	{
		std::cout << "error" <<std::endl;
	}

	if(!tree->getAttribute<Eigen::VectorXd>(std::string("goal"), this->_goal))
	{
		std::cout << "error" <<std::endl;
	}

	if(!tree->getAttribute<Eigen::VectorXd>(std::string("kp"), this->_kp))
	{
		std::cout << "error" <<std::endl;
	}

	if(!tree->getAttribute<Eigen::VectorXd>(std::string("kv"), this->_kv))
	{
		std::cout << "error" <<std::endl;
	}
}

int Controller::getDimensionality() const
{
	return this->_dimensionality;
}

void Controller::setDimensionality(const int& new_dimensionality)
{
	this->_dimensionality = new_dimensionality;
}

Eigen::VectorXd Controller::getGoal() const
{
	return this->_goal;
}

void Controller::setGoal(const Eigen::VectorXd& new_goal)
{
	if(new_goal.size() != this->_dimensionality)
	{
		throw std::string("Controller::setGoal. The size of the new goal does not match the dimensionality of the controller");
	}
	this->_goal = new_goal;
}

Eigen::VectorXd Controller::getKp() const
{
	return this->_kp;
}

void Controller::setKp(const Eigen::VectorXd& new_kp)
{
	if(new_kp.size() != this->_dimensionality)
	{
		throw std::string("Controller::setGoal. The size of the new Kp does not match the dimensionality of the controller");
	}
	this->_kp = new_kp;
}

Eigen::VectorXd Controller::getKv() const
{
	return this->_kv;
}

void Controller::setKv(const Eigen::VectorXd& new_kv)
{
	if(new_kv.size() != this->_dimensionality)
	{
		throw std::string("Controller::setGoal. The size of the new Kv does not match the dimensionality of the controller");
	}
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
}