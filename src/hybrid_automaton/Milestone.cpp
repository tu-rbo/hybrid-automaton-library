#include "Milestone.h"
#include <iostream>

using namespace std;

Milestone::Milestone() : 
MDPNode(),
status_(INVALID),
activated_(false),
name_("default")
{
}

Milestone::Milestone(const std::string& milestone_name) : 
MDPNode(milestone_name),
status_(INVALID),
activated_(false),
name_(milestone_name)
{
}

Milestone::Milestone(const Milestone &milestone_cpy) : 
MDPNode(milestone_cpy),
status_(milestone_cpy.status_),
activated_(milestone_cpy.activated_),
name_(milestone_cpy.name_)
{
}

Milestone::~Milestone()
{
}

Milestone::Status Milestone::getStatus() const 
{
	return this->status_;
}

void Milestone::setStatus(Milestone::Status status) 
{
	this->status_ = status;
}

void Milestone::activate(rxSystem* sys) 
{
	this->activated_ = true;
}

void Milestone::update(const rTime& t) 
{
}

void Milestone::deactivate() 
{
	this->activated_ = false;
}

Milestone* Milestone::clone() const
{
	Milestone* new_milestone = new Milestone(*this);
	return new_milestone;
}

bool Milestone::operator ==(const Milestone & n) const 
{
	return (this->name_ == n.getName());
}

bool Milestone::operator !=(const Milestone & n) const 
{
	return !(this->name_ == n.getName());
}

dVector Milestone::getConfiguration() const
{
	return dVector();
}

std::string Milestone::getName() const
{
	return this->name_;
}

bool Milestone::hasConverged(rxSystem* sys)
{
	//std::string s("[Milestone::hasConverged(rxSystem*)]: Error hasConverged method not implemented");
	//std::cout << s << std::endl;
	//throw s;
	return true;
}

/*
MilestoneType Milestone::getType() const
{
	return this->type_;
}

void Milestone::setType(MilestoneType type)
{
	this->type_ = type;
}
*/
//bool Milestone::operator== (const Node& n) const {
//	return false;
//}
//
//bool Milestone::operator!= (const Node& n) const {
//	return true;
//}
//bool Milestone::operator ==(const Milestone * n) const {
//	return this->status == n->status;
//}