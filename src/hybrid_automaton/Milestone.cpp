#include "Milestone.h"
#include <iostream>

using namespace std;

Milestone::Milestone() : 
Node(),
status_(INVALID)
{
	this->name_ = std::string("default");
}

Milestone::Milestone(std::string milestone_name) : 
Node(),
status_(INVALID)
{
	this->name_ = milestone_name;
}

Milestone::Milestone(const char* milestone_name) : 
Node(),
status_(INVALID)
{
	this->name_ = std::string(milestone_name);
}

Milestone::Milestone(const Milestone &milestone_cpy) : 
Node(),
status_(milestone_cpy.status_)
{
	this->name_ = milestone_cpy.name_;
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

void Milestone::update() 
{
}

std::string Milestone::toStringXML() const 
{
	throw std::string("WARNING: [Milestone::toStringXML()] Empty method. Derived class' method should have been called instead.");
}

TiXmlElement* Milestone::toElementXML() const 
{
	throw std::string("WARNING: [Milestone::toElementXML(TiXmlElement* root)] Empty method. Derived class' method should have been called instead.");
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