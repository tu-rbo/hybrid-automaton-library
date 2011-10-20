#include "Milestone.h"
#include <iostream>

using namespace std;

Milestone::Milestone() : 
Node(),
status_(INVALID)
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

void Milestone::update() 
{
}

std::string Milestone::toStringXML() const 
{
	throw std::string("WARNING: [Milestone::toStringXML()] Empty method. Derived class' method should have been called instead.");
}

void Milestone::toElementXML(TiXmlElement* root) const 
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
	return this->status_ == n.getStatus();
}

bool Milestone::operator !=(const Milestone & n) const 
{
	return !(this->status_ == n.getStatus());
}

dVector Milestone::getConfiguration() const
{
	return dVector();
}

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