#include "msgs\Transform.h"

namespace rlab {

Transform::Transform() :
 position(3), orientation(4)
{
	this->typeid_name = "Transform";
}

Transform::Transform(const std::string& topic) :
 position(3), orientation(4)
{
	this->typeid_name = "Transform";
	this->topic = topic;
}

Transform::Transform(const std::string& topic, const std::vector<double>& position, const std::vector<double>& orientation, const std::string& parent) {
	this->typeid_name = "Transform";
	this->topic = topic;
	this->position = position;
	this->orientation = orientation;
	this->parent = parent;
}

Transform::Transform(const std::vector<double>& position, const std::vector<double>& orientation, const std::string& parent) {
	this->typeid_name = "Transform";
	this->position = position;
	this->orientation = orientation;
	this->parent = parent;
}

//std::string Transform::getType(){
//  return typeid(this).name();
//}

bool Transform::canDeserialize(std::string s){
	std::string str=this->typeid_name;
	std::size_t pos = s.find(str);
	if(pos != 0) return false;
	else return true;
}

rlab::NetworkData* Transform::deserialize(std::string & s) {
	//std::string str=typeid(*this).name(); // move to operator <<
	//std::string vString = s.substr(str.size());

	Transform* vd = new Transform();
	(*vd) << s; //vString;
	return vd;
}

std::string Transform::serialize() {
	std::string s;
	(*this) >> s;
	return s;
}

void Transform::update(void* d) {
	Transform* f = static_cast<Transform*>(d);
	this->position = f->getPosition();
	this->orientation = f->getOrientation();
	this->parent = f->getParent();
	this->updated = true;
}

}
