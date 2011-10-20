#include "msgs\JointState.h"

namespace rlab {

JointState::JointState() {
	this->typeid_name = "JointState";
}

JointState::JointState(const std::string& topic) {
	this->typeid_name = "JointState";
	this->topic = topic;
}

JointState::JointState(const std::string& topic, const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort) {
	this->typeid_name = "JointState";
	this->topic = topic;
	this->position = position;
	this->position = velocity;
	this->position = effort;
}

JointState::JointState(const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort) {
	this->typeid_name = "JointState";
	this->position = position;
	this->position = velocity;
	this->position = effort;
}

//std::string JointState::getType(){
//  return typeid(this).name();
//}

bool JointState::canDeserialize(std::string s){
	std::string str=this->typeid_name;
	std::size_t pos = s.find(str);
	if(pos != 0) return false;
	else return true;
}

rlab::NetworkData* JointState::deserialize(std::string & s) {
	//std::string str=typeid(*this).name(); // move to operator <<
	//std::string vString = s.substr(str.size());

	JointState* vd = new JointState();
	(*vd) << s; //vString;
	return vd;
}

std::string JointState::serialize() {
	std::string s;
	(*this) >> s;
	return s;
}

void JointState::update(void* d) {
	JointState* f = static_cast<JointState*>(d);
	this->position = f->getPosition();
	this->velocity = f->getVelocity();
	this->effort = f->getEffort();
	this->updated = true;
}

}
