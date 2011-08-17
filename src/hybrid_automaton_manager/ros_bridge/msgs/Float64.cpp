#include "msgs\Float64.h"


namespace rlab {

Float64::Float64() {
	this->typeid_name = "Float64";
	this->topic = "";
}

Float64::Float64(const std::string& topic) {
	this->typeid_name = "Float64";
	this->topic = topic;
}

Float64::Float64(const std::string& topic, double data) {
	this->typeid_name = "Float64";
	this->topic = topic;
	this->data = data;
}

Float64::~Float64() {
}

double Float64::get(){
	return data;
}

void Float64::set(double data){
	this->data = data;
}

bool Float64::canDeserialize(std::string s) {
	size_t pos = s.find(this->typeid_name);
	if(pos != 0)
		return false;
	else
		return true;
}

NetworkData* Float64::deserialize(std::string & s) {
	Float64* c = new Float64();
	(*c)<<s;
	return c;
}

std::string Float64::serialize() {
	std::string s;
	(*this) >> s;
	return s;
}

void Float64::update(void* d) {
	Float64* f = static_cast<Float64*>(d);
	this->data = f->get();
	this->updated = true;
}

}
