#include "Float64MultiArray.h"

namespace rlab {

Float64MultiArray::Float64MultiArray() {
	this->typeid_name = "Float64MultiArray";
}

Float64MultiArray::Float64MultiArray(const std::string& topic) {
	this->typeid_name = "Float64MultiArray";
	this->topic = topic;
}

Float64MultiArray::Float64MultiArray(const std::string& topic, const std::vector<double>& data) {
	this->typeid_name = "Float64MultiArray";
	this->topic = topic;
	this->data = data;
}

Float64MultiArray::Float64MultiArray(const std::vector<double>& data) {
	this->typeid_name = "Float64MultiArray";
	this->data = data;
}

//std::string Float64MultiArray::getType(){
//  return typeid(this).name();
//}

bool Float64MultiArray::canDeserialize(std::string s){
	std::string str=this->typeid_name;
	std::size_t pos = s.find(str);
	if(pos != 0) return false;
	else return true;
}

rlab::NetworkData* Float64MultiArray::deserialize(std::string & s) {
	//std::string str=typeid(*this).name(); // move to operator <<
	//std::string vString = s.substr(str.size());

	Float64MultiArray* vd = new Float64MultiArray();
	(*vd) << s; //vString;
	return vd;
}

std::string Float64MultiArray::serialize() {

	std::string s;
	(*this) >> s;
	return s;
}

void Float64MultiArray::update(void* d) {
	Float64MultiArray* f = static_cast<Float64MultiArray*>(d);
	this->data = f->get();
	this->updated = true;
}

}
