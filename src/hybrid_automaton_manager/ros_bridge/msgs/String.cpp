#include "msgs\String.h"


namespace rlab {

String::String() {
	this->typeid_name = "String";
	this->topic = "";
}

String::String(const std::string& topic) {
	this->typeid_name = "String";
	this->topic = topic;
}

String::String(const std::string& topic, const std::string& data) {
	this->typeid_name = "String";
	this->topic = topic;
	this->data = data;
}

String::~String() {
}

std::string String::get() {
	return this->data;
}

void String::set(const std::string& data) {
	this->data = data;
}

bool String::canDeserialize(std::string s) {
	size_t pos = s.find(this->typeid_name);
	if(pos != 0)
		return false;
	else
		return true;
}

NetworkData* String::deserialize(std::string & s) {
	String* c = new String();
	(*c)<<s;
	return c;
}

std::string String::serialize() {
	std::string s;
	(*this) >> s;
	return s;
}

void String::update(void* d) {
	String* f = static_cast<String*>(d);
	this->data = f->get();
	this->updated = true;
}

}
