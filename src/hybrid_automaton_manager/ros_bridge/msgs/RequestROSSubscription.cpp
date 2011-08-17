#include "msgs\RequestROSSubscription.h"


namespace rlab {

RequestROSSubscription::RequestROSSubscription() {
	this->typeid_name = "RequestROSSubscription";
	this->topic = "";
}

RequestROSSubscription::RequestROSSubscription(const std::string& topic) {
	this->typeid_name = "RequestROSSubscription";
	this->topic = topic;
}

RequestROSSubscription::~RequestROSSubscription() {
}

bool RequestROSSubscription::canDeserialize(std::string s) {
	size_t pos = s.find(this->typeid_name);
	if(pos != 0)
		return false;
	else
		return true;
}

NetworkData* RequestROSSubscription::deserialize(std::string & s) {
	RequestROSSubscription* c = new RequestROSSubscription();
	(*c)<<s;
	return c;
}

std::string RequestROSSubscription::serialize() {
	std::string s;
	(*this) >> s;
	return s;
}

void RequestROSSubscription::update(void* d) {
}

}
