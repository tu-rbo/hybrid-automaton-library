#include "msgs\NetworkData.h"

namespace rlab {

bool NetworkData::canDeserialize(std::string s){
	//std::string str=typeid(*this).name();
	//unsigned int pos = s.find(str);
	//if(pos != 0) return false;
	//else return true;

	size_t pos = s.find(this->typeid_name);
	if (pos != std::string::npos)
		return true;
	else
		return false;
}
/*
std::string Subscriber::serialize() {
  std::string s;
  (*this) >> s;
  return s;
}
 */
}
