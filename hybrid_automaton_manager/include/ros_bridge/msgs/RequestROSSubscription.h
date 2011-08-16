/*
 *\brief RequestROSSubscription class
 *\detailed The RequestROSSubscription class is an example for objects that can be sent
 *          over the network (to be deleted in the future)
 *$Author: clemens $
 *$Date: 2011/06/27 $
 *$Revision: 1.5 $
 */

#ifndef __REQUESTROSSUBSCRIPTION__
#define __REQUESTROSSUBSCRIPTION__

#include "ros_bridge\msgs\NetworkData.h"
#include <ostream>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>

namespace rlab {

class RequestROSSubscription : public rlab::NetworkData {

public:
	RequestROSSubscription();
	RequestROSSubscription(const std::string& topic);
	virtual ~RequestROSSubscription();

	friend RequestROSSubscription& operator << (RequestROSSubscription& c, std::string& s){
		std::string type = c.getType();
		unsigned int pos = s.find(type);
		if(pos == 0){
			s.erase(0,type.size());
		}

		std::stringstream vStream;
		vStream << s;

		std::string topic;
		vStream >> topic;
		c.setTopic(topic);

		return c;
	}

	friend std::string& operator >> (RequestROSSubscription& c, std::string& s){
		s = c.getType();
		std::ostringstream o1;
		o1 << c.getTopic() << " ";
		s = s + o1.str().c_str();
		return s;
	}

	friend std::ostream& operator << (std::ostream& os, RequestROSSubscription& c){
		std::string s;
		c>>s;
		os<<s;
		return os;
	}

	virtual bool canDeserialize(std::string s);
	virtual rlab::NetworkData* deserialize(std::string & s);
	std::string serialize();

	virtual void update(void* d);
};

}

#endif
