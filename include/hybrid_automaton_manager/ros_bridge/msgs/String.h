/*
 *\brief String class
 *\detailed The String class is an example for objects that can be sent
 *          over the network (to be deleted in the future)
 *$Author: clemens $
 *$Date: 2011/06/27 $
 *$Revision: 1.0 $
 */

#ifndef __STRING__
#define __STRING__

#include <ostream>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>

#include "NetworkData.h"

//#ifdef HAVE_STDLIB_H
#include <stdlib.h>
//#endif

namespace rlab {

class String : public rlab::NetworkData {

protected:
	std::string data;

public:
	String();
	String(const std::string& topic);
	String(const std::string& topic, const std::string& data);
	virtual ~String();

	std::string get();
	void set(const std::string& data);

	friend String& operator << (String& c, std::string& s) {
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

		c.set(vStream.str().substr(topic.size() + 1));

		return c;
	}

	friend std::string& operator >> (String& c, std::string& s){
		s = c.getType();

		std::ostringstream o1;
		o1 << c.getTopic() << " " << c.get();
		s = s + o1.str().c_str();

		return s;
	}

	friend std::ostream& operator << (std::ostream& os, String& c){
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
