/*
 *\brief Float64 class
 *\detailed The Float64 class is an example for objects that can be sent
 *          over the network (to be deleted in the future)
 *$Author: clemens, dubik $
 *$Date: 2007/12/30 17:54:54 $
 *$Revision: 1.5 $
 */

#ifndef __FLOAT64__
#define __FLOAT64__

#include "ros_bridge\msgs\NetworkData.h"
#include <ostream>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>

namespace rlab {

class Float64 : public rlab::NetworkData {

protected:
	double data;

public:
	Float64();
	Float64(const std::string& topic);
	Float64(const std::string& topic, double data);
	virtual ~Float64();

	double get();
	void set(double data);

	friend Float64& operator << (Float64& c, std::string& s){
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

		double x;
		vStream >> x;
		c.set(x);

		return c;
	}

	friend std::string& operator >> (Float64& c, std::string& s){
		s = c.getType();
		std::ostringstream o1;
		o1 << c.getTopic() << " " << c.get() << " ";
		s = s + o1.str().c_str();
		return s;
	}

	friend std::ostream& operator << (std::ostream& os, Float64& c){
		std::string s;
		c>>s;
		os<<s;
		return os;
	}

	//  virtual std::string getType();
	virtual bool canDeserialize(std::string s);
	virtual rlab::NetworkData* deserialize(std::string & s);
	std::string serialize();

	virtual void update(void* d);
};

}

#endif
