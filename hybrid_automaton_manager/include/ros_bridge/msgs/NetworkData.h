/*
 *\brief Subscriber class
 *\detailed The subscriber class is an interface for objects that can subscribe with
 *          a network object and check whether they can parse received objects
 *$Author: dubik $
 *$Date: 2007/12/30 17:54:54 $
 *$Revision: 1.7 $
 */

#ifndef __NETWORKDATA__
#define __NETWORKDATA__

#include <string>
#include <ostream>
#include <iostream>
#include <sstream>

#include "ros_bridge\msgs\DataObject.h"

template < typename T >
T stream_cast(const std::string & r_s){
	std::stringstream ss;
	ss << r_s;
	T result;
	ss >> result;
	return result;
}

namespace rlab {

class NetworkData : public virtual rlab::DataObject {

public:
	virtual ~NetworkData(){}

	virtual bool canDeserialize(std::string s) = 0;
	virtual rlab::NetworkData* deserialize(std::string & s) = 0;
	virtual std::string serialize()=0;// {(*this)>>s;}
};

}

#endif
