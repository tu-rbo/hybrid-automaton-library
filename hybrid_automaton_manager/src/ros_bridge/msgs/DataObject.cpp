/*
 * DataObject.cpp
 *
 *  Created on: May 13, 2011
 *      Author: clemens
 */

#include "DataObject.h"

namespace rlab {

DataObject::DataObject() :
	updated(false)
{
	// TODO Auto-generated constructor stub

}

DataObject::~DataObject() {
	// TODO Auto-generated destructor stub
}

const std::string& DataObject::getType() {
	return this->typeid_name;
}

const std::string& DataObject::getTopic() {
	return this->topic;
}

void DataObject::setTopic(const std::string& s) {
	this->topic = s;
}

bool DataObject::hasType(const std::string& s) {
	return (s.find(typeid_name) != std::string::npos);
}

bool DataObject::hasTopic(const std::string& s) {
	return (s.find(topic) != std::string::npos);
}

bool DataObject::isUpdated() {
	return updated;
}

void DataObject::setIsUpdated(bool b) {
	updated = b;
}

}
