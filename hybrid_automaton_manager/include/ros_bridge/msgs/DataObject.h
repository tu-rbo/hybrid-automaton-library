/*
 * DataObject.h
 *
 *  Created on: May 13, 2011
 *      Author: clemens
 */

#ifndef DATAOBJECT_H_
#define DATAOBJECT_H_

#include <string>

namespace rlab {

class DataObject {
public:
	DataObject();
	virtual ~DataObject();

	const std::string& getType();
	const std::string& getTopic();

	bool hasType(const std::string& s);
	bool hasTopic(const std::string& s);

	void setTopic(const std::string& s);

	virtual void update(void* s)=0;
	bool isUpdated();
	void setIsUpdated(bool b);

protected:
	std::string typeid_name;
	std::string topic;

	bool updated;
};

}

#endif /* DATAOBJECT_H_ */
