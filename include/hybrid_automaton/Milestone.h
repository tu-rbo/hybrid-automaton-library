/*
* Milestone.h
*
* TODO should/CAN class be abstract? if yes, which member(s) should be pure virtual?
*
* @see Graph
*
*/

#ifndef MILESTONE_
#define MILESTONE_

#include "node.h"
#include <iostream>
#include "tinyxml.h"
#include "rxControlSDK\rxControlSDK.h"

class Milestone : public Node 
{

public:

	enum Status {
		VALID, INVALID, TASK_CONSISTENT, STATUS_NUM
	};

	Milestone();

	virtual ~Milestone();

	Milestone::Status getStatus() const;

	void setStatus(Milestone::Status status);

	virtual void update();

	virtual std::string toStringXML() const;

	virtual void toElementXML(TiXmlElement* root) const;

	virtual Milestone* clone() const;

	virtual bool operator ==(const Milestone & n) const;

	virtual bool operator !=(const Milestone & n) const;

	virtual dVector getConfiguration() const;

private:

	Status status_;

};

#endif // MILESTONE_

//bool operator== (const Node& n) const;
//bool operator!= (const Node& n) const;
