#ifndef __JOINTSTATE__
#define __JOINTSTATE__

#include <vector>

#include "NetworkData.h"

namespace rlab {

class JointState : public rlab::NetworkData {

protected:
	std::vector<double> position;
	std::vector<double> velocity;
	std::vector<double> effort;

public:
	JointState();
	JointState(const std::string& topic);
	JointState(const std::string& topic, const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort);
	JointState(const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort);
	virtual ~JointState() {};

	unsigned int getSize() {return position.size();}
	void setSize(unsigned int size) {position.resize(size); velocity.resize(size); effort.resize(size);}

	std::vector<double>& getPosition() {return position;}
	std::vector<double>& getVelocity() {return velocity;}
	std::vector<double>& getEffort() {return effort;}

	void setPosition(const std::vector<double> p) {position = p;}
	void setVelocity(const std::vector<double> v) {velocity = v;}
	void setEffort(const std::vector<double> e) {effort = e;}

	//  virtual std::string getType();
	virtual bool canDeserialize(std::string s);
	virtual rlab::NetworkData* deserialize(std::string & s);
	virtual std::string serialize();

	virtual void update(void* d);

	friend std::string & operator >> (JointState &vd, std::string &s) {
		s = vd.typeid_name;

		std::stringstream vStream;

		vStream << vd.topic << " " << vd.getSize() << " ";
		for (std::vector<double>::iterator it = vd.getPosition().begin(); it != vd.getPosition().end(); ++it) {
			vStream << (*it) << " ";
		}
		for (std::vector<double>::iterator it = vd.getVelocity().begin(); it != vd.getVelocity().end(); ++it) {
			vStream << (*it) << " ";
		}
		for (std::vector<double>::iterator it = vd.getEffort().begin(); it != vd.getEffort().end(); ++it) {
			vStream << (*it) << " ";
		}

		s = s + vStream.str().c_str();
		return s;
	}

	friend JointState & operator << (JointState &vd, std::string &s) {
		std::string str = vd.typeid_name;
		s = s.substr(str.size());

		std::stringstream vStream;
		vStream << s;

		vStream >> vd.topic;

		//std::cout << vd.topic << std::endl;

		unsigned int size;
		vStream >> size;
		vd.setSize(size);

		for(unsigned int i=0; i<size; i++) {
			vStream >> vd.getPosition()[i];
		}
		for(unsigned int i=0; i<size; i++) {
			vStream >> vd.getVelocity()[i];
		}
		for(unsigned int i=0; i<size; i++) {
			vStream >> vd.getEffort()[i];
		}
		return vd;
	}

	friend std::ostream & operator << (std::ostream &os, JointState &vd) {
		std::string s;
		vd>>s;
		os<<vd;
		return os;
	}
};

}

#endif
