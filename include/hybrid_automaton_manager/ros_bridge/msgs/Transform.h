#ifndef __TRANSFORM__
#define __TRANSFORM__

#include <vector>

#include "NetworkData.h"

namespace rlab {

class Transform : public rlab::NetworkData {

protected:
	std::vector<double> position;
	std::vector<double> orientation;
	std::string parent;

public:
	Transform();
	Transform(const std::string& topic);
	Transform(const std::string& topic, const std::vector<double>& position, const std::vector<double>& orientation, const std::string& parent);
	Transform(const std::vector<double>& position, const std::vector<double>& velocity, const std::string& parent);
	virtual ~Transform() {};

	/*
	unsigned int getSize() {return position.size();}
	void setSize(unsigned int size) {position.resize(size); velocity.resize(size); effort.resize(size);}
	*/

	std::vector<double>& getPosition() {return position;}
	std::vector<double>& getOrientation() {return orientation;}
	std::string& getParent() {return parent;}

	void setPosition(const std::vector<double> p) {position = p;}
	void setOrientation(const std::vector<double> o) {orientation = o;}
	void setParent(const std::string& id) {parent = id;}

	//  virtual std::string getType();
	virtual bool canDeserialize(std::string s);
	virtual rlab::NetworkData* deserialize(std::string & s);
	virtual std::string serialize();

	virtual void update(void* d);

	friend std::string & operator >> (Transform &vd, std::string &s) {
		s = vd.typeid_name;

		std::stringstream vStream;

		vStream << vd.topic << " " << vd.parent << " ";
		for (std::vector<double>::iterator it = vd.getPosition().begin(); it != vd.getPosition().end(); ++it) {
			vStream << (*it) << " ";
		}
		for (std::vector<double>::iterator it = vd.getOrientation().begin(); it != vd.getOrientation().end(); ++it) {
			vStream << (*it) << " ";
		}

		s = s + vStream.str().c_str();
		return s;
	}

	friend Transform & operator << (Transform &vd, std::string &s) {
		std::string str = vd.typeid_name;
		s = s.substr(str.size());

		std::stringstream vStream;
		vStream << s;

		vStream >> vd.topic;

		vStream >> vd.parent;
		//std::cout << vd.topic << std::endl;

		for(unsigned int i=0; i<3; i++) {
			vStream >> vd.getPosition()[i];
		}
		for(unsigned int i=0; i<4; i++) {
			vStream >> vd.getOrientation()[i];
		}
		return vd;
	}

	friend std::ostream & operator << (std::ostream &os, Transform &vd) {
		std::string s;
		vd>>s;
		os<<vd;
		return os;
	}
};

}

#endif
