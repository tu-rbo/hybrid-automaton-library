#ifndef __FLOAT64MULTIARRAY__
#define __FLOAT64MULTIARRAY__

#include <vector>

#include "NetworkData.h"

namespace rlab {

class Float64MultiArray : public rlab::NetworkData {

protected:
	std::vector<double> data;

public:
	Float64MultiArray();
	Float64MultiArray(const std::string& topic);
	Float64MultiArray(const std::string& topic, const std::vector<double>& data);
	Float64MultiArray(const std::vector<double>& data);
	virtual ~Float64MultiArray() {};

	unsigned int getSize() {return data.size();}
	void setSize(unsigned int size) {data.resize(size);}

	std::vector<double>& get() {return data;}
	void set(const std::vector<double> d) {data = d;}

	//  virtual std::string getType();
	virtual bool canDeserialize(std::string s);
	virtual rlab::NetworkData* deserialize(std::string & s);
	virtual std::string serialize();

	virtual void update(void* d);

	friend std::string & operator >> (Float64MultiArray &vd, std::string &s) {
		s=vd.typeid_name;

		std::stringstream vStream;

		vStream << vd.topic << " " << vd.getSize() << " ";
		std::vector<double> data = vd.get();
		std::vector<double>::iterator it = data.begin();
		for(; it != data.end(); ++it) {
			vStream << (*it) << " ";
		}

		s=s + vStream.str().c_str();
		return s;
	}

	friend Float64MultiArray & operator << (Float64MultiArray &vd, std::string &s) {
		std::string str=vd.typeid_name;
		s = s.substr(str.size());

		std::stringstream vStream;
		vStream << s;

		vStream >> vd.topic;

		//std::cout << vd.topic << std::endl;

		unsigned int size;
		vStream >> size;
		vd.setSize(size);

		std::vector<double> data(size);
		for(unsigned int i=0; i<size; i++) {
			vStream >> data[i];
		}
		vd.set(data);
		return vd;
	}

	friend std::ostream & operator << (std::ostream &os, Float64MultiArray &vd) {
		std::string s;
		vd>>s;
		os<<vd;
		return os;
	}
};

}

#endif
