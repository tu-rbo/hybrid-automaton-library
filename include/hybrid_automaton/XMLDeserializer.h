#ifndef XML_DESERIALIZER
#define XML_DESERIALIZER

#include <string>
#include "tinyxml.h"

#include "HybridAutomaton.h"
#include "MotionBehaviour.h"

class XMLDeserializer
{
public:
	XMLDeserializer(TiXmlElement * xml_element_in);
	virtual ~XMLDeserializer();
	
	static HybridAutomaton* createHybridAutomaton(const std::string& xml_string, rxSystem* robot, double dT);
	
	bool deserializeBoolean(const char * field_name);
	bool deserializeBoolean(const char * field_name, bool default_value);

	int deserializeInteger(const char * field_name);
	int deserializeInteger(const char * field_name, int default_value);

	double deserializeDouble(const char * field_name);
	double deserializeDouble(const char * field_name, double default_value);

	std::string deserializeString(const char * field_name);

	ViaPointBase * deserializeViaPoint(ControllerType type_of_controller, int controller_dimension);

	std::vector<double> deserializeVectorDouble(const char * field_name);

private:
	TiXmlElement * xml_element;
};

#endif	//XML_DESERIALIZER