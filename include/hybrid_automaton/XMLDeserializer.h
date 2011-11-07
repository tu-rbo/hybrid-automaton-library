#ifndef XML_DESERIALIZER
#define XML_DESERIALIZER

#include <string>
#include "tinyxml.h"
#include "MotionBehaviour.h"

class XMLDeserializer
{
public:
	XMLDeserializer(TiXmlElement * xml_element_in);

	virtual ~XMLDeserializer();

	bool deserializeBoolean(const char * field_name);

	int deserializeInteger(const char * field_name);

	double deserializeDouble(const char * field_name);

	std::string deserializeString(const char * field_name);

	ViaPointBase * deserializeViaPoint(ControllerType type_of_controller, int controller_dimension);

	std::vector<double> deserializeVectorDouble(const char * field_name);

private:
	TiXmlElement * xml_element;
};

#endif	//XML_DESERIALIZER