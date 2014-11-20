#include "hybrid_automaton/Sensor.h"

namespace ha{

	const std::string Sensor::getType() const
	{
		return this->_type;
	}

	void Sensor::setType(const std::string& new_type)
	{
		this->_type = new_type;
	}
}