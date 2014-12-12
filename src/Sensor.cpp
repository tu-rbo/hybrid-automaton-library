#include "hybrid_automaton/Sensor.h"

namespace ha{

	Sensor::Sensor()
	{
	}

	Sensor::~Sensor()
	{
	}

		/**
		* Copy constructor
		*/
	Sensor::Sensor(const Sensor& ss)
	{
	}

	const std::string Sensor::getType() const
	{
		return this->_type;
	}

	void Sensor::setType(const std::string& new_type)
	{
		this->_type = new_type;
	}

	void Sensor::setSystem(const System::ConstPtr& system)
	{
		this->_system = system;
	}

	void Sensor::initialize(const double& t) 
	{
		this->_initial_sensor_value = this->getCurrentValue();
	}

	void Sensor::terminate() 
	{
	}

	void Sensor::step(const double& t) 
	{
	}

	::Eigen::MatrixXd Sensor::getInitialValue() const
	{
		return this->_initial_sensor_value;
	}

	::Eigen::MatrixXd Sensor::getRelativeCurrentValue() const
	{
		return (this->getCurrentValue() - this->_initial_sensor_value);
	}

}