#include "hybrid_automaton/ClockSensor.h"

namespace ha
{

	HA_SENSOR_REGISTER("ClockSensor", ClockSensor);

	::Eigen::MatrixXd ClockSensor::getCurrentValue() const
	{
		return this->_system->getCurrentTime();
	}

}