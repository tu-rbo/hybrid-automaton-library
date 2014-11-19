#include "hybrid_automaton/ClockSensor.h"

namespace ha
{

	::Eigen::MatrixXd ClockSensor::getCurrentValue() const
	{
		return this->_system->getCurrentTime();
	}

}