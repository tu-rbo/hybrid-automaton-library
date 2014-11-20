#include "hybrid_automaton/ForceTorqueSensor.h"

namespace ha
{
	HA_SENSOR_REGISTER("ForceTorqueSensor", ForceTorqueSensor);

	::Eigen::MatrixXd ForceTorqueSensor::getCurrentValue() const
	{
		return this->_system->getForceTorqueMeasurement();
	}

}