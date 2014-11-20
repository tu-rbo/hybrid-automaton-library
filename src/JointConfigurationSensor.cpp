#include "hybrid_automaton/JointConfigurationSensor.h"

namespace ha
{

	HA_SENSOR_REGISTER("JointConfigurationSensor", JointConfigurationSensor);

	::Eigen::MatrixXd JointConfigurationSensor::getCurrentValue() const
	{
		return this->_system->getConfiguration();
	}

}