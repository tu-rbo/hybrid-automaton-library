#include "hybrid_automaton/JointConfigurationSensor.h"

namespace ha
{

	::Eigen::MatrixXd JointConfigurationSensor::getCurrentValue() const
	{
		return this->_system->getConfiguration();
	}

}