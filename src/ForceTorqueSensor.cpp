#include "hybrid_automaton/ForceTorqueSensor.h"

namespace ha
{

	::Eigen::MatrixXd ForceTorqueSensor::getCurrentValue() const
	{
		return this->_system->getForceTorqueMeasurement();
	}

}