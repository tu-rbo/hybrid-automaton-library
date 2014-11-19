#include "hybrid_automaton/FramePoseSensor.h"

namespace ha
{

	FramePoseSensor::FramePoseSensor(const std::string& frame_id)
		: _frame_id(frame_id)
	{
	}


	::Eigen::MatrixXd FramePoseSensor::getCurrentValue() const
	{
		return this->_system->getFramePose(this->_frame_id);
	}

}