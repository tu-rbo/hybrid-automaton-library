#include "hybrid_automaton/FrameDisplacementSensor.h"

namespace ha
{

	HA_SENSOR_REGISTER("FrameDisplacementSensor", FrameDisplacementSensor);

	FrameDisplacementSensor::FrameDisplacementSensor(const std::string& frame_id)
		: _frame_id(frame_id)
	{
	}

	FrameDisplacementSensor::~FrameDisplacementSensor()
	{
	}

	FrameDisplacementSensor::FrameDisplacementSensor(const FrameDisplacementSensor& ss)
		:Sensor(ss)
	{
	}

	void FrameDisplacementSensor::initialize(const double& t) 
	{
		this->_initial_sensor_value = this->_system->getFramePose(this->_frame_id);
	}

	::Eigen::MatrixXd FrameDisplacementSensor::getCurrentValue() const
	{
		::Eigen::MatrixXd pose = this->_system->getFramePose(this->_frame_id);
		pose = pose.col(3);
		pose.conservativeResize(3,1);
		return pose;
	}

	::Eigen::MatrixXd FrameDisplacementSensor::getInitialValue() const
	{
		::Eigen::MatrixXd ret = this->_initial_sensor_value.col(3);
		ret.conservativeResize(3,1);
		return ret;
	}

	::Eigen::MatrixXd FrameDisplacementSensor::getRelativeCurrentValue() const
	{
		
		::Eigen::MatrixXd pose = this->_system->getFramePose(this->_frame_id);
		pose=_initial_sensor_value.inverse()*pose;
		pose = pose.col(3);
		pose.conservativeResize(3,1);
		
		return pose;
	}

	DescriptionTreeNode::Ptr FrameDisplacementSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

		return tree;
	}

	void FrameDisplacementSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("FrameDisplacementSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("FrameDisplacementSensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		_system = system;
	}
}