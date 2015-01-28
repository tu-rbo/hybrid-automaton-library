#include "hybrid_automaton/FramePoseSensor.h"

namespace ha
{

	HA_SENSOR_REGISTER("FramePoseSensor", FramePoseSensor);

	FramePoseSensor::FramePoseSensor(const std::string& frame_id)
		: _frame_id(frame_id)
	{
	}

	FramePoseSensor::~FramePoseSensor()
	{
	}

	FramePoseSensor::FramePoseSensor(const FramePoseSensor& ss)
		:Sensor(ss)
	{
	}

	::Eigen::MatrixXd FramePoseSensor::getCurrentValue() const
	{
		return this->_system->getFramePose(this->_frame_id);
	}

	::Eigen::MatrixXd FramePoseSensor::getRelativeCurrentValue() const
	{
		//TODO!
		HA_THROW_ERROR("Sensor::getRelativeCurrentValue()", "Not Implemented for FramePoseSensor");
	}

	DescriptionTreeNode::Ptr FramePoseSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

		return tree;
	}

	void FramePoseSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("FramePoseSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("FramePoseSensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}
		
		_system = system;
	}
}