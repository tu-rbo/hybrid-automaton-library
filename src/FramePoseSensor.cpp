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


	void FramePoseSensor::initialize(const double& t) 
	{
		this->_initial_sensor_value = this->_system->getFramePose(this->_frame_id);
	}

	::Eigen::MatrixXd FramePoseSensor::getCurrentValue() const
	{
		return this->_system->getFramePose(this->_frame_id);
	}

	::Eigen::MatrixXd FramePoseSensor::getRelativeCurrentValue() const
	{
		::Eigen::MatrixXd pose = this->_system->getFramePose(this->_frame_id);
		pose = _initial_sensor_value.inverse()*pose;
		return pose;
	}

	::Eigen::MatrixXd FramePoseSensor::getInitialValue() const
	{
		return this->_initial_sensor_value;
	}

	DescriptionTreeNode::Ptr FramePoseSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());
		tree->setAttribute<std::string>(std::string("frame_id"), _frame_id);

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

		if(!tree->getAttribute<std::string>("frame_id", _frame_id))
		{
			HA_WARN("FramePoseSensor::deserialize", "frame_id not defined. using default value EE");
			_frame_id = "EE";
		}

		
		_system = system;
	}
}