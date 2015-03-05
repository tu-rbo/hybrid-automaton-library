#include "hybrid_automaton/ROSTopicSensor.h"

namespace ha
{

	HA_SENSOR_REGISTER("ROSTopicSensor", ROSTopicSensor);

	ROSTopicSensor::ROSTopicSensor() :Sensor() {}

	ROSTopicSensor::~ROSTopicSensor() {}

	ROSTopicSensor::ROSTopicSensor(const ROSTopicSensor& ss)
		:Sensor(ss)
	{
	}

	::Eigen::MatrixXd ROSTopicSensor::getCurrentValue() const
	{
		::Eigen::MatrixXd pose;
		this->_system->getROSPose(this->_ros_topic_name, this->_ros_topic_type, pose);
		return pose;
	}

	DescriptionTreeNode::Ptr ROSTopicSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

		tree->setAttribute<std::string>(std::string("ros_topic_name"), this->_ros_topic_name);
		tree->setAttribute<std::string>(std::string("ros_topic_type"), this->_ros_topic_type);

		return tree;
	}

	void ROSTopicSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("ROSTopicSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("ROSTopicSensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		if(!tree->getAttribute<std::string>("ros_topic_name", _ros_topic_name))
		{
			HA_THROW_ERROR("ROSTopicSensordeserialize", "topic name not defined");
		}
		if(!tree->getAttribute<std::string>("ros_topic_type", _ros_topic_type))
		{
			HA_THROW_ERROR("ROSTopicSensor.deserialize", "topic type not defined");
		}
		
		_system = system;
	}
}