#include "hybrid_automaton/ROSTopicSensor.h"

namespace ha
{

	HA_SENSOR_REGISTER("ROSTopicSensor", ROSTopicSensor);

	ROSTopicSensor::ROSTopicSensor() :Sensor(), _is_subscribed(false) {}

	ROSTopicSensor::~ROSTopicSensor() {}

	ROSTopicSensor::ROSTopicSensor(const ROSTopicSensor& ss)
		:Sensor(ss)
	{
	}

	void ROSTopicSensor::initialize(const double& t) {
		// connect to topic
		if (!_is_subscribed) {
			if (!_system->subscribeToROSMessage(_ros_topic_name)) {
				HA_THROW_ERROR("ROSTopicSensor.initialize", "Unable to connect to topic " << _ros_topic_name << "! Aborting initializion");
			}
			HA_INFO("ROSTopicSensor.initialize", "Successfully subscribed to topic " << _ros_topic_name << "");
			_is_subscribed = true;
		}

		// needs to be executed after connecting to topic because
		// it queries getCurrentValue
		Sensor::initialize(t);
	}

	bool ROSTopicSensor::isActive() const {
		return _system->isROSTopicAvailable(this->_ros_topic_name);
	}

	::Eigen::MatrixXd ROSTopicSensor::getCurrentValue() const
	{
		static int counter = 1;// wait update_rate-1 ticks before considering querying blackboard at all

		::Eigen::MatrixXd pose;
		if ( (_update_rate < 0 && counter == 0) || (counter++ % _update_rate == 0) ) {
			if (!this->_system->getROSPose(this->_ros_topic_name, this->_ros_topic_type, pose)) {
				HA_WARN("ROSTopicSensor.getCurrentValue", "Unable to get Pose from topic " << _ros_topic_name << "! Is the topic still being published?");
			}
		}
		return pose;
	}

	DescriptionTreeNode::Ptr ROSTopicSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

		tree->setAttribute<std::string>(std::string("ros_topic_name"), this->_ros_topic_name);
		tree->setAttribute<std::string>(std::string("ros_topic_type"), this->_ros_topic_type);
		tree->setAttribute<int>(std::string("update_rate"), this->_update_rate);

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

		if(!tree->getAttribute<int>("update_rate", _update_rate))
		{
			_update_rate = 10;
			HA_WARN("ROSTopicSensor.deserialize", "update_rate not defined. setting to default update_rate=" << _update_rate);
		}
		if (_update_rate <= 0) {
			HA_THROW_ERROR("ROSTopicSensor.deserialize", "update_rate must be > 0!");
		}
		
		_system = system;
	}
}