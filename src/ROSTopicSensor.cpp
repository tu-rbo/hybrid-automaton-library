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
				HA_WARN("ROSTopicSensor.initialize", "Unable to connect to topic " << _ros_topic_name);
			}else{
				HA_INFO("ROSTopicSensor.initialize", "Successfully subscribed to topic " << _ros_topic_name << "");
				_is_subscribed = true;
			}
		}

		// needs to be executed after connecting to topic because
		// it queries getCurrentValue
		Sensor::initialize(t);
	}

	bool ROSTopicSensor::isActive() const {
		// If the subscription failed in the initialization we try again
		if(!this->_is_subscribed)
		{
			this->_is_subscribed = _system->subscribeToROSMessage(_ros_topic_name);
			if (!this->_is_subscribed) {
				HA_WARN("ROSTopicSensor.isActive", "Unable to connect to topic " << _ros_topic_name);
			}
			// In the same tick is not possible to subscribe to a topic and that this topic is available
			return false;
		}

		bool available = _system->isROSTopicAvailable(this->_ros_topic_name); 
		if (available && _last_pose.rows() == 0) {
			getCurrentValue();
		}
		return (available && _last_pose.rows() != 0);
	}

	::Eigen::MatrixXd ROSTopicSensor::getCurrentValue() const
	{

		if (this->_system->isROSTopicUpdated(this->_ros_topic_name)) {
			::Eigen::MatrixXd pose;
			if (!this->_system->getROSPose(this->_ros_topic_name, this->_ros_topic_type, pose)) {
				HA_WARN("ROSTopicSensor.getCurrentValue", "Unable to get Pose from topic " << _ros_topic_name << "! Is the topic still being published?");
			}
			if (_last_pose.rows() == 0)
				_initial_sensor_value = pose;

			_last_pose = pose;
		}
		return _last_pose;
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