/*
 * Copyright 2015-2017, Robotics and Biology Lab, TU Berlin
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the 
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
		_last_pose.resize(0,0);
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