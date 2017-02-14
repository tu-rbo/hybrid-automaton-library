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
#ifndef ROS_TOPIC_SENSOR_H
#define ROS_TOPIC_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class ROSTopicSensor;
	typedef boost::shared_ptr<ROSTopicSensor> ROSTopicSensorPtr;
	typedef boost::shared_ptr<const ROSTopicSensor> ROSTopicSensorConstPtr;

    /**
     * @brief An interface to read the value from a ROS topic
     *
    */
	class ROSTopicSensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<ROSTopicSensor> Ptr;
		typedef boost::shared_ptr<const ROSTopicSensor> ConstPtr;

		ROSTopicSensor();

		virtual ~ROSTopicSensor();

		ROSTopicSensor(const ROSTopicSensor& ss);

		ROSTopicSensorPtr clone() const
		{
			return (ROSTopicSensorPtr(_doClone()));
		};

		virtual void initialize(const double& t); 

        /**
         * @brief Return the value of the topic /a _ros_topic_name
         */
		virtual ::Eigen::MatrixXd getCurrentValue() const;
		
		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

        /**
         * @brief Set the topic to listen to
         * @param topic the topic name (leading "/" is important)
         * @param type the datatype of the topic - choose from Bool/Float64/Float64MultiArray/Transform
         */
		virtual void setTopic(const std::string& topic, const std::string& type) {
			_ros_topic_name = topic;
			_ros_topic_type = type;
		}

		virtual bool isActive() const;

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new ROSTopicSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:

		std::string _ros_topic_name;
		std::string _ros_topic_type;

		mutable bool _is_subscribed;

		mutable ::Eigen::MatrixXd _last_pose;

		virtual ROSTopicSensor* _doClone() const
		{
			return (new ROSTopicSensor(*this));
		}

	};

}

#endif
