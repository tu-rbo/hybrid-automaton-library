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
