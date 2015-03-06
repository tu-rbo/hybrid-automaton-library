#ifndef ROS_TOPIC_SENSOR_H
#define ROS_TOPIC_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class ROSTopicSensor;
	typedef boost::shared_ptr<ROSTopicSensor> ROSTopicSensorPtr;
	typedef boost::shared_ptr<const ROSTopicSensor> ROSTopicSensorConstPtr;

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

		virtual ::Eigen::MatrixXd getCurrentValue() const;
		
		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

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

		int _update_rate;

		bool _is_subscribed;

		virtual ROSTopicSensor* _doClone() const
		{
			return (new ROSTopicSensor(*this));
		}

	};

}

#endif
