#ifndef FRAME_POSE_SENSOR_H
#define FRAME_POSE_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class FramePoseSensor;
	typedef boost::shared_ptr<FramePoseSensor> FramePoseSensorPtr;
	typedef boost::shared_ptr<const FramePoseSensor> FramePoseSensorConstPtr;

	class FramePoseSensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<FramePoseSensor> Ptr;
		typedef boost::shared_ptr<const FramePoseSensor> ConstPtr;

		FramePoseSensor(const std::string& frame_id = std::string("EE"));

		virtual ~FramePoseSensor();

		FramePoseSensor(const FramePoseSensor& ss);

		FramePoseSensorPtr clone() const
		{
			return (FramePoseSensorPtr(_doClone()));
		};

		virtual ::Eigen::MatrixXd getCurrentValue() const;
		
		virtual ::Eigen::MatrixXd getRelativeCurrentValue() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new FramePoseSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:

		std::string _frame_id;

		virtual FramePoseSensor* _doClone() const
		{
			return (new FramePoseSensor(*this));
		}

	};

}

#endif
