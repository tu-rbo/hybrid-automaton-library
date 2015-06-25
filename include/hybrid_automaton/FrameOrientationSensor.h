#ifndef FRAME_ORIENTATION_SENSOR_H
#define FRAME_ORIENTATION_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class FrameOrientationSensor;
	typedef boost::shared_ptr<FrameOrientationSensor> FrameOrientationSensorPtr;
	typedef boost::shared_ptr<const FrameOrientationSensor> FrameOrientationSensorConstPtr;

	class FrameOrientationSensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<FrameOrientationSensor> Ptr;
		typedef boost::shared_ptr<const FrameOrientationSensor> ConstPtr;

        /**
         * @brief An interface to measure the orienation of the given robot frame using forward kinematics
         */
		FrameOrientationSensor(const std::string& frame_id = std::string("EE"));

		virtual ~FrameOrientationSensor();

		FrameOrientationSensor(const FrameOrientationSensor& ss);

		FrameOrientationSensorPtr clone() const
		{
			return (FrameOrientationSensorPtr(_doClone()));
		};

		virtual void initialize(const double& t); 

		virtual ::Eigen::MatrixXd getInitialValue() const;

        /**
         * @brief Returns the current frame osition as a 3x3 mrotation matrix
         */
		virtual ::Eigen::MatrixXd getCurrentValue() const;
		
		virtual ::Eigen::MatrixXd getRelativeCurrentValue() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new FrameOrientationSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:

		std::string _frame_id;

		virtual FrameOrientationSensor* _doClone() const
		{
			return (new FrameOrientationSensor(*this));
		}

	};

}

#endif
