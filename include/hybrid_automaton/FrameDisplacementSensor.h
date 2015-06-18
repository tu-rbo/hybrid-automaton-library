#ifndef FRAME_DISPLACEMENT_SENSOR_H
#define FRAME_DISPLACEMENT_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class FrameDisplacementSensor;
	typedef boost::shared_ptr<FrameDisplacementSensor> FrameDisplacementSensorPtr;
	typedef boost::shared_ptr<const FrameDisplacementSensor> FrameDisplacementSensorConstPtr;

    /**
     * @brief An interface to measure the position of the given frame usinf forward kinematics
     */
	class FrameDisplacementSensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<FrameDisplacementSensor> Ptr;
		typedef boost::shared_ptr<const FrameDisplacementSensor> ConstPtr;

		FrameDisplacementSensor(const std::string& frame_id = std::string("EE"));

		virtual ~FrameDisplacementSensor();

		FrameDisplacementSensor(const FrameDisplacementSensor& ss);

		FrameDisplacementSensorPtr clone() const
		{
			return (FrameDisplacementSensorPtr(_doClone()));
		};

		virtual void initialize(const double& t); 
		
		virtual ::Eigen::MatrixXd getRelativeCurrentValue() const;

        /**
         * @brief Returns the current frame position as a 3x1 vector
         */
		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual ::Eigen::MatrixXd getInitialValue() const;
		
		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new FrameDisplacementSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:

		std::string _frame_id;

		virtual FrameDisplacementSensor* _doClone() const
		{
			return (new FrameDisplacementSensor(*this));
		}

	};

}

#endif
