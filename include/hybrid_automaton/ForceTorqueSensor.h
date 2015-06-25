#ifndef FORCE_TORQUE_SENSOR_H
#define FORCE_TORQUE_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class ForceTorqueSensor;
	typedef boost::shared_ptr<ForceTorqueSensor> ForceTorqueSensorPtr;
	typedef boost::shared_ptr<const ForceTorqueSensor> ForceTorqueSensorConstPtr;

    /**
     * @brief An interface to a six-axis force-torque sensor
     */
	class ForceTorqueSensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<ForceTorqueSensor> Ptr;
		typedef boost::shared_ptr<const ForceTorqueSensor> ConstPtr;

		ForceTorqueSensor();

		virtual ~ForceTorqueSensor();

		ForceTorqueSensor(const ForceTorqueSensor& ss);

		ForceTorqueSensorPtr clone() const
		{
			return (ForceTorqueSensorPtr(_doClone()));
		};

        /**
         * @brief Returns the current force-torque sensor reading as a 6x1 vector.
         *
         * Optionally transforms the force into frame _frame_id - PLEASE TEST THIS BEFORE USING!
         *
         * x ,y ,z ,rot_x, rot_y, rot_z
         */
		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new ForceTorqueSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

		virtual void setFrameId(const std::string& frame_id) {
			_frame_id = frame_id;
		}

		virtual std::string getFrameId() const {
			return _frame_id;
		}

	protected:

        /**
         * @brief Optional frame ID to transform the force into
         */
        std::string _frame_id;

		virtual ForceTorqueSensor* _doClone() const
		{
			return (new ForceTorqueSensor(*this));
		}

	};

}

#endif
