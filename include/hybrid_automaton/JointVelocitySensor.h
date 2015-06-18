#ifndef JOINT_VELOCITY_SENSOR_H
#define JOINT_VELOCITY_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class JointVelocitySensor;
	typedef boost::shared_ptr<JointVelocitySensor> JointVelocitySensorPtr;
	typedef boost::shared_ptr<const JointVelocitySensor> JointVelocitySensorConstPtr;

    /**
     * @brief An interface to the current joint velocity (usually estimated from encoders)
     */
	class JointVelocitySensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<JointVelocitySensor> Ptr;
		typedef boost::shared_ptr<const JointVelocitySensor> ConstPtr;

		JointVelocitySensor();

		virtual ~JointVelocitySensor();

		JointVelocitySensor(const JointVelocitySensor& ss);

		JointVelocitySensorPtr clone() const
		{
			return (JointVelocitySensorPtr(_doClone()));
		};

        /**
         * @brief Return the current joint velocity as a dimx1-vector
         */
		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new JointVelocitySensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:

		virtual JointVelocitySensor* _doClone() const
		{
			return (new JointVelocitySensor(*this));
		}

	};

}

#endif
