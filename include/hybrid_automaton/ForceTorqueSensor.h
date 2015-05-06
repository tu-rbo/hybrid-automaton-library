#ifndef FORCE_TORQUE_SENSOR_H
#define FORCE_TORQUE_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class ForceTorqueSensor;
	typedef boost::shared_ptr<ForceTorqueSensor> ForceTorqueSensorPtr;
	typedef boost::shared_ptr<const ForceTorqueSensor> ForceTorqueSensorConstPtr;

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

		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new ForceTorqueSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
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
