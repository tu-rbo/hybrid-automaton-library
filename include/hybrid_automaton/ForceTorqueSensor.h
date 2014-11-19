#ifndef FORCE_TORQUE_SENSOR_H
#define FORCE_TORQUE_SENSOR_H

#include "hybrid_automaton/Sensor.h"

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

	protected:

		virtual ForceTorqueSensor* _doClone() const
		{
			return (new ForceTorqueSensor(*this));
		}

	};

}

#endif
