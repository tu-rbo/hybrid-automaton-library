#ifndef JOINT_CONFIGURATION_SENSOR_H
#define JOINT_CONFIGURATION_SENSOR_H

#include "hybrid_automaton/Sensor.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class JointConfigurationSensor;
	typedef boost::shared_ptr<JointConfigurationSensor> JointConfigurationSensorPtr;
	typedef boost::shared_ptr<const JointConfigurationSensor> JointConfigurationSensorConstPtr;

	class JointConfigurationSensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<JointConfigurationSensor> Ptr;
		typedef boost::shared_ptr<const JointConfigurationSensor> ConstPtr;

		JointConfigurationSensor();

		virtual ~JointConfigurationSensor();

		JointConfigurationSensor(const JointConfigurationSensor& ss);

		JointConfigurationSensorPtr clone() const
		{
			return (JointConfigurationSensorPtr(_doClone()));
		};

		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

	protected:

		virtual JointConfigurationSensor* _doClone() const
		{
			return (new JointConfigurationSensor(*this));
		}

	};

}

#endif
