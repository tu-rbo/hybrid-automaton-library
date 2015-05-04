#include "hybrid_automaton/JointConfigurationSensor.h"

namespace ha
{

	HA_SENSOR_REGISTER("JointConfigurationSensor", JointConfigurationSensor);

	JointConfigurationSensor::JointConfigurationSensor()
	{
	}

	JointConfigurationSensor::~JointConfigurationSensor()
	{
	}

	JointConfigurationSensor::JointConfigurationSensor(const JointConfigurationSensor& ss)
		:Sensor(ss)
	{
	}

	::Eigen::MatrixXd JointConfigurationSensor::getCurrentValue() const
	{
		return this->_system->getJointConfiguration();
	}

	DescriptionTreeNode::Ptr JointConfigurationSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

		return tree;
	}

	void JointConfigurationSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("JointConfigurationSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("JointConfigurationSensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		_system = system;
	}

}