#include "hybrid_automaton/JointVelocitySensor.h"

namespace ha
{

	HA_SENSOR_REGISTER("JointVelocitySensor", JointVelocitySensor);

	JointVelocitySensor::JointVelocitySensor()
	{
	}

	JointVelocitySensor::~JointVelocitySensor()
	{
	}

	JointVelocitySensor::JointVelocitySensor(const JointVelocitySensor& ss)
		:Sensor(ss)
	{
	}

	::Eigen::MatrixXd JointVelocitySensor::getCurrentValue() const
	{
		return this->_system->getJointVelocity();
	}

	DescriptionTreeNode::Ptr JointVelocitySensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

		return tree;
	}

	void JointVelocitySensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("JointVelocitySensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("JointVelocitySensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		_system = system;
	}

}