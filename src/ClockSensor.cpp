#include "hybrid_automaton/ClockSensor.h"

namespace ha
{
	HA_SENSOR_REGISTER("ClockSensor", ClockSensor);

	ClockSensor::ClockSensor()
	{
	}

	ClockSensor::~ClockSensor()
	{
	}

	ClockSensor::ClockSensor(const ClockSensor& ss)
		:Sensor(ss)
	{
	}

	::Eigen::MatrixXd ClockSensor::getCurrentValue() const
	{
		return this->_system->getCurrentTime();
	}

	DescriptionTreeNode::Ptr ClockSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

		return tree;
	}

	void ClockSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("ClockSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("ClockSensor.deserialize", "Sensor type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}
	}

}