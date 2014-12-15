#include "hybrid_automaton/ClockSensor.h"

namespace ha
{
	HA_SENSOR_REGISTER("ClockSensor", ClockSensor);

	ClockSensor::ClockSensor():
	_current_time(0.0)
	{
	}

	ClockSensor::~ClockSensor()
	{
	}

	ClockSensor::ClockSensor(const ClockSensor& ss)
		:Sensor(ss),
		_current_time(ss._current_time)
	{
	}

	::Eigen::MatrixXd ClockSensor::getCurrentValue() const
	{
		::Eigen::MatrixXd ret(1,1);
		ret<<this->_current_time;
		return ret;
	}

	void ClockSensor::initialize(const double& t) 
	{
		this->_current_time = t;
		this->_initial_sensor_value = getCurrentValue();
	}

	void ClockSensor::step(const double& t) 
	{
		this->_current_time = t;
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

		_system = system;
	}

}