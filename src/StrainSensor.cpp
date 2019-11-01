#include <hybrid_automaton/StrainSensor.h>

namespace ha{

  HA_SENSOR_REGISTER("StrainSensor", StrainSensor);

  	StrainSensor::StrainSensor()
	{
	}

	StrainSensor::~StrainSensor()
	{
	}

	StrainSensor::StrainSensor(const StrainSensor& ss)
		:Sensor(ss),
		_current_value(ss._current_value)
	{
	}

	::Eigen::MatrixXd StrainSensor::getCurrentValue() const
	{
		::Eigen::MatrixXd ret(3,1);
		ret = _system->getStrainSensorMeasurement();
		return ret;
	}

	void StrainSensor::initialize(const double& t)
	{
		HA_INFO("StrainSensor.initialize", "Initialize StrainSensor");
		this->_initial_sensor_value = getCurrentValue();
	}

	void StrainSensor::step(const double& t)
	{
		//this->_current_time = t;
	}

	DescriptionTreeNode::Ptr StrainSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

		return tree;
	}

	void StrainSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{

		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("StrainSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("StrainSensor.deserialize", "Sensor type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		_system = system;
	}

}