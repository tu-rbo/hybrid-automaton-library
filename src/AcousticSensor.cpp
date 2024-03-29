#include <hybrid_automaton/AcousticSensor.h>


namespace ha{

  HA_SENSOR_REGISTER("AcousticSensor", AcousticSensor);

  	AcousticSensor::AcousticSensor()
	{
	}

	AcousticSensor::~AcousticSensor()
	{
	}

	AcousticSensor::AcousticSensor(const AcousticSensor& ss)
		:Sensor(ss),
		_current_value(ss._current_value)
	{
	}

	::Eigen::MatrixXd AcousticSensor::getCurrentValue() const
	{
		::Eigen::MatrixXd ret;
		ret = _system->getAcousticSensorMeasurement(this->_n_fingers, this->_n_classes);
		return ret;
	}

	int AcousticSensor::get_n_fingers(){
		return this->_n_fingers;
	}

	int AcousticSensor::get_n_classes(){
		return this->_n_classes;
	}

	void AcousticSensor::initialize(const double& t) 
	{
		HA_INFO("AcousticSensor.initialize", "Initialize AcousticSensor");
		this->_initial_sensor_value = getCurrentValue();
	}

	void AcousticSensor::step(const double& t) 
	{
		//this->_current_time = t;
	}


	DescriptionTreeNode::Ptr AcousticSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

		return tree;
	}

	void AcousticSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
	     		
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("AcousticSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("AcousticSensor.deserialize", "Sensor type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		
		tree->getAttribute<int>("fingers", _n_fingers);
		tree->getAttribute<int>("classes", _n_classes);

		_system = system;
	}

}