#include "hybrid_automaton/SubjointConfigurationSensor.h"

namespace ha
{

    HA_SENSOR_REGISTER("SubjointConfigurationSensor", SubjointConfigurationSensor);

    SubjointConfigurationSensor::SubjointConfigurationSensor()
	{
	}

    SubjointConfigurationSensor::~SubjointConfigurationSensor()
	{
	}

    SubjointConfigurationSensor::SubjointConfigurationSensor(const SubjointConfigurationSensor& ss)
        :Sensor(ss), _index(ss._index)
	{
	}

    ::Eigen::MatrixXd SubjointConfigurationSensor::getCurrentValue() const
	{
        ::Eigen::MatrixXd cfg = this->_system->getJointConfiguration();
        ::Eigen::MatrixXd subcfg(_index.size(), 1);
        for(size_t i=0; i< _index.size(); i++)
            subcfg(i) = cfg(_index[i]);

        return subcfg;
	}

    DescriptionTreeNode::Ptr SubjointConfigurationSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

		Eigen::MatrixXd index_mat(_index.size(), 1);
        for(int j=0; j<index_mat.rows(); j++)
			index_mat(j) = _index[j];

		tree->setAttribute< Eigen::MatrixXd >(std::string("index"), index_mat);

		return tree;
	}

    void SubjointConfigurationSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
            HA_THROW_ERROR("SubjointConfigurationSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

        ::Eigen::MatrixXd index_mat;
        if(!tree->getAttribute< ::Eigen::MatrixXd >("index", index_mat) )
            HA_THROW_ERROR("SubjointConfigurationSensor.deserialize", "This type of sensor needs a value 'index'!");

		_index.resize(index_mat.rows());
        for(int j=0; j<index_mat.rows(); j++)
            _index[j]=(int)index_mat(j);

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
            HA_THROW_ERROR("SubjointConfigurationSensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		_system = system;
	}

}
