#include "hybrid_automaton/SubjointVelocitySensor.h"

namespace ha
{

    HA_SENSOR_REGISTER("SubjointVelocitySensor", SubjointVelocitySensor);

    SubjointVelocitySensor::SubjointVelocitySensor()
	{
	}

    SubjointVelocitySensor::~SubjointVelocitySensor()
	{
	}

    SubjointVelocitySensor::SubjointVelocitySensor(const SubjointVelocitySensor& ss)
        :Sensor(ss), _index(ss._index)
	{
	}

    ::Eigen::MatrixXd SubjointVelocitySensor::getCurrentValue() const
	{
        ::Eigen::MatrixXd cfg = this->_system->getJointVelocity();
        ::Eigen::MatrixXd subcfg(_index.size(), 1);
        for(int i=0; i< _index.size(); i++)
            subcfg(i) = cfg(_index[i]);

        return subcfg;
	}

    DescriptionTreeNode::Ptr SubjointVelocitySensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

		Eigen::MatrixXd index_mat(_index.size(), 1);
        for(int j=0; j<index_mat.rows(); j++)
			index_mat(j) = _index[j];

		tree->setAttribute< Eigen::MatrixXd >(std::string("index"), index_mat);

		return tree;
	}

    void SubjointVelocitySensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
            HA_THROW_ERROR("SubjointVelocitySensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

        ::Eigen::MatrixXd index_mat;
        if(!tree->getAttribute< ::Eigen::MatrixXd >("index", index_mat) )
            HA_THROW_ERROR("SubjointVelocitySensor.deserialize", "This type of sensor needs a value 'index'!");

		_index.resize(index_mat.rows());
        for(int j=0; j<index_mat.rows(); j++)
            _index[j]=index_mat(j);

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
            HA_THROW_ERROR("SubjointVelocitySensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		_system = system;
	}

}
