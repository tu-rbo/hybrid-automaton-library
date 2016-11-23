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
        for(size_t i=0; i< _index.size(); i++)
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

		if(index_mat.rows()> system->getDof() || index_mat.cols() != 1)
			HA_THROW_ERROR("SubjointVelocitySensor::deserialize", "Wrong dimensions for index!! - need at most (" << system->getDof()<<"x1), got ("<<index_mat.rows()<<"x"<<index_mat.cols()<<") !");

		std::vector<long> index_vec(index_mat.rows());
		for(size_t i = 0; i < index_vec.size(); i++)
		{
			if(fabs(index_mat(i,0) - floor(index_mat(i,0)))>0.01)
				HA_THROW_ERROR("SubjointVelocitySensor::deserialize", "Value in index not an integer - instead it's "<<index_mat(i,0)<<" !");
			int index_int = (int)floor(index_mat(i,0));
			if(index_int >= system->getDof() || index_int < 0)
				HA_THROW_ERROR("SubjointVelocitySensor::deserialize", "Value in index does not match a joint index - instead it's "<<index_mat(i,0)<<" !");
			index_vec[i] = index_int;
		}
		for(size_t i = 0; i < index_vec.size(); i++)
		{
			for(size_t j = i+1; j < index_vec.size(); j++)
			{
				if(index_vec[i] == index_vec[j]){
					HA_THROW_ERROR("SubjointVelocitySensor::deserialize", "Value in index occurs twice: "<<index_vec.at(i)<<". These should be unique !");
				}
			}
		}

		_index.resize(index_mat.rows());
        for(size_t j=0; j<index_mat.rows(); j++)
            _index[j]=index_mat(j);

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
            HA_THROW_ERROR("SubjointVelocitySensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		_system = system;
	}

}
