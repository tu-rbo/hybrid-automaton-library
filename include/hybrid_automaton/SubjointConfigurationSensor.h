#ifndef SUBJOINT_CONFIGURATION_SENSOR_H
#define SUBJOINT_CONFIGURATION_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

namespace ha {

    class SubjointConfigurationSensor;
    typedef boost::shared_ptr<SubjointConfigurationSensor> SubjointConfigurationSensorPtr;
    typedef boost::shared_ptr<const SubjointConfigurationSensor> SubjointConfigurationSensorConstPtr;

    class SubjointConfigurationSensor : public Sensor
	{
	public:

        typedef boost::shared_ptr<SubjointConfigurationSensor> Ptr;
        typedef boost::shared_ptr<const SubjointConfigurationSensor> ConstPtr;

        SubjointConfigurationSensor();

        virtual ~SubjointConfigurationSensor();

        SubjointConfigurationSensor(const SubjointConfigurationSensor& ss);

        SubjointConfigurationSensorPtr clone() const
		{
            return (SubjointConfigurationSensorPtr(_doClone()));
        }

        virtual std::vector<int> getIndex() const
        {
            return _index;
        }

        virtual void setIndex(const std::vector<int>& index)
        {
            _index = index;
        }

        virtual void setIndex(const Eigen::MatrixXd& index)
        {
            _index.resize(index.size());
            for(int i=0; i<index.rows(); i++)
            {
                _index.at(i) = (int)index(i,0);
            }
        }

		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
            Sensor::Ptr sensor(new SubjointConfigurationSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:
        virtual SubjointConfigurationSensor* _doClone() const
		{
            return (new SubjointConfigurationSensor(*this));
		}


        std::vector<int> _index;

	};

}

#endif
