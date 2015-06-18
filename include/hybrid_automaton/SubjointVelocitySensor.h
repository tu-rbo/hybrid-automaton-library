#ifndef SUBJOINT_VELOCITY_SENSOR_H
#define SUBJOINT_VELOCITY_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

    class SubjointVelocitySensor;
    typedef boost::shared_ptr<SubjointVelocitySensor> SubjointVelocitySensorPtr;
    typedef boost::shared_ptr<const SubjointVelocitySensor> SubjointVelocitySensorConstPtr;

    class SubjointVelocitySensor : public Sensor
	{
	public:

        typedef boost::shared_ptr<SubjointVelocitySensor> Ptr;
        typedef boost::shared_ptr<const SubjointVelocitySensor> ConstPtr;

        /**
         * @brief An interface to a subset of the current joint velocity
         *
         * Specify the dimensions of interest with the \a index vector
         */
        SubjointVelocitySensor();

        virtual ~SubjointVelocitySensor();

        SubjointVelocitySensor(const SubjointVelocitySensor& ss);

        SubjointVelocitySensorPtr clone() const
		{
            return (SubjointVelocitySensorPtr(_doClone()));
        }

        virtual std::vector<int> getIndex() const
        {
            return _index;
        }

        /**
         * @brief Returns a subset of the joint velcoity vector of as a matrix of size dim(index)x1
         *
         * the output value of this Sensor will be (q_dot[index[0]], ... q_dot[index[n]])^T
         */
        virtual void setIndex(const std::vector<int>& index)
        {
            _index = index;
        }

        /**
         * @brief Returns a subset of the joint velcoity vector of as a matrix of size dim(index)x1
         *
         * the output value of this Sensor will be (q_dot[index[0]], ... q_dot[index[n]])^T
         */
		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
            Sensor::Ptr sensor(new SubjointVelocitySensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:
        virtual SubjointVelocitySensor* _doClone() const
		{
            return (new SubjointVelocitySensor(*this));
		}


        std::vector<int> _index;

	};

}

#endif
