#ifndef ACOUSTICSENSOR
#define ACOUSTICSENSOR
#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha{

    class StrainSensor;
    typedef boost::shared_ptr<StrainSensor> StrainSensorPtr;
    typedef boost::shared_ptr<const StrainSensor> StrainSensorConstPtr;

  class StrainSensor: public Sensor{
    	public:

		typedef boost::shared_ptr<StrainSensor> Ptr;
		typedef boost::shared_ptr<const StrainSensor> ConstPtr;

		StrainSensor();

		virtual ~StrainSensor();

		AcousticSensor(const StrainSensor& ss);

		StrainSensorPtr clone() const
		{
			return (StrainSensorPtr(_doClone()));
        }


         /*
            * @brief Returns the current deformation of one (need to be made flexible in the future)
                     time as a 3x1 deformation matrix --> flexional, lateral, twist
         */
		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual void initialize(const double& t);
		virtual void step(const double& t);

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new StrainSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:

		virtual StrainSensor* _doClone() const
		{
			return (new StrainSensor(*this));
		}
		/**
		* The current time
		*/
		Eigen::MatrixXd _current_value;


  };
}
#endif
