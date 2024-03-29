#ifndef ACOUSTICSENSOR
#define ACOUSTICSENSOR
#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha{
  
  
  	class AcousticSensor;
	typedef boost::shared_ptr<AcousticSensor> AcousticSensorPtr;
	typedef boost::shared_ptr<const AcousticSensor> AcousticSensorConstPtr;
  class AcousticSensor: public Sensor{
    	public:

		typedef boost::shared_ptr<AcousticSensor> Ptr;
		typedef boost::shared_ptr<const AcousticSensor> ConstPtr;

		AcousticSensor();

		virtual ~AcousticSensor();

		AcousticSensor(const AcousticSensor& ss);

		AcousticSensorPtr clone() const
		{
			return (AcousticSensorPtr(_doClone()));
        }
        
        
                /**
         * @brief Returns the current System time as a 1x1 matrix
         */
		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual void initialize(const double& t); 
		virtual void step(const double& t);

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new AcousticSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

		int get_n_fingers();
		int get_n_classes();

	protected:

		virtual AcousticSensor* _doClone() const
		{
			return (new AcousticSensor(*this));
		}

		/**
		* The current time
		*/
		Eigen::MatrixXd _current_value;

		int _n_fingers;
		int _n_classes;
    
    
  };
}
#endif
