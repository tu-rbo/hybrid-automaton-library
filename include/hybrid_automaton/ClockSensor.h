#ifndef CLOCK_SENSOR_H
#define CLOCK_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class ClockSensor;
	typedef boost::shared_ptr<ClockSensor> ClockSensorPtr;
	typedef boost::shared_ptr<const ClockSensor> ClockSensorConstPtr;

	class ClockSensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<ClockSensor> Ptr;
		typedef boost::shared_ptr<const ClockSensor> ConstPtr;

		ClockSensor();

		virtual ~ClockSensor();

		ClockSensor(const ClockSensor& ss);

		ClockSensorPtr clone() const
		{
			return (ClockSensorPtr(_doClone()));
		};

		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual void initialize(const double& t); 
		virtual void step(const double& t);

		virtual void resetTimer();

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new ClockSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:

		virtual ClockSensor* _doClone() const
		{
			return (new ClockSensor(*this));
		}

		/**
		* The current time
		*/
		double _time;

		/**
		* The last time this sensor was resetted
		*/
		double _timerStart;

	};

}

#endif
