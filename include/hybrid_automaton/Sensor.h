#ifndef HYBRID_AUTOMATON_SENSOR_H
#define HYBRID_AUTOMATON_SENSOR_H

#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/error_handling.h"
#include "hybrid_automaton/System.h"
#include "hybrid_automaton/HybridAutomatonRegistration.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class Sensor;
	typedef boost::shared_ptr<Sensor> SensorPtr;
	typedef boost::shared_ptr<const Sensor> SensorConstPtr;

	class Sensor : public Serializable
	{
	public:

		typedef boost::shared_ptr<Sensor> Ptr;
		typedef boost::shared_ptr<const Sensor> ConstPtr;

		Sensor();

		virtual ~Sensor();

		/**
		* Copy constructor
		*/
		Sensor(const Sensor& ss);

		SensorPtr clone() const
		{
			return (SensorPtr(_doClone()));
		};

		virtual ::Eigen::MatrixXd getCurrentValue() const = 0;

		virtual ::Eigen::MatrixXd getRelativeCurrentValue() const;

        // automatically implemented by HA_SENSOR_INSTANCE macro
        virtual const std::string getType() const;
		virtual void setType(const std::string& new_type);

		virtual ::Eigen::MatrixXd getInitialValue() const;

		virtual void setSystem(const System::ConstPtr& system);

		virtual void initialize(const double& t); 
		virtual void terminate();
		virtual void step(const double& t);

		/**
		 * @brief If a sensor is inactive (e.g. it is still booting)
		 *  its JumpCondition will interpret its state as false
		 *
		 * By default, a sensor is always active. Override this method
		 * to change behavior.
		 */
		virtual bool isActive() const {
			return true;
		}

	protected:
		System::ConstPtr _system;

		virtual Sensor* _doClone() const = 0;

		std::string _type;

		/**
		 * @brief Initial sensor value
		 *
		 * Required to compute relative sensor value.
		 *
		 * It is mutable as some sensors might not be able
		 * to fill it in the initialize method but only when
		 * becoming active (e.g. ROSTopicSensor)
		 */
		mutable ::Eigen::MatrixXd _initial_sensor_value;

	};

}

#endif
