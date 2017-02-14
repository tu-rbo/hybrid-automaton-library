/*
 * Copyright 2015-2017, Robotics and Biology Lab, TU Berlin
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the 
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
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

        /**
        * @brief The interface for all Sensors to use in a JumpCondition
        *
        * implementations need to overload getCurrentValue and possibly getRelativeCurrentValue
        */
		Sensor();

		virtual ~Sensor();

		/**
		* Copy constructor
		*/
		Sensor(const Sensor& ss);

		SensorPtr clone() const
		{
			return (SensorPtr(_doClone()));
        }

        /**
        * @brief The current value of this sensor
        */
		virtual ::Eigen::MatrixXd getCurrentValue() const = 0;

        /**
        * @brief The value of this sensor, relative to the initial sensor value (at activation time)
        *
        * The default implementation returns current_value - initial_value,
        * this might need to be adapted, i.e. if your sensor value is a homogenuous transform
        */
		virtual ::Eigen::MatrixXd getRelativeCurrentValue() const;

        // automatically implemented by HA_SENSOR_INSTANCE macro
        virtual const std::string getType() const;
		virtual void setType(const std::string& new_type);

		virtual ::Eigen::MatrixXd getInitialValue() const;

		virtual void setSystem(const System::ConstPtr& system);

        /**
        * @brief Activates the sensor and stores the initial sensor value - is called from the JumpCondition
        */
		virtual void initialize(const double& t); 

        /**
        * @brief Deactivates the sensor - is called from the JumpCondition
        */
        virtual void terminate();

        /**
        * @brief Update sensor value - is called from the JumpCondition once per control cycle
        */
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
