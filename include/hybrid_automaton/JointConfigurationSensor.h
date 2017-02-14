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
#ifndef JOINT_CONFIGURATION_SENSOR_H
#define JOINT_CONFIGURATION_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class JointConfigurationSensor;
	typedef boost::shared_ptr<JointConfigurationSensor> JointConfigurationSensorPtr;
	typedef boost::shared_ptr<const JointConfigurationSensor> JointConfigurationSensorConstPtr;

    /**
     * @brief An interface to the current joint encoder readings
     */
	class JointConfigurationSensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<JointConfigurationSensor> Ptr;
		typedef boost::shared_ptr<const JointConfigurationSensor> ConstPtr;

		JointConfigurationSensor();

		virtual ~JointConfigurationSensor();

		JointConfigurationSensor(const JointConfigurationSensor& ss);

		JointConfigurationSensorPtr clone() const
		{
			return (JointConfigurationSensorPtr(_doClone()));
		};

        /**
         * @brief Return the current robot configuration as a dimx1-vector
         */
		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new JointConfigurationSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:

		virtual JointConfigurationSensor* _doClone() const
		{
			return (new JointConfigurationSensor(*this));
		}

	};

}

#endif
