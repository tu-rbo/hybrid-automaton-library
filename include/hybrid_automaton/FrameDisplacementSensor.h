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
#ifndef FRAME_DISPLACEMENT_SENSOR_H
#define FRAME_DISPLACEMENT_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class FrameDisplacementSensor;
	typedef boost::shared_ptr<FrameDisplacementSensor> FrameDisplacementSensorPtr;
	typedef boost::shared_ptr<const FrameDisplacementSensor> FrameDisplacementSensorConstPtr;

    /**
     * @brief An interface to measure the position of the given frame usinf forward kinematics
     */
	class FrameDisplacementSensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<FrameDisplacementSensor> Ptr;
		typedef boost::shared_ptr<const FrameDisplacementSensor> ConstPtr;

		FrameDisplacementSensor(const std::string& frame_id = std::string("EE"));

		virtual ~FrameDisplacementSensor();

		FrameDisplacementSensor(const FrameDisplacementSensor& ss);

		FrameDisplacementSensorPtr clone() const
		{
			return (FrameDisplacementSensorPtr(_doClone()));
		};

		virtual void initialize(const double& t); 
		
		virtual ::Eigen::MatrixXd getRelativeCurrentValue() const;

        /**
         * @brief Returns the current frame position as a 3x1 vector
         */
		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual ::Eigen::MatrixXd getInitialValue() const;
		
		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new FrameDisplacementSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:

		std::string _frame_id;

		virtual FrameDisplacementSensor* _doClone() const
		{
			return (new FrameDisplacementSensor(*this));
		}

	};

}

#endif
