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
#ifndef FRAME_POSE_SENSOR_H
#define FRAME_POSE_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class FramePoseSensor;
	typedef boost::shared_ptr<FramePoseSensor> FramePoseSensorPtr;
	typedef boost::shared_ptr<const FramePoseSensor> FramePoseSensorConstPtr;

    /**
     * @brief An interface to measure the pose of the given robot frame using forward kinematics
     */
	class FramePoseSensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<FramePoseSensor> Ptr;
		typedef boost::shared_ptr<const FramePoseSensor> ConstPtr;

		FramePoseSensor(const std::string& frame_id = std::string("EE"));

		virtual ~FramePoseSensor();

		FramePoseSensor(const FramePoseSensor& ss);

		FramePoseSensorPtr clone() const
		{
			return (FramePoseSensorPtr(_doClone()));
		};

		virtual void initialize(const double& t); 


        /**
         * @brief Returns the current frame pose as a 4x4 homogenuous transformation matrix
         */
		virtual ::Eigen::MatrixXd getCurrentValue() const;
		
		virtual ::Eigen::MatrixXd getRelativeCurrentValue() const;

		virtual ::Eigen::MatrixXd getInitialValue() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new FramePoseSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}

	protected:

		std::string _frame_id;

    std::string _reference_frame;

		virtual FramePoseSensor* _doClone() const
		{
			return (new FramePoseSensor(*this));
		}

	};

}

#endif
