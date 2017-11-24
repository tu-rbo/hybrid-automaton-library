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
#ifndef FORCE_TORQUE_SENSOR_H
#define FORCE_TORQUE_SENSOR_H

#include "hybrid_automaton/Sensor.h"
#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

	class ForceTorqueSensor;
	typedef boost::shared_ptr<ForceTorqueSensor> ForceTorqueSensorPtr;
	typedef boost::shared_ptr<const ForceTorqueSensor> ForceTorqueSensorConstPtr;

	/**
	* @brief An interface to a six-axis force-torque sensor
	*/
	class ForceTorqueSensor : public Sensor
	{
	public:

		typedef boost::shared_ptr<ForceTorqueSensor> Ptr;
		typedef boost::shared_ptr<const ForceTorqueSensor> ConstPtr;

		ForceTorqueSensor();

		virtual ~ForceTorqueSensor();

		ForceTorqueSensor(const ForceTorqueSensor& ss);

		ForceTorqueSensorPtr clone() const
		{
			return (ForceTorqueSensorPtr(_doClone()));
		};

		/**
		* @brief Returns the current force-torque sensor reading as a 6x1 vector.
		*
		* Optionally transforms the force into frame _frame if parameter is given
		*
		* x ,y ,z ,rot_x, rot_y, rot_z
		*/
		virtual ::Eigen::MatrixXd getCurrentValue() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		// required to enable deserialization of this sensor
		HA_SENSOR_INSTANCE(node, system, ha) {
			Sensor::Ptr sensor(new ForceTorqueSensor());
			sensor->deserialize(node, system, ha);
			return sensor;
		}


		virtual void setFrameId(const std::string& frame_id) {
			_frame_id = frame_id;
		}

		virtual std::string getFrameId() const {
			return _frame_id;
		}

		virtual void setFrame(const Eigen::MatrixXd& frame) {
			_frame = frame;
		}

		virtual ::Eigen::MatrixXd getFrame() const {
			return _frame;
		}

	protected:

        /**
         * @brief Optional frame ID to transform the force. Can be either "EE", for force in end-effector frame
		 * or "world" for force in world coordinate system
         */
        std::string _frame_id;

		/**
		* @brief Optional frame to transform the force into (relative to frame_id)
		*/
		::Eigen::MatrixXd _frame;

		// The port of the force-torque sensor to query values from
		int _port;

		virtual ForceTorqueSensor* _doClone() const
		{
			return (new ForceTorqueSensor(*this));
		}

		::Eigen::MatrixXd transformWrench(const ::Eigen::MatrixXd& wrench, const ::Eigen::MatrixXd& transform) const;
	};

}

#endif
