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
#include "hybrid_automaton/FrameDisplacementSensor.h"

namespace ha
{

	HA_SENSOR_REGISTER("FrameDisplacementSensor", FrameDisplacementSensor);

	FrameDisplacementSensor::FrameDisplacementSensor(const std::string& frame_id)
		: _frame_id(frame_id)
	{
	}

	FrameDisplacementSensor::~FrameDisplacementSensor()
	{
	}

	FrameDisplacementSensor::FrameDisplacementSensor(const FrameDisplacementSensor& ss)
		:Sensor(ss)
	{
	}

	void FrameDisplacementSensor::initialize(const double& t) 
	{
		this->_initial_sensor_value = this->_system->getFramePose(this->_frame_id);
	}

	::Eigen::MatrixXd FrameDisplacementSensor::getCurrentValue() const
	{
		::Eigen::MatrixXd pose = this->_system->getFramePose(this->_frame_id);
		pose = pose.col(3);
		pose.conservativeResize(3,1);
		return pose;
	}

	::Eigen::MatrixXd FrameDisplacementSensor::getInitialValue() const
	{
		::Eigen::MatrixXd ret = this->_initial_sensor_value.col(3);
		ret.conservativeResize(3,1);
		return ret;
	}

	::Eigen::MatrixXd FrameDisplacementSensor::getRelativeCurrentValue() const
	{
		
		::Eigen::MatrixXd pose = this->_system->getFramePose(this->_frame_id);
		pose=_initial_sensor_value.inverse()*pose;
		pose = pose.col(3);
		pose.conservativeResize(3,1);
		
		return pose;
	}

	DescriptionTreeNode::Ptr FrameDisplacementSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());
		tree->setAttribute<std::string>(std::string("frame_id"), _frame_id);

		return tree;
	}

	void FrameDisplacementSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("FrameDisplacementSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("FrameDisplacementSensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}
		
		if(!tree->getAttribute<std::string>("frame_id", _frame_id))
		{
			HA_WARN("FramePoseSensor::deserialize", "frame_id not defined. using default value EE");
			_frame_id = "EE";
		}

		_system = system;
	}
}