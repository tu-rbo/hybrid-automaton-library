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
#include "hybrid_automaton/FrameOrientationSensor.h"

namespace ha
{

	HA_SENSOR_REGISTER("FrameOrientationSensor", FrameOrientationSensor);

	FrameOrientationSensor::FrameOrientationSensor(const std::string& frame_id)
		: _frame_id(frame_id)
	{
	}

	FrameOrientationSensor::~FrameOrientationSensor()
	{
	}

	FrameOrientationSensor::FrameOrientationSensor(const FrameOrientationSensor& ss)
		:Sensor(ss)
	{
	}

	void FrameOrientationSensor::initialize(const double& t) 
	{
		::Eigen::MatrixXd pose = this->_system->getFramePose(this->_frame_id);
		pose.conservativeResize(3,3);
		this->_initial_sensor_value = pose;
	}

	::Eigen::MatrixXd FrameOrientationSensor::getCurrentValue() const
	{
		::Eigen::MatrixXd pose = this->_system->getFramePose(this->_frame_id);
		pose.conservativeResize(3,3);
		return pose;
	}
	
	::Eigen::MatrixXd FrameOrientationSensor::getInitialValue() const
	{
		return this->_initial_sensor_value;
	}

	::Eigen::MatrixXd FrameOrientationSensor::getRelativeCurrentValue() const
	{
		::Eigen::MatrixXd pose = this->_system->getFramePose(this->_frame_id);
		pose.conservativeResize(3,3);
		pose = _initial_sensor_value.inverse()*pose;
		return pose;
	}

	DescriptionTreeNode::Ptr FrameOrientationSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());
		tree->setAttribute<std::string>(std::string("frame_id"), _frame_id);

		return tree;
	}

	void FrameOrientationSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("FrameOrientationSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("FrameOrientationSensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		if(!tree->getAttribute<std::string>("frame_id", _frame_id))
		{
			HA_WARN("FrameOrientationSensor::deserialize", "frame_id not defined. using default value EE");
			_frame_id = "EE";
		}

		
		_system = system;
	}
}