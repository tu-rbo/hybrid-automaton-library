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
#include "hybrid_automaton/FramePoseSensor.h"

namespace ha
{

	HA_SENSOR_REGISTER("FramePoseSensor", FramePoseSensor);

	FramePoseSensor::FramePoseSensor(const std::string& frame_id)
		: _frame_id(frame_id)
	{
	}

	FramePoseSensor::~FramePoseSensor()
	{
	}

	FramePoseSensor::FramePoseSensor(const FramePoseSensor& ss)
		:Sensor(ss)
	{
	}


	void FramePoseSensor::initialize(const double& t) 
	{
		this->_initial_sensor_value = this->_system->getFramePose(this->_frame_id);
	}

	::Eigen::MatrixXd FramePoseSensor::getCurrentValue() const
	{
		return this->_system->getFramePose(this->_frame_id);
	}

	::Eigen::MatrixXd FramePoseSensor::getRelativeCurrentValue() const
	{
		::Eigen::MatrixXd pose = this->_system->getFramePose(this->_frame_id);
		pose = _initial_sensor_value.inverse()*pose;
		return pose;
	}

	::Eigen::MatrixXd FramePoseSensor::getInitialValue() const
	{
		return this->_initial_sensor_value;
	}

	DescriptionTreeNode::Ptr FramePoseSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());
		tree->setAttribute<std::string>(std::string("frame_id"), _frame_id);

		return tree;
	}

	void FramePoseSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("FramePoseSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("FramePoseSensor.deserialize", "SensorType type '" << _type << "' "
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