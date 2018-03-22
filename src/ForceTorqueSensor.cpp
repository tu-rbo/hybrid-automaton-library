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
#include "hybrid_automaton/ForceTorqueSensor.h"

namespace ha
{
	HA_SENSOR_REGISTER("ForceTorqueSensor", ForceTorqueSensor);

	ForceTorqueSensor::ForceTorqueSensor() : _port(DEFAULT_FT_PORT)
	{
	}

	ForceTorqueSensor::~ForceTorqueSensor()
	{
	}

	ForceTorqueSensor::ForceTorqueSensor(const ForceTorqueSensor& ss)
		:Sensor(ss)
	{
		_port = ss._port;
	}
	::Eigen::MatrixXd ForceTorqueSensor::transformWrench(const ::Eigen::MatrixXd& wrench, const ::Eigen::MatrixXd& transform) const
	{
		::Eigen::Matrix3d frameRot = transform.block(0,0,3,3);
		::Eigen::Vector3d frameTrans(transform(0,3), transform(1,3), transform(2,3));
		::Eigen::Vector3d forcePart(wrench(0,0), wrench(1,0),wrench(2,0));
		::Eigen::Vector3d momentPart(wrench(3,0), wrench(4,0),wrench(5,0));

		forcePart = frameRot.transpose()*forcePart;
		momentPart = frameRot.transpose()*(momentPart - frameTrans.cross(forcePart));

		Eigen::MatrixXd wrenchOut(6,1);
		wrenchOut(0) = forcePart(0); wrenchOut(1) = forcePart(1); wrenchOut(2) = forcePart(2);
		wrenchOut(3) = momentPart(0); wrenchOut(4) = momentPart(1); wrenchOut(5) = momentPart(2);
		return wrenchOut;
	}

	::Eigen::MatrixXd ForceTorqueSensor::getCurrentValue() const
	{
		//This is the F/T wrench from the hardware - It must be in EE frame
		::Eigen::MatrixXd forceTorque = _system->getForceTorqueMeasurement(_port);
		Eigen::MatrixXd ftOut = forceTorque;
		
		if(_frame_id == "world")
		{
			::Eigen::MatrixXd eeFrame = _system->getFramePose("EE");

			//Transform FT wrench to world frame
			ftOut = transformWrench(forceTorque, eeFrame.inverse());
		}

		//Transform FT wrench to given frame
		ftOut = transformWrench(ftOut, _frame);

		//if((k++)%2000 == 0)
		//	HA_INFO("ForceTorqueSensor.getCurrentValue","ftout: "<<ftOut.transpose());

		return ftOut;
	}

	DescriptionTreeNode::Ptr ForceTorqueSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");
		tree->setAttribute<std::string>(std::string("type"), this->getType());
		tree->setAttribute<int>(std::string("port"), _port);
		tree->setAttribute<Eigen::MatrixXd>(std::string("frame"), _frame);
		tree->setAttribute<std::string>(std::string("frame_id"), _frame_id);
		return tree;
	}

	void ForceTorqueSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("ForceTorqueSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");
		tree->getAttribute<int>("port", _port, DEFAULT_FT_PORT);

		if(_port > 2 || _port <0)
		{
			HA_THROW_ERROR("ForceTorqueSensor.deserialize", "Port number " << _port << " "
				<< "invalid - it must be between 0 and 2!");
		}

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("ForceTorqueSensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}
		
		tree->getAttribute<std::string>("frame_id", _frame_id, "EE");
		if(_frame_id != "EE" && _frame_id != "world")
		{
			HA_WARN("ForceTorqueSensor.deserialize", "Currently only frame_id EE or world supported. Found "<<_frame_id<<". Will use default value EE!");
			_frame_id = "EE";
		}

		_frame.resize(4,4);
		_frame.setIdentity();
		if(tree->getAttribute< Eigen::MatrixXd>("frame", _frame))
		{
			HA_INFO("ForceTorqueSensor.deserialize", "Using external frame to express F/T value in");
			if(_frame.cols()!=4 || _frame.rows()!=4)
			{
				HA_THROW_ERROR("ForceTorqueSensor.deserialize", "frame parameter must be 4x4 homogeneous transform!");
			}
		}

		_system = system;
	}
}
