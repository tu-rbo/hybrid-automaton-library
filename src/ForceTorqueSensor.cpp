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

	::Eigen::MatrixXd ForceTorqueSensor::getCurrentValue() const
	{
    //This is the F/T wrench from the hardware - It must be in EE frame
    ::Eigen::MatrixXd forceTorque = _system->getForceTorqueMeasurement(DEFAULT_FT_PORT);

    ::Eigen::Matrix3d frameRot = _frame.block(0,0,3,3);
    ::Eigen::Vector3d frameTrans(_frame(0,3), _frame(1,3), _frame(2,3));
    ::Eigen::Vector3d forcePart(forceTorque(0,0), forceTorque(1,0),forceTorque(2,0));
    ::Eigen::Vector3d momentPart(forceTorque(3,0), forceTorque(4,0),forceTorque(5,0));

    forcePart = frameRot*forcePart;
    momentPart = frameRot*(momentPart - frameTrans.cross(forcePart));

    Eigen::MatrixXd ftOut(6,1);
    ftOut(0) = forcePart(0); ftOut(1) = forcePart(1); ftOut(2) = forcePart(2);
    ftOut(3) = momentPart(0); ftOut(4) = momentPart(1); ftOut(5) = momentPart(2);

    return ftOut;
	}

	DescriptionTreeNode::Ptr ForceTorqueSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");
		tree->setAttribute<std::string>(std::string("type"), this->getType());
		tree->setAttribute<int>(std::string("port"), _port);
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
