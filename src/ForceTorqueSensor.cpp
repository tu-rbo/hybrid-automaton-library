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

        ForceTorqueSensor::ForceTorqueSensor() : _frame_id("")
	{
	}

	ForceTorqueSensor::~ForceTorqueSensor()
	{
	}

	ForceTorqueSensor::ForceTorqueSensor(const ForceTorqueSensor& ss)
		:Sensor(ss)
	{
	}

	::Eigen::MatrixXd ForceTorqueSensor::getCurrentValue() const
	{
        ::Eigen::MatrixXd forceTorque = _system->getForceTorqueMeasurement();

        if (this->_frame_id != "") {
            // transform the force torque into a relative frame
            if (forceTorque.rows() == 6 && forceTorque.cols() == 1) {
                ::Eigen::MatrixXd pose = this->_system->getFramePose(this->_frame_id);
                if (pose.rows() == 4 && pose.cols() == 4) {
                    // get inverse
                    //pose = pose.inverse();

                    // apply transformation

                    ::Eigen::MatrixXd rot = pose.topLeftCorner(3,3);
					rot = rot.inverse();
                    ::Eigen::Vector3d translation = pose.topRightCorner(3, 1);
                    ::Eigen::Vector3d force = forceTorque.topLeftCorner(3, 1);
                    ::Eigen::Vector3d torque = forceTorque.bottomLeftCorner(3, 1);

                    ::Eigen::MatrixXd ft(6,1);
                    // force
                    ft.topLeftCorner(3,1) = rot*force;
                    // torque
                    ft.bottomLeftCorner(3,1) = rot*torque; // + translation.cross(force);

					/*static int i = 0;
					if ( (i++ % 500) == 0)
					{
						std::cout << "ori: "<< forceTorque.transpose() << std::endl;
						std::cout << "rot: "<<ft.transpose() << std::endl;

					}*/

                    forceTorque = ft;


                } else {
                    static bool error_logged=false;
                    if (!error_logged) {
                        HA_ERROR("ForceTorqueSensor.getCurrentValue", "Pose reading for frame " << _frame_id <<  " has wrong dimensionality: "
                                 << pose.rows() << ", " << pose.cols() << " - expected 4.4");
                        error_logged = true;
                    }
                }


            } else {
                static bool error_logged=false;
                if (!error_logged) {
                    HA_ERROR("ForceTorqueSensor.getCurrentValue", "Force torque reading has wrong dimensionality: "
                             << forceTorque.rows() << ", " << forceTorque.cols() << " - expected 6,1");
                    error_logged = true;
                }
            }


        }

        return forceTorque;
	}

	DescriptionTreeNode::Ptr ForceTorqueSensor::serialize(const DescriptionTree::ConstPtr& factory) const
	{
		DescriptionTreeNode::Ptr tree = factory->createNode("Sensor");

		tree->setAttribute<std::string>(std::string("type"), this->getType());

        tree->setAttribute<std::string>(std::string("frame_id"), _frame_id);
        return tree;
	}

	void ForceTorqueSensor::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "Sensor") {
			HA_THROW_ERROR("ForceTorqueSensor.deserialize", "DescriptionTreeNode must have type 'Sensor', not '" << tree->getType() << "'!");
		}
		tree->getAttribute<std::string>("type", _type, "");

        tree->getAttribute<std::string>("frame_id", _frame_id, "");

		if (_type == "" || !HybridAutomaton::isSensorRegistered(_type)) {
			HA_THROW_ERROR("ForceTorqueSensor.deserialize", "SensorType type '" << _type << "' "
				<< "invalid - empty or not registered with HybridAutomaton!");
		}

		_system = system;
	}
}
