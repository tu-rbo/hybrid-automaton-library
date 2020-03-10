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
#ifndef HYBRID_AUTOMATON_SYSTEM_H_
#define HYBRID_AUTOMATON_SYSTEM_H_

#include "hybrid_automaton/error_handling.h"

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>


#define DEFAULT_FT_PORT 0


namespace ha {

	class System;
	typedef boost::shared_ptr<System> SystemPtr;
	typedef boost::shared_ptr<const System> SystemConstPtr;

	class System {

	protected:

	public:
		typedef boost::shared_ptr<System> Ptr;
		typedef boost::shared_ptr<const System> ConstPtr;

        /**
        * @brief An interface to your robot system
        *
        * Overload all virtual functions to connect to your hardware
        */
		System() {
		}

		virtual ~System() {
		}

        /**
        * @brief Return the number degrees of freedom of your system
        */
		virtual int getDof() const = 0;

        /**
        * @brief Return the current joint configuration vector (dimx1)
        */
		virtual ::Eigen::MatrixXd getJointConfiguration() const = 0;

        /**
        * @brief Return the current joint velocity vector (dimx1)
        */
		virtual ::Eigen::MatrixXd getJointVelocity() const = 0;

        /**
        * @brief Return the current Force-torque measurement of your sensor (6x1)
        */
		virtual ::Eigen::MatrixXd getForceTorqueMeasurement(const int& port = DEFAULT_FT_PORT) const = 0;
		
	    /**
        * @brief Return the current contact-classification measurement of your acoustic-finger-sensor (6x1)
                 6-Classes and one finger are assumed in the moment.
        */
		virtual ::Eigen::MatrixXd getAcousticSensorMeasurement(int n_fingers, int n_classes) const = 0;

        /**
        * @brief Return the current strain-deformation-prediction of your strain-sensorized-finger-sensor (3x1)
                 3-values that represent deformation in flexional, lateral and twist direction
                 additionally one finger is assumed in the moment.
        */
		virtual ::Eigen::MatrixXd getStrainSensorMeasurement() const = 0;

        /**
        * @brief Return the pose of a frame with id \a frame_id (4x4)
        */
	virtual ::Eigen::MatrixXd getFramePose(const std::string& frame_id) const = 0;

	virtual bool subscribeToROSMessage(const std::string& topic) const {
		HA_THROW_ERROR("System.subscribeToROSMessage", "Not implemented");
	}
	virtual bool subscribeToTransform(const std::string& frame, const std::string& parent) const {
		HA_THROW_ERROR("System.subscribeToTransform", "Not implemented");
	}

	virtual bool isROSTopicAvailable(const std::string& topic_name) const  {
		HA_THROW_ERROR("System.isROSTopicAvailable", "Not implemented");
	}

	virtual bool isROSTopicUpdated(const std::string& topic_name) const  {
		HA_THROW_ERROR("System.isROSTopicUpdated", "Not implemented");
	}

	virtual bool getROSPose(const std::string& topic_name, const std::string& topic_type, ::Eigen::MatrixXd& pose) const  {
		HA_THROW_ERROR("System.getROSPose", "Not implemented");
	}
	virtual bool getROSTfPose(const std::string& child, const std::string& parent, ::Eigen::MatrixXd& pose) const  {
		HA_THROW_ERROR("System.getROSTfPose", "Not implemented");
	}

  };

}

#endif // HYBRID_AUTOMATON_SYSTEM_H_
