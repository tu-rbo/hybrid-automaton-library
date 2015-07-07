#ifndef HYBRID_AUTOMATON_SYSTEM_H_
#define HYBRID_AUTOMATON_SYSTEM_H_

#include "hybrid_automaton/error_handling.h"

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

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
        * @brief Return the current Force-torque mieasurement of your sensor (6x1)
        */
		virtual ::Eigen::MatrixXd getForceTorqueMeasurement(const std::string& frame_id = "ee") const = 0;

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
