#ifndef HYBRID_AUTOMATON_SYSTEM_H_
#define HYBRID_AUTOMATON_SYSTEM_H_

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

		System() {
		}

		virtual ~System() {
		}

		virtual int getDof() const = 0;
		virtual ::Eigen::MatrixXd getConfiguration() const = 0;
		virtual ::Eigen::MatrixXd getForceTorqueMeasurement() const = 0;
		virtual ::Eigen::MatrixXd getFramePose(const std::string& frame_id) const = 0;

    virtual ::Eigen::MatrixXd getROSTfPose(const std::string& child) const {
      return getROSTfPose(child, "");
    }
    virtual ::Eigen::MatrixXd getROSTfPose(const std::string& child, const std::string& parent) const = 0;

  };

}

#endif // HYBRID_AUTOMATON_SYSTEM_H_
