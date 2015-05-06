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
                    // apply transformation
                    // FIXME were are only changing the orientation!
                    ::Eigen::MatrixXd rot = pose.topLeftCorner(3,3);
                    forceTorque.topLeftCorner(3,1) = rot*forceTorque.topLeftCorner(3, 1);
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
