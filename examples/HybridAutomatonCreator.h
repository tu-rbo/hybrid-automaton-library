/*
 * HybridAutomatonCreator.h
 *
 *  Created on: Feb 9, 2015
 *      Author: arne
 */

#ifndef HYBRIDAUTOMATONCREATOR_H_
#define HYBRIDAUTOMATONCREATOR_H_

#include "hybrid_automaton/HybridAutomaton.h"
#include "apc/ApcTask.h"

namespace apc{

class HybridAutomatonCreator{
public:
    /**
     * @brief Helper class to create hybrid automata from apc tasks.
     *
     * Most parameters are deifned in the constructor
     */
    HybridAutomatonCreator();

    /**
     * @brief Destructor
     *
     */
    ~HybridAutomatonCreator(){}

    /**
     * @brief Creates an initial HA that is immediately sent and that:
     * - Puts the robot in Gravity Compensation
     * - Switches off the Vacuum cleaner (if it is present)
     *
     * @param tool The tool used by the robot
     * @return ha::HybridAutomaton::Ptr Pointer to the generated HA
     */
    ha::HybridAutomaton::Ptr createInitialHybridAutomaton(int tool);

    /**
     * @brief Create an empty hybrid automaton which performs gravity compensation
     */
    ha::HybridAutomaton::Ptr createEmptyHybridAutomaton();

    /**
     * @brief Create a hybrid automaton that performs the given apc task
     */
    ha::HybridAutomaton::Ptr createHybridAutomatonFromTask(const apc::ApcTask &task, int tool, int localization, int shelf_tracking);

    /**
     * @brief Create a hybrid automaton that reaches the pre-grasp positions
     *  but neither reaches into the bins nor grasps the objects.
     *
     * The resulting HA is required for automated collection of training data
     * for object recognition.
     */
    ha::HybridAutomaton::Ptr createHybridAutomatonFromTaskWithoutGrasping(const apc::ApcTask &task, int tool, int localization, int shelf_tracking);


private:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// GENERATION OF CONTROL SETS
    ///

    /**
     * @brief Create a control set for joint controllers
     *
     * @param ctrl The controller in the controlset
     * @return ha::ControlSet::Ptr The generated control set
     */
    ha::ControlSet::Ptr _createJointSpaceControlSet(ha::Controller::Ptr ctrl);

    /**
     * @brief Create a control set for joint controllers for the whole body control
     *
     * @param ctrl_arm The controller for the arm in the controlset
     * @param ctrl_base The controller for the base in the controlset
     * @return ha::ControlSet::Ptr The generated control set
     *
     */
    ha::ControlSet::Ptr _createJointSpaceWholeBodyControlSet(ha::Controller::Ptr ctrl_arm, ha::Controller::Ptr ctrl_base);

    /**
     * @brief Create a control set for task space controllers
     *
     * @param ctrl The controller in the controlset
     * @param useBase True if the control set should move both base and arm
     * @return ha::ControlSet::Ptr The generated control set
     */
    ha::ControlSet::Ptr _createTaskSpaceControlSet(ha::Controller::Ptr ctrl, bool useBase);


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// GENERATION OF CONTROLLERS
    ///

    /**
     * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param goal the goal configuration
     * @param completionTime the desired time to arrive for the interpolator
     * @return ha::Controller::Ptr The generated controller
     */
    ha::Controller::Ptr _createJointSpaceController(std::string name, const Eigen::MatrixXd &goal, double completionTime);

    /**
     * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only arm)
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param goal The goal configuration
     * @param max_velocity The maximum velocity for the joints of the arm
     * @return ha::Controller::Ptr The generated controller
     */
    ha::Controller::Ptr _createJointSpaceArmController(std::string name, const Eigen::MatrixXd &goal, const Eigen::MatrixXd& max_velocity);

    /**
     * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only base)
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param topic The topic where the goal for the controller is published
     * @param max_velocity The maximum velocity for the base
     * @return ha::Controller::Ptr The generated controller
     */
    ha::Controller::Ptr _createJointSpaceBaseController(std::string name, const std::string& topic, const Eigen::MatrixXd& max_velocity);

    /**
     * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only base)
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param goal The goal configuration
     * @param max_velocity The maximum velocity for the base
     * @return ha::Controller::Ptr The generated controller
     */
    ha::Controller::Ptr _createJointSpaceBaseController(std::string name, const Eigen::MatrixXd& goal, const Eigen::MatrixXd& max_velocity);

    /**
     * @brief Create an interpolated joint space controller to follow a trajectory from the current robots position
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param goal The goal configurations - list of configurations
     * @param vel_max the maximum velocity for each joint
     * @return ha::Controller::Ptr The generated controller
     */
    ha::Controller::Ptr _createJointSpaceControllerMoreThanOneGoal(std::string name, const Eigen::MatrixXd &goals, const Eigen::MatrixXd vel_max);

    /**
     * @brief Create an interpolated task space controller to move the end-effector from the current robots position to a goal frame
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param pos the goal end-effector position
     * @param ori the goal end-effector orientation
     * @param completionTime the desired time to arrive for the interpolator
     */
    ha::Controller::Ptr _createTaskSpaceController(std::string name, const Eigen::MatrixXd &pos, const Eigen::MatrixXd &ori, double completionTime);

    /**
     * @brief Create an interpolated task space controller that receives several frames (via points) from a ROS topic
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param topic The topic name where the series of frames are published
     * @param max_displacement_velocity Maximum linear velocity of the controller in op space
     * @param max_rotational_velocity Maximum angular velocity of the controller in op space
     * @param relative_goal True if the goal(s) is relative to the current pose of the EE
     * @return ha::Controller::Ptr The generated controller
     */
    ha::Controller::Ptr _createTaskSpaceTrajController(std::string name, const std::string topic, double max_displacement_velocity, double max_rotational_velocity, bool relative_goal);

    /**
     * @brief Create an interpolated task space controller to move the end-effector from the current robots position to a goal frame given as a /tf frame
     *
     * The controller will continuously reinterpolate to the goal from the tf topic
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param frame the target tf frame - the goal will be queried relative to /base_link
     * @param max_displacement_velocity the maximal end-effector translational velocity in m/s
     * @param max_rotational_velocity the maximal end-effector rotational velocity in rad/s
     */
    ha::Controller::Ptr _createTaskSpaceControllerTF(std::string name, const std::string frame, double max_displacement_velocity, double max_rotational_velocity);

    /**
     * @brief Create an interpolated task space controller to move the end-effector from the current robots position to a goal frame given as a /tf frame
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param HT_trajectory The trajectory as a series of frames
     * @param max_displacement_velocity Maximum linear velocity of the controller in op space
     * @param max_rotational_velocity Maximum angular velocity of the controller in op space
     * @param relative_goal True if the goal(s) is relative to the current pose of the EE
     * @return ha::Controller::Ptr The generated controller
     */
    ha::Controller::Ptr CreateResamplingGraspController(std::string name, const Eigen::MatrixXd& HT_trajectory, double max_displacement_velocity, double max_rotational_velocity, bool relative_goal);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// GENERATION OF JUMP CONDITIONS
    ///

    /**
     * @brief Create a jump condition that checks if a joint space controller converges
     *
     * @param ctrl A pointer to the goal controller
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr _createJointSpaceConvergenceCondition(ha::ControllerConstPtr ctrl);

    /**
     * @brief Create a jump condition that checks if a joint space controller has zero velocity
     *
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr _createJointSpaceConvergenceWithZeroVelCondition();

    /**
     * @brief Create a jump condition that checks if an op space space controller for the arm converges
     *
     * @param ctrl A pointer to the goal arm controller
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr _createJointSpaceArmConvergenceCondition(ha::ControllerConstPtr ctrl);

    /**
     * @brief Create a jump condition that checks if an op space controller for the arm has zero velocity
     *
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr _createJointSpaceArmConvergenceWithZeroVelCondition();

    /**
     * @brief Create a jump condition that checks if an op space space controller for the base converges
     *
     * @param ctrl A pointer to the goal base controller
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr _createJointSpaceBaseConvergenceCondition(ha::ControllerConstPtr ctrl);

    /**
     * @brief Create a jump condition that checks if an op space controller for the base has zero velocity
     *
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr _createJointSpaceBaseConvergenceWithZeroVelCondition();

    /**
     * @brief Create a jump condition that checks if the base is too close to the dropping bin (this distance
     * is published in a topic as a double number)
     *
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr _createDistanceToDroppingBinCondition();

    /**
     * @brief Create a jump condition that checks for the passing of a certain time
     *
     * @param max_time Time for the condition to trigger
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr _createMaxTimeCondition(double max_time);

    /**
     * @brief Create a jump condition that checks if a task space controller converges
     *
     * @param ctrl a pointer to the goal controller
     */
    ha::JumpCondition::Ptr _createTaskSpaceConvergenceCondition(ha::ControllerConstPtr ctrl, bool relative = false);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// GENERATION OF CONTROL SWITCHES
    ///

    /**
     * @brief Create a control switch that triggers when some max time is exceeded
     *
     * @param mode_name Name of the mode. Used to generate a name for the CS
     * @param max_time Max time to wait before triggering
     * @return ha::ControlSwitch::Ptr The generated CS
     */
    ha::ControlSwitch::Ptr CreateMaxTimeControlSwitch(const std::string& mode_name, double max_time);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// GENERATION OF CONTROL MODES
    ///

    /**
     * @brief Create a control mode that commands the robot to gravity compensation
     *
     * @param cm_ptr The pointer of the generated CM
     * @param name The name for the control mode
     * @return bool
     */
    bool CreateGCCM(const ha::ControlMode::Ptr& cm_ptr, const std::string& name);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// GENERATION OF CONTROL MODES AND CONTROL SWITCHES
    ///

    /**
     * @brief Create a CM to go to home position (joint space - arm+base) and a CS for its convergence
     *
     * @param cm_ptr Pointer to the resulting CM
     * @param cs_ptr Pointer to the resulting CS
     * @param name Name used as root for the CM and the CS
     * @param goal_cfg Goal of the CM and the convergence
     * @return bool
     */
    bool CreateGoToHomeCMAndConvergenceCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name, const Eigen::MatrixXd& goal_cfg);

    /**
     * @brief Create a CM to go to the viewing position (joint space - arm+base) and a CS for its convergence (both position and zero velocity)
     *
     * @param cm_ptr Pointer to the resulting CM
     * @param cs_ptr Pointer to the resulting CS
     * @param name Name used as root for the CM and the CS
     * @param localization The map used for localization (viewing poses depend on it)
     * @param shelf_tracking The model used for tracking (viewing poses depend on it)
     * @param bin_id The bin that has to be observed (viewing poses depend on it)
     * @return bool
     */
    bool CreateGoToViewCMAndConvergenceCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& cs_ptr,
                                          const std::string& name, int localization, int shelf_tracking, int bin_id);

    /**
     * @brief Create a CM to grasp (close the hand or activate vacuum cleaner + joint space of the arm to maintain pose) and a CS that indicates successful grasp
     * (waits 3 seconds AND if the tool is the vacuum cleaner, the pressure should go over a threshold)
     *
     * @param cm_ptr Pointer to the resulting CM
     * @param cs_ptr Pointer to the resulting CS
     * @param name Name used as root for the CM and the CS
     * @param tool The type of tool used (vacuum cleaner, soft hand or no tool)
     * @return bool
     */
    bool CreateGraspCMAndCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name, int tool);

    /**
     * @brief Create a CM to ungrasp (open the hand or deactivate vacuum cleaner + joint space of the arm to maintain pose) and a CS that indicates successful ungrasp
     * (waits 15 seconds OR the pressure should go under a threshold - only for vacuum cleaner!)
     *
     * @param cm_ptr Pointer to the resulting CM
     * @param cs_ptr Pointer to the resulting CS1 (pressure under threshold)
     * @param cs_ptr2 Pointer to the resulting CS2 (15 seconds)
     * @param name Name used as root for the CM and the CSs
     * @param tool The type of tool used (vacuum cleaner, soft hand or no tool)
     * @return bool
     */
    bool CreateUngraspCMAndCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& cs_ptr, const ha::ControlSwitch::Ptr& cs_ptr2, const std::string& name, int tool);

    /**
     * @brief Create a CM to move to a frame defined in a ROS tf or a topic (op space - arm+base) and a CS that indicates convergence
     *
     * @param cm_ptr Pointer to the resulting CM
     * @param cs_ptr Pointer to the resulting CS
     * @param name Name used as root for the CM and the CS
     * @param frame_name Name of the frame (or the topic) to be listened to from ROS
     * @param lin_vel Linear velocity for the interpolation
     * @param ang_vel Angular velocity for the interpolation
     * @param relative True if the goal is relative to the current pose
     * @param useTf True if the goal is a tf, false if the goal is a set of frames published in a topic
     * @param useBase True if we want to move the base
     * @return bool
     */
    bool CreateGoToBBCMAndConvergenceCS(const ha::ControlMode::Ptr& cm_ptr,
                                        const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name, const std::string& frame_name, double lin_vel, double ang_vel, bool relative, bool useTf, bool useBase);

    /**
     * @brief Create a CS for the case that the end-effector adheres to the shelf (force/torque limit when retracting AND the distance to the initial pose should be small - the
     * latter avoids thinking that the EE is adhered to the shelf when it is far away but carrying a heavy object), a CM for this case that sets the robot in gravity compensation
     * and switches off the vacuum cleaner, and a final CS to check that the detaching from the shelf is finished (time condition + pressure should be under a threshold)
     *
     * @param cs_ptr Pointer to the first CS that triggers if the vacuum cleaner is stucked
     * @param cm_ptr Pointer to the CM that ungrasps
     * @param cs2_ptr Pointer to the resulting CS for time after ungrasping
     * @param cs3_ptr Pointer to the resulting CS for pressure threshold after grasping
     * @param name Name used as root for the CM and the CSs
     * @param force_threshold Threshold in the force torque signal to detect that the EE is adhered
     * @param tool The type of tool used (vacuum cleaner, soft hand or no tool)
     * @return bool
     */
    bool CreateAdherenceCSAndCMAndTimeCS(const ha::ControlSwitch::Ptr& cs_ptr, const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& cs2_ptr, const ha::ControlSwitch::Ptr& cs3_ptr, const std::string& name, int force_threshold, int tool );

    /**
     * @brief Create CM to move the arm for the grasping motion (and activates the vacuum cleaner already if the object is the only object in the bin) and the CS
     * that triggers when the arm presses on something
     *
     * @param cm_ptr Pointer to the CM that moves the EE to grasp
     * @param cs_ptr Pointer to the CS that triggers when the EE presses on something
     * @param name Name used as root for the CM and the CS
     * @param frame_name Name of the frame (or the topic) to be listened to from ROS
     * @param lin_vel Linear velocity for the interpolation
     * @param ang_vel Angular velocity for the interpolation
     * @param relative True if the goal is relative to the current pose
     * @param useTf True if the goal is a tf, false if the goal is a set of frames published in a topic
     * @param useBase True if we want to move the base
     * @param num_other_objs Number of objects in the bin (to know if we already activate the vacuum cleaner during the motion or we wait until we touch something)
     * @param tool The type of tool used (vacuum cleaner, soft hand or no tool)
     * @return bool
     */
    bool CreateGoToBBGraspCMAndCS(const ha::ControlMode::Ptr& cm_ptr,
                                  const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name, const std::string& frame_name, double lin_vel, double ang_vel, bool relative, bool useTf, bool useBase, int num_other_objs, int tool);

    /**
     * @brief Create a CM to move to a frame defined in a ROS tf (op space - arm+base) and a CS that indicates convergence
     *
     * @param cm_ptr Pointer to the resulting CM
     * @param cs_ptr Pointer to the resulting CS
     * @param name Name used as root for the CM and the CS
     * @param frame_name Name of the frame to be listened to from ROS
     * @param lin_vel Linear velocity for the interpolation
     * @param ang_vel Angular velocity for the interpolation
     * @param useBase True if we want to move the base
     * @return bool
     */
    bool CreateGoToTfCMAndConvergenceCS(const ha::ControlMode::Ptr& cm_ptr,
                                        const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name, const std::string& frame_name, double lin_vel, double ang_vel, bool useBase);

    /**
     * @brief Create a CM that maintains the current pose (joint space) and calls the object recognition service (the name of the CM triggers the call), and the CSs that listens to a ros topic
     * that indicates when the service has returned either successfully or unsuccessfully
     *
     * @param cm_ptr Pointer to the resulting CM
     * @param cs_success_ptr Pointer to the resulting CS that triggers if ObjRec succeeds
     * @param cs_failure_ptr Pointer to the resulting CS that triggers if ObjRec fails
     * @param name Name used as root for the CM and the CS
     * @return bool
     */
    bool CreateRunObjRecCMAndCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& cs_success_ptr,const ha::ControlSwitch::Ptr& cs_failure_ptr, const std::string& name);

    /**
     * @brief Create a CM that maintains the current pose (joint space) and stops the object recognition service (the name of the CM triggers the call),
     * and the CS that indicates that enough time passed since the
     * service stop command
     *
     * @param cm_ptr Pointer to the resulting CM
     * @param cs_ptr Pointer to the resulting CS that triggers if enough time passed since the service stop command was sent
     * @param name Name used as root for the CM and the CS
     * @return bool
     */
    bool CreateStopObjRecCMAndCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name);

    /**
     * @brief Create a CM that moves the arm (joint space) and stops the object recognition service (the name of the CM triggers the call),
     * and the CS that indicates the convergence of the motion
     *
     * @param cm_ptr Pointer to the resulting CM
     * @param cs_ptr Pointer to the resulting CS that triggers if the motion converged
     * @param name Name used as root for the CM and the CS
     * @return bool
     */
    bool CreateStopObjRecAndGoToPrehomeCMAndCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name);

    /**
     * @brief Create a CS that triggers the resampling if the pressure is too low after the grasping motion, the CM for the resampling, that moves around the first pose (op space - arm)
     * and the CSs that trigger if the robot grasps something (change in the pressure) or if a max time is spent (failure)
     *
     * @param cm_ptr Pointer to the resulting CM
     * @param to_resampling_cs_ptr Pointer to the resulting CS that triggers if the initial grasp failed and starts the resampling
     * @param successful_resampling_grasp_cs_ptr Pointer to the resulting CS that triggers if the resampling succeeds (pressure over a threshold)
     * @param failed_resampling_grasp_cs_ptr Pointer to the resulting CS that triggers if the resampling fails (time over a threshold)
     * @param name Name used as root for the CM and the CS
     * @param grasp_object_name Name of the object to grasp, to have different resampling motions (more extreme) for certain objects
     * @return bool
     */
    bool CreateResamplingGraspCMtoandfromCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& to_resampling_cs_ptr,  const ha::ControlSwitch::Ptr& successful_resampling_grasp_cs_ptr, const ha::ControlSwitch::Ptr& failed_resampling_grasp_cs_ptr, const std::string& name, std::string grasp_object_name);

    /**
     * @brief Create a CS that triggers the wobbling if the pressure is too low after the grasping motion, the CM for the wobbling, that rotates the EE (joint space - arm)
     * and the CSs that trigger if the robot grasps something (change in the pressure) or if a max time is spent (failure)
     *
     * @param cm_ptr Pointer to the resulting CM
     * @param cs_ptr Pointer to the resulting CS that triggers if the initial grasp failed and starts the resampling
     * @param cs_ptr_2 Pointer to the resulting CS that triggers if the resampling succeeds (pressure over a threshold)
     * @param cs_ptr_3 Pointer to the resulting CS that triggers if the resampling fails (time over a threshold)
     * @param name Name used as root for the CM and the CS
     * @return bool
     */
    bool CreateWobbleCMandtoandfromCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& cs_ptr, const ha::ControlSwitch::Ptr& cs_ptr_2,  const ha::ControlSwitch::Ptr& cs_ptr_3, const std::string& name);

    /**
     * @brief Create a CM to go to an intermediate safe pose (joint space - only arm) after grasping, a CS for its convergence, another CM to go to the dropping bin
     * after the intermediate pose (joint space trajectory, 2 poses for arm; tf defined op space pose for base) and a CS that triggers because convergence
     *
     * @param cm1_ptr Pointer to the resulting CM to go to the intermediate safe pose
     * @param cs1_ptr Pointer to the resulting CS that triggers when the robot is in the intermediate pose
     * @param cm2_ptr Pointer to the resulting CM to go to the dropping bin
     * @param cs2_ptr Pointer to the resulting CS that triggers when the robot converges to the dropping pose
     * @param name Name used as root for the CM and the CS
     * @return bool
     */
    bool CreateJointSpaceTrajectoryToBinCMandCS(const ha::ControlMode::Ptr& cm1_ptr, const ha::ControlSwitch::Ptr& cs1_ptr,const ha::ControlMode::Ptr& cm2_ptr, const ha::ControlSwitch::Ptr& cs2_ptr, const std::string& name);


    ///index vectors for 10-dof control
    std::string _base_index_str;
    std::string _arm_index_str;
    std::vector<int> _base_index_vec;
    std::vector<int> _arm_index_vec;

    ///Maximum joint space velocity for arm
    Eigen::MatrixXd _max_js_vel_arm;

    ///Maximum velocity (x,y,theta) for base
    Eigen::MatrixXd _max_js_vel_base;

    /// joint space gains (7 dof)
    Eigen::MatrixXd _kp_jointspace;
    Eigen::MatrixXd _kv_jointspace;

    /// joint space gains for base controller (3 dof)
    Eigen::MatrixXd _kp_base_jointspace;
    Eigen::MatrixXd _kv_base_jointspace;

    /// opspace gains (3x orientation + 3x position)
    Eigen::MatrixXd _kp_opspace;
    Eigen::MatrixXd _kv_opspace;

    /// joint space integration gains for Nakamura control set (7 dof)
    Eigen::MatrixXd _kp_js_nakamura;
    Eigen::MatrixXd _kv_js_nakamura;
    ///weights for 10dof task space control
    Eigen::MatrixXd _joint_weights_nakamura;
    ///weights for 7dof task space control
    Eigen::MatrixXd _joint_weights_arm_nakamura;

    /// gains while grasping (more compliant)
    Eigen::MatrixXd _kp_grasp;
    Eigen::MatrixXd _kv_grasp;

    /// gains while droping
    Eigen::MatrixXd _kp_drop;
    Eigen::MatrixXd _kv_drop;

    /// gains while weighing
    Eigen::MatrixXd _kp_weigh;
    Eigen::MatrixXd _kv_weigh;


    /// home configuration - the robot will return here before all grasp attempts
    Eigen::MatrixXd _home_config;

    ///A good starting configuration for the lower bins
    Eigen::MatrixXd _home_config_low_bin;
    ///A good starting configuration for the higher bins
    Eigen::MatrixXd _home_config_high_bin;

    /// A configuration for a good viewing pose for each of the bins
    std::vector<Eigen::MatrixXd> _bin_view_positions_ikea_cb1;
    std::vector<Eigen::MatrixXd> _bin_view_positions_ikea_cb2;
    std::vector<Eigen::MatrixXd> _bin_view_positions_whiteshelf_cb1;
    std::vector<Eigen::MatrixXd> _bin_view_positions_whiteshelf_cb2;
    std::vector<Eigen::MatrixXd> _bin_view_positions_ikea_woodenshelf;
    std::vector<Eigen::MatrixXd> _bin_view_positions_kiva;

    /// convergence radius of a joint controller
    double _joint_epsilon;   
    double _joint_base_epsilon;

    double _joint_vel_epsilon;
    double _joint_base_vel_epsilon;

    ///convergence radius of an opspace controller
    double _op_epsilon;

    //The values below are only needed if USE_TF is false.

    /// The position in front of each bin
    std::vector<Eigen::MatrixXd> _bin_positions;

    /// The EE orientation while entering and leaving a bin
    Eigen::MatrixXd _bin_orientation;

    /// The EE orientation while grasping
    Eigen::MatrixXd _bin_orientation_grasp;

    /// The position of the dropping basket
    Eigen::MatrixXd _basket_position;
    Eigen::MatrixXd _basket_orientation;
};

}

#endif



