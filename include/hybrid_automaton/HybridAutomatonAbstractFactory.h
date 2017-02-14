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
#ifndef HYBRID_AUTOMATON_ABSTRACT_FACTORY_H
#define HYBRID_AUTOMATON_ABSTRACT_FACTORY_H

#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

/**
 * Things you have to implement in your own Factory:
 *
 * Control Sets:
 *  virtual ha::ControlSet::Ptr createJointSpaceControlSet(const HybridAutomatonAbstractParams& params,ha::Controller::Ptr ctrl) = 0;
 *  virtual ha::ControlSet::Ptr createJointSpaceControlSet(const HybridAutomatonAbstractParams& params,const std::vector<ha::Controller::Ptr>& ctrls) = 0;
 *  virtual ha::ControlSet::Ptr createTaskSpaceControlSet(const HybridAutomatonAbstractParams& params, ha::Controller::Ptr ctrl, bool move_base ) = 0;
 *
 * Controllers:
 *  virtual ha::Controller::Ptr createGraspController(const HybridAutomatonAbstractParams& params, std::string name) = 0;
 *  virtual ha::Controller::Ptr createJointSpaceController(const HybridAutomatonAbstractParams& params, std::string name, const Eigen::MatrixXd &goal_js, double completion_time, bool goal_relative) = 0;
 *  virtual ha::Controller::Ptr createSubjointSpaceController(const HybridAutomatonAbstractParams& params, std::string name, const Eigen::MatrixXd& goal_js, const Eigen::MatrixXd& index_vec, bool is_relative) = 0;
 *  virtual ha::Controller::Ptr createBBSubjointSpaceController(const HybridAutomatonAbstractParams& params, std::string name, bool use_tf, const std::string& topic_name, const std::string& tf_parent, const Eigen::MatrixXd& index_vec, bool is_relative) = 0;
 *  virtual ha::Controller::Ptr createBBSubjointSpaceControllerBase(const HybridAutomatonAbstractParams& params, const std::string name, bool use_tf, const std::string& topic_name, const std::string& tf_parent, bool is_relative) = 0;
 *  virtual ha::Controller::Ptr createOperationalSpaceController(const HybridAutomatonAbstractParams& params, std::string name, const Eigen::MatrixXd &goal_op_translation, const Eigen::MatrixXd &goal_op_rot_matrix, double completion_time, bool is_relative)= 0;
 *  virtual ha::Controller::Ptr createOperationalSpaceController(const HybridAutomatonAbstractParams& params, std::string name, const Eigen::MatrixXd &goal_op_translation, const Eigen::MatrixXd &goal_op_rot_matrix, bool is_relative)=0;
 *
 * Various:
 *  virtual HybridAutomatonAbstractFactory* _doClone() const = 0;
 */
namespace ha {

enum GripperType
{
    NO_GRIPPER,
    SOFT_HAND,
    SOFT_HAND_8CHANNELS,
    SUCTION_CUP,
    BARRETT_HAND,
    PR2_GRIPPER
};

class HybridAutomatonAbstractFactory;
typedef boost::shared_ptr<HybridAutomatonAbstractFactory> HybridAutomatonAbstractFactoryPtr;
typedef boost::shared_ptr<const HybridAutomatonAbstractFactory> HybridAutomatonAbstractFactoryConstPtr;

struct HybridAutomatonAbstractParams
{
    /// Grasp parameters
    GripperType gripper;
    double grasp_strength;
    int grasp_type;

    /// Indices that define the convention for the joints of the arm and the base
    std::string _index_str_base;
    std::string _index_str_arm;
    Eigen::MatrixXd _index_vec_base;
    Eigen::MatrixXd _index_vec_arm;

    /// Convergence radius for the velocity of a joint space (js) controller
    double _vel_epsilon_js_arm;
    double _vel_epsilon_js_base;

    //double pos_epsilon_js;
    //double vel_epsilon_js;

    /// Maximum joint space (os) velocity
    double _max_vel_os_linear;
    double _max_vel_os_angular;

    /// Convergence radius for the position of a joint space (js) controller
    double _pos_epsilon_js_arm;
    double _pos_epsilon_js_base;

    /// Convergence radius of an operational space (os) controller
    double _pos_epsilon_os_linear;
    double _pos_epsilon_os_angular;

    /// Convergence radius for the velocity of an operational space (os) controller
    double _vel_epsilon_os_linear;
    double _vel_epsilon_os_angular;

    /// Number of degrees of freedom of the arm
    int num_dof_arm;

    /// Number of degrees of freedom of the base
    int num_dof_base;

    /// Home configuration - usually a good initial position to begin the interaction and/or a safe
    /// position to return to
    Eigen::MatrixXd _home_config_js_arm;
    Eigen::MatrixXd _home_config_js_base;

};

/**
     * @brief A class to easily generate basic hybrid automata, control modes, control switches, jump conditions...
     *
     */
class HybridAutomatonAbstractFactory
{
public:

    typedef boost::shared_ptr<HybridAutomatonAbstractFactory> Ptr;
    typedef boost::shared_ptr<const HybridAutomatonAbstractFactory> ConstPtr;

    /**
         * @brief Default constructor
         *
         */
    HybridAutomatonAbstractFactory();

    /**
         * @brief Constructor
         *
         * @param num_dof_arm Number of degrees of freedom of the arm
         * @param num_dof_base Number of degrees of freedom of the base
         */
    HybridAutomatonAbstractFactory(const int& num_dof_arm, const int& num_dof_base);

    /**
         * @brief Destructor
         *
         */
    virtual ~HybridAutomatonAbstractFactory();

    /**
         * @brief Copy constructor
         *
         * @param haf Object to make a copy from
         */
    HybridAutomatonAbstractFactory(const HybridAutomatonAbstractFactory& haf);

    /**
         * @brief Clone function
         *
         * @return HybridAutomatonAbstractFactoryPtr Pointer to the generated clone
         */
    HybridAutomatonAbstractFactoryPtr clone() const
    {
        return (HybridAutomatonAbstractFactoryPtr(_doClone()));
    }

    /**
         * @brief Creates an initial HA that is immediately sent and that:
         * - Puts the robot in Gravity Compensation
         * - Switches off the Vacuum cleaner (if it is present)
         * - Opens the hand (if it is present)
         *
         * @param tool The tool used by the robot
         * @return ha::HybridAutomaton::Ptr Pointer to the generated HA
         */
    ha::HybridAutomaton::Ptr createInitialHybridAutomaton(const HybridAutomatonAbstractParams& params);

    /**
         * @brief Create an empty hybrid automaton which performs gravity compensation
         */
    ha::HybridAutomaton::Ptr createEmptyHybridAutomaton();

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
    virtual ha::ControlSet::Ptr createControlSet(const HybridAutomatonAbstractParams& params,ha::Controller::Ptr ctrl) = 0;

    virtual ha::ControlSet::Ptr createControlSet(const HybridAutomatonAbstractParams& params,const std::vector<ha::Controller::Ptr>& ctrls) = 0;

    /**
         * @brief Create a control set for task space controllers
         *
         * @param ctrl The controller in the controlset
         * @param move_base True if the control set should move both base and arm
         * @return ha::ControlSet::Ptr The generated control set
         */
    virtual ha::ControlSet::Ptr createTaskSpaceControlSet(const HybridAutomatonAbstractParams& params,
                                                           ha::Controller::Ptr ctrl,
                                                           bool move_base ) = 0;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// GENERATION OF CONTROLLERS
    ///

    /**
     * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param goal the goal configuration
     * @param completion_time the desired time to arrive for the interpolator
     * @return ha::Controller::Ptr The generated controller
     */

    virtual ha::Controller::Ptr createGraspController(const HybridAutomatonAbstractParams& params,
                                                      std::string name) = 0;

    virtual ha::Controller::Ptr createJointSpaceController(const HybridAutomatonAbstractParams& params,
                                                           std::string name,
                                                           const Eigen::MatrixXd &goal_js,
                                                           double completion_time,
                                                           bool goal_relative) = 0;

    virtual ha::Controller::Ptr createSubjointSpaceController(const HybridAutomatonAbstractParams& params,
                                                              std::string name,
                                                              const Eigen::MatrixXd& goal_js,
                                                              const Eigen::MatrixXd& index_vec,
                                                              bool is_relative) = 0;

    virtual ha::Controller::Ptr createBBSubjointSpaceController(const HybridAutomatonAbstractParams& params,
                                                                std::string name,
                                                                bool use_tf,
                                                                const std::string& topic_name,
                                                                const std::string& tf_parent,
                                                                const Eigen::MatrixXd& index_vec,
                                                                bool is_relative) = 0;

    /**
     * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only arm)
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param goal The goal configuration
     * @param max_velocity The maximum velocity for the joints of the arm
     * @return ha::Controller::Ptr The generated controller
     */
    virtual ha::Controller::Ptr createSubjointSpaceControllerArm(const HybridAutomatonAbstractParams& params,
                                                                 const std::string name,
                                                                 const Eigen::MatrixXd& goal_js_arm,
                                                                 bool is_relative);

    /**
         * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only base)
         *
         * @param name The controller name - must be unique within a hybrid automaton
         * @param goal The goal configuration
         * @param max_velocity The maximum velocity for the base
         * @return ha::Controller::Ptr The generated controller
         */
    virtual ha::Controller::Ptr createSubjointSpaceControllerBase(const HybridAutomatonAbstractParams& params,
                                                                  const std::string name,
                                                                  const Eigen::MatrixXd& goal_js_base,
                                                                  bool is_relative);

    /**
      * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only base)
      *
      * @param name The controller name - must be unique within a hybrid automaton
      * @param topic The topic where the goal for the controller is published
      * @param max_velocity The maximum velocity for the base
      * @return ha::Controller::Ptr The generated controller
      */
    virtual ha::Controller::Ptr createBBSubjointSpaceControllerBase(const HybridAutomatonAbstractParams& params,
                                                                    const std::string name,
                                                                    bool use_tf,
                                                                    const std::string& topic_name,
                                                                    const std::string& tf_parent,
                                                                    bool is_relative) = 0;

    virtual ha::Controller::Ptr createOperationalSpaceController(const HybridAutomatonAbstractParams& params,
                                                                 std::string name,
                                                                 const Eigen::MatrixXd &goal_op_translation,
                                                                 const Eigen::MatrixXd &goal_op_rot_matrix,
                                                                 double completion_time,
                                                                 bool is_relative)= 0;

    virtual ha::Controller::Ptr createOperationalSpaceController(const HybridAutomatonAbstractParams& params,
                                                                 std::string name,
                                                                 const Eigen::MatrixXd &goal_op_translation,
                                                                 const Eigen::MatrixXd &goal_op_rot_matrix,
                                                                 bool is_relative)=0;

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
    virtual ha::Controller::Ptr createBBOperationalSpaceController(const HybridAutomatonAbstractParams& params,
                                                                   std::string name,
                                                                   bool trajectory,
                                                                   bool use_tf,
                                                                   const std::string frame,
                                                                   const std::string parent_frame,
                                                                   bool is_relative) = 0;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// GENERATION OF JUMP CONDITIONS
    ///


    virtual ha::JumpCondition::Ptr createJointSpaceConvergenceCondition(const HybridAutomatonAbstractParams& params,
                                                                ha::ControllerConstPtr js_ctrl,
                                                                double epsilon
                                                                );

    /**
     * @brief Create a jump condition that checks if a joint space controller converges
     *
     * @param ctrl A pointer to the goal controller
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    virtual ha::JumpCondition::Ptr createSubjointSpaceConvergenceCondition(const HybridAutomatonAbstractParams& params,
                                                                   ha::ControllerConstPtr subjs_ctrl,
                                                                   const Eigen::MatrixXd& index_vec,
                                                                   double epsilon
                                                                   //                                                                   ,
                                                                   //                                                                   const double& pos_epsilon_js
                                                                   );

    /**
     * @brief Create a jump condition that checks if an op space space controller for the arm converges
     *
     * @param ctrl A pointer to the goal arm controller
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    virtual ha::JumpCondition::Ptr createSubjointSpaceConvergenceConditionArm(const HybridAutomatonAbstractParams& params,
                                                                      ha::ControllerConstPtr subjs_ctrl
                                                                      //                                                                      ,
                                                                      //                                                                      const Eigen::MatrixXd& index_vec_arm = Eigen::MatrixXd(),
                                                                      //                                                                      const double& pos_epsilon_js_arm = -1
                                                                      );
    /**
     * @brief Create a jump condition that checks if an op space space controller for the base converges
     *
     * @param ctrl A pointer to the goal base controller
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    virtual ha::JumpCondition::Ptr createSubjointSpaceConvergenceConditionBase(const HybridAutomatonAbstractParams& params,
                                                                       ha::ControllerConstPtr subjs_ctrl
                                                                       //                                                                       ,
                                                                       //                                                                       const Eigen::MatrixXd& index_vec_base = Eigen::MatrixXd(),
                                                                       //                                                                       const double &pos_epsilon_js_base = -1
                                                                       );

    virtual ha::JumpCondition::Ptr createJointSpaceVelocityCondition(const HybridAutomatonAbstractParams& params,
                                                             const Eigen::MatrixXd& index_vec,
                                                             const Eigen::MatrixXd& vel_goal_js,
                                                             double epsilon
                                                             //                                                             ,
                                                             //                                                             const double& vel_epsilon_js = -1
                                                             );

    /**
     * @brief Create a jump condition that checks if a joint space controller has zero velocity
     *
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    virtual ha::JumpCondition::Ptr createJointSpaceZeroVelocityCondition(const HybridAutomatonAbstractParams& params,
                                                                 const Eigen::MatrixXd& index_vec,
                                                                 double epsilon
                                                                 //                                                                 ,
                                                                 //                                                                 const double& vel_epsilon_js = -1
                                                                 );

    /**
     * @brief Create a jump condition that checks if an op space controller for the arm has zero velocity
     *
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    virtual ha::JumpCondition::Ptr createJointSpaceZeroVelocityConditionArm(const HybridAutomatonAbstractParams& params
                                                                    //                                                                    const Eigen::MatrixXd& index_vec_arm = Eigen::MatrixXd(),
                                                                    //                                                                    const double& vel_epsilon_js_arm = -1
                                                                    );

    virtual ha::JumpCondition::Ptr createJointSpaceVelocityConditionArm(const HybridAutomatonAbstractParams& params,
                                                                const Eigen::MatrixXd& vel_goal_js_arm  = Eigen::MatrixXd()
            //                                                                   const Eigen::MatrixXd& index_vec_arm  = Eigen::MatrixXd(),
            //                                                                    const double& vel_epsilon_js_arm = -1
            );

    /**
     * @brief Create a jump condition that checks if an op space controller for the base has zero velocity
     *
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    virtual ha::JumpCondition::Ptr createJointSpaceZeroVelocityConditionBase(const HybridAutomatonAbstractParams& params);

    virtual ha::JumpCondition::Ptr createOperationalSpaceConvergenceCondition(const HybridAutomatonAbstractParams& params,
                                                                      ha::ControllerConstPtr ctrl,
                                                                      bool relative,
                                                                      bool only_displacement = false
            );


    virtual ha::JumpCondition::Ptr createMaxTimeCondition(const HybridAutomatonAbstractParams& params,
                                                  double max_time);

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
    virtual ha::ControlSwitch::Ptr CreateMaxTimeControlSwitch(const HybridAutomatonAbstractParams& params,
                                                      const std::string& mode_name,
                                                      double max_time);

    virtual ha::ControlSwitch::Ptr CreateMaxForceTorqueControlSwitch(const HybridAutomatonAbstractParams& params,
                                                             const std::string& name,
                                                             const Eigen::MatrixXd& ft_weights,
                                                             const Eigen::MatrixXd& ft_max_val,
                                                             const ha::JumpCondition::JumpCriterion ft_criterion,
                                                             const bool negate_ft_condition=false,
                                                             const float epsilon=0.0);

    virtual void CreateMaxForceTorqueControlSwitch(const HybridAutomatonAbstractParams& p,
                                           const ha::ControlSwitch::Ptr& cs_ptr,
                                           const std::string& name,
                                           const Eigen::MatrixXd& ft_weights,
                                           const Eigen::MatrixXd& ft_max_val,
                                           const ha::JumpCondition::JumpCriterion ft_criterion,
                                           const bool negate_ft_condition=false,
                                           const float epsilon = 0.0);

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
    virtual void CreateGCCM(const HybridAutomatonAbstractParams& p,
                    const ha::ControlMode::Ptr& cm_ptr, const std::string& name) = 0;

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
    virtual void CreateGoToHomeCMAndConvergenceCSArm(const HybridAutomatonAbstractParams& p,
                                             const ha::ControlMode::Ptr& cm_ptr,
                                             const ha::ControlSwitch::Ptr& cs_ptr,
                                             const std::string& name);

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
    virtual void CreateGraspCMAndCS(const HybridAutomatonAbstractParams& p,
                            const ha::ControlMode::Ptr& cm_ptr,
                            const ha::ControlSwitch::Ptr& cs_ptr,
                            const std::string& name
//                            ,
//                            const GripperType& gripper,
//                            const Eigen::MatrixXd& kp_grasp = Eigen::MatrixXd(),
//                            const Eigen::MatrixXd& kv_grasp = Eigen::MatrixXd(),
//                            //softhand
//                            const double grasp_strength=0.2,
//                            const int graps_type=4
            );

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
    //    void CreateUngraspCMAndCS(const HybridAutomatonAbstractParams& p,
    //                              const ha::ControlMode::Ptr& cm_ptr,
    //                              const ha::ControlSwitch::Ptr& cs_ptr,
    //                              const ha::ControlSwitch::Ptr& cs_ptr2,
    //                              const std::string& name,
    //                              const GripperType& gripper,
    //                              const Eigen::MatrixXd& kp_drop = Eigen::MatrixXd(),
    //                              const Eigen::MatrixXd& kv_drop = Eigen::MatrixXd(),
    //                              const double& grasp_strength=4.0,
    //                              const int& grasp_type=0);

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
    virtual void CreateGoToBBCMAndConvergenceCS(const HybridAutomatonAbstractParams& p,
                                        const ha::ControlMode::Ptr& cm_ptr,
                                        const ha::ControlSwitch::Ptr& cs_ptr,
                                        const std::string& name,
                                        const std::string& frame_name,
                                        const std::string& parent_frame_name,
                                        bool use_tf,
                                        bool use_base,
                                        bool is_relative);

    virtual void CreateGoToCMAndConvergenceCS(const HybridAutomatonAbstractParams& p,
                                      const ha::ControlMode::Ptr& cm_ptr,
                                      const ha::ControlSwitch::Ptr& cs_ptr,
                                      const std::string& name,
                                      const Eigen::MatrixXd &goal_op_pos,
                                      const Eigen::MatrixXd &goal_op_ori,
                                      bool use_base,
                                      bool is_relative);

    virtual void CreateGoToCMConvergenceCSAndMaxForceCS(const HybridAutomatonAbstractParams& p,
                                                const ha::ControlMode::Ptr& cm_ptr,
                                                const ha::ControlSwitch::Ptr& convergence_cs_ptr,
                                                const ha::ControlSwitch::Ptr& max_force_cs_ptr,
                                                const std::string& name,
                                                const Eigen::MatrixXd &goal_op_pos,
                                                const Eigen::MatrixXd &goal_op_ori,
                                                const Eigen::MatrixXd &ft_weights,
                                                const Eigen::MatrixXd &max_ft,
                                                const ha::JumpCondition::JumpCriterion ft_criterion,
                                                bool use_base,
                                                bool negate_ft_condition,
                                                double ft_epsilon,
                                                bool is_relative);



    /**
     * @brief
     *
     * @param ha
     * @return std::string
     */
    std::string HybridAutomatonToString(ha::HybridAutomaton::ConstPtr ha);

protected:

    /**
         * @brief Performs the cloning operation (this solves some issues of inheritance and smart pointers)
         *
         * @return HybridAutomatonAbstractFactory Pointer to the generated clone
         */
    virtual HybridAutomatonAbstractFactory* _doClone() const = 0;

    /**
         * @brief Concatenates two vectors, one for the arm and one for the base
         *
         * @param arm_vector Vector for the arm. It will be the first _num_dof_arm values of the resulting vector
         * @param base_vector Vector for the base. It will be the last _num_dof_base of the resulting vector
         * @return Eigen::MatrixXd Resulting vector that contains both input vectors
         */
    virtual Eigen::MatrixXd _combineArmAndBase(const Eigen::MatrixXd& arm_vector, const Eigen::MatrixXd base_vector);


};

}

#endif
