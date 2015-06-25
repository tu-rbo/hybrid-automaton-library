#ifndef HYBRID_AUTOMATON_FACTORY_H
#define HYBRID_AUTOMATON_FACTORY_H

#include "hybrid_automaton/HybridAutomaton.h"

#include <boost/shared_ptr.hpp>

namespace ha {

enum GripperType
{
    NO_GRIPPER,
    SOFT_HAND,
    SUCTION_CUP,
    BARRETT_HAND
};

class HybridAutomatonFactory;
typedef boost::shared_ptr<HybridAutomatonFactory> HybridAutomatonFactoryPtr;
typedef boost::shared_ptr<const HybridAutomatonFactory> HybridAutomatonFactoryConstPtr;

/**
     * @brief A class to easily generate basic hybrid automata, control modes, control switches, jump conditions...
     *
     */
class HybridAutomatonFactory
{
public:

    typedef boost::shared_ptr<HybridAutomatonFactory> Ptr;
    typedef boost::shared_ptr<const HybridAutomatonFactory> ConstPtr;

    /**
         * @brief Default constructor
         *
         */
    HybridAutomatonFactory();

    /**
         * @brief Constructor
         *
         * @param num_dof_arm Number of degrees of freedom of the arm
         * @param num_dof_base Number of degrees of freedom of the base
         */
    HybridAutomatonFactory(const int& num_dof_arm, const int& num_dof_base);

    /**
         * @brief Destructor
         *
         */
    virtual ~HybridAutomatonFactory();

    /**
         * @brief Copy constructor
         *
         * @param haf Object to make a copy from
         */
    HybridAutomatonFactory(const HybridAutomatonFactory& haf);

    /**
         * @brief Clone function
         *
         * @return HybridAutomatonFactoryPtr Pointer to the generated clone
         */
    HybridAutomatonFactoryPtr clone() const
    {
        return (HybridAutomatonFactoryPtr(_doClone()));
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
    ha::HybridAutomaton::Ptr createInitialHybridAutomaton(const GripperType& gripper,
                                                          const Eigen::MatrixXd& home_config_js_arm = Eigen::MatrixXd(),
                                                          const Eigen::MatrixXd& home_config_js_base = Eigen::MatrixXd(),
                                                          const Eigen::MatrixXd& max_vel_js_arm = Eigen::MatrixXd(),
                                                          const Eigen::MatrixXd& max_vel_js_base = Eigen::MatrixXd(),
                                                          const Eigen::MatrixXd& index_vec_arm = Eigen::MatrixXd(),
                                                          const double& pos_epsilon_js_arm = -1,
                                                          const double& grasp_strength=4.0,
                                                          const int& grasp_type=0);

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
    ha::ControlSet::Ptr createControlSet(ha::Controller::Ptr ctrl);

    ha::ControlSet::Ptr createControlSet(const std::vector<ha::Controller::Ptr>& ctrls);

    /**
         * @brief Create a control set for task space controllers
         *
         * @param ctrl The controller in the controlset
         * @param move_base True if the control set should move both base and arm
         * @return ha::ControlSet::Ptr The generated control set
         */
    ha::ControlSet::Ptr createTPNakamuraControlSet(ha::Controller::Ptr ctrl,
                                                                 bool move_base,
                                                                 const Eigen::MatrixXd &kp_js_nakamura_arm = Eigen::MatrixXd(),
                                                                 const Eigen::MatrixXd &kp_js_nakamura_base = Eigen::MatrixXd(),
                                                                 const Eigen::MatrixXd &kv_js_nakamura_arm = Eigen::MatrixXd(),
                                                                 const Eigen::MatrixXd &kv_js_nakamura_base = Eigen::MatrixXd(),
                                                                 const Eigen::MatrixXd &joint_weights_nakamura_arm = Eigen::MatrixXd(),
                                                                 const Eigen::MatrixXd &joint_weights_nakamura_base = Eigen::MatrixXd());


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
    ha::Controller::Ptr createJointSpaceController(std::string name,
                                                   const Eigen::MatrixXd &goal_js,
                                                   double completion_time,
                                                   const Eigen::MatrixXd &kp_js = Eigen::MatrixXd(),
                                                   const Eigen::MatrixXd &kv_js = Eigen::MatrixXd(),
                                                   bool goal_relative = false);

    ha::Controller::Ptr createSubjointSpaceController(std::string name,
                                                      const Eigen::MatrixXd& goal_js,
                                                      const Eigen::MatrixXd& max_velocity,
                                                      const Eigen::MatrixXd& index_vec,
                                                      const Eigen::MatrixXd& kp_js = Eigen::MatrixXd(),
                                                      const Eigen::MatrixXd& kv_js= Eigen::MatrixXd(),
                                                      bool is_relative=false);



    ha::Controller::Ptr createBBSubjointSpaceController(std::string name,
                                                        bool use_tf,
                                                        const std::string& topic_name,
                                                        const std::string& tf_parent,
                                                                                const Eigen::MatrixXd& max_velocity,
                                                                                const Eigen::MatrixXd& index_vec,
                                                                                const Eigen::MatrixXd& kp_js= Eigen::MatrixXd(),
                                                                                const Eigen::MatrixXd& kv_js= Eigen::MatrixXd(),
                                                                                bool is_relative=false,
                                                                                int update_rate=-1);

    /**
     * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only arm)
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param goal The goal configuration
     * @param max_velocity The maximum velocity for the joints of the arm
     * @return ha::Controller::Ptr The generated controller
     */
    ha::Controller::Ptr createSubjointSpaceControllerArm(std::string name,
                                                         const Eigen::MatrixXd& goal_js_arm,
                                                         const Eigen::MatrixXd& max_vel_js_arm = Eigen::MatrixXd(),
                                                         const Eigen::MatrixXd& index_vec_arm = Eigen::MatrixXd(),
                                                         const Eigen::MatrixXd& kp_js_arm= Eigen::MatrixXd(),
                                                         const Eigen::MatrixXd& kv_js_arm= Eigen::MatrixXd(),
                                                         bool is_relative =false);

    /**
         * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only base)
         *
         * @param name The controller name - must be unique within a hybrid automaton
         * @param goal The goal configuration
         * @param max_velocity The maximum velocity for the base
         * @return ha::Controller::Ptr The generated controller
         */
    ha::Controller::Ptr createSubjointSpaceControllerBase(std::string name,
                                                          const Eigen::MatrixXd& goal_js_base,
                                                          const Eigen::MatrixXd& max_vel_js_base = Eigen::MatrixXd(),
                                                          const Eigen::MatrixXd& index_vec_base = Eigen::MatrixXd(),
                                                          const Eigen::MatrixXd& kp_js_base= Eigen::MatrixXd(),
                                                          const Eigen::MatrixXd& kv_js_base= Eigen::MatrixXd(),
                                                          bool is_relative =false);

    /**
      * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only base)
      *
      * @param name The controller name - must be unique within a hybrid automaton
      * @param topic The topic where the goal for the controller is published
      * @param max_velocity The maximum velocity for the base
      * @return ha::Controller::Ptr The generated controller
      */
    ha::Controller::Ptr createBBSubjointSpaceControllerBase(std::string name,
                                                            bool use_tf,
                                                            const std::string& topic_name,
                                                            const std::string& tf_parent = std::string("odom"),
                                                            const Eigen::MatrixXd& max_vel_js_base = Eigen::MatrixXd(),
                                                            const Eigen::MatrixXd& index_vec_base = Eigen::MatrixXd(),
                                                            const Eigen::MatrixXd& kp_js_base = Eigen::MatrixXd(),
                                                            const Eigen::MatrixXd& kv_js_base = Eigen::MatrixXd(),
                                                            bool is_relative = false,
                                                            int update_rate = 100);

    ha::Controller::Ptr createOperationalSpaceController(std::string name,
                                                         const Eigen::MatrixXd &goal_op_pos,
                                                         const Eigen::MatrixXd &goal_op_ori,
                                                         double completion_time,
                                                         const Eigen::MatrixXd &kp_os_linear = Eigen::MatrixXd(),
                                                         const Eigen::MatrixXd &kp_os_angular = Eigen::MatrixXd(),
                                                         const Eigen::MatrixXd &kv_os_linear = Eigen::MatrixXd(),
                                                         const Eigen::MatrixXd &kv_os_angular = Eigen::MatrixXd(),
                                                         bool is_relative = false);

    ha::Controller::Ptr createOperationalSpaceController(std::string name,
                                                         const Eigen::MatrixXd &goal_op_pos,
                                                         const Eigen::MatrixXd &goal_op_ori,
                                                         double max_vel_os_linear = -1,
                                                         double max_vel_os_angular = -1,
                                                         const Eigen::MatrixXd &kp_os_linear = Eigen::MatrixXd(),
                                                         const Eigen::MatrixXd &kp_os_angular = Eigen::MatrixXd(),
                                                         const Eigen::MatrixXd &kv_os_linear = Eigen::MatrixXd(),
                                                         const Eigen::MatrixXd &kv_os_angular = Eigen::MatrixXd(),
                                                         bool is_relative = false);

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
    ha::Controller::Ptr createBBOperationalSpaceController(std::string name,
                                                           bool trajectory,
                                                           bool use_tf,
                                                           double max_vel_os_linear,
                                                           double max_vel_os_angular,
                                                           const std::string frame,
                                                           const std::string parent_frame="/odom",
                                                           const Eigen::MatrixXd &kp_os_linear = Eigen::MatrixXd(),
                                                           const Eigen::MatrixXd &kp_os_angular = Eigen::MatrixXd(),
                                                           const Eigen::MatrixXd &kv_os_linear = Eigen::MatrixXd(),
                                                           const Eigen::MatrixXd &kv_os_angular = Eigen::MatrixXd(),
                                                           bool is_relative = false,
                                                           int update_rate = 50);


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// GENERATION OF JUMP CONDITIONS
    ///


    ha::JumpCondition::Ptr createJointSpaceConvergenceCondition(ha::ControllerConstPtr js_ctrl,
                                                                              const double &pos_epsilon_js);

    /**
     * @brief Create a jump condition that checks if a joint space controller converges
     *
     * @param ctrl A pointer to the goal controller
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr createSubjointSpaceConvergenceCondition(ha::ControllerConstPtr subjs_ctrl,
                                                                   const Eigen::MatrixXd& index_vec,
                                                                   const double& pos_epsilon_js);

    /**
     * @brief Create a jump condition that checks if an op space space controller for the arm converges
     *
     * @param ctrl A pointer to the goal arm controller
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr createSubjointSpaceConvergenceConditionArm(ha::ControllerConstPtr subjs_ctrl,
                                                                   const Eigen::MatrixXd& index_vec_arm = Eigen::MatrixXd(),
                                                                   const double& pos_epsilon_js_arm = -1);
    /**
     * @brief Create a jump condition that checks if an op space space controller for the base converges
     *
     * @param ctrl A pointer to the goal base controller
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr createSubjointSpaceConvergenceConditionBase(ha::ControllerConstPtr subjs_ctrl,
                                                                       const Eigen::MatrixXd& index_vec_base = Eigen::MatrixXd(),
                                                                       const double &pos_epsilon_js_base = -1);

    ha::JumpCondition::Ptr createJointSpaceVelocityCondition(const Eigen::MatrixXd& index_vec,
                                                                            const Eigen::MatrixXd& vel_goal_js,
                                                                            const double& vel_epsilon_js = -1);

    /**
     * @brief Create a jump condition that checks if a joint space controller has zero velocity
     *
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr createJointSpaceZeroVelocityCondition(const Eigen::MatrixXd& index_vec,
                                                                                const double& vel_epsilon_js = -1);

    /**
     * @brief Create a jump condition that checks if an op space controller for the arm has zero velocity
     *
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr createJointSpaceZeroVelocityConditionArm(const Eigen::MatrixXd& index_vec_arm = Eigen::MatrixXd(),
                                                                                   const double& vel_epsilon_js_arm = -1);

    ha::JumpCondition::Ptr createJointSpaceVelocityConditionArm(const Eigen::MatrixXd& index_vec_arm  = Eigen::MatrixXd(),
                                                                                                       const Eigen::MatrixXd& vel_goal_js_arm  = Eigen::MatrixXd(),
                                                                                                       const double& vel_epsilon_js_arm = -1);

    /**
     * @brief Create a jump condition that checks if an op space controller for the base has zero velocity
     *
     * @return ha::JumpCondition::Ptr Pointer to the generated jump condition
     */
    ha::JumpCondition::Ptr createJointSpaceZeroVelocityConditionBase(const Eigen::MatrixXd &index_vec_base = Eigen::MatrixXd(),
                                                                                    const double& vel_epsilon_js_base =  -1);

    ha::JumpCondition::Ptr createOperationalSpaceConvergenceCondition(ha::ControllerConstPtr ctrl,
                                                                                              bool relative,
                                                                                              const double& pos_epsilon_os);


    ha::JumpCondition::Ptr createMaxTimeCondition(double max_time);

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
    void CreateGCCM(const ha::ControlMode::Ptr& cm_ptr, const std::string& name);

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
    void CreateGoToHomeCMAndConvergenceCSArm(const ha::ControlMode::Ptr& cm_ptr,
                                          const ha::ControlSwitch::Ptr& cs_ptr,
                                          const std::string& name,
                                          //Arguments to createSubjointSpaceControllerArm
                                          const Eigen::MatrixXd& goal_cfg,
                                          const Eigen::MatrixXd& max_vel_js_arm = Eigen::MatrixXd(),
                                          const Eigen::MatrixXd& index_vec_arm = Eigen::MatrixXd(),
                                          const Eigen::MatrixXd& kp_js_arm= Eigen::MatrixXd(),
                                          const Eigen::MatrixXd& kv_js_arm= Eigen::MatrixXd(),
                                          bool is_relative =false,
                                          //createSubjointSpaceConvergenceConditionArm
                                          const double& pos_epsilon_js_arm = -1,
                                          //createJointSpaceZeroVelocityConditionArm
                                          const double& vel_epsilon_js_arm = -1);

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
    void CreateGraspCMAndCS(const ha::ControlMode::Ptr& cm_ptr,
                            const ha::ControlSwitch::Ptr& cs_ptr,
                            const std::string& name,
                            const GripperType& gripper,
                            const Eigen::MatrixXd& kp_grasp,
                            const Eigen::MatrixXd& kv_grasp,
                            //softhand
                            const double grasp_strength=0.1,
                            const int graps_type=4);

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
    void CreateUngraspCMAndCS(const ha::ControlMode::Ptr& cm_ptr,
                              const ha::ControlSwitch::Ptr& cs_ptr,
                              const ha::ControlSwitch::Ptr& cs_ptr2,
                              const std::string& name,
                              const GripperType& gripper,
                              const Eigen::MatrixXd& kp_drop,
                              const Eigen::MatrixXd& kv_drop,
                              const double& grasp_strength=4.0,
                              const int& grasp_type=1);

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
    void CreateGoToBBCMAndConvergenceCS(const ha::ControlMode::Ptr& cm_ptr,
                                        const ha::ControlSwitch::Ptr& cs_ptr,
                                        const std::string& name,
                                        double max_vel_os_linear,
                                        double max_vel_os_angular,
                                        const double& pos_epsilon_os,
                                        const std::string& frame_name,
                                        const std::string& parent_frame_name,
                                        bool use_tf,
                                        bool use_base,
                                        bool is_relative = false,
                                        const Eigen::MatrixXd &kp_os_linear = Eigen::MatrixXd(),
                                        const Eigen::MatrixXd &kp_os_angular = Eigen::MatrixXd(),
                                        const Eigen::MatrixXd &kv_os_linear = Eigen::MatrixXd(),
                                        const Eigen::MatrixXd &kv_os_angular = Eigen::MatrixXd(),
                                        int update_rate = 50);

    void CreateGoToCMAndConvergenceCS(const ha::ControlMode::Ptr& cm_ptr,
                                      const ha::ControlSwitch::Ptr& cs_ptr,
                                      const std::string& name,
                                      const Eigen::MatrixXd &goal_op_pos,
                                      const Eigen::MatrixXd &goal_op_ori,
                                      double max_vel_os_linear,
                                      double max_vel_os_angular,
                                      const double& pos_epsilon_os,
                                      bool use_base,
                                      bool is_relative = false,
                                      const Eigen::MatrixXd &kp_os_linear = Eigen::MatrixXd(),
                                      const Eigen::MatrixXd &kp_os_angular = Eigen::MatrixXd(),
                                      const Eigen::MatrixXd &kv_os_linear = Eigen::MatrixXd(),
                                      const Eigen::MatrixXd &kv_os_angular = Eigen::MatrixXd());

    Eigen::MatrixXd max_vel_js_arm() const;
    void setMax_vel_js_arm(const Eigen::MatrixXd &max_vel_js_arm);

    Eigen::MatrixXd max_vel_js_base() const;
    void setMax_vel_js_base(const Eigen::MatrixXd &max_vel_js_base);

    Eigen::MatrixXd kp_js_arm() const;
    void setKp_js_arm(const Eigen::MatrixXd &kp_js_arm);

    Eigen::MatrixXd kp_js_base() const;
    void setKp_js_base(const Eigen::MatrixXd &kp_js_base);

    Eigen::MatrixXd kv_js_arm() const;
    void setKv_js_arm(const Eigen::MatrixXd &kv_js_arm);

    Eigen::MatrixXd kv_js_base() const;
    void setKv_js_base(const Eigen::MatrixXd &kv_js_base);

    Eigen::MatrixXd kp_os_linear() const;
    void setKp_os_linear(const Eigen::MatrixXd &kp_os_linear);

    Eigen::MatrixXd kp_os_angular() const;
    void setKp_os_angular(const Eigen::MatrixXd &kp_os_angular);

    Eigen::MatrixXd kv_os_linear() const;
    void setKv_os_linear(const Eigen::MatrixXd &kv_os_linear);

    Eigen::MatrixXd kv_os_angular() const;
    void setKv_os_angular(const Eigen::MatrixXd &kv_os_angular);

    Eigen::MatrixXd kp_js_nakamura_arm() const;
    void setKp_js_nakamura_arm(const Eigen::MatrixXd &kp_js_nakamura_arm);

    Eigen::MatrixXd kp_js_nakamura_base() const;
    void setKp_js_nakamura_base(const Eigen::MatrixXd &kp_js_nakamura_base);

    Eigen::MatrixXd kv_js_nakamura_arm() const;
    void setKv_js_nakamura_arm(const Eigen::MatrixXd &kv_js_nakamura_arm);

    Eigen::MatrixXd kv_js_nakamura_base() const;
    void setKv_js_nakamura_base(const Eigen::MatrixXd &kv_js_nakamura_base);

    Eigen::MatrixXd joint_weights_nakamura_arm() const;
    void setJoint_weights_nakamura_arm(const Eigen::MatrixXd &joint_weights_nakamura_arm);

    Eigen::MatrixXd joint_weights_nakamura_base() const;
    void setJoint_weights_nakamura_base(const Eigen::MatrixXd &joint_weights_nakamura_base);

    Eigen::MatrixXd joint_weights_nakamura_base_no_rotation() const;
    void setJoint_weights_nakamura_base_no_rotation(const Eigen::MatrixXd &joint_weights_nakamura_base_no_rotation);

    Eigen::MatrixXd joint_weights_nakamura_base_little_motion() const;
    void setJoint_weights_nakamura_base_little_motion(const Eigen::MatrixXd &joint_weights_nakamura_base_little_motion);

    Eigen::MatrixXd home_config_js_arm() const;
    void setHome_config_js_arm(const Eigen::MatrixXd &home_config_js_arm);

    Eigen::MatrixXd home_config_js_base() const;
    void setHome_config_js_base(const Eigen::MatrixXd &home_config_js_base);

    double pos_epsilon_js_arm() const;
    void setPos_epsilon_js_arm(const double &pos_epsilon_js_arm);

    double pos_epsilon_js_base() const;
    void setPos_epsilon_js_base(const double &pos_epsilon_js_base);

    double vel_epsilon_js_arm() const;
    void setVel_epsilon_js_arm(const double &vel_epsilon_js_arm);

    double vel_epsilon_js_base() const;
    void setVel_epsilon_js_base(const double &vel_epsilon_js_base);

    double pos_epsilon_os_linear() const;
    void setPos_epsilon_os_linear(const double &pos_epsilon_os_linear);

    double pos_epsilon_os_angular() const;
    void setPos_epsilon_os_angular(const double &pos_epsilon_os_angular);

    double vel_epsilon_os_linear() const;
    void setVel_epsilon_os_linear(const double &vel_epsilon_os_linear);

    double vel_epsilon_os_angular() const;
    void setVel_epsilon_os_angular(const double &vel_epsilon_os_angular);

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
         * @return HybridAutomatonFactory Pointer to the generated clone
         */
    virtual HybridAutomatonFactory* _doClone() const
    {
        return (new HybridAutomatonFactory(*this));
    }

    /**
         * @brief Concatenates two vectors, one for the arm and one for the base
         *
         * @param arm_vector Vector for the arm. It will be the first _num_dof_arm values of the resulting vector
         * @param base_vector Vector for the base. It will be the last _num_dof_base of the resulting vector
         * @return Eigen::MatrixXd Resulting vector that contains both input vectors
         */
    virtual Eigen::MatrixXd _combineArmAndBase(const Eigen::MatrixXd& arm_vector, const Eigen::MatrixXd base_vector);

    /// Number of degrees of freedom of the arm
    int _num_dof_arm;

    /// Number of degrees of freedom of the base
    int _num_dof_base;

    /// Indices that define the convention for the joints of the arm and the base
    std::string _index_str_base;
    std::string _index_str_arm;
    Eigen::MatrixXd _index_vec_base;
    Eigen::MatrixXd _index_vec_arm;

    /// Maximum joint space (js) velocity
    Eigen::MatrixXd _max_vel_js_arm;
    Eigen::MatrixXd _max_vel_js_base;

    /// Joint space (js) gains
    Eigen::MatrixXd _kp_js_arm;
    Eigen::MatrixXd _kp_js_base;
    Eigen::MatrixXd _kv_js_arm;
    Eigen::MatrixXd _kv_js_base;

    /// Operational space gains (3x orientation + 3x position)
    Eigen::MatrixXd _kp_os_linear;
    Eigen::MatrixXd _kp_os_angular;
    Eigen::MatrixXd _kv_os_linear;
    Eigen::MatrixXd _kv_os_angular;

    /// Joint space integration gains for operational space Nakamura control set
    Eigen::MatrixXd _kp_js_nakamura_arm;
    Eigen::MatrixXd _kp_js_nakamura_base;
    Eigen::MatrixXd _kv_js_nakamura_arm;
    Eigen::MatrixXd _kv_js_nakamura_base;

    /// Weights of each joint for operational space Nakamura control set
    Eigen::MatrixXd _joint_weights_nakamura_arm;
    Eigen::MatrixXd _joint_weights_nakamura_base;
    Eigen::MatrixXd _joint_weights_nakamura_base_no_rotation;
    Eigen::MatrixXd _joint_weights_nakamura_base_little_motion;

    /// Home configuration - usually a good initial position to begin the interaction and/or a safe
    /// position to return to
    Eigen::MatrixXd _home_config_js_arm;
    Eigen::MatrixXd _home_config_js_base;

    /// Maximum joint space (os) velocity
    double _max_vel_os_linear;
    double _max_vel_os_angular;

    /// Convergence radius for the position of a joint space (js) controller
    double _pos_epsilon_js_arm;
    double _pos_epsilon_js_base;

    /// Convergence radius for the velocity of a joint space (js) controller
    double _vel_epsilon_js_arm;
    double _vel_epsilon_js_base;

    /// Convergence goal for the velocity of a joint space (js) controller
    Eigen::MatrixXd _vel_goal_js_arm;
    Eigen::MatrixXd _vel_goal_js_base;

    /// Convergence radius of an operational space (os) controller
    double _pos_epsilon_os_linear;
    double _pos_epsilon_os_angular;

    /// Convergence radius for the velocity of an operational space (os) controller
    double _vel_epsilon_os_linear;
    double _vel_epsilon_os_angular;

    /// Convergence goal for the velocity of an operational space (os) controller
    Eigen::MatrixXd _vel_goal_os_linear;
    Eigen::MatrixXd _vel_goal_os_angular;

};

}

#endif
