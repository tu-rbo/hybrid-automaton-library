#ifndef HYBRID_AUTOMATON_RBO_FACTORY_H
#define HYBRID_AUTOMATON_RBO_FACTORY_H

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/HybridAutomatonAbstractFactory.h"

#include <boost/shared_ptr.hpp>

namespace ha {

enum GripperType
{
    NO_GRIPPER,
    SOFT_HAND,
    SUCTION_CUP,
    BARRETT_HAND
};

class HybridAutomatonRBOFactory;
typedef boost::shared_ptr<HybridAutomatonRBOFactory> HybridAutomatonRBOFactoryPtr;
typedef boost::shared_ptr<const HybridAutomatonRBOFactory> HybridAutomatonRBOFactoryConstPtr;

struct HybridAutomatonRBOParams : public HybridAutomatonAbstractParams
{
    std::string name, topic_name, parent_frame, child_frame;
    Eigen::MatrixXd goal1, goal2;
    Eigen::MatrixXd kp1, kp2;
    Eigen::MatrixXd kv1, kv2;
    Eigen::MatrixXd max_velocity1, max_velocity2;
    Eigen::MatrixXd index_vec;
    bool goal_relative, use_tf, trajectory;
    int update_rate;
    double completion_time;
};

/**
     * @brief A class to easily generate basic hybrid automata, control modes, control switches, jump conditions...
     *
     */
class HybridAutomatonRBOFactory : public HybridAutomatonAbstractFactory
{
public:

    typedef boost::shared_ptr<HybridAutomatonRBOFactory> Ptr;
    typedef boost::shared_ptr<const HybridAutomatonRBOFactory> ConstPtr;

    /**
         * @brief Default constructor
         *
         */
    HybridAutomatonRBOFactory();

    /**
         * @brief Constructor
         *
         * @param num_dof_arm Number of degrees of freedom of the arm
         * @param num_dof_base Number of degrees of freedom of the base
         */
    HybridAutomatonRBOFactory(const int& num_dof_arm, const int& num_dof_base);

    /**
         * @brief Destructor
         *
         */
    virtual ~HybridAutomatonRBOFactory();

    /**
         * @brief Copy constructor
         *
         * @param haf Object to make a copy from
         */
    HybridAutomatonRBOFactory(const HybridAutomatonRBOFactory& haf);

    /**
         * @brief Clone function
         *
         * @return HybridAutomatonRBOFactoryPtr Pointer to the generated clone
         */
    HybridAutomatonRBOFactoryPtr clone() const
    {
        return (HybridAutomatonRBOFactoryPtr(_doClone()));
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
    virtual ha::Controller::Ptr createJointSpaceController(const HybridAutomatonAbstractParams& params);

    virtual ha::Controller::Ptr createSubjointSpaceController(const HybridAutomatonAbstractParams& params);

    virtual ha::Controller::Ptr createBBSubjointSpaceController(const HybridAutomatonAbstractParams& params);

    /**
     * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only arm)
     *
     * @param name The controller name - must be unique within a hybrid automaton
     * @param goal The goal configuration
     * @param max_velocity The maximum velocity for the joints of the arm
     * @return ha::Controller::Ptr The generated controller
     */
    virtual ha::Controller::Ptr createSubjointSpaceControllerArm(const HybridAutomatonAbstractParams& params);

    /**
         * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only base)
         *
         * @param name The controller name - must be unique within a hybrid automaton
         * @param goal The goal configuration
         * @param max_velocity The maximum velocity for the base
         * @return ha::Controller::Ptr The generated controller
         */
    virtual ha::Controller::Ptr createSubjointSpaceControllerBase(const HybridAutomatonAbstractParams& params);

    /**
      * @brief Create an interpolated joint space controller to move from the current robots position to a goal configuration (only base)
      *
      * @param name The controller name - must be unique within a hybrid automaton
      * @param topic The topic where the goal for the controller is published
      * @param max_velocity The maximum velocity for the base
      * @return ha::Controller::Ptr The generated controller
      */
    virtual ha::Controller::Ptr createBBSubjointSpaceControllerBase(const HybridAutomatonAbstractParams& params);

    virtual ha::Controller::Ptr createOperationalSpaceController(const HybridAutomatonAbstractParams& params);

    virtual ha::Controller::Ptr createOperationalSpaceController(const HybridAutomatonAbstractParams& params);

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
    virtual ha::Controller::Ptr createBBOperationalSpaceController(const HybridAutomatonAbstractParams& params);

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

    virtual void _initializeDefaultValues();

    /**
         * @brief Performs the cloning operation (this solves some issues of inheritance and smart pointers)
         *
         * @return HybridAutomatonRBOFactory Pointer to the generated clone
         */
    virtual HybridAutomatonRBOFactory* _doClone() const
    {
        return (new HybridAutomatonRBOFactory(*this));
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

    /// The default update rate
    int _update_rate;
};

}

#endif
