#include "hybrid_automaton/HybridAutomatonFactory.h"

#include "hybrid_automaton/ControlMode.h"
#include "hybrid_automaton/ControlSet.h"
#include "hybrid_automaton/Controller.h"

//Sensors
#include "hybrid_automaton/JointConfigurationSensor.h"
#include "hybrid_automaton/JointVelocitySensor.h"
#include "hybrid_automaton/SubjointConfigurationSensor.h"
#include "hybrid_automaton/SubjointVelocitySensor.h"
#include "hybrid_automaton/ForceTorqueSensor.h"
#include "hybrid_automaton/ClockSensor.h"
#include "hybrid_automaton/FrameDisplacementSensor.h"
#include "hybrid_automaton/FramePoseSensor.h"

#define DEFAULT_NUM_DOF_ARM 7
#define DEFAULT_NUM_DOF_BASE 3

#define DEFAULT_MAX_VEL_JS_ARM 0.3
#define DEFAULT_MAX_VEL_JS_BASE 0.15

#define DEFAULT_KP_JS_ARM 0.0
#define DEFAULT_KV_JS_ARM 0.0
#define DEFAULT_KP_JS_BASE 40.0
#define DEFAULT_KV_JS_BASE 7.0

#define DEFAULT_KP_JS_NAKAMURA_ARM 15.0
#define DEFAULT_KP_JS_NAKAMURA_BASE 0.0
#define DEFAULT_KV_JS_NAKAMURA_ARM 0.5
#define DEFAULT_KV_JS_NAKAMURA_BASE 5.0

#define DEFAULT_JOINT_WEIGHTS_NAKAMURA_ARM 1.0
#define DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE 0.5
#define DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE_LITTLE_MOTION 0.01

#define DEFAULT_KP_OS_LINEAR 0.0
#define DEFAULT_KP_OS_ANGULAR 0.0
#define DEFAULT_KV_OS_LINEAR 10.0
#define DEFAULT_KV_OS_ANGULAR 10.0

#define DEFAULT_HOME_CONFIG_JS_ARM 0.0
#define DEFAULT_HOME_CONFIG_JS_BASE 0.0

#define DEFAULT_POS_EPSILON_JS_ARM 0.13
#define DEFAULT_POS_EPSILON_JS_BASE 0.05
#define DEFAULT_VEL_EPSILON_JS_ARM 0.01
#define DEFAULT_VEL_EPSILON_JS_BASE 0.001
#define DEFAULT_VEL_GOAL_JS_ARM 0.0
#define DEFAULT_VEL_GOAL_JS_BASE 0.0

#define DEFAULT_POS_EPSILON_OS_LINEAR 0.01
#define DEFAULT_POS_EPSILON_OS_ANGULAR 0.01
#define DEFAULT_VEL_EPSILON_OS_LINEAR 0.01
#define DEFAULT_VEL_EPSILON_OS_ANGULAR 0.01
#define DEFAULT_VEL_GOAL_OS_LINEAR 0.0
#define DEFAULT_VEL_GOAL_OS_ANGULAR 0.0

namespace ha
{
HybridAutomatonFactory::HybridAutomatonFactory()
    : HybridAutomatonFactory(DEFAULT_NUM_DOF_ARM, DEFAULT_NUM_DOF_BASE)
{

}

HybridAutomatonFactory::HybridAutomatonFactory(const int& num_dof_arm, const int& num_dof_base)
    :_num_dof_arm(num_dof_arm), _num_dof_base(num_dof_base)
{

    _index_vec_arm.resize(_num_dof_arm, 0.0);
    std::stringstream index_arm_ss;
    index_arm_ss << "[" << _num_dof_arm << ",1]";
    const char* separator_arm = "";
    for(int idx_arm=0; idx_arm<_num_dof_arm; idx_arm++)
    {
        _index_vec_arm.push_back(idx_arm);
        index_arm_ss << separator_arm << idx_arm;
        separator_arm = ";";
    }
    _index_str_arm = index_arm_ss.str();

    _index_vec_base.resize(_num_dof_base, 0.0);
    std::stringstream index_base_ss;
    index_base_ss << "[" << _num_dof_base << ",1]";
    const char* separator_base = "";
    for(int idx_base=_num_dof_arm; idx_base<_num_dof_arm+_num_dof_base; idx_base++)
    {
        _index_vec_base.push_back(idx_base);
        index_base_ss << separator_base << idx_base;
        separator_base = ";";
    }
    _index_str_base = index_base_ss.str();

    _max_vel_js_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_MAX_VEL_JS_ARM);
    _max_vel_js_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_MAX_VEL_JS_BASE);

    _kp_js_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_KP_JS_ARM);
    if(_num_dof_arm == DEFAULT_NUM_DOF_ARM)
    {
        _kp_js_arm << 300.0, 200.0, 150.0, 120.0, 10.0, 10.0, 10.0;
    }
    _kp_js_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_KP_JS_BASE);

    _kv_js_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_KV_JS_ARM);
    if(_num_dof_arm == DEFAULT_NUM_DOF_ARM)
    {
        _kv_js_arm << 2.0, 4.0, 2.0, 1.2, 0.2, 0.3, 0.02;
    }
    _kv_js_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_KV_JS_BASE);

    _kp_os_linear = Eigen::MatrixXd::Constant(3, 1, DEFAULT_KP_OS_LINEAR);
    _kp_os_angular = Eigen::MatrixXd::Constant(3, 1, DEFAULT_KP_OS_ANGULAR);
    _kv_os_linear = Eigen::MatrixXd::Constant(3, 1, DEFAULT_KV_OS_LINEAR);
    _kv_os_angular = Eigen::MatrixXd::Constant(3, 1, DEFAULT_KV_OS_ANGULAR);

    _kp_js_nakamura_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_KP_JS_NAKAMURA_ARM);
    if(_num_dof_arm == DEFAULT_NUM_DOF_ARM)
    {
        _kp_js_nakamura_arm << 30.0, 20.0, 15.0, 20.0, 10.0, 10.0, 10.0;
    }
    _kp_js_nakamura_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_KP_JS_NAKAMURA_BASE);

    _kv_js_nakamura_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_KV_JS_NAKAMURA_ARM);
    if(_num_dof_arm == DEFAULT_NUM_DOF_ARM)
    {
        _kv_js_nakamura_arm << 1.0, 2.0, 1.0, 0.4, 0.1, 0.1, 0.01;
    }
    _kv_js_nakamura_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_KV_JS_NAKAMURA_BASE);
    if(_num_dof_base== DEFAULT_NUM_DOF_BASE)
    {
        _kv_js_nakamura_base << 10.0, 10.0, 2.0;
    }

    _joint_weights_nakamura_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_JOINT_WEIGHTS_NAKAMURA_ARM);
    _joint_weights_nakamura_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE);
    _joint_weights_nakamura_base_no_rotation = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE);
    if(_num_dof_base== DEFAULT_NUM_DOF_BASE)
    {
        _joint_weights_nakamura_base_no_rotation << DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE, DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE, DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE_LITTLE_MOTION;
    }
    _joint_weights_nakamura_base_little_motion = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE_LITTLE_MOTION);

    _home_config_js_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_HOME_CONFIG_JS_ARM);
    if(_num_dof_arm == DEFAULT_NUM_DOF_ARM)
    {
        _home_config_js_arm << 0.0, -0.14, 0.0, 2.18, 0.0, 0.2, -0.13;
    }
    _home_config_js_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_HOME_CONFIG_JS_BASE);

    _pos_epsilon_js_arm  = DEFAULT_POS_EPSILON_JS_ARM;
    _pos_epsilon_js_base = DEFAULT_POS_EPSILON_JS_BASE;

    _vel_epsilon_js_arm = DEFAULT_VEL_EPSILON_JS_ARM;
    _vel_epsilon_js_base =  DEFAULT_VEL_EPSILON_JS_BASE;

    _vel_goal_js_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_VEL_GOAL_JS_ARM);
    _vel_goal_js_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_VEL_GOAL_JS_BASE);

    _pos_epsilon_os_linear =  DEFAULT_POS_EPSILON_OS_LINEAR;
    _pos_epsilon_os_angular =  DEFAULT_POS_EPSILON_OS_ANGULAR;

    _vel_epsilon_os_linear =  DEFAULT_VEL_EPSILON_OS_LINEAR;
    _vel_epsilon_os_angular =  DEFAULT_VEL_EPSILON_OS_ANGULAR;

    _vel_goal_os_linear =  DEFAULT_VEL_GOAL_OS_LINEAR;
    _vel_goal_os_angular =  DEFAULT_VEL_GOAL_OS_ANGULAR;

}

HybridAutomatonFactory::~HybridAutomatonFactory()
{

}

HybridAutomatonFactory::HybridAutomatonFactory(const HybridAutomatonFactory& haf)
{

}

Eigen::MatrixXd HybridAutomatonFactory::max_vel_js_arm() const
{
    return _max_vel_js_arm;
}

void HybridAutomatonFactory::setMax_vel_js_arm(const Eigen::MatrixXd &max_vel_js_arm)
{
    _max_vel_js_arm = max_vel_js_arm;
}
Eigen::MatrixXd HybridAutomatonFactory::max_vel_js_base() const
{
    return _max_vel_js_base;
}

void HybridAutomatonFactory::setMax_vel_js_base(const Eigen::MatrixXd &max_vel_js_base)
{
    _max_vel_js_base = max_vel_js_base;
}
Eigen::MatrixXd HybridAutomatonFactory::kp_js_arm() const
{
    return _kp_js_arm;
}

void HybridAutomatonFactory::setKp_js_arm(const Eigen::MatrixXd &kp_js_arm)
{
    _kp_js_arm = kp_js_arm;
}
Eigen::MatrixXd HybridAutomatonFactory::kp_js_base() const
{
    return _kp_js_base;
}

void HybridAutomatonFactory::setKp_js_base(const Eigen::MatrixXd &kp_js_base)
{
    _kp_js_base = kp_js_base;
}
Eigen::MatrixXd HybridAutomatonFactory::kv_js_arm() const
{
    return _kv_js_arm;
}

void HybridAutomatonFactory::setKv_js_arm(const Eigen::MatrixXd &kv_js_arm)
{
    _kv_js_arm = kv_js_arm;
}
Eigen::MatrixXd HybridAutomatonFactory::kv_js_base() const
{
    return _kv_js_base;
}

void HybridAutomatonFactory::setKv_js_base(const Eigen::MatrixXd &kv_js_base)
{
    _kv_js_base = kv_js_base;
}
Eigen::MatrixXd HybridAutomatonFactory::kp_os_linear() const
{
    return _kp_os_linear;
}

void HybridAutomatonFactory::setKp_os_linear(const Eigen::MatrixXd &kp_os_linear)
{
    _kp_os_linear = kp_os_linear;
}
Eigen::MatrixXd HybridAutomatonFactory::kp_os_angular() const
{
    return _kp_os_angular;
}

void HybridAutomatonFactory::setKp_os_angular(const Eigen::MatrixXd &kp_os_angular)
{
    _kp_os_angular = kp_os_angular;
}
Eigen::MatrixXd HybridAutomatonFactory::kv_os_linear() const
{
    return _kv_os_linear;
}

void HybridAutomatonFactory::setKv_os_linear(const Eigen::MatrixXd &kv_os_linear)
{
    _kv_os_linear = kv_os_linear;
}
Eigen::MatrixXd HybridAutomatonFactory::kv_os_angular() const
{
    return _kv_os_angular;
}

void HybridAutomatonFactory::setKv_os_angular(const Eigen::MatrixXd &kv_os_angular)
{
    _kv_os_angular = kv_os_angular;
}
Eigen::MatrixXd HybridAutomatonFactory::kp_js_nakamura_arm() const
{
    return _kp_js_nakamura_arm;
}

void HybridAutomatonFactory::setKp_js_nakamura_arm(const Eigen::MatrixXd &kp_js_nakamura_arm)
{
    _kp_js_nakamura_arm = kp_js_nakamura_arm;
}
Eigen::MatrixXd HybridAutomatonFactory::kp_js_nakamura_base() const
{
    return _kp_js_nakamura_base;
}

void HybridAutomatonFactory::setKp_js_nakamura_base(const Eigen::MatrixXd &kp_js_nakamura_base)
{
    _kp_js_nakamura_base = kp_js_nakamura_base;
}
Eigen::MatrixXd HybridAutomatonFactory::kv_js_nakamura_arm() const
{
    return _kv_js_nakamura_arm;
}

void HybridAutomatonFactory::setKv_js_nakamura_arm(const Eigen::MatrixXd &kv_js_nakamura_arm)
{
    _kv_js_nakamura_arm = kv_js_nakamura_arm;
}
Eigen::MatrixXd HybridAutomatonFactory::kv_js_nakamura_base() const
{
    return _kv_js_nakamura_base;
}

void HybridAutomatonFactory::setKv_js_nakamura_base(const Eigen::MatrixXd &kv_js_nakamura_base)
{
    _kv_js_nakamura_base = kv_js_nakamura_base;
}
Eigen::MatrixXd HybridAutomatonFactory::joint_weights_nakamura_arm() const
{
    return _joint_weights_nakamura_arm;
}

void HybridAutomatonFactory::setJoint_weights_nakamura_arm(const Eigen::MatrixXd &joint_weights_nakamura_arm)
{
    _joint_weights_nakamura_arm = joint_weights_nakamura_arm;
}
Eigen::MatrixXd HybridAutomatonFactory::joint_weights_nakamura_base() const
{
    return _joint_weights_nakamura_base;
}

void HybridAutomatonFactory::setJoint_weights_nakamura_base(const Eigen::MatrixXd &joint_weights_nakamura_base)
{
    _joint_weights_nakamura_base = joint_weights_nakamura_base;
}
Eigen::MatrixXd HybridAutomatonFactory::joint_weights_nakamura_base_no_rotation() const
{
    return _joint_weights_nakamura_base_no_rotation;
}

void HybridAutomatonFactory::setJoint_weights_nakamura_base_no_rotation(const Eigen::MatrixXd &joint_weights_nakamura_base_no_rotation)
{
    _joint_weights_nakamura_base_no_rotation = joint_weights_nakamura_base_no_rotation;
}
Eigen::MatrixXd HybridAutomatonFactory::joint_weights_nakamura_base_little_motion() const
{
    return _joint_weights_nakamura_base_little_motion;
}

void HybridAutomatonFactory::setJoint_weights_nakamura_base_little_motion(const Eigen::MatrixXd &joint_weights_nakamura_base_little_motion)
{
    _joint_weights_nakamura_base_little_motion = joint_weights_nakamura_base_little_motion;
}
Eigen::MatrixXd HybridAutomatonFactory::home_config_js_arm() const
{
    return _home_config_js_arm;
}

void HybridAutomatonFactory::setHome_config_js_arm(const Eigen::MatrixXd &home_config_js_arm)
{
    _home_config_js_arm = home_config_js_arm;
}
Eigen::MatrixXd HybridAutomatonFactory::home_config_js_base() const
{
    return _home_config_js_base;
}

void HybridAutomatonFactory::setHome_config_js_base(const Eigen::MatrixXd &home_config_js_base)
{
    _home_config_js_base = home_config_js_base;
}
Eigen::MatrixXd HybridAutomatonFactory::pos_epsilon_js_arm() const
{
    return _pos_epsilon_js_arm;
}

void HybridAutomatonFactory::setPos_epsilon_js_arm(const Eigen::MatrixXd &pos_epsilon_js_arm)
{
    _pos_epsilon_js_arm = pos_epsilon_js_arm;
}
Eigen::MatrixXd HybridAutomatonFactory::pos_epsilon_js_base() const
{
    return _pos_epsilon_js_base;
}

void HybridAutomatonFactory::setPos_epsilon_js_base(const Eigen::MatrixXd &pos_epsilon_js_base)
{
    _pos_epsilon_js_base = pos_epsilon_js_base;
}
Eigen::MatrixXd HybridAutomatonFactory::vel_epsilon_js_arm() const
{
    return _vel_epsilon_js_arm;
}

void HybridAutomatonFactory::setVel_epsilon_js_arm(const Eigen::MatrixXd &vel_epsilon_js_arm)
{
    _vel_epsilon_js_arm = vel_epsilon_js_arm;
}
Eigen::MatrixXd HybridAutomatonFactory::vel_epsilon_js_base() const
{
    return _vel_epsilon_js_base;
}

void HybridAutomatonFactory::setVel_epsilon_js_base(const Eigen::MatrixXd &vel_epsilon_js_base)
{
    _vel_epsilon_js_base = vel_epsilon_js_base;
}
Eigen::MatrixXd HybridAutomatonFactory::pos_epsilon_os_linear() const
{
    return _pos_epsilon_os_linear;
}

void HybridAutomatonFactory::setPos_epsilon_os_linear(const Eigen::MatrixXd &pos_epsilon_os_linear)
{
    _pos_epsilon_os_linear = pos_epsilon_os_linear;
}
Eigen::MatrixXd HybridAutomatonFactory::pos_epsilon_os_angular() const
{
    return _pos_epsilon_os_angular;
}

void HybridAutomatonFactory::setPos_epsilon_os_angular(const Eigen::MatrixXd &pos_epsilon_os_angular)
{
    _pos_epsilon_os_angular = pos_epsilon_os_angular;
}
Eigen::MatrixXd HybridAutomatonFactory::vel_epsilon_os_linear() const
{
    return _vel_epsilon_os_linear;
}

void HybridAutomatonFactory::setVel_epsilon_os_linear(const Eigen::MatrixXd &vel_epsilon_os_linear)
{
    _vel_epsilon_os_linear = vel_epsilon_os_linear;
}
Eigen::MatrixXd HybridAutomatonFactory::vel_epsilon_os_angular() const
{
    return _vel_epsilon_os_angular;
}

void HybridAutomatonFactory::setVel_epsilon_os_angular(const Eigen::MatrixXd &vel_epsilon_os_angular)
{
    _vel_epsilon_os_angular = vel_epsilon_os_angular;
}

ha::HybridAutomaton::Ptr HybridAutomatonFactory::createInitialHybridAutomaton(const GripperType& gripper,
                                                                              const Eigen::MatrixXd& home_config_js_arm,
                                                                              const Eigen::MatrixXd& home_config_js_base,
                                                                              const Eigen::MatrixXd& max_vel_js_arm,
                                                                              const Eigen::MatrixXd& max_vel_js_base,
                                                                              const Eigen::MatrixXd& index_vec_arm,
                                                                              const Eigen::MatrixXd& pos_epsilon_js_arm)
{

    //create Hybrid Automaton
    ha::HybridAutomaton::Ptr initial_ha = ha::HybridAutomaton::Ptr(new ha::HybridAutomaton());
    initial_ha->setName("Initial_HA");

    ha::ControlMode::Ptr move_home_initial_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr move_home_initial_cs(new ha::ControlSwitch());

    move_home_initial_cm->setName("move_home_initial_cm");

    ha::Controller::Ptr home_ctrl_arm = createJointSpaceArmController("move_home_initial_arm_ctrl",
                                                                      (home_config_js_arm.size() == 0 ? _home_config_js_arm : home_config_js_arm),
                                                                      (max_vel_js_arm.size() == 0 ? _max_vel_js_arm : max_vel_js_arm));
    ha::Controller::Ptr home_ctrl_base = createJointSpaceBaseController("move_home_initial_base_ctrl",
                                                                        (home_config_js_base.size() == 0 ? _home_config_js_base : home_config_js_base),
                                                                        (max_vel_js_base.size() == 0 ? _max_vel_js_base : max_vel_js_base));
    std::vector<ha::Controller::Ptr> home_ctrls;
    home_ctrls.push_back(home_ctrl_arm);
    home_ctrls.push_back(home_ctrl_base);
    ha::ControlSet::Ptr goto_home_cs = createJointSpaceControlSet(home_ctrls);
    move_home_initial_cm->setControlSet(goto_home_cs);

    //add controller to ControlSet
    ha::Controller::Ptr ungrasp_ctrl(new ha::Controller());
    switch(gripper)
    {
    case NO_GRIPPER:
        break;
    case SOFT_HAND:
    {
        ungrasp_ctrl->setName("FeixGraspControl");
        ungrasp_ctrl->setType("FeixGraspSoftHandController");
        ungrasp_ctrl->setArgument("grasp_strength", "4.0");
        ungrasp_ctrl->setArgument("grasp_type", "0");
        goto_home_cs->appendController(ungrasp_ctrl);
    }
        break;
    case SUCTION_CUP:
    {
        ungrasp_ctrl->setName("VacuumGraspControl");
        ungrasp_ctrl->setType("VacuumCleanerController");
        ungrasp_ctrl->setArgument("power", "0");
        goto_home_cs->appendController(ungrasp_ctrl);
    }
        break;
    case BARRETT_HAND:
        break;
    default:
        break;
    }

    //Create first ControlSwitch
    move_home_initial_cs->setName("move_home_initial_cs");
    ha::JumpConditionPtr initial_convergence_arm_jc =
            createSubjointSpaceControllerArmGoalConvergenceCondition(home_ctrl_arm,
                                                    (index_vec_arm.size() == 0 ? _index_vec_arm : index_vec_arm),
                                                    (pos_epsilon_js_arm.size() == 0 ? _pos_epsilon_js_arm : pos_epsilon_js_arm));
    ha::JumpConditionPtr initial_convergence_vel_arm_jc =
            createJointSpaceArmConvergenceWithZeroVelCondition();
    ha::JumpConditionPtr initial_convergence_base_jc = createJointSpaceBaseConvergenceCondition(home_ctrl_base);
    ha::JumpConditionPtr initial_convergence_vel_base_jc = createJointSpaceBaseConvergenceWithZeroVelCondition();
    move_home_initial_cs->add(initial_convergence_arm_jc);
    move_home_initial_cs->add(initial_convergence_vel_arm_jc);
    move_home_initial_cs->add(initial_convergence_base_jc);
    move_home_initial_cs->add(initial_convergence_vel_base_jc);

    initial_ha->addControlMode(move_home_initial_cm);

    ha::ControlMode::Ptr grav_cm(new ha::ControlMode());
    grav_cm->setName("finished");
    ha::ControlSet::Ptr grav_cs(new ha::ControlSet());
    grav_cs->setType("rxControlSet");
    grav_cs->setName("empty");
    grav_cm->setControlSet(grav_cs);

    initial_ha->addControlSwitchAndMode(move_home_initial_cm->getName(), move_home_initial_cs, grav_cm);

    initial_ha->setCurrentControlMode(move_home_initial_cm->getName());

    return initial_ha;
}

ha::HybridAutomaton::Ptr HybridAutomatonFactory::createEmptyHybridAutomaton()
{

    //create Hybrid Automaton
    ha::HybridAutomaton::Ptr grav_comp_ha = ha::HybridAutomaton::Ptr(new ha::HybridAutomaton());
    grav_comp_ha->setName("Grav_Comp_HA");

    ha::ControlMode::Ptr grav_cm(new ha::ControlMode());
    grav_cm->setName("finished");
    ha::ControlSet::Ptr grav_cs(new ha::ControlSet());
    grav_cs->setType("rxControlSet");
    grav_cs->setName("empty");
    grav_cm->setControlSet(grav_cs);

    grav_comp_ha->addControlMode(grav_cm);

    grav_comp_ha->setCurrentControlMode(grav_cm->getName());

    return grav_comp_ha;
}

ha::ControlSet::Ptr HybridAutomatonFactory::createJointSpaceControlSet(ha::Controller::Ptr ctrl)
{
    ha::ControlSet::Ptr cs(new ha::ControlSet());
    cs->setType("rxControlSet");
    cs->appendController(ctrl);
    return cs;
}

ha::ControlSet::Ptr HybridAutomatonFactory::createJointSpaceControlSet(const std::vector<ha::Controller::Ptr>& ctrls)
{
    ha::ControlSet::Ptr cs(new ha::ControlSet());
    cs->setType("rxControlSet");
    for(int i=0; i<ctrls.size(); i++)
        cs->appendController(ctrls.at(i));
    return cs;
}

ha::ControlSet::Ptr HybridAutomatonFactory::createOperationalSpaceNakamuraControlSet(ha::Controller::Ptr ctrl, bool move_base,
                                                                                     const Eigen::MatrixXd& kp_js_nakamura_arm,
                                                                                     const Eigen::MatrixXd& kp_js_nakamura_base,
                                                                                     const Eigen::MatrixXd& kv_js_nakamura_arm,
                                                                                     const Eigen::MatrixXd& kv_js_nakamura_base,
                                                                                     const Eigen::MatrixXd& joint_weights_nakamura_arm,
                                                                                     const Eigen::MatrixXd& joint_weights_nakamura_base)
{
    ha::ControlSet::Ptr cs(new ha::ControlSet());
    cs->setType("TPNakamuraControlSet");
    cs->setArgument<Eigen::MatrixXd>("js_kp", _combineArmAndBase((kp_js_nakamura_arm.size() == 0 ? _kp_js_nakamura_arm : kp_js_nakamura_arm),
                                                                 (kp_js_nakamura_base.size() == 0 ? _kp_js_nakamura_base : kp_js_nakamura_base)));
    cs->setArgument<Eigen::MatrixXd>("js_kd", _combineArmAndBase((kv_js_nakamura_arm.size() == 0 ? _kv_js_nakamura_arm : kv_js_nakamura_arm),
                                                                 (kv_js_nakamura_base.size() == 0 ? _kv_js_nakamura_base : kv_js_nakamura_base)));

    if(move_base)
        cs->setArgument<Eigen::MatrixXd>("joint_weights",
                                         _combineArmAndBase((joint_weights_nakamura_arm.size() == 0 ? _joint_weights_nakamura_arm : joint_weights_nakamura_arm),
                                                            (joint_weights_nakamura_base.size() == 0 ? _joint_weights_nakamura_base_no_rotation : joint_weights_nakamura_base)));
    else
        cs->setArgument<Eigen::MatrixXd>("joint_weights",
                                         _combineArmAndBase((joint_weights_nakamura_arm.size() == 0 ? _joint_weights_nakamura_arm : joint_weights_nakamura_arm),
                                                            (joint_weights_nakamura_base.size() == 0 ? _joint_weights_nakamura_base_little_motion : joint_weights_nakamura_base)));


    cs->appendController(ctrl);
    return cs;
}

ha::Controller::Ptr HybridAutomatonFactory::createJointSpaceController(std::string name,
                                                                       const Eigen::MatrixXd &goal_js,
                                                                       double completion_time,
                                                                       const Eigen::MatrixXd &kp_js,
                                                                       const Eigen::MatrixXd &kv_js,
                                                                       bool goal_relative)
{
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("InterpolatedJointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ctrl->setGoal(goal);
    ctrl->setKp((kp_js.size() == 0 ? _combineArmAndBase(_kp_js_arm, _kp_js_base) : kp_js));
    ctrl->setKv((kv_js.size() == 0 ? _combineArmAndBase(_kv_js_arm, _kv_js_base) : kv_js));
    ctrl->setCompletionTime(completion_time);
    ctrl->setGoalIsRelative(goal_relative);
    return ctrl;
}


ha::Controller::Ptr HybridAutomatonFactory::createSubjointSpaceController(std::string name,
                                                                          const Eigen::MatrixXd& goal_js,
                                                                          const Eigen::MatrixXd& max_velocity,
                                                                          const Eigen::MatrixXd& index_vec,
                                                                          const Eigen::MatrixXd& kp_js,
                                                                          const Eigen::MatrixXd& kv_js,
                                                                          bool is_relative)
{
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("InterpolatedSubjointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ha_ostringstream index_vec_ss;
    index_vec_ss << index_vec;
    ctrl->setArgument("index", index_vec_ss.str());
    ctrl->setGoal(goal_js);
    ctrl->setKp(kp_js);
    ctrl->setKv(kv_js);
    ctrl->setMaximumVelocity(max_velocity);
    ctrl->setGoalIsRelative(is_relative);
    return ctrl;
}

ha::Controller::Ptr HybridAutomatonFactory::createSubjointSpaceControllerArm(std::string name,
                                                                             const Eigen::MatrixXd& goal_js_arm,
                                                                             const Eigen::MatrixXd& max_vel_js_arm,
                                                                             const Eigen::MatrixXd& index_vec_arm,
                                                                             const Eigen::MatrixXd& kp_js_arm,
                                                                             const Eigen::MatrixXd& kv_js_arm,
                                                                             bool is_relative)
{
    return createSubjointSpaceController(name,
                                         goal_js_arm,
                                         (max_vel_js_arm.size() == 0 ? _max_vel_js_arm : max_vel_js_arm),
                                         (index_vec_arm.size() == 0 ? _index_vec_arm : index_vec_arm),
                                         (kp_js_arm.size() == 0 ? _kp_js_arm : kp_js_arm),
                                         (kv_js_arm.size() == 0 ? _kv_js_arm : kv_js_arm),
                                         is_relative);
}

ha::Controller::Ptr HybridAutomatonFactory::createSubjointSpaceControllerBase(std::string name,
                                                                              const Eigen::MatrixXd& goal_js_base,
                                                                              const Eigen::MatrixXd& max_vel_js_base,
                                                                              const Eigen::MatrixXd& index_vec_base,
                                                                              const Eigen::MatrixXd& kp_js_base,
                                                                              const Eigen::MatrixXd& kv_js_base,
                                                                              bool is_relative)
{
    return createSubjointSpaceController(name,
                                         goal_js_base,
                                         (max_vel_js_base.size() == 0 ? _max_vel_js_base : max_vel_js_base),
                                         (index_vec_base.size() == 0 ? _index_vec_base : index_vec_base),
                                         (kp_js_base.size() == 0 ? _kp_js_base : kp_js_base),
                                         (kv_js_base.size() == 0 ? _kv_js_base : kv_js_base),
                                         is_relative);
}

ha::Controller::Ptr HybridAutomatonFactory::createSubjointSpaceController(std::string name,
                                                                          const string& topic_name,
                                                                          const Eigen::MatrixXd& index_vec,
                                                                          const Eigen::MatrixXd& kp_js,
                                                                          const Eigen::MatrixXd& kv_js,
                                                                          const Eigen::MatrixXd& max_velocity,
                                                                          bool is_relative)
{
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("InterpolatedSubjointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ha_ostringstream index_vec_ss;
    index_vec_ss << index_vec;
    ctrl->setArgument("index", index_vec_ss.str());
    ctrl->setGoal(goal_js);
    ctrl->setKp(kp_js);
    ctrl->setKv(kv_js);
    ctrl->setMaximumVelocity(max_velocity);
    ctrl->setGoalIsRelative(is_relative);
    return ctrl;
}

ha::Controller::Ptr HybridAutomatonFactory::createJointSpaceBaseController(std::string name, const std::string& topic, const Eigen::MatrixXd& max_velocity){
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("BlackboardInterpolatedSubjointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ctrl->setArgument("index", _base_index_str);
    ctrl->setArgument("reinterpolation", "1");
    ctrl->setArgument("use_tf", "1");
    ctrl->setArgument("topic_name", topic);
    ctrl->setArgument("tf_parent", "odom");
    ctrl->setArgument("update_rate", 100);
    ctrl->setKp(_kp_base_jointspace);
    ctrl->setKv(_kv_base_jointspace);
    ctrl->setMaximumVelocity(max_velocity);
    ctrl->setGoalIsRelative(0);
    return ctrl;
}

ha::Controller::Ptr HybridAutomatonFactory::createJointSpaceControllerMoreThanOneGoal(std::string name, const Eigen::MatrixXd &goals, const Eigen::MatrixXd vel_max){
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("InterpolatedJointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ctrl->setGoal(goals);
    ctrl->setKp(_kp_jointspace);
    ctrl->setKv(_kv_jointspace);
    //ctrl->setCompletionTimes(completionTimes);
    ctrl->setMaximumVelocity(vel_max);
    return ctrl;
}


ha::Controller::Ptr HybridAutomatonFactory::createTaskSpaceController(std::string name, const Eigen::MatrixXd &pos, const Eigen::MatrixXd &ori, double completionTime){

    Eigen::MatrixXd bin_home_frame;
    bin_home_frame.resize(4,4);
    bin_home_frame.setIdentity();
    bin_home_frame.block(0,0,3,3) = ori;
    bin_home_frame.block(0,3,3,1) = pos;

    //Endeffector Frame Controller
    ha::Controller::Ptr ctrl(new ha::Controller);
    ctrl->setName(name);
    ctrl->setType("InterpolatedHTransformController");
    ctrl->setArgument("interpolation_type", "cubic");

    ctrl->setGoal(bin_home_frame);
    ctrl->setKp(_kp_opspace);
    ctrl->setKv(_kv_opspace);
    ctrl->setCompletionTime(completionTime);
    ctrl->setGoalIsRelative(0);
    ctrl->setArgument("operational_frame", "EE");

    return ctrl;
}

ha::Controller::Ptr HybridAutomatonFactory::createTaskSpaceControllerTF(std::string name, const std::string frame, double max_displacement_velocity, double max_rotational_velocity){

    //Endeffector Frame Controller
    ha::Controller::Ptr ctrl(new ha::Controller);
    ctrl->setName(name);
    ctrl->setType("BlackboardInterpolatedHTransformController");
    ctrl->setArgument("interpolation_type", "cubic");
    ctrl->setArgument("reinterpolation", "1");
    ctrl->setArgument("use_tf", "1");
    ctrl->setArgument("topic_name", frame);
    ctrl->setArgument("tf_parent", "odom");
    ctrl->setArgument("update_rate", 50);

    Eigen::MatrixXd max_vel(2,1);
    max_vel << max_rotational_velocity, max_displacement_velocity;
    ctrl->setMaximumVelocity(max_vel);

    ctrl->setKp( _kp_opspace);
    ctrl->setKv( _kv_opspace);
    ctrl->setGoalIsRelative(0);
    ctrl->setArgument("operational_frame", "EE");

    return ctrl;
}

ha::Controller::Ptr HybridAutomatonFactory::createTaskSpaceTrajController(std::string name, const std::string topic, double max_displacement_velocity, double max_rotational_velocity, bool relative_goal){

    //Endeffector Frame Controller
    ha::Controller::Ptr ctrl(new ha::Controller);
    ctrl->setName(name);
    ctrl->setType("BlackboardInterpolatedHTransformTrajectoryController");
    ctrl->setArgument("interpolation_type", "cubic");
    ctrl->setArgument("reinterpolation", "1");
    ctrl->setArgument("use_tf", "0");
    ctrl->setArgument("topic_name", topic);
    ctrl->setArgument("update_rate", -1);

    Eigen::MatrixXd max_vel(2,1);
    max_vel << max_rotational_velocity, max_displacement_velocity;
    ctrl->setMaximumVelocity(max_vel);

    ctrl->setKp( _kp_opspace);
    ctrl->setKv( _kv_opspace);
    ctrl->setGoalIsRelative(relative_goal);
    ctrl->setArgument("operational_frame", "EE");

    return ctrl;
}

ha::JumpCondition::Ptr HybridAutomatonFactory::createJointSpaceControllerGoalConvergenceCondition(ha::ControllerConstPtr js_ctrl,
                                                                                                  const Eigen::MatrixXd& pos_epsilon_js)
{
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SensorPtr sensor(new ha::JointConfigurationSensor());
    jc->setSensor(sensor);
    jc->setControllerGoal(ctrl);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon((pos_epsilon_js.size() == 0 ? _combineArmAndBase(_pos_epsilon_js_arm, _pos_epsilon_js_base) : pos_epsilon_js));

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonFactory::createSubjointSpaceControllerGoalConvergenceCondition(ha::ControllerConstPtr subjs_ctrl,
                                                                                       const std::vector<int>& index_vec,
                                                                                       const Eigen::MatrixXd& pos_epsilon_js)
{
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SubjointConfigurationSensorPtr sensor(new ha::SubjointConfigurationSensor());
    sensor->setIndex(index_vec);
    jc->setSensor(sensor);
    jc->setControllerGoal(subjs_ctrl);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon(pos_epsilon_js);

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonFactory::createSubjointSpaceControllerArmGoalConvergenceCondition(ha::ControllerConstPtr subjs_ctrl,
                                                                                       const Eigen::MatrixXd& index_vec_arm,
                                                                                       const Eigen::MatrixXd& pos_epsilon_js_arm)
{
    return createSubjointSpaceControllerGoalConvergenceCondition(subjs_ctrl,
                                                                 (index_vec_arm.size() == 0 ? _index_vec_arm : index_vec_arm),
                                                                 (pos_epsilon_js_arm.size() == 0 ? _pos_epsilon_js_arm : pos_epsilon_js_arm));
}

ha::JumpCondition::Ptr HybridAutomatonFactory::createSubjointSpaceControllerBaseGoalConvergenceCondition(ha::ControllerConstPtr subjs_ctrl,
                                                                                        const Eigen::MatrixXd& index_vec_base,
                                                                                        const Eigen::MatrixXd& pos_epsilon_js_base)
 {
     return createSubjointSpaceControllerGoalConvergenceCondition(subjs_ctrl,
                                                                  (index_vec_base.size() == 0 ? _index_vec_base : index_vec_base),
                                                                  (pos_epsilon_js_base.size() == 0 ? _pos_epsilon_js_base : pos_epsilon_js_base));
 }

ha::JumpCondition::Ptr HybridAutomatonFactory::createJointSpaceConvergenceWithVelocityCondition(const Eigen::MatrixXd& index_vec,
                                                                                                const Eigen::MatrixXd& vel_goal_js,
                                                                                                const Eigen::MatrixXd& vel_epsilon_js){
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SubjointVelocitySensorPtr sensor(new ha::SubjointVelocitySensor());
    sensor->setIndex(index_vec);
    jc->setSensor(sensor);
    jc->setConstantGoal(vel_goal_js);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon(vel_epsilon_js);

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonFactory::createJointSpaceConvergenceWithZeroVelocityCondition(const Eigen::MatrixXd& index_vec,
                                                                                                    const Eigen::MatrixXd& vel_epsilon_js)
{
    Eigen::MatrixXd zero_goal = Eigen::MatrixXd::Contant(vel_epsilon_js.size(), 1, 0.0);
    return createJointSpaceConvergenceWithVelocityCondition(index_vec, zero_goal, vel_epsilon_js);
}

ha::JumpCondition::Ptr HybridAutomatonFactory::createJointSpaceArmConvergenceWithZeroVelocityCondition(const Eigen::MatrixXd& index_vec_arm,
                                                                                                       const Eigen::MatrixXd& vel_epsilon_js_arm)
{
    return createJointSpaceConvergenceWithZeroVelocityCondition((index_vec_arm.size() == 0 ? _index_vec_arm : index_vec_arm),
                                                                (vel_epsilon_js_arm.size() == 0 ? _vel_epsilon_js_arm : vel_epsilon_js_arm));
}

ha::JumpCondition::Ptr HybridAutomatonFactory::createJointSpaceArmConvergenceWithVelocityCondition(const Eigen::MatrixXd& index_vec_arm,
                                                                                                   const Eigen::MatrixXd& vel_goal_js_arm,
                                                                                                   const Eigen::MatrixXd& vel_epsilon_js_arm)
{
    return createJointSpaceConvergenceWithVelocityCondition((index_vec_arm.size() == 0 ? _index_vec_arm : index_vec_arm),
                                                            (vel_goal_js_arm.size() == 0 ? _vel_goal_js_arm : vel_goal_js_arm)
                                                            (vel_epsilon_js_arm.size() == 0 ? _vel_epsilon_js_arm : vel_epsilon_js_arm));
}

ha::JumpCondition::Ptr HybridAutomatonFactory::createJointSpaceBaseConvergenceWithZeroVelocityCondition(const Eigen::MatrixXd& index_vec_base,
                                                                                                        const Eigen::MatrixXd& vel_epsilon_js_base)
{
    return createJointSpaceConvergenceWithZeroVelocityCondition((index_vec_base.size() == 0 ? _index_vec_base : index_vec_base),
                                                                (vel_epsilon_js_base.size() == 0 ? _vel_epsilon_js_base : vel_epsilon_js_base));
}

ha::JumpCondition::Ptr HybridAutomatonFactory::createTaskSpaceConvergenceCondition(ha::ControllerConstPtr ctrl, bool relative){
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SensorPtr sensor(new ha::FramePoseSensor());
    jc->setSensor(sensor);
    jc->setControllerGoal(ctrl);
    jc->setJumpCriterion(ha::JumpCondition::NORM_TRANSFORM);
    jc->setEpsilon(_op_epsilon);

    if(relative)
        jc->setGoalRelative();
    else
        jc->setGoalAbsolute();

    return jc;
}

Eigen::MatrixXd HybridAutomatonFactory::_combineArmAndBase(const Eigen::MatrixXd& arm_vector, const Eigen::MatrixXd base_vector)
{
    Eigen::MatrixXd combined_vector(arm_vector.rows()+base_vector.rows(), arm_vector.cols());
    combined_vector <<  arm_vector,
            base_vector;
    return combined_vector;
}

}
