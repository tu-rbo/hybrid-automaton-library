#include "hybrid_automaton/HybridAutomatonAbstractFactory.h"

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
#include "hybrid_automaton/ROSTopicSensor.h"

#include "hybrid_automaton/DescriptionTreeXML.h"
#include "hybrid_automaton/HybridAutomatonRBOFactory.h"


namespace ha
{
HybridAutomatonAbstractFactory::HybridAutomatonAbstractFactory()
{

}





HybridAutomatonAbstractFactory::~HybridAutomatonAbstractFactory()
{

}

HybridAutomatonAbstractFactory::HybridAutomatonAbstractFactory(const HybridAutomatonAbstractFactory& haf)
{

}

//Eigen::MatrixXd HybridAutomatonAbstractFactory::max_vel_js_arm() const
//{
//    return _max_vel_js_arm;
//}

//void HybridAutomatonAbstractFactory::setMax_vel_js_arm(const Eigen::MatrixXd &max_vel_js_arm)
//{
//    _max_vel_js_arm = max_vel_js_arm;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::max_vel_js_base() const
//{
//    return _max_vel_js_base;
//}

//void HybridAutomatonAbstractFactory::setMax_vel_js_base(const Eigen::MatrixXd &max_vel_js_base)
//{
//    _max_vel_js_base = max_vel_js_base;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kp_js_arm() const
//{
//    return _kp_js_arm;
//}

//void HybridAutomatonAbstractFactory::setKp_js_arm(const Eigen::MatrixXd &kp_js_arm)
//{
//    _kp_js_arm = kp_js_arm;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kp_js_base() const
//{
//    return _kp_js_base;
//}

//void HybridAutomatonAbstractFactory::setKp_js_base(const Eigen::MatrixXd &kp_js_base)
//{
//    _kp_js_base = kp_js_base;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kv_js_arm() const
//{
//    return _kv_js_arm;
//}

//void HybridAutomatonAbstractFactory::setKv_js_arm(const Eigen::MatrixXd &kv_js_arm)
//{
//    _kv_js_arm = kv_js_arm;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kv_js_base() const
//{
//    return _kv_js_base;
//}

//void HybridAutomatonAbstractFactory::setKv_js_base(const Eigen::MatrixXd &kv_js_base)
//{
//    _kv_js_base = kv_js_base;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kp_os_linear() const
//{
//    return _kp_os_linear;
//}

//void HybridAutomatonAbstractFactory::setKp_os_linear(const Eigen::MatrixXd &kp_os_linear)
//{
//    _kp_os_linear = kp_os_linear;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kp_os_angular() const
//{
//    return _kp_os_angular;
//}

//void HybridAutomatonAbstractFactory::setKp_os_angular(const Eigen::MatrixXd &kp_os_angular)
//{
//    _kp_os_angular = kp_os_angular;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kv_os_linear() const
//{
//    return _kv_os_linear;
//}

//void HybridAutomatonAbstractFactory::setKv_os_linear(const Eigen::MatrixXd &kv_os_linear)
//{
//    _kv_os_linear = kv_os_linear;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kv_os_angular() const
//{
//    return _kv_os_angular;
//}

//void HybridAutomatonAbstractFactory::setKv_os_angular(const Eigen::MatrixXd &kv_os_angular)
//{
//    _kv_os_angular = kv_os_angular;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kp_js_nakamura_arm() const
//{
//    return _kp_js_nakamura_arm;
//}

//void HybridAutomatonAbstractFactory::setKp_js_nakamura_arm(const Eigen::MatrixXd &kp_js_nakamura_arm)
//{
//    _kp_js_nakamura_arm = kp_js_nakamura_arm;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kp_js_nakamura_base() const
//{
//    return _kp_js_nakamura_base;
//}

//void HybridAutomatonAbstractFactory::setKp_js_nakamura_base(const Eigen::MatrixXd &kp_js_nakamura_base)
//{
//    _kp_js_nakamura_base = kp_js_nakamura_base;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kv_js_nakamura_arm() const
//{
//    return _kv_js_nakamura_arm;
//}

//void HybridAutomatonAbstractFactory::setKv_js_nakamura_arm(const Eigen::MatrixXd &kv_js_nakamura_arm)
//{
//    _kv_js_nakamura_arm = kv_js_nakamura_arm;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::kv_js_nakamura_base() const
//{
//    return _kv_js_nakamura_base;
//}

//void HybridAutomatonAbstractFactory::setKv_js_nakamura_base(const Eigen::MatrixXd &kv_js_nakamura_base)
//{
//    _kv_js_nakamura_base = kv_js_nakamura_base;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::joint_weights_nakamura_arm() const
//{
//    return _joint_weights_nakamura_arm;
//}

//void HybridAutomatonAbstractFactory::setJoint_weights_nakamura_arm(const Eigen::MatrixXd &joint_weights_nakamura_arm)
//{
//    _joint_weights_nakamura_arm = joint_weights_nakamura_arm;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::joint_weights_nakamura_base() const
//{
//    return _joint_weights_nakamura_base;
//}

//void HybridAutomatonAbstractFactory::setJoint_weights_nakamura_base(const Eigen::MatrixXd &joint_weights_nakamura_base)
//{
//    _joint_weights_nakamura_base = joint_weights_nakamura_base;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::joint_weights_nakamura_base_no_rotation() const
//{
//    return _joint_weights_nakamura_base_no_rotation;
//}

//void HybridAutomatonAbstractFactory::setJoint_weights_nakamura_base_no_rotation(const Eigen::MatrixXd &joint_weights_nakamura_base_no_rotation)
//{
//    _joint_weights_nakamura_base_no_rotation = joint_weights_nakamura_base_no_rotation;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::joint_weights_nakamura_base_little_motion() const
//{
//    return _joint_weights_nakamura_base_little_motion;
//}

//void HybridAutomatonAbstractFactory::setJoint_weights_nakamura_base_little_motion(const Eigen::MatrixXd &joint_weights_nakamura_base_little_motion)
//{
//    _joint_weights_nakamura_base_little_motion = joint_weights_nakamura_base_little_motion;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::home_config_js_arm() const
//{
//    return _home_config_js_arm;
//}

//void HybridAutomatonAbstractFactory::setHome_config_js_arm(const Eigen::MatrixXd &home_config_js_arm)
//{
//    _home_config_js_arm = home_config_js_arm;
//}
//Eigen::MatrixXd HybridAutomatonAbstractFactory::home_config_js_base() const
//{
//    return _home_config_js_base;
//}

//void HybridAutomatonAbstractFactory::setHome_config_js_base(const Eigen::MatrixXd &home_config_js_base)
//{
//    _home_config_js_base = home_config_js_base;
//}
//double HybridAutomatonAbstractFactory::pos_epsilon_js_arm() const
//{
//    return _pos_epsilon_js_arm;
//}

//void HybridAutomatonAbstractFactory::setPos_epsilon_js_arm(const double &pos_epsilon_js_arm)
//{
//    _pos_epsilon_js_arm = pos_epsilon_js_arm;
//}
//double HybridAutomatonAbstractFactory::pos_epsilon_js_base() const
//{
//    return _pos_epsilon_js_base;
//}

//void HybridAutomatonAbstractFactory::setPos_epsilon_js_base(const double &pos_epsilon_js_base)
//{
//    _pos_epsilon_js_base = pos_epsilon_js_base;
//}
//double HybridAutomatonAbstractFactory::vel_epsilon_js_arm() const
//{
//    return _vel_epsilon_js_arm;
//}

//void HybridAutomatonAbstractFactory::setVel_epsilon_js_arm(const double &vel_epsilon_js_arm)
//{
//    _vel_epsilon_js_arm = vel_epsilon_js_arm;
//}
//double HybridAutomatonAbstractFactory::vel_epsilon_js_base() const
//{
//    return _vel_epsilon_js_base;
//}

//void HybridAutomatonAbstractFactory::setVel_epsilon_js_base(const double &vel_epsilon_js_base)
//{
//    _vel_epsilon_js_base = vel_epsilon_js_base;
//}
//double HybridAutomatonAbstractFactory::pos_epsilon_os_linear() const
//{
//    return _pos_epsilon_os_linear;
//}

//void HybridAutomatonAbstractFactory::setPos_epsilon_os_linear(const double &pos_epsilon_os_linear)
//{
//    _pos_epsilon_os_linear = pos_epsilon_os_linear;
//}
//double HybridAutomatonAbstractFactory::pos_epsilon_os_angular() const
//{
//    return _pos_epsilon_os_angular;
//}

//void HybridAutomatonAbstractFactory::setPos_epsilon_os_angular(const double &pos_epsilon_os_angular)
//{
//    _pos_epsilon_os_angular = pos_epsilon_os_angular;
//}
//double HybridAutomatonAbstractFactory::vel_epsilon_os_linear() const
//{
//    return _vel_epsilon_os_linear;
//}

//void HybridAutomatonAbstractFactory::setVel_epsilon_os_linear(const double &vel_epsilon_os_linear)
//{
//    _vel_epsilon_os_linear = vel_epsilon_os_linear;
//}
//double HybridAutomatonAbstractFactory::vel_epsilon_os_angular() const
//{
//    return _vel_epsilon_os_angular;
//}

//void HybridAutomatonAbstractFactory::setVel_epsilon_os_angular(const double &vel_epsilon_os_angular)
//{
//    _vel_epsilon_os_angular = vel_epsilon_os_angular;
//}
ha::HybridAutomaton::Ptr HybridAutomatonAbstractFactory::createInitialHybridAutomaton(const HybridAutomatonAbstractParams& p)
//ha::HybridAutomaton::Ptr HybridAutomatonAbstractFactory::createInitialHybridAutomaton(const GripperType& gripper,
//                                                                              const Eigen::MatrixXd& home_config_js_arm,
//                                                                              const Eigen::MatrixXd& home_config_js_base,
//                                                                              const Eigen::MatrixXd& max_vel_js_arm,
//                                                                              const Eigen::MatrixXd& max_vel_js_base,
//                                                                              const Eigen::MatrixXd& index_vec_arm,
//                                                                              const double& pos_epsilon_js_arm,
//                                                                              const double& grasp_strength,
//                                                                              const int& grasp_type)
{

    //create Hybrid Automaton
    ha::HybridAutomaton::Ptr initial_ha = ha::HybridAutomaton::Ptr(new ha::HybridAutomaton());
    initial_ha->setName("Initial_HA");

    ha::ControlMode::Ptr move_home_initial_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr move_home_initial_cs(new ha::ControlSwitch());

    move_home_initial_cm->setName("move_home_initial_cm");

    ha::Controller::Ptr home_ctrl_arm = createSubjointSpaceControllerArm(p,
                                                                         "move_home_initial_arm_ctrl",
                                                                         p._home_config_js_arm,
                                                                         false
                                                                         );
    ha::Controller::Ptr home_ctrl_base = createSubjointSpaceControllerBase(p,
                                                                           "move_home_initial_base_ctrl",
                                                                           p._home_config_js_base,
                                                                           true);
    std::vector<ha::Controller::Ptr> home_ctrls;
    home_ctrls.push_back(home_ctrl_arm);
    home_ctrls.push_back(home_ctrl_base);
    ha::ControlSet::Ptr goto_home_cs = createJointSpaceControlSet(p,home_ctrls);
    move_home_initial_cm->setControlSet(goto_home_cs);

    //add controller to ControlSet
    ha::Controller::Ptr ungrasp_ctrl(new ha::Controller());
    switch(p.gripper)
    {
    case NO_GRIPPER:
        break;
    case SOFT_HAND:
    {
        ungrasp_ctrl->setName("FeixGraspControl");
        ungrasp_ctrl->setType("FeixGraspSoftHandController");
        ungrasp_ctrl->setArgument("grasp_strength", p.grasp_strength);
        ungrasp_ctrl->setArgument("grasp_type", p.grasp_type);
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
            createSubjointSpaceConvergenceConditionArm(p,home_ctrl_arm);
    ha::JumpConditionPtr initial_convergence_vel_arm_jc =
            createJointSpaceZeroVelocityConditionArm(p);
    ha::JumpConditionPtr initial_convergence_base_jc = createSubjointSpaceConvergenceConditionBase(p,home_ctrl_base);
    ha::JumpConditionPtr initial_convergence_vel_base_jc = createJointSpaceZeroVelocityConditionBase(p);
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

ha::HybridAutomaton::Ptr HybridAutomatonAbstractFactory::createEmptyHybridAutomaton()
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


ha::Controller::Ptr HybridAutomatonAbstractFactory::createSubjointSpaceControllerArm(const HybridAutomatonAbstractParams& params,
                                                                                std::string name,
                                                                             const Eigen::MatrixXd& goal_js_arm,
                                                                             bool is_relative)
{
    return createSubjointSpaceController(params,
                                         name,
                                         goal_js_arm,
                                         params._index_vec_arm,
                                         is_relative);
}

ha::Controller::Ptr HybridAutomatonAbstractFactory::createSubjointSpaceControllerBase(const HybridAutomatonAbstractParams& params,
                                                                                 std::string name,
                                                                              const Eigen::MatrixXd& goal_js_base,
                                                                              bool is_relative)
{
    return createSubjointSpaceController(params,
                                         name,
                                         goal_js_base,
                                         params._index_vec_base,
                                         is_relative);
}


ha::JumpCondition::Ptr HybridAutomatonAbstractFactory::createJointSpaceConvergenceCondition(const HybridAutomatonAbstractParams& p, ha::ControllerConstPtr js_ctrl,double epsilon)
{
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SensorPtr sensor(new ha::JointConfigurationSensor());
    jc->setSensor(sensor);
    jc->setControllerGoal(js_ctrl);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon(epsilon);

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonAbstractFactory::createSubjointSpaceConvergenceCondition(const HybridAutomatonAbstractParams& p,
                                                                                               ha::ControllerConstPtr subjs_ctrl,
                                                                                               const Eigen::MatrixXd& index_vec,
                                                                                               double epsilon)
{
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SubjointConfigurationSensorPtr sensor(new ha::SubjointConfigurationSensor());
    sensor->setIndex(index_vec);
    jc->setSensor(sensor);
    jc->setControllerGoal(subjs_ctrl);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon(epsilon);

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonAbstractFactory::createSubjointSpaceConvergenceConditionArm(const HybridAutomatonAbstractParams& p,
                                                                                                  ha::ControllerConstPtr subjs_ctrl)
{
    return createSubjointSpaceConvergenceCondition(p, subjs_ctrl, p._index_vec_arm,p._pos_epsilon_js_arm);
}

ha::JumpCondition::Ptr HybridAutomatonAbstractFactory::createSubjointSpaceConvergenceConditionBase(const HybridAutomatonAbstractParams& p,
                                                                                                   ha::ControllerConstPtr subjs_ctrl)
{
    return createSubjointSpaceConvergenceCondition(p, subjs_ctrl, p._index_vec_base,p._pos_epsilon_js_base);
}

ha::JumpCondition::Ptr HybridAutomatonAbstractFactory::createJointSpaceVelocityCondition(const HybridAutomatonAbstractParams& p,
                                                                                         const Eigen::MatrixXd& index_vec,
                                                                                         const Eigen::MatrixXd& vel_goal_js,
                                                                                         const double epsilon){
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SubjointVelocitySensorPtr sensor(new ha::SubjointVelocitySensor());
    sensor->setIndex(index_vec);
    jc->setSensor(sensor);
    jc->setConstantGoal(vel_goal_js);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon(epsilon);//p.vel_epsilon_js <= 0 ? std::min(p._vel_epsilon_js_arm,p._vel_epsilon_js_base): p.vel_epsilon_js);

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonAbstractFactory::createJointSpaceZeroVelocityCondition(const HybridAutomatonAbstractParams& p,
                                                                                             const Eigen::MatrixXd& index_vec,
                                                                                             const double epsilon)
{
    Eigen::MatrixXd zero_goal = Eigen::MatrixXd::Constant(index_vec.size(), 1, 0.0);
    return createJointSpaceVelocityCondition(p,
                                             index_vec,
                                             zero_goal,
                                             epsilon);
}

ha::JumpCondition::Ptr HybridAutomatonAbstractFactory::createJointSpaceZeroVelocityConditionArm(const HybridAutomatonAbstractParams& p)
{
    return createJointSpaceZeroVelocityCondition(p,
                                                 p._index_vec_arm,
                                                 p._vel_epsilon_js_arm);
}

ha::JumpCondition::Ptr HybridAutomatonAbstractFactory::createJointSpaceVelocityConditionArm(const HybridAutomatonAbstractParams& p,
                                                                                            const Eigen::MatrixXd& vel_goal_js_arm)
{
    return createJointSpaceVelocityCondition(p,
                                             p._index_vec_arm,
                                             vel_goal_js_arm,
                                             p._vel_epsilon_js_arm);
}

ha::JumpCondition::Ptr HybridAutomatonAbstractFactory::createJointSpaceZeroVelocityConditionBase(const HybridAutomatonAbstractParams& p)
{
    return createJointSpaceZeroVelocityCondition(p, p._index_vec_base,p._vel_epsilon_js_base);
}

ha::JumpCondition::Ptr HybridAutomatonAbstractFactory::createOperationalSpaceConvergenceCondition(const HybridAutomatonAbstractParams& p,
                                                                                                  ha::ControllerConstPtr ctrl,
                                                                                                  bool relative,
                                                                                                  bool only_displacement)
{
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    if(!only_displacement)
    {
        ha::SensorPtr sensor(new ha::FramePoseSensor());
        jc->setSensor(sensor);
        jc->setControllerGoal(ctrl);
        jc->setJumpCriterion(ha::JumpCondition::NORM_TRANSFORM);

        jc->setEpsilon(p._pos_epsilon_os_linear);
    }else{
        ha::SensorPtr sensor(new ha::FrameDisplacementSensor());
        jc->setSensor(sensor);
        jc->setControllerGoal(ctrl);
        jc->setJumpCriterion(ha::JumpCondition::NORM_L2);

        jc->setEpsilon(p._pos_epsilon_os_linear);
    }



    if(relative)
        jc->setGoalRelative();
    else
        jc->setGoalAbsolute();

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonAbstractFactory::createMaxTimeCondition(const HybridAutomatonAbstractParams& p,
                                                                              double max_time){
    ha::JumpCondition::Ptr max_time_cond(new ha::JumpCondition());
    ha::ClockSensor::Ptr time_sensor(new ha::ClockSensor());
    max_time_cond->setSensor(time_sensor);
    max_time_cond->setConstantGoal(max_time);
    max_time_cond->setGoalRelative();
    max_time_cond->setEpsilon(0);
    max_time_cond->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    return max_time_cond;
}

ha::ControlSwitch::Ptr HybridAutomatonAbstractFactory::CreateMaxForceTorqueControlSwitch(const HybridAutomatonAbstractParams& p,
                                                                                         const std::string& name,
                                                                                 const Eigen::MatrixXd& ft_weights,
                                                                                 const Eigen::MatrixXd& ft_max_val,
                                                                                 const ha::JumpCondition::JumpCriterion ft_criterion,
                                                                                 const bool negate_ft_condition,
                                                                                 const float epsilon)
{
    ha::ControlSwitch::Ptr max_ft_cs(new ha::ControlSwitch());
    CreateMaxForceTorqueControlSwitch(p, max_ft_cs, name, ft_weights, ft_max_val, ft_criterion, negate_ft_condition,epsilon);
    return max_ft_cs;
}

void HybridAutomatonAbstractFactory::CreateMaxForceTorqueControlSwitch(const HybridAutomatonAbstractParams& p,
                                                                       const ha::ControlSwitch::Ptr& cs_ptr,
                                                               const std::string& name,
                                                               const Eigen::MatrixXd& ft_weights,
                                                               const Eigen::MatrixXd& ft_max_val,
                                                               const ha::JumpCondition::JumpCriterion ft_criterion,
                                                               const bool negate_ft_condition,
                                                               const float epsilon)
{
    cs_ptr->setName(name + std::string("_max_ft_cs"));
    ha::JumpConditionPtr max_ft_jc(new ha::JumpCondition());
    ha::ForceTorqueSensorPtr ft_sensor(new ha::ForceTorqueSensor());
    max_ft_jc->setSensor(ft_sensor);
    max_ft_jc->setGoalRelative();
    max_ft_jc->setConstantGoal(ft_max_val);
    max_ft_jc->setJumpCriterion(ft_criterion ,ft_weights);
    max_ft_jc->setEpsilon(epsilon);
    max_ft_jc->setNegate(negate_ft_condition);
    cs_ptr->add(max_ft_jc);
}

ha::ControlSwitch::Ptr HybridAutomatonAbstractFactory::CreateMaxTimeControlSwitch(const HybridAutomatonAbstractParams& p,const std::string& mode_name, double max_time){
    ha::ControlSwitchPtr time_switch(new ha::ControlSwitch);
    time_switch->setName(mode_name + "_time_cs");
    ha::JumpConditionPtr time_cond = createMaxTimeCondition(p, max_time);
    time_switch->add(time_cond);
    return time_switch;
}

void  HybridAutomatonAbstractFactory::CreateGCCM(const HybridAutomatonAbstractParams& p,
                                                 const ha::ControlMode::Ptr& cm_ptr,
                                         const std::string& name){
    cm_ptr->setName(name);
    ha::ControlSet::Ptr gravity_cs(new ha::ControlSet());
    gravity_cs->setType("rxControlSet");
    cm_ptr->setControlSet(gravity_cs);
}

void HybridAutomatonAbstractFactory::CreateGoToHomeCMAndConvergenceCSArm(const HybridAutomatonAbstractParams& p,
                                                                         const ha::ControlMode::Ptr& cm_ptr,
                                                                 const ha::ControlSwitch::Ptr& cs_ptr,
                                                                 const std::string& name){
    cm_ptr->setName(name + std::string("_cm"));

    //ha::Controller::Ptr home_ctrl_arm = _createJointSpaceArmController(name + std::string("_arm_ctrl"), goal_cfg, _max_js_vel_arm);
    ha::Controller::Ptr home_ctrl_arm = createSubjointSpaceControllerArm(p,
                                                                         name + std::string("_arm_ctrl"),
                                                                         p._home_config_js_arm,
                                                                         false);


    ha::ControlSet::Ptr goto_home_cs =  createJointSpaceControlSet(p,
                                                         home_ctrl_arm);

    cm_ptr->setControlSet(goto_home_cs);

    //Create first ControlSwitch
    cs_ptr->setName(name + std::string("_cs"));
    //ha::JumpConditionPtr initial_convergence_arm_jc = _createJointSpaceArmConvergenceCondition(home_ctrl_arm);
    ha::JumpCondition::Ptr initial_convergence_arm_jc = createSubjointSpaceConvergenceConditionArm(p,
                                                                                                   home_ctrl_arm);


    //ha::JumpConditionPtr initial_convergence_vel_arm_jc = _createJointSpaceArmConvergenceWithZeroVelCondition();
    ha::JumpCondition::Ptr initial_convergence_vel_arm_jc = createJointSpaceZeroVelocityConditionArm(p);

    //ha::JumpConditionPtr initial_convergence_base_jc = _createJointSpaceBaseConvergenceCondition(home_ctrl_base);
    cs_ptr->add(initial_convergence_arm_jc);
    cs_ptr->add(initial_convergence_vel_arm_jc);
    //cs_ptr->add(initial_convergence_base_jc);
}




void HybridAutomatonAbstractFactory::CreateGraspCMAndCS(const HybridAutomatonAbstractParams& p,
                                                        const ha::ControlMode::Ptr& cm_ptr,
                                                const ha::ControlSwitch::Ptr& cs_ptr,
                                                const std::string& name
//                                                        ,
//                                                const GripperType& gripper,
//                                                const Eigen::MatrixXd& kp_grasp,
//                                                const Eigen::MatrixXd& kv_grasp,
//                                                const double grasp_strength,
//                                                const int grasp_type
                                                        ){
    cm_ptr->setName(name+ std::string("_cm"));
    ha::ControlSet::Ptr grasp_cs(new ha::ControlSet());

    grasp_cs->setType("rxControlSet");
    cm_ptr->setControlSet(grasp_cs);

    ha::Controller::Ptr grasp_ctrl = createGraspController(p,name + std::string("_grasp_ctrl"));
Eigen::MatrixXd grasp_zeros_goal = Eigen::MatrixXd::Constant(p.num_dof_arm, 1, 0.0);

    ha::Controller::Ptr grasp_joint_ctrl = createSubjointSpaceControllerArm(p,
                                                                            name + std::string("_grasp_joint_ctrl"),
                                                                            grasp_zeros_goal,
                                                                            true);



//    grasp_joint_ctrl->setName("grasp_joint_ctrl");
//    grasp_joint_ctrl->setType("JointController");

//    grasp_joint_ctrl->setGoal(grasp_zeros_goal);
//    grasp_joint_ctrl->setGoalIsRelative(1);
//    grasp_joint_ctrl->setKp((kp_grasp.size() == 0 ? _combineArmAndBase(_kp_js_arm, _kp_js_base) : kp_grasp));
//    grasp_joint_ctrl->setKv((kv_grasp.size() == 0 ? _combineArmAndBase(_kv_js_arm, _kv_js_base) : kv_grasp));


    if(p.gripper!=NO_GRIPPER){
        //add controller to ControlSet
        grasp_cs->appendController(grasp_ctrl);
        grasp_cs->appendController(grasp_joint_ctrl);
    }

    //Create ControlSwitch
    cs_ptr->setName(name + std::string("_pressure_time_cs"));

    ha::JumpConditionPtr to_move_up_jc(new ha::JumpCondition());
    ha::SensorPtr to_move_up_clock_sensor(new ha::ClockSensor());
    to_move_up_jc->setSensor(to_move_up_clock_sensor);
    to_move_up_jc->setConstantGoal(3.0);
    to_move_up_jc->setGoalRelative();
    to_move_up_jc->setEpsilon(0);
    to_move_up_jc->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    cs_ptr->add(to_move_up_jc);

    if(p.gripper == SUCTION_CUP) //Vacuum cleaner
    {
        ha::JumpConditionPtr to_move_up_pressure_jc(new ha::JumpCondition());
        ha::ROSTopicSensor::Ptr to_move_up_pressure_sensor(new ha::ROSTopicSensor());
        to_move_up_pressure_sensor->setTopic("/pneumaticbox/pressure_0", "Float64");
        to_move_up_pressure_jc->setSensor(to_move_up_pressure_sensor);
        to_move_up_pressure_jc->setConstantGoal(18.8);
        to_move_up_pressure_jc->setEpsilon(0);
        to_move_up_pressure_jc->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
        cs_ptr->add(to_move_up_pressure_jc);
    }
}

//void HybridAutomatonAbstractFactory::CreateUngraspCMAndCS(const HybridAutomatonAbstractParams& p,
//                                                          const ha::ControlMode::Ptr& cm_ptr,
//                                                  const ha::ControlSwitch::Ptr& cs_ptr,
//                                                  const ha::ControlSwitch::Ptr& cs_ptr2,
//                                                  const std::string& name){
//    cm_ptr->setName(name+ std::string("_cm"));
//    ha::ControlSet::Ptr ungrasp_cs(new ha::ControlSet());
//    ungrasp_cs->setType("rxControlSet");
//    cm_ptr->setControlSet(ungrasp_cs);

//    //add controller to ControlSet
//    ha::Controller::Ptr ungrasp_ctrl(new ha::Controller());

//    switch(gripper)
//    {
//    case NO_GRIPPER:
//        break;
//    case SOFT_HAND:
//    {
//        ungrasp_ctrl->setName("FeixGraspControl");
//        ungrasp_ctrl->setType("FeixGraspSoftHandController");
//        ungrasp_ctrl->setArgument("grasp_strength", grasp_strength);
//        ungrasp_ctrl->setArgument("grasp_type", grasp_type);
//        ungrasp_cs->appendController(ungrasp_ctrl);
//    }
//        break;
//    case SUCTION_CUP:
//    {
//        ungrasp_ctrl->setName("VacuumGraspControl");
//        ungrasp_ctrl->setType("VacuumCleanerController");
//        ungrasp_ctrl->setArgument("power", "0");
//        ungrasp_cs->appendController(ungrasp_ctrl);
//    }
//        break;
//    case BARRETT_HAND:
//        break;
//    default:
//        break;
//    }

//    Eigen::MatrixXd drop_zeros_goal(7,1);
//    drop_zeros_goal << 0,0,0,0,0,0,0;
//    ha::Controller::Ptr ungrasp_joint_ctrl = createJointSpaceController(p, name + std::string("_drop_joint_ctrl"),
//                                                                        drop_zeros_goal, 1.0, true);
//    ungrasp_cs->appendController(ungrasp_joint_ctrl);

//    cs_ptr->setName(name + std::string("_pressure_cs"));
//    ha::JumpConditionPtr to_stop_or_pressure_jc(new ha::JumpCondition());
//    ha::ROSTopicSensor::Ptr to_stop_or_sensor(new ha::ROSTopicSensor());
//    to_stop_or_sensor->setTopic("/pneumaticbox/pressure_0", "Float64");
//    to_stop_or_pressure_jc->setSensor(to_stop_or_sensor);
//    to_stop_or_pressure_jc->setConstantGoal(0.5);
//    to_stop_or_pressure_jc->setEpsilon(0);
//    to_stop_or_pressure_jc->setJumpCriterion(ha::JumpCondition::THRESH_LOWER_BOUND);
//    cs_ptr->add(to_stop_or_pressure_jc);

//    // To free from adherence wait 15 seconds or for pressure sensor
//    cs_ptr2->setName(name + std::string("_time_cs"));
//    ha::JumpConditionPtr to_stop_or_time_jc(new ha::JumpCondition());
//    ha::SensorPtr free_sensor_time(new ha::ClockSensor());
//    to_stop_or_time_jc->setSensor(free_sensor_time);
//    switch(gripper)
//    {
//    case SOFT_HAND:
//        to_stop_or_time_jc->setConstantGoal(3);
//        break;
//    case SUCTION_CUP:
//    case NO_GRIPPER:
//    case BARRETT_HAND:
//    default:
//        to_stop_or_time_jc->setConstantGoal(15);
//        break;
//    }

//    to_stop_or_time_jc->setGoalRelative();
//    to_stop_or_time_jc->setEpsilon(0);
//    to_stop_or_time_jc->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
//    cs_ptr2->add(to_stop_or_time_jc);
//}

void HybridAutomatonAbstractFactory::CreateGoToBBCMAndConvergenceCS(const HybridAutomatonAbstractParams& p,
                                                                    const ha::ControlMode::Ptr& cm_ptr,
                                                            const ha::ControlSwitch::Ptr& cs_ptr,
                                                            const std::string& name,
                                                            const std::string& frame_name,
                                                            const std::string& parent_frame_name,
                                                            bool use_tf,
                                                            bool use_base,
                                                            bool is_relative){
    cm_ptr->setName(name + std::string("_cm"));

    //Endeffector Frame Controller
    ha::Controller::Ptr bb_ctrl;
    bb_ctrl = createBBOperationalSpaceController(p,
                name + std::string("_ctrl"),
                                                 false,
                                                 use_tf,
                                                 frame_name,
                                                 parent_frame_name,
                                                 is_relative);

    ha::ControlSet::Ptr bb_cs = createTaskSpaceControlSet(p, bb_ctrl, use_base);
    cm_ptr->setControlSet(bb_cs);

    cs_ptr->setName(name + std::string("_cs"));
    ha::JumpConditionPtr convergence_jc = createOperationalSpaceConvergenceCondition(p, bb_ctrl,
                                                                                     is_relative);
    cs_ptr->add(convergence_jc);
}

void HybridAutomatonAbstractFactory::CreateGoToCMAndConvergenceCS(const HybridAutomatonAbstractParams& p,
                                                                  const ha::ControlMode::Ptr& cm_ptr,
                                                          const ha::ControlSwitch::Ptr& cs_ptr,
                                                          const std::string& name,
                                                          const Eigen::MatrixXd &goal_op_pos,
                                                          const Eigen::MatrixXd &goal_op_ori,
                                                          bool use_base,

                                                          bool is_relative
                                                          )
{
    cm_ptr->setName(name + std::string("_cm"));

    //Endeffector Frame Controller
    ha::Controller::Ptr os_ctrl;
    os_ctrl = createOperationalSpaceController(p,
                name + std::string("_ctrl"),
                                               goal_op_pos,
                                               goal_op_ori,
                                               is_relative);

    ha::ControlSet::Ptr os_cs = createTaskSpaceControlSet(p, os_ctrl, use_base);
    cm_ptr->setControlSet(os_cs);

    bool only_displacement = false;
    if(goal_op_ori.size() == 0)
    {
        only_displacement = true;
    }

    cs_ptr->setName(name + std::string("_cs"));
    ha::JumpConditionPtr convergence_jc = createOperationalSpaceConvergenceCondition(p, os_ctrl,
                                                                                     is_relative,
                                                                                     only_displacement);
    cs_ptr->add(convergence_jc);
}

void HybridAutomatonAbstractFactory::CreateGoToCMConvergenceCSAndMaxForceCS(const HybridAutomatonAbstractParams& p,
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
                                                                    bool is_relative)
{
    std::cout<<"generating gotocmandconvergence"<<std::endl;
    CreateGoToCMAndConvergenceCS(p, cm_ptr, convergence_cs_ptr, name, goal_op_pos, goal_op_ori,
                                 use_base,
                                 is_relative);
    std::cout<<"generating maxftcs"<<std::endl;
    CreateMaxForceTorqueControlSwitch(p, max_force_cs_ptr,name, ft_weights, max_ft, ft_criterion,negate_ft_condition,ft_epsilon);

}

Eigen::MatrixXd HybridAutomatonAbstractFactory::_combineArmAndBase(const Eigen::MatrixXd& arm_vector, const Eigen::MatrixXd base_vector)
{
    Eigen::MatrixXd combined_vector(arm_vector.rows()+base_vector.rows(), arm_vector.cols());
    combined_vector <<  arm_vector,
            base_vector;
    return combined_vector;
}

std::string HybridAutomatonAbstractFactory::HybridAutomatonToString(ha::HybridAutomaton::ConstPtr ha)
{
    ha::DescriptionTreeXML::Ptr tree(new ha::DescriptionTreeXML);
    ha::DescriptionTreeNode::Ptr ha_serialized;

    try{
        ha_serialized = ha->serialize(tree);
    }
    catch(std::string err)
    {
        std::cerr << "[HybridAutomatonAbstractFactory.HybridAutomatonToString] Failed to serialize Hybrid Automaton: " << err << std::endl;
    }

    tree->setRootNode(ha_serialized);
    return tree->writeTreeXML();
}
}
