#include "hybrid_automaton/HybridAutomatonRBOFactory.h"

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

#include <exception>

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

#define DEFAULT_MAX_VEL_OS_LINEAR 0.03
#define DEFAULT_MAX_VEL_OS_ANGULAR 0.5

#define DEFAULT_UPDATE_RATE 100

namespace ha
{
HybridAutomatonRBOFactory::HybridAutomatonRBOFactory()
{
    //_initializeDefaultValues();
}


HybridAutomatonRBOFactory::~HybridAutomatonRBOFactory()
{

}

HybridAutomatonRBOFactory::HybridAutomatonRBOFactory(const HybridAutomatonRBOFactory& haf)
{

}



ha::HybridAutomaton::Ptr HybridAutomatonRBOFactory::createEmptyHybridAutomaton()
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

ha::ControlSet::Ptr HybridAutomatonRBOFactory::createControlSet(const HybridAutomatonAbstractParams& params, ha::Controller::Ptr ctrl)
{
    ha::ControlSet::Ptr cs(new ha::ControlSet());
    cs->setType("rxControlSet");
    cs->appendController(ctrl);
    return cs;
}

ha::ControlSet::Ptr HybridAutomatonRBOFactory::createControlSet(const HybridAutomatonAbstractParams& params, const std::vector<ha::Controller::Ptr>& ctrls)
{
    ha::ControlSet::Ptr cs(new ha::ControlSet());
    cs->setType("rxControlSet");
    for(int i=0; i<ctrls.size(); i++)
        cs->appendController(ctrls.at(i));
    return cs;
}

ha::ControlSet::Ptr HybridAutomatonRBOFactory::createTaskSpaceControlSet(const HybridAutomatonAbstractParams& params, ha::Controller::Ptr ctrl, bool move_base)
{
    HybridAutomatonRBOParams& p = (HybridAutomatonRBOParams&)params;
    ha::ControlSet::Ptr cs(new ha::ControlSet());
    cs->setType("TPNakamuraControlSet");
    cs->setArgument<Eigen::MatrixXd>("js_kp", _combineArmAndBase(p._kp_js_nakamura_arm,
                                                                 p._kp_js_nakamura_base));
    cs->setArgument<Eigen::MatrixXd>("js_kd", _combineArmAndBase(p._kv_js_nakamura_arm,
                                                                 p._kv_js_nakamura_base));

    if(move_base)
        cs->setArgument<Eigen::MatrixXd>("joint_weights",
                                         _combineArmAndBase(p._joint_weights_nakamura_arm,
                                                            p._joint_weights_nakamura_base_no_rotation));
    else
        cs->setArgument<Eigen::MatrixXd>("joint_weights",
                                         _combineArmAndBase(p._joint_weights_nakamura_arm,
                                                            p._joint_weights_nakamura_base_little_motion));


    cs->appendController(ctrl);
    return cs;
}

ha::Controller::Ptr HybridAutomatonRBOFactory::createJointSpaceController(const HybridAutomatonAbstractParams& params,
                                                                          std::string name,
                                                                          const Eigen::MatrixXd &goal_js,
                                                                          double completion_time,
                                                                          bool goal_relative)
{
    HybridAutomatonRBOParams& p = (HybridAutomatonRBOParams&)params;
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("InterpolatedJointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ctrl->setGoal(goal_js);
    ctrl->setKp(_combineArmAndBase(p.kp_js_arm, p.kp_js_base));
    ctrl->setKv(_combineArmAndBase(p.kv_js_arm, p.kv_js_base));
    ctrl->setCompletionTime(completion_time);
    ctrl->setGoalIsRelative(goal_relative);
    return ctrl;
}


ha::Controller::Ptr HybridAutomatonRBOFactory::createSubjointSpaceController(const HybridAutomatonAbstractParams& params,
                                                                             std::string name,
                                                                             const Eigen::MatrixXd& goal_js,
                                                                             const Eigen::MatrixXd& index_vec,
                                                                             bool is_relative)
{
    HybridAutomatonRBOParams& p = (HybridAutomatonRBOParams&)params;
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("InterpolatedSubjointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ha_ostringstream index_vec_ss;
    index_vec_ss << index_vec;
    ctrl->setArgument("index", index_vec_ss.str());
    ctrl->setGoal(goal_js);
    Eigen::VectorXd kp_js_masked = mask_by_index(p.kp_js,index_vec);
    ctrl->setKp(kp_js_masked);
    Eigen::VectorXd kv_js_masked = mask_by_index(p.kv_js,index_vec);
    ctrl->setKv(kv_js_masked);
    Eigen::VectorXd max_vel_js_masked = mask_by_index(p.max_vel_js,index_vec);
    ctrl->setMaximumVelocity(max_vel_js_masked);
    ctrl->setGoalIsRelative(is_relative);
    return ctrl;
}



ha::Controller::Ptr HybridAutomatonRBOFactory::createBBSubjointSpaceController(const HybridAutomatonAbstractParams& params,
                                                                               std::string name,
                                                                               bool use_tf,
                                                                               const std::string& topic_name,
                                                                               const std::string& tf_parent,
                                                                               const Eigen::MatrixXd& index_vec,
                                                                               bool is_relative)
{
    HybridAutomatonRBOParams& p = (HybridAutomatonRBOParams&)params;
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("BlackboardInterpolatedSubjointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ha_ostringstream index_vec_ss;
    index_vec_ss << index_vec;
    ctrl->setArgument("index", index_vec_ss.str());
    ctrl->setArgument("reinterpolation", "1");
    if (use_tf){
        ctrl->setArgument("use_tf", "1");
        ctrl->setArgument("tf_parent", tf_parent);
    } else {
        ctrl->setArgument("use_tf", "0");
    }
    ctrl->setArgument("topic_name", topic_name);
    ctrl->setArgument("update_rate", p._update_rate);
    ctrl->setKp(_combineArmAndBase(p.kp_js_arm, p.kp_js_base));
    ctrl->setKv(_combineArmAndBase(p.kv_js_arm, p.kv_js_base));
    ctrl->setMaximumVelocity(p.max_vel_js);
    ctrl->setGoalIsRelative(is_relative);
    return ctrl;
}



ha::Controller::Ptr HybridAutomatonRBOFactory::createBBSubjointSpaceControllerBase(const HybridAutomatonAbstractParams& params,
                                                                                   std::string name,
                                                                                   bool use_tf,
                                                                                   const std::string& topic_name,
                                                                                   const std::string& tf_parent,
                                                                                   bool is_relative
                                                                                   )
{
    HybridAutomatonRBOParams& p = (HybridAutomatonRBOParams&)params;
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("BlackboardInterpolatedSubjointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ha_ostringstream index_vec_ss;
    index_vec_ss << p._index_vec_base ;
    ctrl->setArgument("index", index_vec_ss.str());
    ctrl->setArgument("reinterpolation", "1");
    if (use_tf){
        ctrl->setArgument("use_tf", "1");
        ctrl->setArgument("tf_parent", tf_parent);
    } else {
        ctrl->setArgument("use_tf", "0");
    }
    ctrl->setArgument("topic_name", topic_name);
    ctrl->setArgument("tf_parent", tf_parent);
    ctrl->setArgument("update_rate", p._update_rate);
    ctrl->setKp(p.kp_js_base);
    ctrl->setKv(p.kv_js_base);
    ctrl->setMaximumVelocity(p.max_vel_js_base);
    ctrl->setGoalIsRelative(is_relative);
    return ctrl;
}


ha::Controller::Ptr HybridAutomatonRBOFactory::createOperationalSpaceController(const HybridAutomatonAbstractParams& params,
                                                                                std::string name,
                                                                                const Eigen::MatrixXd &goal_op_translation,
                                                                                const Eigen::MatrixXd &goal_op_rot_matrix,
                                                                                double completion_time,
                                                                                bool is_relative){
    HybridAutomatonRBOParams& p = (HybridAutomatonRBOParams&)params;
    Eigen::MatrixXd bin_home_frame;
    bin_home_frame.resize(4,4);
    bin_home_frame.setIdentity();
    if(goal_op_rot_matrix.size() != 0)
    {
        bin_home_frame.block(0,0,3,3) = goal_op_rot_matrix;
    }
    bin_home_frame.block(0,3,3,1) = goal_op_translation;

    //Endeffector Frame Controller
    ha::Controller::Ptr ctrl(new ha::Controller);
    ctrl->setName(name);
    ctrl->setType("InterpolatedHTransformController");
    ctrl->setArgument("interpolation_type", "cubic");

    ctrl->setGoal(bin_home_frame);
    ctrl->setKp(_combineArmAndBase(p._kp_os_linear,
                                   p._kp_os_angular));
    ctrl->setKv(_combineArmAndBase(p._kv_os_linear,
                                   p._kv_os_angular));

    ctrl->setCompletionTime(completion_time);
    ctrl->setGoalIsRelative(is_relative);
    ctrl->setArgument("operational_frame", "EE");

    return ctrl;
}

ha::Controller::Ptr HybridAutomatonRBOFactory::createOperationalSpaceController(const HybridAutomatonAbstractParams& params,
                                                                                std::string name,
                                                                                const Eigen::MatrixXd &goal_op_translation,
                                                                                const Eigen::MatrixXd &goal_op_rot_matrix,
                                                                                bool is_relative)
{
    HybridAutomatonRBOParams& p = (HybridAutomatonRBOParams&)params;
    ha::Controller::Ptr ctrl(new ha::Controller);
    Eigen::MatrixXd os_goal;

    if(goal_op_rot_matrix.size() != 0)
    {

        int n_targets = goal_op_translation.cols();
        int os_goal_rows = 4;
        int os_goal_cols = 4 * n_targets;

        os_goal.resize(os_goal_rows,os_goal_cols);
        //os_goal.setIdentity();

        for(int i=0; i<n_targets;++i){
            int offset = 4 * i;
            int offset_trans = 1 * i;
            int offset_rot = 3 * i;

            //set the translation
            os_goal.block(0,offset,4,4).setIdentity();
            //set the rotation
            os_goal.block(0,offset + 0,3,3) = goal_op_rot_matrix.block(0,offset_rot + 0, 3, 3);

            os_goal.block(0,offset + 3,3,1) = goal_op_translation.block(0, offset_trans + 0, 3, 1);

        }

        //Endeffector Frame Controller

        ctrl->setName(name);
        ctrl->setType("InterpolatedHTransformController");
        ctrl->setArgument("interpolation_type", "cubic");

        ctrl->setGoal(os_goal);
        ctrl->setKp(_combineArmAndBase(p._kp_os_linear,
                                       p._kp_os_angular));
        ctrl->setKv(_combineArmAndBase(p._kv_os_linear,
                                       p._kv_os_angular));

        Eigen::MatrixXd max_vel(2,1);
        max_vel <<  p._max_vel_os_angular,p._max_vel_os_linear;
        ctrl->setMaximumVelocity(max_vel);
        ctrl->setGoalIsRelative(is_relative);
        ctrl->setArgument("operational_frame", "EE");

    }else{
        os_goal = goal_op_translation;

        //Endeffector Frame Controller
        ctrl->setName(name);
        ctrl->setType("InterpolatedDisplacementController");
        ctrl->setArgument("interpolation_type", "cubic");

        ctrl->setGoal(os_goal);
        ctrl->setKp(p._kp_os_linear);
        ctrl->setKv(p._kv_os_linear);

        Eigen::MatrixXd max_vel = Eigen::MatrixXd::Constant(3,1,(p._max_vel_os_linear));
        ctrl->setMaximumVelocity(max_vel);
        ctrl->setGoalIsRelative(is_relative);
        ctrl->setArgument("operational_frame", "EE");

    }

    return ctrl;
}

ha::Controller::Ptr HybridAutomatonRBOFactory::createGraspController(const HybridAutomatonAbstractParams& params,
                                                                     std::string name){
    HybridAutomatonRBOParams& p = (HybridAutomatonRBOParams&)params;
    ha::Controller::Ptr grasp_ctrl(new ha::Controller());

    switch(p.gripper)
    {
    case NO_GRIPPER:
        break;
    case SOFT_HAND:
    {
        grasp_ctrl->setName("FeixGraspControl");
        grasp_ctrl->setType("FeixGraspSoftHandController");
        grasp_ctrl->setArgument("grasp_strength", p.grasp_strength);
        grasp_ctrl->setArgument("grasp_type", p.grasp_type);
    }
        break;
    case SOFT_HAND_8CHANNELS:
    {
        grasp_ctrl->setName("FeixGraspControl");
        grasp_ctrl->setType("FeixGraspSoftHand8ChannelsController");
        grasp_ctrl->setArgument("grasp_strength", p.grasp_strength);
        grasp_ctrl->setArgument("grasp_type", p.grasp_type);
    }
        break;
    case SUCTION_CUP:
    {
        grasp_ctrl->setName("VacuumGraspControl");
        grasp_ctrl->setType("VacuumCleanerController");
        grasp_ctrl->setArgument("power", p.grasp_strength);
    }
        break;
    case BARRETT_HAND:
        break;
    default:
        throw std::invalid_argument( "received unkown grasp_type" );
        break;
    }
    return grasp_ctrl;
}


ha::Controller::Ptr HybridAutomatonRBOFactory::createBBOperationalSpaceController(const HybridAutomatonAbstractParams& params,
                                                                                  std::string name,
                                                                                  bool trajectory,
                                                                                  bool use_tf,
                                                                                  const std::string frame,
                                                                                  const std::string parent_frame,
                                                                                  bool is_relative){
    HybridAutomatonRBOParams& p = (HybridAutomatonRBOParams&)params;
    //Endeffector Frame Controller
    ha::Controller::Ptr ctrl(new ha::Controller);
    ctrl->setName(name);
    if(trajectory)
        ctrl->setType("BlackboardInterpolatedHTransformTrajectoryController");
    else
        ctrl->setType("BlackboardInterpolatedHTransformController");
    ctrl->setArgument("interpolation_type", "cubic");
    ctrl->setArgument("reinterpolation", "1");
    if (use_tf){
        ctrl->setArgument("use_tf", "1");
        ctrl->setArgument("tf_parent", parent_frame);
    } else {
        ctrl->setArgument("use_tf", "0");
    }
    ctrl->setArgument("topic_name", frame);

    ctrl->setArgument("update_rate", p._update_rate);

    Eigen::MatrixXd max_vel(2,1);
    max_vel <<  p._max_vel_os_angular, p._max_vel_os_linear;
    ctrl->setMaximumVelocity(max_vel);

    ctrl->setKp(_combineArmAndBase(p._kp_os_linear,
                                   p._kp_os_angular));
    ctrl->setKv(_combineArmAndBase(p._kv_os_linear,
                                   p._kv_os_angular));
    ctrl->setGoalIsRelative(is_relative);
    ctrl->setArgument("operational_frame", "EE");

    return ctrl;

}



Eigen::MatrixXd _combineArmAndBase(const Eigen::MatrixXd& arm_vector, const Eigen::MatrixXd base_vector)
{
    Eigen::MatrixXd combined_vector(arm_vector.rows()+base_vector.rows(), arm_vector.cols());
    combined_vector <<  arm_vector,
            base_vector;
    return combined_vector;
}
Eigen::MatrixXd mask_by_index(const Eigen::MatrixXd m, const Eigen::MatrixXd index_vec)
{
    Eigen::VectorXd masked;
        masked.resize(index_vec.size());
        for (int i=0;i<index_vec.size();++i){
            masked(i)=m(index_vec(i));
        }
        return masked;
}

std::string HybridAutomatonRBOFactory::HybridAutomatonToString(ha::HybridAutomaton::ConstPtr ha)
{
    ha::DescriptionTreeXML::Ptr tree(new ha::DescriptionTreeXML);
    ha::DescriptionTreeNode::Ptr ha_serialized;

    try{
        ha_serialized = ha->serialize(tree);
    }
    catch(std::string err)
    {
        std::cerr << "[HybridAutomatonRBOFactory.HybridAutomatonToString] Failed to serialize Hybrid Automaton: " << err << std::endl;
    }

    tree->setRootNode(ha_serialized);
    return tree->writeTreeXML();
}

void  HybridAutomatonRBOFactory::CreateGCCM(const ha::HybridAutomatonAbstractParams& p,
                                                 const ha::ControlMode::Ptr& cm_ptr,
                                         const std::string& name){
    cm_ptr->setName(name);
    ha::ControlSet::Ptr gravity_cs(new ha::ControlSet());
    gravity_cs->setType("rxControlSet");
    cm_ptr->setControlSet(gravity_cs);
}
} // namespace
