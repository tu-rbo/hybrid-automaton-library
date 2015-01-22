#include <iostream>
#include <fstream>

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/JointConfigurationSensor.h"
#include "hybrid_automaton/ForceTorqueSensor.h"
#include "hybrid_automaton/ClockSensor.h"

#include "hybrid_automaton/DescriptionTreeXML.h"

using namespace ha;

int main() {

//    HybridAutomaton ha;

//    ControlModePtr cm1(new ControlMode("cm1"));
//    ControlModePtr cm2(new ControlMode("cm2"));

//    ControlSetPtr cset1(new ControlSet);
//    cset1->setName("MyControlSet");

//    ControlSwitchPtr cs1(new ControlSwitch);
//    cs1->setName("CS1");
//    JumpConditionPtr jc1(new JumpCondition);
//    cs1->add(jc1);

//    ha.addControlMode(cm1);
//    ha.addControlMode(cm2);
//    ha.addControlSwitch("cm1", cs1, "cm2");

////    std::cout << ha.existsControlMode("cm1") << std::endl;
////    std::cout << ha.existsControlMode("cmfe1") << std::endl;
//    std::cout << cm1 << std::endl;
//    std::cout << ha.getControlModeByName("cm1") << std::endl;


//    //The destination of the output file
//    _outFile.open ("\\\\tsclient\\run_ha\\ha_FT_grasp.xml"); //("C:/rswin/applications/trajectory_recorder/grasp_ha.xml");

    // joint space gains
    Eigen::MatrixXd kp(7, 1);
    Eigen::MatrixXd kv(7, 1);

    kp(0,0) = 900.0;
    kp(1,0) = 2500.0;
    kp(2,0) = 600.0;
    kp(3,0) = 500.0;
    kp(4,0) = 10.0;
    kp(5,0) = 10.0;
    kp(6,0) = 10.0;
    kv(0,0) = 10.0;
    kv(1,0) = 20.0;
    kv(2,0) = 5.0;
    kv(3,0) = 2.0;
    kv(4,0) = 0.5;
    kv(5,0) = 0.5;
    kv(6,0) = 0.05;


    ha::HybridAutomaton::Ptr new_ha = ha::HybridAutomaton::Ptr(new ha::HybridAutomaton());
    new_ha->setName("GraspHA");

    ha::ControlMode::Ptr new_cm(new ha::ControlMode());
    ha::ControlMode::Ptr new_cm2(new ha::ControlMode());
    ha::ControlMode::Ptr grasp_cm(new ha::ControlMode());
    ha::ControlMode::Ptr new_cm3(new ha::ControlMode());
    ha::ControlMode::Ptr deflate_cm(new ha::ControlMode());
    ha::ControlMode::Ptr grav_cm(new ha::ControlMode());

    new_cm->setName("Goto_Init");
    new_cm2->setName("Playback_Traj");
    grasp_cm->setName("Grasp");
    new_cm3->setName("Playback_Traj_2");
    deflate_cm->setName("Deflate");
    grav_cm->setName("Gravity Compensation");
    //TODO: opspace control for mocve_ee

    ha::ControlSet::Ptr new_cs(new ha::ControlSet());
    ha::ControlSet::Ptr new_cs2(new ha::ControlSet());
    ha::ControlSet::Ptr grasp_cs(new ha::ControlSet());
    ha::ControlSet::Ptr new_cs3(new ha::ControlSet());
    ha::ControlSet::Ptr deflate_cs(new ha::ControlSet());
    ha::ControlSet::Ptr grav_cs(new ha::ControlSet());

    new_cs->setType("rxControlSet");
    new_cs2->setType("rxControlSet");
    grasp_cs->setType("rxControlSet");
    new_cs3->setType("rxControlSet");
    deflate_cs->setType("rxControlSet");
    grav_cs->setType("rxControlSet");

    ha::Controller::Ptr new_ctrl(new ha::Controller());
    ha::Controller::Ptr new_ctrl2(new ha::Controller());
    ha::Controller::Ptr new_ctrl22(new ha::Controller());
    ha::Controller::Ptr new_ctrl222(new ha::Controller());
    ha::Controller::Ptr grasp_ctrl(new ha::Controller());
    ha::Controller::Ptr new_ctrl3(new ha::Controller());
    ha::Controller::Ptr deflate_ctrl(new ha::Controller());

    new_ctrl->setName("HomeCtrl");
    new_ctrl2->setName("HomeCtrl");
    new_ctrl22->setName("BlaCtrl_3");
    new_ctrl222->setName("BlubCtrl");
    grasp_ctrl->setName("FeixGraspControl");
    new_ctrl3->setName("HomeCtrl_2");
    deflate_ctrl->setName("FeixGraspControl");

    new_ctrl->setPriority(2);
    new_ctrl22->setPriority(0);
    new_ctrl222->setPriority(1);

    new_ctrl->setType("InterpolatedJointController");
    new_ctrl2->setType("InterpolatedJointController");
    new_ctrl22->setType("InterpolatedJointController");
    new_ctrl222->setType("InterpolatedJointController");
    grasp_ctrl->setType("FeixGraspSoftHandController");
    new_ctrl3->setType("InterpolatedJointController");
    deflate_ctrl->setType("FeixGraspSoftHandController");

    new_ctrl->setArgument("interpolation_type", "linear");
    new_ctrl2->setArgument("interpolation_type", "linear");
    new_ctrl22->setArgument("interpolation_type", "linear");
    grasp_ctrl->setArgument("grasp_strength", "0.1");
    grasp_ctrl->setArgument("grasp_type", "4");
    new_ctrl3->setArgument("interpolation_type", "linear");
    deflate_ctrl->setArgument("grasp_strength", "4.0");
    deflate_ctrl->setArgument("grasp_type", "0");

    //only the first configuration of the trajectory
//    new_ctrl->setGoal(_trajectory.col(0));

//    //now set the first trajecotry
//    new_ctrl2->setGoal(_trajectory);

//    //set the second trajectory
//    new_ctrl3->setGoal(_trajectory_2);

    //The first point might be far away - take 10 seconds time
    new_ctrl->setCompletionTime(5.0);
    new_ctrl2->setCompletionTime(2.0);
    new_ctrl3->setCompletionTime(6.0);

    new_ctrl->setKp(kp);
    new_ctrl->setKv(kv);
    new_ctrl2->setKp(kp);
    new_ctrl2->setKv(kv);
    new_ctrl3->setKp(kp);
    new_ctrl3->setKv(kv);

    new_cs->appendController(new_ctrl);
    new_cs->appendController(new_ctrl22);
    new_cs->appendController(new_ctrl222);
    new_cs2->appendController(new_ctrl2);
    grasp_cs->appendController(grasp_ctrl);
    //grasp_cs->appendController(hold_ctrl);
    new_cs3->appendController(new_ctrl3);
    deflate_cs->appendController(deflate_ctrl);
    //TODO: add behaviour for wrist here as a second controller!

    new_cm->setControlSet(new_cs);
    new_cm2->setControlSet(new_cs2);
    grasp_cm->setControlSet(grasp_cs);
    new_cm3->setControlSet(new_cs3);
    deflate_cm->setControlSet(deflate_cs);
    grav_cm->setControlSet(grav_cs);


    new_ha->addControlMode(new_cm);

    ha::ControlSwitch::Ptr new_s(new ha::ControlSwitch());
    new_s->setName("initial_convergence");
    ha::JumpConditionPtr new_j(new ha::JumpCondition());
    ha::SensorPtr new_sensor(new ha::JointConfigurationSensor());
    //new_sensor->setSystem(_system);
    new_j->setSensor(new_sensor);
    new_j->setControllerGoal(new_ctrl);
    new_j->setEpsilon(0.05);
    new_s->add(new_j);
    new_ha->addControlSwitchAndMode(new_cm->getName(), new_s, new_cm2);
    std::cout<<"2"<<std::endl;

    ha::ControlSwitch::Ptr grasp_s(new ha::ControlSwitch());
    grasp_s->setName("switch_to_grasp");
    ha::JumpConditionPtr grasp_j(new ha::JumpCondition());
    ha::SensorPtr grasp_sensor(new ha::ForceTorqueSensor());
    //grasp_sensor->setSystem(_system);
    grasp_j->setSensor(grasp_sensor);
    //grasp_j->setControllerGoal(new_ctrl2);
    grasp_j->setGoalRelative();
    Eigen::MatrixXd goal(6,1);
    goal << 0.0 , 2.0, 0.0 , 0.0 , 0.0 , 0.0;
    grasp_j->setConstantGoal(goal);
    Eigen::MatrixXd weights(6,1);
    weights << 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0;
    grasp_j->setEpsilon(0.00);
    grasp_j->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND , weights);
    grasp_s->add(grasp_j);
    //new_ha->addControlMode(grasp_cm);
    new_ha->addControlSwitchAndMode(new_cm2->getName(), grasp_s, grasp_cm);

    ha::ControlSwitch::Ptr traj2_s(new ha::ControlSwitch());
    traj2_s->setName("switch_to_traj2");
    ha::JumpConditionPtr traj2_j(new ha::JumpCondition());
    ha::SensorPtr traj2_sensor(new ha::ClockSensor());
//    traj2_sensor->setSystem(_system);
    traj2_j->setSensor(traj2_sensor);
    traj2_j->setConstantGoal(5.0);
    traj2_j->setGoalRelative();
    traj2_s->add(traj2_j);
    new_ha->addControlSwitchAndMode(grasp_cm->getName(), traj2_s, new_cm3);

    ha::ControlSwitch::Ptr deflate_s(new ha::ControlSwitch());
    deflate_s->setName("switch_to_deflate");
    ha::JumpConditionPtr deflate_j(new ha::JumpCondition());
    ha::SensorPtr deflate_sensor(new ha::JointConfigurationSensor());
//    deflate_sensor->setSystem(_system);
    deflate_j->setSensor(deflate_sensor);
    deflate_j->setControllerGoal(new_ctrl3);
    deflate_j->setEpsilon(0.05);
    deflate_s->add(deflate_j);
    //new_ha->addControlMode(grasp_cm);
    new_ha->addControlSwitchAndMode(new_cm3->getName(), deflate_s, deflate_cm);

    ha::ControlSwitch::Ptr grav_s(new ha::ControlSwitch());
    grav_s->setName("switch_to_gravity");
    ha::JumpConditionPtr grav_j(new ha::JumpCondition());
    ha::SensorPtr grav_sensor(new ha::ClockSensor());
//    grav_sensor->setSystem(_system);
    grav_j->setSensor(grav_sensor);
    grav_j->setConstantGoal(3.0);
    grav_s->add(grav_j);
    //new_ha->addControlMode(grasp_cm);
    new_ha->addControlSwitchAndMode(deflate_cm->getName(), grav_s, grav_cm);

    new_ha->setCurrentControlMode(new_cm->getName());

    /*
    ha::DescriptionTreeNode::Ptr ha_serialized;
    ha::DescriptionTreeXML::Ptr tree(new ha::DescriptionTreeXML);

    ha_serialized = new_ha->serialize(tree);
    tree->setRootNode(ha_serialized);

    std::string ha_string = tree->writeTreeXML();
    _outFile<<ha_string;
    _outFile.close();

    //std::cout<<_trajectory<<std::endl;
 //std::cout<<"---------------------------------------------------------"<<std::endl;
    std::cout<<ha_string<<std::endl;
    */


    ha::DescriptionTreeXML::Ptr dt(new ha::DescriptionTreeXML);
    ha::DescriptionTreeNode::Ptr node = new_ha->serialize(dt);
    dt->setRootNode(node);
    std::ofstream out;
    out.open("test.xml");
    out << dt->writeTreeXML();
    out.close();


    new_ha->visualizeGraph("test.dot");


}
