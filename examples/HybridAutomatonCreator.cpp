/*
 * HybridAutomatonCreator.cpp
 *
 *  Created on: Feb 4, 2015
 *      Author: newtop-1
 */

#include <boost/assign/std/vector.hpp>

#include "apc/HybridAutomatonCreator.h"

#include "apc/ApcDefinitions.h"

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



#ifdef OBJECT_RECOGNITION_ON
#include "hybrid_automaton/ROSTopicSensor.h"
#endif

#include "hybrid_automaton/DescriptionTreeXML.h"

#include <ros/ros.h>

//#include <fstream>
//#include <iostream>

// CAREFUL
//#define BASE_GAINS_FOR_BIG_LAB 1

using namespace boost::assign; // bring 'operator+=()' into scope

namespace apc{

HybridAutomatonCreator::HybridAutomatonCreator()
{
    _arm_index_str = "[7,1]0;1;2;3;4;5;6";
    _base_index_str = "[3,1]7;8;9";

    _base_index_vec += 7,8,9;
    _arm_index_vec += 0,1,2,3,4,5,6;

    //Velocity for Interpolated Joint Controller
    _max_js_vel_arm.resize(7,1);
    _max_js_vel_arm<<0.3,0.3,0.3,0.3,0.3,0.3,0.3;
    _max_js_vel_base.resize(3,1);
    _max_js_vel_base<<0.15,0.15,0.15;

    //gains for joint space integration of nakamura control set
    _kp_js_nakamura.resize(10,1);
    _kv_js_nakamura.resize(10,1);
    _kp_js_nakamura(0,0) = 30.0;
    _kp_js_nakamura(1,0) = 20.0;
    _kp_js_nakamura(2,0) = 15.0;
    _kp_js_nakamura(3,0) = 20.0;
    _kp_js_nakamura(4,0) = 10.0;
    _kp_js_nakamura(5,0) = 10.0;
    _kp_js_nakamura(6,0) = 10.0;
    _kp_js_nakamura(7,0) = 0.0;
    _kp_js_nakamura(8,0) = 0.0;
    _kp_js_nakamura(9,0) = 0.0;
    _kv_js_nakamura(0,0) = 1.0;
    _kv_js_nakamura(1,0) = 2.0;
    _kv_js_nakamura(2,0) = 1.0;
    _kv_js_nakamura(3,0) = 0.4;
    _kv_js_nakamura(4,0) = 0.1;
    _kv_js_nakamura(5,0) = 0.1;
    _kv_js_nakamura(6,0) = 0.01;
    _kv_js_nakamura(7,0) = 10.0;
    _kv_js_nakamura(8,0) = 10.0;
    _kv_js_nakamura(9,0) = 2.0;

    //gains for joint space controllers
    _kp_jointspace.resize(10,1);
    _kv_jointspace.resize(10,1);
    _kp_jointspace(0,0) = 300.0;
    _kp_jointspace(1,0) = 200.0;
    _kp_jointspace(2,0) = 150.0;
    _kp_jointspace(3,0) = 120.0;
    _kp_jointspace(4,0) = 10.0;
    _kp_jointspace(5,0) = 10.0;
    _kp_jointspace(6,0) = 10.0;
    _kp_jointspace(7,0) = 0.0;
    _kp_jointspace(8,0) = 0.0;
    _kp_jointspace(9,0) = 0.0;
    _kv_jointspace(0,0) = 2.0;
    _kv_jointspace(1,0) = 4.0;
    _kv_jointspace(2,0) = 2.0;
    _kv_jointspace(3,0) = 1.2;
    _kv_jointspace(4,0) = 0.2;
    _kv_jointspace(5,0) = 0.3;
    _kv_jointspace(6,0) = 0.02;
    _kv_jointspace(7,0) = 0.0;
    _kv_jointspace(8,0) = 0.0;
    _kv_jointspace(9,0) = 0.0;

    //These weights move arm and base in taskspace - the base never rotates
    _joint_weights_nakamura.resize(10,1);
    _joint_weights_nakamura(0,0) = 1.0;
    _joint_weights_nakamura(1,0) = 1.0;
    _joint_weights_nakamura(2,0) = 1.0;
    _joint_weights_nakamura(3,0) = 1.0;
    _joint_weights_nakamura(4,0) = 1.0;
    _joint_weights_nakamura(5,0) = 1.0;
    _joint_weights_nakamura(6,0) = 1.0;
    _joint_weights_nakamura(7,0) = 0.5;
    _joint_weights_nakamura(8,0) = 0.5;
    _joint_weights_nakamura(9,0) = 0.01;

    //These weights only move the arm in taskspace
    _joint_weights_arm_nakamura.resize(10,1);
    _joint_weights_arm_nakamura(0,0) = 1.0;
    _joint_weights_arm_nakamura(1,0) = 1.0;
    _joint_weights_arm_nakamura(2,0) = 1.0;
    _joint_weights_arm_nakamura(3,0) = 1.0;
    _joint_weights_arm_nakamura(4,0) = 1.0;
    _joint_weights_arm_nakamura(5,0) = 1.0;
    _joint_weights_arm_nakamura(6,0) = 1.0;
    _joint_weights_arm_nakamura(7,0) = 0.01;
    _joint_weights_arm_nakamura(8,0) = 0.01;
    _joint_weights_arm_nakamura(9,0) = 0.01;

    _kp_base_jointspace.resize(3,1);
    _kv_base_jointspace.resize(3,1);
    _kp_base_jointspace(0,0) = 40.0;
    _kp_base_jointspace(1,0) = 40.0;
    _kp_base_jointspace(2,0) = 40.0;
    _kv_base_jointspace(0,0) = 7.0;
    _kv_base_jointspace(1,0) = 7.0;
    _kv_base_jointspace(2,0) = 7.0;

#ifdef BASE_GAINS_FOR_BIG_LAB

    _kp_base_jointspace(0,0) = 35.0;
    _kp_base_jointspace(1,0) = 35.0;
    _kp_base_jointspace(2,0) = 30.0;
    _kv_base_jointspace(0,0) = 8.0;
    _kv_base_jointspace(1,0) = 8.0;
    _kv_base_jointspace(2,0) = 5.0;

    ROS_ERROR_STREAM("Careful! You are using base gains to be used only in the big lab! Arne, wake up!!!" << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl );

#endif


    // op space gains
    _kp_opspace.resize(6,1);
    _kv_opspace.resize(6,1);
    _kp_opspace(0,0) = 0;
    _kp_opspace(1,0) = 0;
    _kp_opspace(2,0) = 0;
    _kp_opspace(3,0) = 0;
    _kp_opspace(4,0) = 0;
    _kp_opspace(5,0) = 0;

    _kv_opspace(0,0) = 10;
    _kv_opspace(1,0) = 10;
    _kv_opspace(2,0) = 10;
    _kv_opspace(3,0) = 10;
    _kv_opspace(4,0) = 10;
    _kv_opspace(5,0) = 10;


    // gains while grasping
    _kp_grasp.resize(10, 1);
    _kv_grasp.resize(10, 1);

    _kp_grasp(0,0) = 900.0;
    _kp_grasp(1,0) = 2500.0;
    _kp_grasp(2,0) = 600.0;
    _kp_grasp(3,0) = 500.0;
    _kp_grasp(4,0) = 10.0; //30.0;
    _kp_grasp(5,0) = 10.0; //30.0;
    _kp_grasp(6,0) = 10.0; //20.0;
    _kp_grasp(7,0) = 0.0; //20.0;
    _kp_grasp(8,0) = 0.0; //20.0;
    _kp_grasp(9,0) = 0.0; //20.0;


    _kv_grasp(0,0) = 10.0;
    _kv_grasp(1,0) = 20.0;
    _kv_grasp(2,0) = 5.0;
    _kv_grasp(3,0) = 2.0;
    _kv_grasp(4,0) = 0.5;
    _kv_grasp(5,0) = 0.5;
    _kv_grasp(6,0) = 0.05;
    _kv_grasp(7,0) = 0.0;
    _kv_grasp(8,0) = 0.0;
    _kv_grasp(9,0) = 0.0;


    // gains while droping
    _kp_drop.resize(10, 1);
    _kv_drop.resize(10, 1);

    _kp_drop(0,0) = 900.0;
    _kp_drop(1,0) = 2500.0;
    _kp_drop(2,0) = 600.0;
    _kp_drop(3,0) = 500.0;
    _kp_drop(4,0) = 10.0; //30.0;
    _kp_drop(5,0) = 10.0; //30.0;
    _kp_drop(6,0) = 10.0; //20.0;
    _kp_drop(7,0) = 0.0;
    _kp_drop(8,0) = 0.0;
    _kp_drop(9,0) = 0.0;
    _kv_drop(0,0) = 10.0;
    _kv_drop(1,0) = 20.0;
    _kv_drop(2,0) = 5.0;
    _kv_drop(3,0) = 2.0;
    _kv_drop(4,0) = 0.5;
    _kv_drop(5,0) = 0.5;
    _kv_drop(6,0) = 0.05;
    _kv_drop(7,0) = 0.0;
    _kv_drop(8,0) = 0.0;
    _kv_drop(9,0) = 0.0;

    //Define epsilons

    _joint_epsilon = 0.13;
    _joint_vel_epsilon = 0.01;
    _joint_base_epsilon = 0.05;
    _joint_base_vel_epsilon = 0.001;
    _op_epsilon = 0.01;

    //Define poses
    _home_config.resize(7,1);
    //_home_config << 0 ,-0.66, 0, 2.58, 0, 0.2, -0.09;
    _home_config <<0.001, -0.14, -0.07, 2.18, 0.05, 0.2, -0.13;

    _home_config_low_bin.resize(7,1);
    _home_config_low_bin <<0.001, 1.15, 0, 1.82, 0.0, -1.37, 0.12;

    _home_config_high_bin.resize(7,1);
    _home_config_high_bin <<0.001, -0.14, -0.07, 2.18, 0.05, 0.2, -0.13;

    _bin_view_positions_ikea_cb1.resize(12);
    _bin_view_positions_ikea_cb2.resize(12);
    _bin_view_positions_whiteshelf_cb1.resize(12);
    _bin_view_positions_whiteshelf_cb2.resize(12);
    _bin_view_positions_ikea_woodenshelf.resize(12);
    _bin_view_positions_kiva.resize(12);
    for(int i = 0; i<12; i++)
    {
        _bin_view_positions_ikea_cb1[i].resize(7,1);
        _bin_view_positions_ikea_cb2[i].resize(7,1);
        _bin_view_positions_whiteshelf_cb1[i].resize(7,1);
        _bin_view_positions_whiteshelf_cb2[i].resize(7,1);
        _bin_view_positions_ikea_woodenshelf[i].resize(7,1);
        _bin_view_positions_kiva[i].resize(7,1);
    }

    _bin_view_positions_ikea_cb1[0] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb1[1] << -0.349492, -0.591655, 0.243022, 2.578710, 0.136161,  0.258721, 0.362997;
    _bin_view_positions_ikea_cb1[2] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb1[3] << -1.171230, -0.905564, 1.379860, 2.516070, 0.351630,  0.218632, 0.385499;
    _bin_view_positions_ikea_cb1[4] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb1[5] << 0.768597, -0.744021, -1.353490, 2.438690, -0.863536, 0.273824, 0.606605;
    _bin_view_positions_ikea_cb1[6] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb1[7] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb1[8] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb1[9] << -1.167470, -1.305320, 1.823670, 2.482410, 0.690608, 0.362304, 0.362689;
    _bin_view_positions_ikea_cb1[10] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb1[11] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!

    _bin_view_positions_ikea_cb2[0] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb2[1] << -0.349492, -0.591655, 0.243022, 2.578710, 0.136161,  0.258721, 0.362997;
    _bin_view_positions_ikea_cb2[2] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb2[3] << -1.171230, -0.905564, 1.379860, 2.516070, 0.351630,  0.218632, 0.385499;
    _bin_view_positions_ikea_cb2[4] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb2[5] << 0.768597, -0.744021, -1.353490, 2.438690, -0.863536, 0.273824, 0.606605;
    _bin_view_positions_ikea_cb2[6] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb2[7] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb2[8] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb2[9] << -1.167470, -1.305320, 1.823670, 2.482410, 0.690608, 0.362304, 0.362689;
    _bin_view_positions_ikea_cb2[10] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_ikea_cb2[11] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!

    _bin_view_positions_whiteshelf_cb1[0] << -0.573197, -0.0364083, 0.965564, 1.86242, -0.220688, 0.419156, 0.524615; //FIXME!
    _bin_view_positions_whiteshelf_cb1[1] << -0.349492, -0.591655, 0.243022, 2.578710, 0.136161,  0.258721, 0.362997;
    _bin_view_positions_whiteshelf_cb1[2] << 0.573197, -0.0364083, -0.965564, 1.86242, -0.220688, 0.419156, 0.524615; //FIXME!
    _bin_view_positions_whiteshelf_cb1[3] << -1.171230, -0.905564, 1.379860, 2.516070, 0.351630,  0.218632, 0.385499;
    _bin_view_positions_whiteshelf_cb1[4] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_whiteshelf_cb1[5] << 0.768597, -0.744021, -1.353490, 2.438690, -0.863536, 0.273824, 0.606605;
    _bin_view_positions_whiteshelf_cb1[6] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_whiteshelf_cb1[7] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_whiteshelf_cb1[8] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_whiteshelf_cb1[9] << -1.167470, -1.305320, 1.823670, 2.482410, 0.690608, 0.362304, 0.362689;
    _bin_view_positions_whiteshelf_cb1[10] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_whiteshelf_cb1[11] << 1.167470, -1.305320, -1.823670, 2.482410, 0.690608, 0.362304, 0.362689; //FIXME!


    _bin_view_positions_whiteshelf_cb2[0] << -0.573197, -0.0364083, 0.965564, 1.86242, -0.220688, 0.419156, 0.524615; //FIXME!
    _bin_view_positions_whiteshelf_cb2[1] << -0.349492, -0.591655, 0.243022, 2.578710, 0.136161,  0.258721, 0.362997;
    _bin_view_positions_whiteshelf_cb2[2] << 0.573197, -0.0364083, -0.965564, 1.86242, -0.220688, 0.419156, 0.524615; //FIXME!
    _bin_view_positions_whiteshelf_cb2[3] << -1.171230, -0.905564, 1.379860, 2.516070, 0.351630,  0.218632, 0.385499;
    _bin_view_positions_whiteshelf_cb2[4] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_whiteshelf_cb2[5] << 0.768597, -0.744021, -1.353490, 2.438690, -0.863536, 0.273824, 0.606605;
    _bin_view_positions_whiteshelf_cb2[6] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_whiteshelf_cb2[7] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_whiteshelf_cb2[8] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_whiteshelf_cb2[9] << -1.167470, -1.305320, 1.823670, 2.482410, 0.690608, 0.362304, 0.362689;
    _bin_view_positions_whiteshelf_cb2[10] << 0.077722, -0.646796, -0.139254, 2.841530, -0.213808, 0.300945, 0.577323; //FIXME!
    _bin_view_positions_whiteshelf_cb2[11] << 1.167470, -1.305320, -1.823670, 2.482410, 0.690608, 0.362304, 0.362689; //FIXME!


    _bin_view_positions_ikea_woodenshelf[0] << 0.00562459, 0.23159, -0.00118592, 1.72334, -0.0406426, 0.907421, -0.0685308;
    _bin_view_positions_ikea_woodenshelf[1] << 0.00562459, 0.23159, -0.00118592, 1.72334, -0.0406426, 0.907421, -0.0685308;
    _bin_view_positions_ikea_woodenshelf[2] << 0.00562459, 0.23159, -0.00118592, 1.72334, -0.0406426, 0.907421, -0.0685308;
    _bin_view_positions_ikea_woodenshelf[3] << 0.00745076, 0.612071, 0.00757162, 1.59653, 0.144542, 0.79767, -0.131102;
    _bin_view_positions_ikea_woodenshelf[4] << 0.00745076, 0.612071, 0.00757162, 1.59653, 0.144542, 0.79767, -0.131102;
    _bin_view_positions_ikea_woodenshelf[5] << 0.00745076, 0.612071, 0.00757162, 1.59653, 0.144542, 0.79767, -0.131102;
    _bin_view_positions_ikea_woodenshelf[6] << 0.00661072, 1.18668, 0.0295567, 0.893971, 0.09109, 0.856974, -0.0947307;
    _bin_view_positions_ikea_woodenshelf[7] << 0.00661072, 1.18668, 0.0295567, 0.893971, 0.09109, 0.856974, -0.0947307;
    _bin_view_positions_ikea_woodenshelf[8] << 0.00661072, 1.18668, 0.0295567, 0.893971, 0.09109, 0.856974, -0.0947307;
    _bin_view_positions_ikea_woodenshelf[9] << -0.000840036, 1.70367, -0.0551907, 0.300831, 0.0559033, 0.886151, -0.0948335;
    _bin_view_positions_ikea_woodenshelf[10] <<-0.000840036, 1.70367, -0.0551907, 0.300831, 0.0559033, 0.886151, -0.0948335;
    _bin_view_positions_ikea_woodenshelf[11] <<-0.000840036, 1.70367, -0.0551907, 0.300831, 0.0559033, 0.886151, -0.0948335;

    _bin_view_positions_kiva[0] << 0.00562459, 0.23159, -0.00118592, 1.72334, -0.0406426, 0.907421, -0.0685308;
    _bin_view_positions_kiva[1] << 0.00562459, 0.23159, -0.00118592, 1.72334, -0.0406426, 0.907421, -0.0685308;
    _bin_view_positions_kiva[2] << 0.00562459, 0.23159, -0.00118592, 1.72334, -0.0406426, 0.907421, -0.0685308;
    _bin_view_positions_kiva[3] << 0.00745076, 0.612071, 0.00757162, 1.59653, 0.144542, 0.79767, -0.131102;
    _bin_view_positions_kiva[4] << 0.00745076, 0.612071, 0.00757162, 1.59653, 0.144542, 0.79767, -0.131102;
    _bin_view_positions_kiva[5] << 0.00745076, 0.612071, 0.00757162, 1.59653, 0.144542, 0.79767, -0.131102;
    _bin_view_positions_kiva[6] << 0.00661072, 1.18668, 0.0295567, 0.893971, 0.09109, 0.856974, -0.0947307;
    _bin_view_positions_kiva[7] << 0.00661072, 1.18668, 0.0295567, 0.893971, 0.09109, 0.856974, -0.0947307;
    _bin_view_positions_kiva[8] << 0.00661072, 1.18668, 0.0295567, 0.893971, 0.09109, 0.856974, -0.0947307;
    _bin_view_positions_kiva[9] << -0.000840036, 1.70367, -0.0551907, 0.300831, 0.0559033, 0.886151, -0.0948335;
    _bin_view_positions_kiva[10] <<-0.000840036, 1.70367, -0.0551907, 0.300831, 0.0559033, 0.886151, -0.0948335;
    _bin_view_positions_kiva[11] <<-0.000840036, 1.70367, -0.0551907, 0.300831, 0.0559033, 0.886151, -0.0948335;
}

ha::ControlSet::Ptr HybridAutomatonCreator::_createJointSpaceControlSet(ha::Controller::Ptr ctrl){
    ha::ControlSet::Ptr cs(new ha::ControlSet());
    cs->setType("rxControlSet");
    cs->appendController(ctrl);
    return cs;
}

ha::ControlSet::Ptr HybridAutomatonCreator::_createJointSpaceWholeBodyControlSet(ha::Controller::Ptr ctrl_arm, ha::Controller::Ptr ctrl_base){
    ha::ControlSet::Ptr cs(new ha::ControlSet());
    cs->setType("rxControlSet");

    if(ctrl_arm)
        cs->appendController(ctrl_arm);
    if(ctrl_base)
        cs->appendController(ctrl_base);
    return cs;
}

ha::Controller::Ptr HybridAutomatonCreator::_createJointSpaceController(std::string name, const Eigen::MatrixXd &goal, double completionTime){
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("InterpolatedJointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ctrl->setGoal(goal);
    ctrl->setKp(_kp_jointspace);
    ctrl->setKv(_kv_jointspace);
    ctrl->setCompletionTime(completionTime);
    ctrl->setGoalIsRelative(0);
    return ctrl;
}


ha::Controller::Ptr HybridAutomatonCreator::_createJointSpaceArmController(std::string name, const Eigen::MatrixXd &goal, const Eigen::MatrixXd& max_velocity){
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("InterpolatedSubjointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ctrl->setArgument("index", _arm_index_str);
    ctrl->setGoal(goal);
    ctrl->setKp(_kp_jointspace);
    ctrl->setKv(_kv_jointspace);
    ctrl->setMaximumVelocity(max_velocity);
    ctrl->setGoalIsRelative(0);
    return ctrl;
}

ha::Controller::Ptr HybridAutomatonCreator::_createJointSpaceBaseController(std::string name, const Eigen::MatrixXd& goal, const Eigen::MatrixXd& max_velocity){
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("InterpolatedSubjointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ctrl->setArgument("index", _base_index_str);
    ctrl->setGoal(goal);
    ctrl->setKp(_kp_base_jointspace);
    ctrl->setKv(_kv_base_jointspace);
    ctrl->setMaximumVelocity(max_velocity);
    ctrl->setGoalIsRelative(0);
    return ctrl;
}

ha::Controller::Ptr HybridAutomatonCreator::_createJointSpaceBaseController(std::string name, const std::string& topic, const Eigen::MatrixXd& max_velocity){
    ha::Controller::Ptr ctrl(new ha::Controller());
    ctrl->setName(name);
    ctrl->setType("BlackboardInterpolatedSubjointController");
    ctrl->setArgument("interpolation_type", "quintic");
    ctrl->setArgument("index", _base_index_str);
    ctrl->setArgument("reinterpolation", "1");
    ctrl->setArgument("use_tf", "1");
    ctrl->setArgument("topic_name", topic);
    ctrl->setArgument("tf_parent", "rlab_origin");
    ctrl->setArgument("update_rate", 100);
    ctrl->setKp(_kp_base_jointspace);
    ctrl->setKv(_kv_base_jointspace);
    ctrl->setMaximumVelocity(max_velocity);
    ctrl->setGoalIsRelative(0);
    return ctrl;
}

ha::Controller::Ptr HybridAutomatonCreator::_createJointSpaceControllerMoreThanOneGoal(std::string name, const Eigen::MatrixXd &goals, const Eigen::MatrixXd vel_max){
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


ha::ControlSet::Ptr HybridAutomatonCreator::_createTaskSpaceControlSet(ha::Controller::Ptr ctrl, bool useBase){
    ha::ControlSet::Ptr cs(new ha::ControlSet());
    cs->setType("TPNakamuraControlSet");
    cs->setArgument<Eigen::MatrixXd>("js_kp", _kp_js_nakamura);
    cs->setArgument<Eigen::MatrixXd>("js_kd", _kv_js_nakamura);
    if(useBase)
        cs->setArgument<Eigen::MatrixXd>("joint_weights", _joint_weights_nakamura);
    else
        cs->setArgument<Eigen::MatrixXd>("joint_weights", _joint_weights_arm_nakamura);

    cs->appendController(ctrl);
    return cs;
}


ha::Controller::Ptr HybridAutomatonCreator::_createTaskSpaceController(std::string name, const Eigen::MatrixXd &pos, const Eigen::MatrixXd &ori, double completionTime){

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

ha::Controller::Ptr HybridAutomatonCreator::_createTaskSpaceControllerTF(std::string name, const std::string frame, double max_displacement_velocity, double max_rotational_velocity){

    //Endeffector Frame Controller
    ha::Controller::Ptr ctrl(new ha::Controller);
    ctrl->setName(name);
    ctrl->setType("BlackboardInterpolatedHTransformController");
    ctrl->setArgument("interpolation_type", "cubic");
    ctrl->setArgument("reinterpolation", "1");
    ctrl->setArgument("use_tf", "1");
    ctrl->setArgument("topic_name", frame);
    ctrl->setArgument("tf_parent", "rlab_origin");
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

ha::Controller::Ptr HybridAutomatonCreator::_createTaskSpaceTrajController(std::string name, const std::string topic, double max_displacement_velocity, double max_rotational_velocity, bool relative_goal){

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


ha::JumpCondition::Ptr HybridAutomatonCreator::_createJointSpaceConvergenceCondition(ha::ControllerConstPtr ctrl){
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SensorPtr sensor(new ha::JointConfigurationSensor());    
    jc->setSensor(sensor);
    jc->setControllerGoal(ctrl);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon(_joint_epsilon);

	return jc;
}

ha::JumpCondition::Ptr HybridAutomatonCreator::_createJointSpaceConvergenceWithZeroVelCondition(){

    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SensorPtr sensor(new ha::JointVelocitySensor());
    jc->setSensor(sensor);
    ::Eigen::MatrixXd zero_goal;
    zero_goal.resize(7,1);
    zero_goal << 0,0,0,0,0,0,0;
    jc->setConstantGoal(zero_goal);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon(_joint_vel_epsilon);

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonCreator::_createJointSpaceArmConvergenceCondition(ha::ControllerConstPtr ctrl){
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SubjointConfigurationSensorPtr sensor(new ha::SubjointConfigurationSensor());
    sensor->setIndex(_arm_index_vec);
    jc->setSensor(sensor);
    jc->setControllerGoal(ctrl);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon(_joint_epsilon);

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonCreator::_createJointSpaceArmConvergenceWithZeroVelCondition(){
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SubjointVelocitySensorPtr sensor(new ha::SubjointVelocitySensor());
    sensor->setIndex(_arm_index_vec);
    jc->setSensor(sensor);
    ::Eigen::MatrixXd zero_goal;
    zero_goal.resize(7,1);
    zero_goal << 0,0,0,0,0,0,0;
    jc->setConstantGoal(zero_goal);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon(_joint_vel_epsilon);

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonCreator::_createJointSpaceBaseConvergenceCondition(ha::ControllerConstPtr ctrl){
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SubjointConfigurationSensorPtr sensor(new ha::SubjointConfigurationSensor());
    sensor->setIndex(_base_index_vec);
    jc->setSensor(sensor);
    jc->setControllerGoal(ctrl);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon(_joint_base_epsilon);

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonCreator::_createJointSpaceBaseConvergenceWithZeroVelCondition(){
    ha::JumpConditionPtr jc(new ha::JumpCondition());
    ha::SubjointVelocitySensorPtr sensor(new ha::SubjointVelocitySensor());
    sensor->setIndex(_base_index_vec);
    jc->setSensor(sensor);
    ::Eigen::MatrixXd zero_goal;
    zero_goal.resize(3,1);
    zero_goal << 0,0,0;
    jc->setConstantGoal(zero_goal);
    jc->setJumpCriterion(ha::JumpCondition::NORM_L_INF);
    jc->setEpsilon(_joint_base_vel_epsilon);

    return jc;
}

ha::JumpCondition::Ptr HybridAutomatonCreator::_createTaskSpaceConvergenceCondition(ha::ControllerConstPtr ctrl, bool relative){
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

ha::JumpCondition::Ptr HybridAutomatonCreator::_createMaxTimeCondition(double max_time){
    ha::JumpCondition::Ptr max_time_cond(new ha::JumpCondition());
    ha::ClockSensor::Ptr time_sensor(new ha::ClockSensor());
    max_time_cond->setSensor(time_sensor);
    max_time_cond->setConstantGoal(max_time);
    max_time_cond->setGoalRelative();
    max_time_cond->setEpsilon(0);
    max_time_cond->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    return max_time_cond;
}

ha::JumpCondition::Ptr HybridAutomatonCreator::_createDistanceToDroppingBinCondition(){
    ha::JumpCondition::Ptr too_close_to_bin_cond(new ha::JumpCondition());
    ha::ROSTopicSensor::Ptr distance_topic_sensor(new ha::ROSTopicSensor());
    distance_topic_sensor->setTopic("/basket_distance","Float64");
    too_close_to_bin_cond->setSensor(distance_topic_sensor);
    too_close_to_bin_cond->setConstantGoal(0.41);
    too_close_to_bin_cond->setEpsilon(0.0);
    too_close_to_bin_cond->setJumpCriterion(ha::JumpCondition::THRESH_LOWER_BOUND);
    return too_close_to_bin_cond;
}

ha::HybridAutomaton::Ptr HybridAutomatonCreator::createInitialHybridAutomaton(int tool){

    //create Hybrid Automaton
    ha::HybridAutomaton::Ptr initial_ha = ha::HybridAutomaton::Ptr(new ha::HybridAutomaton());
    initial_ha->setName("Initial_HA");

    ha::ControlMode::Ptr move_home_initial_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr move_home_initial_cs(new ha::ControlSwitch());

    move_home_initial_cm->setName("move_home_initial_cm");

    ha::Controller::Ptr home_ctrl_arm = _createJointSpaceArmController("move_home_initial_arm_ctrl", _home_config, _max_js_vel_arm);
    ha::Controller::Ptr home_ctrl_base = _createJointSpaceBaseController("move_home_initial_base_ctrl", "/view_frames/bin_E", _max_js_vel_base);
    ha::ControlSet::Ptr goto_home_cs = _createJointSpaceWholeBodyControlSet(home_ctrl_arm,home_ctrl_base);
    move_home_initial_cm->setControlSet(goto_home_cs);

    //add controller to ControlSet
    ha::Controller::Ptr ungrasp_ctrl(new ha::Controller());
    switch(tool)
    {
    case 0:  // Vacuum cleaner
    {
        ungrasp_ctrl->setName("VacuumGraspControl");
        ungrasp_ctrl->setType("VacuumCleanerController");
        ungrasp_ctrl->setArgument("power", "0");
        goto_home_cs->appendController(ungrasp_ctrl);
    }
        break;

    case 1: //Soft hand
    {
        ungrasp_ctrl->setName("FeixGraspControl");
        ungrasp_ctrl->setType("FeixGraspSoftHandController");
        ungrasp_ctrl->setArgument("grasp_strength", "4.0");
        ungrasp_ctrl->setArgument("grasp_type", "0");
        goto_home_cs->appendController(ungrasp_ctrl);
    }
        break;
    default:
        break;
    }

    //Create first ControlSwitch
    move_home_initial_cs->setName("move_home_initial_cs");
    ha::JumpConditionPtr initial_convergence_arm_jc = _createJointSpaceArmConvergenceCondition(home_ctrl_arm);
    ha::JumpConditionPtr initial_convergence_vel_arm_jc = _createJointSpaceArmConvergenceWithZeroVelCondition();
    ha::JumpConditionPtr initial_convergence_base_jc = _createJointSpaceBaseConvergenceCondition(home_ctrl_base);
    ha::JumpConditionPtr initial_convergence_vel_base_jc = _createJointSpaceBaseConvergenceWithZeroVelCondition();
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

ha::HybridAutomaton::Ptr HybridAutomatonCreator::createEmptyHybridAutomaton(){

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

ha::ControlSwitch::Ptr HybridAutomatonCreator::CreateMaxTimeControlSwitch(const std::string& mode_name, double max_time){
    ha::ControlSwitchPtr time_switch(new ha::ControlSwitch);
    time_switch->setName(mode_name + "_time_cs");
    ha::JumpConditionPtr time_cond = _createMaxTimeCondition(max_time);
    time_switch->add(time_cond);
    return time_switch;
}


bool HybridAutomatonCreator::CreateGoToViewCMAndConvergenceCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& cs_ptr,
                                                              const std::string& name, int localization, int shelf_tracking, int bin_id){

    cm_ptr->setName(name + std::string("_cm"));

    std::vector<Eigen::MatrixXd> bin_view_positions;
    //Interpolated Joint Controller
    switch(localization)
    {
    case 0: //No localization -> We use the Ikea
    case 1: //Localization mockup -> We use the Ikea
    case 2: //Ikea shelf
    {
        switch(shelf_tracking)
        {
        case 0: //No shelf tracking -> We use cardboard1
        case 1: //Shelf tracking mockup -> We use cardboard1
        case 2: //Cardboard 1
            bin_view_positions = _bin_view_positions_ikea_cb1;
            break;
        case 3:
            bin_view_positions = _bin_view_positions_ikea_cb2;
            break;
        case 4:
            bin_view_positions = _bin_view_positions_ikea_woodenshelf;
            break;
        case 5:
            ROS_ERROR_STREAM_NAMED("HybridAutomatonCreator.createHybridAutomatonFromTask","The combination of localization with ikea shelf and tracking of the kiva shelf is not possible");
            break;
        default:
            ROS_ERROR_STREAM_NAMED("HybridAutomatonCreator.createHybridAutomatonFromTask","WRONG VALUE!!");
            break;
        }
        break;
    }
    case 3: //White shelf
    {
        switch(shelf_tracking)
        {
        case 0: //No shelf tracking -> We use cardboard1
        case 1: //Shelf tracking mockup -> We use cardboard1
        case 2: //Cardboard 1
            bin_view_positions = _bin_view_positions_whiteshelf_cb1;
            break;
        case 3:
            bin_view_positions = _bin_view_positions_whiteshelf_cb2;
            break;
        case 4:
            bin_view_positions = _bin_view_positions_ikea_woodenshelf;
            break;
        case 5:
            ROS_ERROR_STREAM_NAMED("HybridAutomatonCreator.createHybridAutomatonFromTask","The combination of localization with the white shelf and tracking of the kiva shelf is not possible");
            break;
        default:
            ROS_ERROR_STREAM_NAMED("HybridAutomatonCreator.createHybridAutomatonFromTask","WRONG VALUE!!");
            break;
        }
        break;
    }
    case 4: // Kiva
    {
        switch(shelf_tracking)
        {
        case 5:
            bin_view_positions = _bin_view_positions_kiva;
            break;
        default:
            ROS_ERROR_STREAM_NAMED("HybridAutomatonCreator.createHybridAutomatonFromTask", "Kiva shelf needs shelf_tracking == 5, not " << shelf_tracking);
        }
        break;
    }
    default:
        ROS_ERROR_STREAM_NAMED("HybridAutomatonCreator.createHybridAutomatonFromTask","Unknown localization mode: " << localization);
        break;
    }

    ha::Controller::Ptr view_ctrl_arm = _createJointSpaceArmController("view_ctrl_arm", bin_view_positions[bin_id], _max_js_vel_arm);
    apc::APCDefinitions& apc_def = apc::APCDefinitions::getInstance();
    std::string view_frame = "/view_frames/" + apc_def.getBinLabelById(bin_id);
    ha::Controller::Ptr view_ctrl_base = _createJointSpaceBaseController("view_ctrl_base", view_frame, _max_js_vel_base);

    ha::ControlSet::Ptr goto_view_cs = _createJointSpaceWholeBodyControlSet(view_ctrl_arm, view_ctrl_base);
    cm_ptr->setControlSet(goto_view_cs);

    cs_ptr->setName(name + std::string("_cs"));
    ha::JumpConditionPtr view_convergence_arm_jc = _createJointSpaceArmConvergenceCondition(view_ctrl_arm);
    ha::JumpConditionPtr view_convergence_vel_arm_jc = _createJointSpaceArmConvergenceWithZeroVelCondition();
    ha::JumpConditionPtr view_convergence_base_jc = _createJointSpaceBaseConvergenceCondition(view_ctrl_base);
    ha::JumpConditionPtr view_convergence_vel_base_jc = _createJointSpaceBaseConvergenceWithZeroVelCondition();
    cs_ptr->add(view_convergence_arm_jc);
    cs_ptr->add(view_convergence_vel_arm_jc);
    cs_ptr->add(view_convergence_base_jc);
    cs_ptr->add(view_convergence_vel_base_jc);
}

bool HybridAutomatonCreator::CreateGoToHomeCMAndConvergenceCS(const ha::ControlMode::Ptr& cm_ptr,
                                                              const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name, const Eigen::MatrixXd& goal_cfg){
    cm_ptr->setName(name + std::string("_cm"));

    ha::Controller::Ptr home_ctrl_arm = _createJointSpaceArmController(name + std::string("_arm_ctrl"), goal_cfg, _max_js_vel_arm);
    //ha::Controller::Ptr home_ctrl_base = _createJointSpaceBaseController(name + std::string("_base_ctrl"), "/view_frames/home", _max_js_vel_base);
    ha::ControlSet::Ptr goto_home_cs = _createJointSpaceWholeBodyControlSet(home_ctrl_arm, ha::ControllerPtr());
    cm_ptr->setControlSet(goto_home_cs);

    //Create first ControlSwitch
    cs_ptr->setName(name + std::string("_cs"));
    ha::JumpConditionPtr initial_convergence_arm_jc = _createJointSpaceArmConvergenceCondition(home_ctrl_arm);
    ha::JumpConditionPtr initial_convergence_vel_arm_jc = _createJointSpaceArmConvergenceWithZeroVelCondition();
    //ha::JumpConditionPtr initial_convergence_base_jc = _createJointSpaceBaseConvergenceCondition(home_ctrl_base);
    cs_ptr->add(initial_convergence_arm_jc);
    cs_ptr->add(initial_convergence_vel_arm_jc);
   //cs_ptr->add(initial_convergence_base_jc);
}

bool HybridAutomatonCreator::CreateAdherenceCSAndCMAndTimeCS(const ha::ControlSwitch::Ptr& cs_ptr, const ha::ControlMode::Ptr& cm_ptr,
                                                             const ha::ControlSwitch::Ptr& cs2_ptr, const ha::ControlSwitch::Ptr& cs3_ptr, const std::string& name, int force_threshold, int tool ){
    cm_ptr->setName(name + std::string("_cm"));

    //Create Switch
    // threshold for FT sensor when triggering jump condition and weights
    Eigen::MatrixXd adherence_jc_FT_sensitivity(6,1);
    adherence_jc_FT_sensitivity << force_threshold,0,0,0,0,0;
    Eigen::MatrixXd adherence_jc_FT_sensitivity_weights(6,1);
    adherence_jc_FT_sensitivity_weights << 1,0,0,0,0,0;

    //force condition - go up until a force threshold
    //(sometimes the F/T triggers at the switch of modes)
    cs_ptr->setName(name + std::string("_cs"));
    ha::JumpConditionPtr adherence_jc_force(new ha::JumpCondition());
    ha::SensorPtr adherence_sensor(new ha::ForceTorqueSensor());
    adherence_jc_force->setSensor(adherence_sensor);
    adherence_jc_force->setGoalRelative();
    adherence_jc_force->setConstantGoal(adherence_jc_FT_sensitivity);
    adherence_jc_force->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND ,adherence_jc_FT_sensitivity_weights);
    adherence_jc_force->setEpsilon(0.00);
    cs_ptr->add(adherence_jc_force);

    // over a specific distance controlswitch should not be triggerd due to problems with heavy objects
    // If endeffector is in adherence, movement is not possible.
    ha::JumpConditionPtr adherence_jc_distance(new ha::JumpCondition());
    ha::SensorPtr adherence_sensor_distance(new ha::FramePoseSensor());
    adherence_jc_distance->setSensor(adherence_sensor_distance);
    //adherence_jc_distance->setControllerGoal(ctrl);

    // if there is any big movement of the endeffector --> no adherence
    // epsilon is compared to the distance from the start frame of this controlmode

    Eigen::MatrixXd adherence_jc_distance_goal(4,4);
    adherence_jc_distance_goal << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    adherence_jc_distance->setConstantGoal(adherence_jc_distance_goal);
    adherence_jc_distance->setGoalRelative();
    adherence_jc_distance->setEpsilon(0.02);
    Eigen::MatrixXd adherence_jc_distance_weights(2,1);
    adherence_jc_distance_weights << 0,-1;
    adherence_jc_distance->setJumpCriterion(ha::JumpCondition::NORM_TRANSFORM ,adherence_jc_distance_weights);
    cs_ptr->add(adherence_jc_distance);


    cm_ptr->setName(name + std::string("_cm"));
    ha::ControlSet::Ptr grav_cs(new ha::ControlSet());
    grav_cs->setType("rxControlSet");
    cm_ptr->setControlSet(grav_cs);

    ha::Controller::Ptr ungrasp_ctrl(new ha::Controller());
    switch(tool)
    {
    case 0:  // Vacuum cleaner
    {
        ungrasp_ctrl->setName("VacuumGraspControl");
        ungrasp_ctrl->setType("VacuumCleanerController");
        ungrasp_ctrl->setArgument("power", "0");
        grav_cs->appendController(ungrasp_ctrl);
    }
        break;

    case 1: //Soft hand
    {
        ungrasp_ctrl->setName("FeixGraspControl");
        ungrasp_ctrl->setType("FeixGraspSoftHandController");
        ungrasp_ctrl->setArgument("grasp_strength", "4.0");
        ungrasp_ctrl->setArgument("grasp_type", "0");
        grav_cs->appendController(ungrasp_ctrl);
    }
        break;
    default:
        break;
    }

    // To free from adherence wait 10 seconds or for pressure sensor
    ha::JumpConditionPtr free_jc_time(new ha::JumpCondition());
    ha::SensorPtr free_sensor_time(new ha::ClockSensor());
    free_jc_time->setSensor(free_sensor_time);
    free_jc_time->setConstantGoal(10);
    free_jc_time->setGoalRelative();
    free_jc_time->setEpsilon(0);
    free_jc_time->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    cs2_ptr->add(free_jc_time);

    ha::JumpConditionPtr free_jc_pressure(new ha::JumpCondition());
    ha::ROSTopicSensor::Ptr free_sensor_pressure(new ha::ROSTopicSensor());
    free_sensor_pressure->setTopic("/pneumaticbox/pressure_0", "Float64");
    free_jc_pressure->setSensor(free_sensor_pressure);
    free_jc_pressure->setConstantGoal(0.5);
    free_jc_pressure->setEpsilon(0);
    free_jc_pressure->setJumpCriterion(ha::JumpCondition::THRESH_LOWER_BOUND);
    cs3_ptr->add(free_jc_pressure);

}

bool HybridAutomatonCreator::CreateRunObjRecCMAndCS(const ha::ControlMode::Ptr& cm_ptr,
                                                              const ha::ControlSwitch::Ptr& cs_success_ptr,const ha::ControlSwitch::Ptr& cs_failure_ptr, const std::string& name){
    cm_ptr->setName(name); // name is important!
    // FIXME: no control set should be possible
    //Interpolated Joint Controller

    Eigen::MatrixXd _zero_config;
    _zero_config.resize(10,1);
    _zero_config <<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    ha::Controller::Ptr maintain_pose_ctrl = _createJointSpaceController(name + std::string("_ctrl"), _zero_config, 6.0);
    maintain_pose_ctrl->setGoalIsRelative(true);
    ha::ControlSet::Ptr run_obj_rec_cs = _createJointSpaceControlSet(maintain_pose_ctrl);
    cm_ptr->setControlSet(run_obj_rec_cs);

    //Create ControlSwitch for success
    cs_success_ptr->setName(name + std::string("_success"));
    ha::JumpConditionPtr to_run_obj_rec_jc(new ha::JumpCondition());
    ha::ROSTopicSensor::Ptr to_run_obj_rec_sensor(new ha::ROSTopicSensor());
    to_run_obj_rec_sensor->setTopic("/object_recognition/has_result", "Float64");
    to_run_obj_rec_jc->setSensor(to_run_obj_rec_sensor);
    to_run_obj_rec_jc->setConstantGoal(1);
    to_run_obj_rec_jc->setEpsilon(0.0); // exact match required
    cs_success_ptr->add(to_run_obj_rec_jc);

    //Create ControlSwitch for failure
    cs_failure_ptr->setName(name + std::string("_failure"));
    ha::JumpConditionPtr to_run_obj_rec_failure_jc(new ha::JumpCondition());
    ha::ROSTopicSensor::Ptr to_run_obj_rec_failure_sensor(new ha::ROSTopicSensor());
    to_run_obj_rec_failure_sensor->setTopic("/object_recognition/has_result", "Float64");
    to_run_obj_rec_failure_jc->setSensor(to_run_obj_rec_sensor);
    to_run_obj_rec_failure_jc->setConstantGoal(-1);
    to_run_obj_rec_failure_jc->setEpsilon(0.0); // exact match required
    cs_failure_ptr->add(to_run_obj_rec_failure_jc);

}

bool HybridAutomatonCreator::CreateStopObjRecCMAndCS(const ha::ControlMode::Ptr& cm_ptr,
                                                              const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name){
    cm_ptr->setName(name);
    ha::ControlSet::Ptr stop_obj_rec_cs(new ha::ControlSet());
    stop_obj_rec_cs->setType("rxControlSet");
    cm_ptr->setControlSet(stop_obj_rec_cs);

    ha::Controller::Ptr stop_or_joint_ctrl(new ha::Controller());
    stop_or_joint_ctrl->setName("stop_or_joint_ctrl");
    stop_or_joint_ctrl->setType("JointController");
    Eigen::MatrixXd grasp_zeros_goal(10,1);
    grasp_zeros_goal << 0,0,0,0,0,0,0,0,0,0;
    stop_or_joint_ctrl->setGoal(grasp_zeros_goal);
    stop_or_joint_ctrl->setGoalIsRelative(1);
    stop_or_joint_ctrl->setKp(_kp_grasp);
    stop_or_joint_ctrl->setKv(_kv_grasp);

    stop_obj_rec_cs->appendController(stop_or_joint_ctrl);

    cs_ptr->setName(name + std::string("_cs"));
    ha::JumpConditionPtr to_gravity_jc(new ha::JumpCondition());
    ha::SensorPtr to_gravity_sensor(new ha::ClockSensor());
    to_gravity_jc->setSensor(to_gravity_sensor);
    to_gravity_jc->setConstantGoal(2.0);
    to_gravity_jc->setGoalRelative();
    to_gravity_jc->setEpsilon(0);
    to_gravity_jc->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    cs_ptr->add(to_gravity_jc);
}

bool HybridAutomatonCreator::CreateStopObjRecAndGoToPrehomeCMAndCS(const ha::ControlMode::Ptr& cm_ptr,
                                                              const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name){
    cm_ptr->setName(name);
    ha::ControlSet::Ptr stop_obj_rec_cs(new ha::ControlSet());
    stop_obj_rec_cs->setType("rxControlSet");
    cm_ptr->setControlSet(stop_obj_rec_cs);

    Eigen::MatrixXd grasp_zeros_goal(7,1);
    grasp_zeros_goal << 0,-0.4,0,0,0,0,0;

    ha::Controller::Ptr stop_or_joint_ctrl = _createJointSpaceArmController(std::string("stop_or_joint_ctrl"), grasp_zeros_goal, _max_js_vel_arm);
    stop_or_joint_ctrl->setGoalIsRelative(true);
    stop_or_joint_ctrl->setName("stop_or_joint_ctrl");

    stop_obj_rec_cs->appendController(stop_or_joint_ctrl);

    cs_ptr->setName(name + std::string("_cs"));
    ha::JumpCondition::Ptr convergence_js_ctrl = _createJointSpaceArmConvergenceCondition(stop_or_joint_ctrl);
    convergence_js_ctrl->setGoalRelative();
    cs_ptr->add(convergence_js_ctrl);
}

bool HybridAutomatonCreator::CreateGoToTfCMAndConvergenceCS(const ha::ControlMode::Ptr& cm_ptr,
                                                              const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name, const std::string& frame_name, double lin_vel, double ang_vel, bool useBase){
    cm_ptr->setName(name + std::string("_cm"));

    //Endeffector Frame Controller
    ha::Controller::Ptr bb_ctrl = _createTaskSpaceControllerTF(name + std::string("_ctrl"), frame_name, lin_vel, ang_vel);

    ha::ControlSet::Ptr bb_cs = _createTaskSpaceControlSet(bb_ctrl, useBase);
    cm_ptr->setControlSet(bb_cs);

    cs_ptr->setName(name + std::string("_cs"));
    ha::JumpConditionPtr convergence_jc = _createTaskSpaceConvergenceCondition(bb_ctrl);
    cs_ptr->add(convergence_jc);
}

bool HybridAutomatonCreator::CreateGoToBBCMAndConvergenceCS(const ha::ControlMode::Ptr& cm_ptr,
                                                              const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name, const std::string& frame_name, double lin_vel, double ang_vel,
                                                              bool relative, bool useTf, bool useBase){
    cm_ptr->setName(name + std::string("_cm"));

    //Endeffector Frame Controller
    ha::Controller::Ptr bb_ctrl;
    if(useTf)
        bb_ctrl = _createTaskSpaceControllerTF(name + std::string("_ctrl"), frame_name, lin_vel, ang_vel);
    else
        bb_ctrl = _createTaskSpaceTrajController(name + std::string("_ctrl"), frame_name, lin_vel, ang_vel, relative);


    ha::ControlSet::Ptr bb_cs = _createTaskSpaceControlSet(bb_ctrl, useBase);
    cm_ptr->setControlSet(bb_cs);

    cs_ptr->setName(name + std::string("_cs"));
    ha::JumpConditionPtr convergence_jc = _createTaskSpaceConvergenceCondition(bb_ctrl, relative);
    cs_ptr->add(convergence_jc);
}

bool HybridAutomatonCreator::CreateGoToBBGraspCMAndCS(const ha::ControlMode::Ptr& cm_ptr,
                                                              const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name, const std::string& frame_name, double lin_vel, double ang_vel, bool relative, bool useTf, bool useBase, int num_other_objs, int tool){
    cm_ptr->setName(name + std::string("_cm"));

    ha::Controller::Ptr bb_ctrl;
    //Endeffector Frame Controller
    if(useTf)
        bb_ctrl = _createTaskSpaceControllerTF(name + std::string("_ctrl"), frame_name, lin_vel, ang_vel);
    else
        bb_ctrl = _createTaskSpaceTrajController(name + std::string("_ctrl"), frame_name, lin_vel, ang_vel, relative);


    ha::ControlSet::Ptr bb_cs = _createTaskSpaceControlSet(bb_ctrl, useBase);
    cm_ptr->setControlSet(bb_cs);

    // turn on VC for single item bins

    if (num_other_objs == 0){
        ha::Controller::Ptr grasp_ctrl(new ha::Controller());
        switch(tool)
        {
        case 0:  // Vacuum cleaner
        {
            grasp_ctrl->setName("VacuumGraspControl");
            grasp_ctrl->setType("VacuumCleanerController");
            grasp_ctrl->setArgument("power", "1");
            bb_cs->appendController(grasp_ctrl);
        }
            break;

        case 1: //Soft hand
        {
            grasp_ctrl->setName("FeixGraspControl");
            grasp_ctrl->setType("FeixGraspSoftHandController");
            grasp_ctrl->setArgument("grasp_strength", "0.1");
            grasp_ctrl->setArgument("grasp_type", "4");
            bb_cs->appendController(grasp_ctrl);
        }
            break;
        default:
            break;
        }

    }



    // threshold for FT sensor when triggering jump condition and weights
    Eigen::MatrixXd grasp_jc_FT_sensitivity(6,1);
    grasp_jc_FT_sensitivity << -4,0,0,0,0,0;
    Eigen::MatrixXd grasp_jc_FT_sensitivity_weights(6,1);
    grasp_jc_FT_sensitivity_weights << 1,0,0,0,0,0;

    //Combined force and time condition - go down until a force threshold AND at least 0.1 seconds
    //(sometimes the F/T triggers at the switch of modes)
    cs_ptr->setName(name + std::string("_cs"));
    ha::JumpConditionPtr to_grasp_jc_force(new ha::JumpCondition());
    ha::SensorPtr to_grasp_sensor(new ha::ForceTorqueSensor());
    to_grasp_jc_force->setSensor(to_grasp_sensor);
    to_grasp_jc_force->setGoalRelative();
    to_grasp_jc_force->setConstantGoal(grasp_jc_FT_sensitivity);
    to_grasp_jc_force->setJumpCriterion(ha::JumpCondition::THRESH_LOWER_BOUND ,grasp_jc_FT_sensitivity_weights);
    to_grasp_jc_force->setEpsilon(0.00);
    cs_ptr->add(to_grasp_jc_force);

    ha::JumpConditionPtr to_grasp_jc_time(new ha::JumpCondition());
    ha::SensorPtr to_grasp_sensor_time(new ha::ClockSensor());
    to_grasp_jc_time->setSensor(to_grasp_sensor_time);
    to_grasp_jc_time->setConstantGoal(0.1);
    to_grasp_jc_time->setGoalRelative();
    to_grasp_jc_time->setEpsilon(0);
    to_grasp_jc_time->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    cs_ptr->add(to_grasp_jc_time);

}

bool HybridAutomatonCreator::CreateGraspCMAndCS(const ha::ControlMode::Ptr& cm_ptr,
                                                              const ha::ControlSwitch::Ptr& cs_ptr, const std::string& name, int tool){
    cm_ptr->setName(name+ std::string("_cm"));
    ha::ControlSet::Ptr grasp_cs(new ha::ControlSet());

    grasp_cs->setType("rxControlSet");
    cm_ptr->setControlSet(grasp_cs);

    ha::Controller::Ptr grasp_ctrl(new ha::Controller());
    switch(tool)
    {
    case 0:  // Vacuum cleaner
    {
        grasp_ctrl->setName("VacuumGraspControl");
        grasp_ctrl->setType("VacuumCleanerController");
        grasp_ctrl->setArgument("power", "1");
    }
        break;

    case 1: //Soft hand
    {
        grasp_ctrl->setName("FeixGraspControl");
        grasp_ctrl->setType("FeixGraspSoftHandController");
        grasp_ctrl->setArgument("grasp_strength", "0.1");
        grasp_ctrl->setArgument("grasp_type", "4");
    }
        break;
    default:
        break;
    }

    ha::Controller::Ptr grasp_joint_ctrl(new ha::Controller());
    grasp_joint_ctrl->setName("grasp_joint_ctrl");
    grasp_joint_ctrl->setType("JointController");
    Eigen::MatrixXd grasp_zeros_goal(10,1);
    grasp_zeros_goal << 0,0,0,0,0,0,0,0,0,0;
    grasp_joint_ctrl->setGoal(grasp_zeros_goal);
    grasp_joint_ctrl->setGoalIsRelative(1);
    grasp_joint_ctrl->setKp(_kp_grasp);
    grasp_joint_ctrl->setKv(_kv_grasp);

    if(tool != 2)
    {
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

    if(tool == 0) //Vacuum cleaner
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

bool HybridAutomatonCreator::CreateUngraspCMAndCS(const ha::ControlMode::Ptr& cm_ptr,
                                                  const ha::ControlSwitch::Ptr& cs_ptr,
                                                  const ha::ControlSwitch::Ptr& cs_ptr2, const std::string& name, int tool){
    cm_ptr->setName(name+ std::string("_cm"));
    ha::ControlSet::Ptr ungrasp_cs(new ha::ControlSet());
    ungrasp_cs->setType("rxControlSet");
    cm_ptr->setControlSet(ungrasp_cs);

    //add controller to ControlSet
    ha::Controller::Ptr ungrasp_ctrl(new ha::Controller());
    switch(tool)
    {
    case 0:  // Vacuum cleaner
    {
        ungrasp_ctrl->setName("VacuumGraspControl");
        ungrasp_ctrl->setType("VacuumCleanerController");
        ungrasp_ctrl->setArgument("power", "0");
        ungrasp_cs->appendController(ungrasp_ctrl);
    }
        break;

    case 1: //Soft hand
    {
        ungrasp_ctrl->setName("FeixGraspControl");
        ungrasp_ctrl->setType("FeixGraspSoftHandController");
        ungrasp_ctrl->setArgument("grasp_strength", "4.0");
        ungrasp_ctrl->setArgument("grasp_type", "0");
        ungrasp_cs->appendController(ungrasp_ctrl);
    }
        break;
    default:
        break;
    }

    ha::Controller::Ptr ungrasp_joint_ctrl(new ha::Controller());
    ungrasp_joint_ctrl->setName("drop_joint_ctrl");
    ungrasp_joint_ctrl->setType("JointController");
    Eigen::MatrixXd drop_zeros_goal(7,1);
    drop_zeros_goal << 0,0,0,0,0,0,0;
    ungrasp_joint_ctrl->setGoal(drop_zeros_goal);
    ungrasp_joint_ctrl->setGoalIsRelative(1);
    ungrasp_joint_ctrl->setKp(_kp_drop);
    ungrasp_joint_ctrl->setKv(_kv_drop);
    ungrasp_cs->appendController(ungrasp_joint_ctrl);

    cs_ptr->setName(name + std::string("_pressure_cs"));
    ha::JumpConditionPtr to_stop_or_pressure_jc(new ha::JumpCondition());
    ha::ROSTopicSensor::Ptr to_stop_or_sensor(new ha::ROSTopicSensor());
    to_stop_or_sensor->setTopic("/pneumaticbox/pressure_0", "Float64");
    to_stop_or_pressure_jc->setSensor(to_stop_or_sensor);
    to_stop_or_pressure_jc->setConstantGoal(0.5);
    to_stop_or_pressure_jc->setEpsilon(0);
    to_stop_or_pressure_jc->setJumpCriterion(ha::JumpCondition::THRESH_LOWER_BOUND);
    cs_ptr->add(to_stop_or_pressure_jc);

    // To free from adherence wait 15 seconds or for pressure sensor
    cs_ptr2->setName(name + std::string("_time_cs"));
    ha::JumpConditionPtr to_stop_or_time_jc(new ha::JumpCondition());
    ha::SensorPtr free_sensor_time(new ha::ClockSensor());
    to_stop_or_time_jc->setSensor(free_sensor_time);
    to_stop_or_time_jc->setConstantGoal(15);
    to_stop_or_time_jc->setGoalRelative();
    to_stop_or_time_jc->setEpsilon(0);
    to_stop_or_time_jc->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    cs_ptr2->add(to_stop_or_time_jc);
}

bool HybridAutomatonCreator::CreateJointSpaceTrajectoryToBinCMandCS(const ha::ControlMode::Ptr& cm1_ptr, const ha::ControlSwitch::Ptr& cs1_ptr,const ha::ControlMode::Ptr& cm2_ptr, const ha::ControlSwitch::Ptr& cs2_ptr, const std::string& name)
{

    Eigen::MatrixXd arm_vels = _max_js_vel_arm;
    arm_vels(3,0) = 0.4;

    //first move arm and base
    cm1_ptr->setName(name+ std::string("_1_cm")); // name is important!

    cs1_ptr->setName(name+ std::string("_1_cs"));

    //Interpolated Joint Controller
    Eigen::MatrixXd to_bin_js_traj_arm_1;
    to_bin_js_traj_arm_1.resize(1,7);
    //0.0, -0.38, 0.0, 3.09285, 0.0, -1.14859, -0.1647
    to_bin_js_traj_arm_1 << 0.196788, -0.607917, -0.183771, 3.09234, -0.083262, -0.979138, -0.141788;

    Eigen::MatrixXd to_bin_js_traj_arm_t_1 = to_bin_js_traj_arm_1.transpose();

    ha::Controller::Ptr js_ctrl_arm2_1 = _createJointSpaceArmController(name + std::string("_arm_ctrl"), to_bin_js_traj_arm_t_1, arm_vels);
    js_ctrl_arm2_1->setGoalIsRelative(false);



    ha::ControlSet::Ptr js_cs2_1 = _createJointSpaceWholeBodyControlSet(js_ctrl_arm2_1, ha::Controller::Ptr());
    js_cs2_1->setName("new_name");
    cm1_ptr->setControlSet(js_cs2_1);

    ha::JumpCondition::Ptr arm_conv_jc2_1 = _createJointSpaceArmConvergenceCondition(js_ctrl_arm2_1);

    // Arm is not converging for joint 6 when carrying heavy objects -> increase epsilon
    double joint_epsilon_modified_j6 = _joint_epsilon + 0.1;
    arm_conv_jc2_1->setEpsilon(joint_epsilon_modified_j6);

    cs1_ptr->add(arm_conv_jc2_1);

    //then move only arm
    cm2_ptr->setName(name+ std::string("_2_cm")); // name is important!

    cs2_ptr->setName(name+ std::string("_2_cs"));
    Eigen::MatrixXd to_bin_js_traj_arm_2;
    to_bin_js_traj_arm_2.resize(2,7);
    //0.0, -0.38, 0.0, 3.09285, 0.0, -1.14859, -0.1647
    to_bin_js_traj_arm_2 << -2.56375, -0.166729, 0.112981, 2.78912, 0.186371, -1.10565, -0.116102,
            -2.53855, 0.869481, 0.0592502, 1.93086, 0.0445171, -0.831908, -0.0799355;

    Eigen::MatrixXd to_bin_js_traj_arm_t_2 = to_bin_js_traj_arm_2.transpose();
    //Eigen::MatrixXd arm_vels = _max_js_vel_arm;
    arm_vels(0,0) = 0.6;
    arm_vels(1,0) = 0.4;
    arm_vels(3,0) = 0.4;
    ha::Controller::Ptr js_ctrl_arm2_2 = _createJointSpaceArmController(name + std::string("_arm_ctrl"), to_bin_js_traj_arm_t_2, arm_vels);
    js_ctrl_arm2_2->setGoalIsRelative(false);

    ha::Controller::Ptr js_ctrl_base2_2 = _createJointSpaceBaseController(name + std::string("_base_ctrl"), "bin_drop_base", _max_js_vel_base);
    js_ctrl_base2_2->setGoalIsRelative(false);

    ha::JumpCondition::Ptr base_conv_jc2_2 = _createJointSpaceBaseConvergenceCondition(js_ctrl_base2_2);

    ha::ControlSet::Ptr js_cs2_2 = _createJointSpaceWholeBodyControlSet(js_ctrl_arm2_2, js_ctrl_base2_2);
    js_cs2_2->setName("new_name");
    cm2_ptr->setControlSet(js_cs2_2);

    ha::JumpCondition::Ptr arm_conv_jc2_2 = _createJointSpaceArmConvergenceCondition(js_ctrl_arm2_2);
    cs2_ptr->add(arm_conv_jc2_2);
    cs2_ptr->add(base_conv_jc2_2);

}


ha::Controller::Ptr HybridAutomatonCreator::CreateResamplingGraspController(std::string name, const Eigen::MatrixXd& HT_trajectory, double max_displacement_velocity, double max_rotational_velocity, bool relative_goal)
{

    //Endeffector Frame Controller
    ha::Controller::Ptr ctrl(new ha::Controller);
    ctrl->setName(name);
    ctrl->setType("InterpolatedHTransformController");
    ctrl->setArgument("interpolation_type", "cubic");
    ctrl->setArgument("reinterpolation", "1");

    Eigen::MatrixXd max_vel(2,1);
    max_vel << max_rotational_velocity, max_displacement_velocity;
    ctrl->setMaximumVelocity(max_vel);

    ctrl->setKp( _kp_opspace);
    ctrl->setKv( _kv_opspace);
    ctrl->setGoalIsRelative(relative_goal);
    ctrl->setArgument("operational_frame", "EE");

    ctrl->setGoal(HT_trajectory);

    return ctrl;
}

bool HybridAutomatonCreator::CreateResamplingGraspCMtoandfromCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& to_resampling_cs_ptr,  const ha::ControlSwitch::Ptr& successful_resampling_grasp_cs_ptr, const ha::ControlSwitch::Ptr& failed_resampling_grasp_cs_ptr, const std::string& name, std::string grasp_object_name){
    cm_ptr->setName(name+ std::string("_cm")); // name is important!

    //Interpolated Joint Controller
    int number_of_HTs = 8;
    bool larger_wobbling = false;
    float height =-0.02;
    double op_space_wobbling_vel = 0.04;
    if(grasp_object_name.compare("kyjen_squeakin_eggs_plush_puppies")==0 || grasp_object_name.compare("mommys_helper_outlet_plugs")==0 ||grasp_object_name.compare("kong_air_dog_squeakair_tennis_ball")==0||grasp_object_name.compare("dr_browns_bottle_brush")==0 )
    {
        height =-0.05;
        larger_wobbling = true;
        op_space_wobbling_vel = 0.05;
    }
    Eigen::MatrixXd up;
    up.resize(4,4);
    up << 1.0, 0.0, 0.0, height,
                   0.0, 1.0, 0.0, 0.0,
                   0.0, 0.0, 1.0, 0.0,
                   0.0 ,0.0, 0.0, 1.0;

    Eigen::MatrixXd back;
    back.resize(4,4);
    back << 1.0, 0.0, 0.0, 0.01,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.03,
            0.0 ,0.0, 0.0, 1.0;

    Eigen::MatrixXd front;
    front.resize(4,4);
    front << 1.0, 0.0, 0.0, 0.01,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, -0.03,
            0.0 ,0.0, 0.0, 1.0;

    Eigen::MatrixXd left;
    left.resize(4,4);
    left << 1.0, 0.0, 0.0, 0.01,
            0.0, 1.0, 0.0, 0.03,
            0.0, 0.0, 1.0, 0.0,
            0.0 ,0.0, 0.0, 1.0;

    Eigen::MatrixXd right;
    right.resize(4,4);
    right << 1.0, 0.0, 0.0, 0.01,
            0.0, 1.0, 0.0, -0.03,
            0.0, 0.0, 1.0, 0.0,
            0.0 ,0.0, 0.0, 1.0;

    Eigen::MatrixXd ht_trajectory;
    ht_trajectory.resize(4, number_of_HTs*4);
    ht_trajectory.block(0,0,4,4) = up;
    ht_trajectory.block(0,4,4,4) = back;
    ht_trajectory.block(0,8,4,4) = up;
    ht_trajectory.block(0,12,4,4) = front;
    ht_trajectory.block(0,16,4,4) = up;
    ht_trajectory.block(0,20,4,4) = left;
    ht_trajectory.block(0,24,4,4) = up;
    ht_trajectory.block(0,28,4,4) = right;

    ha::Controller::Ptr resampling_grasping_ctrl = CreateResamplingGraspController(name + std::string("_ctrl"), ht_trajectory, op_space_wobbling_vel, 0.25, true);
    ha::ControlSet::Ptr resampling_grasping_cs = _createTaskSpaceControlSet(resampling_grasping_ctrl, false);
    cm_ptr->setControlSet(resampling_grasping_cs);

    //CS to resample grasping CM
    //CS to wobble CM
    to_resampling_cs_ptr->setName(name + std::string("_pressure_time_cs"));
    ha::JumpConditionPtr to_resampling_grasping_pressure_jc(new ha::JumpCondition());
    ha::ROSTopicSensor::Ptr to_resampling_grasping_sensor(new ha::ROSTopicSensor());
    to_resampling_grasping_sensor->setTopic("/pneumaticbox/pressure_0", "Float64");
    to_resampling_grasping_pressure_jc->setSensor(to_resampling_grasping_sensor);
    to_resampling_grasping_pressure_jc->setConstantGoal(18.8);
    to_resampling_grasping_pressure_jc->setEpsilon(0);
    to_resampling_grasping_pressure_jc->setJumpCriterion(ha::JumpCondition::THRESH_LOWER_BOUND);
    to_resampling_cs_ptr->add(to_resampling_grasping_pressure_jc);
    // Giving some time to establish contact
    ha::JumpConditionPtr to_resampling_grasping_time_jc(new ha::JumpCondition());
    ha::SensorPtr free_sensor_time(new ha::ClockSensor());
    to_resampling_grasping_time_jc->setSensor(free_sensor_time);
    to_resampling_grasping_time_jc->setConstantGoal(3);
    to_resampling_grasping_time_jc->setGoalRelative();
    to_resampling_grasping_time_jc->setEpsilon(0);
    to_resampling_grasping_time_jc->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    to_resampling_cs_ptr->add(to_resampling_grasping_time_jc);

    //CS from wobble CM
    successful_resampling_grasp_cs_ptr->setName(std::string("from_") + name + std::string("_cs"));
    ha::JumpConditionPtr from_resampling_grasping_pressure_jc(new ha::JumpCondition());
    ha::ROSTopicSensor::Ptr from_resampling_grasping_sensor(new ha::ROSTopicSensor());
    from_resampling_grasping_sensor->setTopic("/pneumaticbox/pressure_0", "Float64");
    from_resampling_grasping_pressure_jc->setSensor(from_resampling_grasping_sensor);
    from_resampling_grasping_pressure_jc->setConstantGoal(18.8);
    from_resampling_grasping_pressure_jc->setEpsilon(0);
    from_resampling_grasping_pressure_jc->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    successful_resampling_grasp_cs_ptr->add(from_resampling_grasping_pressure_jc);

    // Giving some time to establish contact
    failed_resampling_grasp_cs_ptr->setName(std::string("from_") + name + std::string("_fail_cs"));

    ha::JumpConditionPtr to_fail_time_jc(new ha::JumpCondition());
    ha::SensorPtr fail_sensor_time(new ha::ClockSensor());
    to_fail_time_jc->setSensor(fail_sensor_time);
    if(larger_wobbling)
    {
        to_fail_time_jc->setConstantGoal(40);
    }else{
        to_fail_time_jc->setConstantGoal(20);
    }
    to_fail_time_jc->setGoalRelative();
    to_fail_time_jc->setEpsilon(0);
    to_fail_time_jc->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    failed_resampling_grasp_cs_ptr->add(to_fail_time_jc);
}


bool HybridAutomatonCreator::CreateWobbleCMandtoandfromCS(const ha::ControlMode::Ptr& cm_ptr, const ha::ControlSwitch::Ptr& cs_ptr, const ha::ControlSwitch::Ptr& cs_ptr_2, const ha::ControlSwitch::Ptr& cs_ptr_3, const std::string& name){
    cm_ptr->setName(name+ std::string("_cm")); // name is important!

    //Interpolated Joint Controller
    Eigen::MatrixXd wobble_config;
    wobble_config.resize(3,7);
    wobble_config << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.348,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.694,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.348;
    Eigen::MatrixXd wobble_config_transpose = wobble_config.transpose();
    Eigen::MatrixXd max_vel;
    max_vel.resize(7,1);
    max_vel << 0.2,0.2,0.2,0.2,0.2,0.2,0.2;
    ha::Controller::Ptr wobble_ctrl = _createJointSpaceArmController(name + std::string("_ctrl"), wobble_config_transpose, max_vel);
    wobble_ctrl->setGoalIsRelative(true);
    ha::ControlSet::Ptr wobble_cs = _createJointSpaceControlSet(wobble_ctrl);
    cm_ptr->setControlSet(wobble_cs);

    //CS to wobble CM
    cs_ptr->setName(name + std::string("_pressure_time_cs"));
    ha::JumpConditionPtr to_wobble_pressure_jc(new ha::JumpCondition());
    ha::ROSTopicSensor::Ptr to_wobble_sensor(new ha::ROSTopicSensor());
    to_wobble_sensor->setTopic("/pneumaticbox/pressure_0", "Float64");
    to_wobble_pressure_jc->setSensor(to_wobble_sensor);
    to_wobble_pressure_jc->setConstantGoal(18.8);
    to_wobble_pressure_jc->setEpsilon(0);
    to_wobble_pressure_jc->setJumpCriterion(ha::JumpCondition::THRESH_LOWER_BOUND);
    cs_ptr->add(to_wobble_pressure_jc);

    // Giving some time to establish contact

    ha::JumpConditionPtr to_wobble_time_jc(new ha::JumpCondition());
    ha::SensorPtr free_sensor_time(new ha::ClockSensor());
    to_wobble_time_jc->setSensor(free_sensor_time);
    to_wobble_time_jc->setConstantGoal(3);
    to_wobble_time_jc->setGoalRelative();
    to_wobble_time_jc->setEpsilon(0);
    to_wobble_time_jc->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    cs_ptr->add(to_wobble_time_jc);

    //CS from wobble CM
    cs_ptr_2->setName(std::string("from_") + name + std::string("_cs"));
    ha::JumpConditionPtr from_wobble_pressure_jc(new ha::JumpCondition());
    ha::ROSTopicSensor::Ptr from_wobble_sensor(new ha::ROSTopicSensor());
    from_wobble_sensor->setTopic("/pneumaticbox/pressure_0", "Float64");
    from_wobble_pressure_jc->setSensor(from_wobble_sensor);
    from_wobble_pressure_jc->setConstantGoal(18.8);
    from_wobble_pressure_jc->setEpsilon(0);
    from_wobble_pressure_jc->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    cs_ptr_2->add(from_wobble_pressure_jc);

    // Giving some time to establish contact
    cs_ptr_3->setName(std::string("from_") + name + std::string("_fail_cs"));

    ha::JumpConditionPtr to_fail_time_jc(new ha::JumpCondition());
    ha::SensorPtr fail_sensor_time(new ha::ClockSensor());
    to_fail_time_jc->setSensor(fail_sensor_time);
    to_fail_time_jc->setConstantGoal(15);
    to_fail_time_jc->setGoalRelative();
    to_fail_time_jc->setEpsilon(0);
    to_fail_time_jc->setJumpCriterion(ha::JumpCondition::THRESH_UPPER_BOUND);
    cs_ptr_3->add(to_fail_time_jc);

}

bool  HybridAutomatonCreator::CreateGCCM(const ha::ControlMode::Ptr& cm_ptr,
                                         const std::string& name){
    cm_ptr->setName(name);
    ha::ControlSet::Ptr gravity_cs(new ha::ControlSet());
    gravity_cs->setType("rxControlSet");
    cm_ptr->setControlSet(gravity_cs);
}

ha::HybridAutomaton::Ptr HybridAutomatonCreator::createHybridAutomatonFromTask(const apc::ApcTask &task, int tool, int localization, int shelf_tracking){

    static int task_counter = 0;
    std::string pre_grasp_topic = "/object_recognition/grasp_pregrasp";
    std::string grasp_topic = "/object_recognition/grasp";
    std::string reposition_topic = "/object_recognition/grasp_reposition";
    std::string reorientation_topic = "/object_recognition/grasp_reorientation";
    std::string post_grasp_topic = "/object_recognition/post_grasp";

    std::string move_outside_frame = "/object_recognition/grasp_move_outside";
    std::string basket_drop_frame   = "/basket_drop";

    //create Hybrid Automaton
    ha::HybridAutomaton::Ptr grasp_ha = ha::HybridAutomaton::Ptr(new ha::HybridAutomaton());

    std::stringstream ss;
    ss << task_counter++;
    grasp_ha->setName("Grasp_HA" + ss.str());

    /////////////////////////////////////////////////////////////
    //Create failure Mode
    //gravity compensation
    ha::ControlMode::Ptr failure_cm(new ha::ControlMode());
    this->CreateGCCM(failure_cm, "failure");

    //add controlmode to HA
    grasp_ha->addControlMode(failure_cm);

    /////////////////////////////////////////////////////////////
    // Create first Mode
    // Go to Home Position
    ha::ControlMode::Ptr goto_home1_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr goto_home1_convergence_cs(new ha::ControlSwitch());
    std::string goto_home1_name = "Goto_Home1";
    this->CreateGoToHomeCMAndConvergenceCS(goto_home1_cm, goto_home1_convergence_cs, goto_home1_name, _home_config);
    ha::ControlSwitch::Ptr goto_home1_time_cs = this->CreateMaxTimeControlSwitch(goto_home1_name, 20.0);

    //add Mode to HA
    grasp_ha->addControlMode(goto_home1_cm);
    grasp_ha->setCurrentControlMode(goto_home1_cm->getName());    

    /////////////////////////////////////////////////////////////
    // Create second Mode
    // Go to View Position
    ha::ControlMode::Ptr goto_view_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr goto_view_convergence_cs(new ha::ControlSwitch());
    std::string goto_view_name = "Goto_view";
    this->CreateGoToViewCMAndConvergenceCS(goto_view_cm, goto_view_convergence_cs, goto_view_name, localization, shelf_tracking, task.getBinID());
    ha::ControlSwitch::Ptr goto_view_time_cs = this->CreateMaxTimeControlSwitch(goto_view_name, 30.0);


    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(goto_home1_cm->getName(), goto_home1_convergence_cs, goto_view_cm);
    grasp_ha->addControlSwitch(goto_home1_cm->getName(), goto_home1_time_cs, goto_view_cm->getName());

    /////////////////////////////////////////////////////////////
    //Create third Mode
    // Run object recognition
    ha::ControlMode::Ptr run_obj_rec_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr after_objrec_success_cs(new ha::ControlSwitch());
    ha::ControlSwitch::Ptr after_objrec_failure_cs(new ha::ControlSwitch());
    std::string run_objrec_name = "run_object_recognition"; //don't change this modes name!
    this->CreateRunObjRecCMAndCS(run_obj_rec_cm, after_objrec_success_cs, after_objrec_failure_cs, run_objrec_name);
    ha::ControlSwitch::Ptr after_objrec_time_cs = CreateMaxTimeControlSwitch(run_objrec_name, 30);

    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(goto_view_cm->getName(), goto_view_convergence_cs, run_obj_rec_cm);
    grasp_ha->addControlSwitch(goto_view_cm->getName(), goto_view_time_cs, run_obj_rec_cm->getName());


    /////////////////////////////////////////////////////////////
    // Create fourth Mode
    // Go back to second home position for low bins
    ha::ControlMode::Ptr goto_home2_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr goto_home2_convergence_cs(new ha::ControlSwitch());

    std::string goto_home2_name = "Goto_Home2";

    if(task.getBinID()<6)
        this->CreateGoToHomeCMAndConvergenceCS(goto_home2_cm, goto_home2_convergence_cs, goto_home2_name, _home_config_high_bin);
    else
        this->CreateGoToHomeCMAndConvergenceCS(goto_home2_cm, goto_home2_convergence_cs, goto_home2_name, _home_config_low_bin);

    ha::ControlSwitch::Ptr goto_home2_time_cs = CreateMaxTimeControlSwitch(goto_home2_name, 30);


    // We do not add the control switch here. We do it at the end, depending of if we grasp or not
    grasp_ha->addControlMode(goto_home2_cm);

    /////////////////////////////////////////////////////////////
    //Create fifth Mode
    //Move to Bin
    ha::ControlMode::Ptr pre_grasp_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr pre_grasp_convergence_cs(new ha::ControlSwitch());
    std::string pre_grasp_cm_name = "Pre_grasp";
    this->CreateGoToBBCMAndConvergenceCS(pre_grasp_cm, pre_grasp_convergence_cs, pre_grasp_cm_name, pre_grasp_topic, 0.08, 0.25, false, true, true);
    ha::ControlSwitch::Ptr pre_grasp_time_cs = CreateMaxTimeControlSwitch(pre_grasp_cm_name, 20);


    //add controlmode and switch to HA
    //grasp_ha->addControlMode(pre_grasp_cm);
    grasp_ha->addControlSwitchAndMode(goto_home2_cm->getName(), goto_home2_convergence_cs, pre_grasp_cm);
    grasp_ha->addControlSwitch(goto_home2_cm->getName(), goto_home2_time_cs, pre_grasp_cm->getName());

    /////////////////////////////////////////////////////////////
    //Create fifth Mode 0)
    //Reorient EE
    ha::ControlMode::Ptr reorient_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr reorient_convergence_cs(new ha::ControlSwitch());
    std::string reorient_cm_name = "Reorient_in_front_of_bin";
    this->CreateGoToBBCMAndConvergenceCS(reorient_cm, reorient_convergence_cs, reorient_cm_name, reorientation_topic, 0.06, 0.3, false, true, false);
    ha::ControlSwitch::Ptr reorient_time_cs = CreateMaxTimeControlSwitch(reorient_cm_name, 20);


    //add controlmode and switch to HA
    //grasp_ha->addControlMode(pre_grasp_cm);
    grasp_ha->addControlSwitchAndMode(pre_grasp_cm->getName(), pre_grasp_convergence_cs, reorient_cm);
    grasp_ha->addControlSwitch(pre_grasp_cm->getName(), pre_grasp_time_cs, reorient_cm->getName());


    /////////////////////////////////////////////////////////////
    //Create fifth Mode a): move inside bin
    //Move down in Bin
    ha::ControlMode::Ptr move_in_bin_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr move_in_bin_cs(new ha::ControlSwitch());
    std::string move_in_bin_cm_name = "Move_inside_Bin";
    this->CreateGoToBBCMAndConvergenceCS(move_in_bin_cm, move_in_bin_cs, move_in_bin_cm_name, reposition_topic, 0.04 , 0.25, false, true, true);
    ha::ControlSwitch::Ptr move_in_bin_time_cs = CreateMaxTimeControlSwitch(move_in_bin_cm_name, 30);


    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(reorient_cm->getName(), reorient_convergence_cs, move_in_bin_cm);
    grasp_ha->addControlSwitch(reorient_cm->getName(), reorient_time_cs, move_in_bin_cm->getName());

    /////////////////////////////////////////////////////////////
    //Create fifth Mode b): reposition ee
    //Move down in Bin
    ha::ControlMode::Ptr reposition_in_bin_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr reposition_in_bin_cs(new ha::ControlSwitch());
    std::string reposition_in_bin_cm_name = "Reposition_in_Bin";
    this->CreateGoToBBCMAndConvergenceCS(reposition_in_bin_cm, reposition_in_bin_cs, reposition_in_bin_cm_name, reposition_topic, 0.04 , 0.25, false, true, false);
    ha::ControlSwitch::Ptr reposition_in_bin_time_cs = CreateMaxTimeControlSwitch(reposition_in_bin_cm_name, 5);


    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(move_in_bin_cm->getName(), move_in_bin_cs, reposition_in_bin_cm);

    /////////////////////////////////////////////////////////////
    //Create sixth Mode
    //Move down in Bin
    ha::ControlMode::Ptr move_down_bin_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr move_down_bin_cs(new ha::ControlSwitch());
    std::string Move_down_in_bin_cm_name = "Move_Down_in_Bin";
    this->CreateGoToBBGraspCMAndCS(move_down_bin_cm, move_down_bin_cs, Move_down_in_bin_cm_name, grasp_topic, 0.04 , 0.25, false, true, false, (int) task.getOtherObjectIDs().size(),tool);
    ha::ControlSwitch::Ptr move_down_bin_time_cs = CreateMaxTimeControlSwitch(Move_down_in_bin_cm_name, 20);


    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(reposition_in_bin_cm->getName(), reposition_in_bin_cs, move_down_bin_cm);
    grasp_ha->addControlSwitch(reposition_in_bin_cm->getName(), reposition_in_bin_time_cs, move_down_bin_cm->getName());


    /////////////////////////////////////////////////////////////
    //Create seventh Mode
    //Grasp
    ha::ControlMode::Ptr grasp_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr grasp_cs(new ha::ControlSwitch());
    this->CreateGraspCMAndCS(grasp_cm, grasp_cs, "Grasp", tool);

    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(move_down_bin_cm->getName(), move_down_bin_cs, grasp_cm);
    grasp_ha->addControlSwitch(move_down_bin_cm->getName(), move_down_bin_time_cs, grasp_cm->getName());

    /////////////////////////////////////////////////////////////
    //Create eighth Mode
    //Move up and out again
    ha::ControlMode::Ptr post_grasp_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr post_grasp_cs(new ha::ControlSwitch());
    std::string post_grasp_cm_name = "post_grasp";
    this->CreateGoToBBCMAndConvergenceCS(post_grasp_cm, post_grasp_cs, post_grasp_cm_name, post_grasp_topic, 0.065, 0.25, true, false, true);
    ha::ControlSwitch::Ptr post_grasp_time_cs = CreateMaxTimeControlSwitch(post_grasp_cm_name, 30);

    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(grasp_cm->getName(), grasp_cs, post_grasp_cm);

    ha::ControlMode::Ptr resample_grasping_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr to_resample_grasping_cs(new ha::ControlSwitch());
    ha::ControlSwitch::Ptr successful_resample_grasping_cs(new ha::ControlSwitch());
    ha::ControlSwitch::Ptr fail_resample_grasping_cs(new ha::ControlSwitch());
    CreateResamplingGraspCMtoandfromCS(resample_grasping_cm, to_resample_grasping_cs, successful_resample_grasping_cs, fail_resample_grasping_cs, "resample_grasping", task.getGraspObjectName());
    grasp_ha->addControlSwitchAndMode(grasp_cm->getName(), to_resample_grasping_cs, resample_grasping_cm);
    grasp_ha->addControlSwitch(resample_grasping_cm->getName(), successful_resample_grasping_cs, post_grasp_cm->getName());


    //////////////////////////////////////////////////////////////
        //Parallel modes and switches for adherence to shelf

        // add ControlSwitch for recognizing adherence to shelf with FT-Sensor
        ha::ControlSwitch::Ptr post_grasp_adherence_cs(new ha::ControlSwitch());

        /////////////////////////////////////////////////////////////
        //Create ninth Mode B
        //Wait til freedom
        ha::ControlMode::Ptr adherence_handle_cm(new ha::ControlMode());
        ha::ControlSwitch::Ptr post_adherence_time_cs(new ha::ControlSwitch());
        ha::ControlSwitch::Ptr post_adherence_pressure_cs(new ha::ControlSwitch());

        this->CreateAdherenceCSAndCMAndTimeCS(post_grasp_adherence_cs, adherence_handle_cm,
                                              post_adherence_time_cs, post_adherence_pressure_cs, "failure_VC_off", 10, tool);


        grasp_ha->addControlSwitchAndMode(post_grasp_cm->getName(), post_grasp_adherence_cs, adherence_handle_cm);
        grasp_ha->addControlSwitch( resample_grasping_cm->getName(),fail_resample_grasping_cs, adherence_handle_cm->getName());


        /////////////////////////////////////////////////////////////
        //Create extra Mode
        //move_outside_bin after failure
        ha::ControlMode::Ptr move_outside_bin_failure_cm(new ha::ControlMode());
        ha::ControlSwitch::Ptr move_outside_bin_failure_cs(new ha::ControlSwitch());
        std::string move_outside_bin_failure_cm_name = "move_outside_bin_failure";
        this->CreateGoToTfCMAndConvergenceCS(move_outside_bin_failure_cm, move_outside_bin_failure_cs, move_outside_bin_failure_cm_name, move_outside_frame, 0.15, 0.25, true);


        //add controlmode and switch to HA
        grasp_ha->addControlSwitchAndMode(adherence_handle_cm->getName(), post_adherence_time_cs, move_outside_bin_failure_cm);
        grasp_ha->addControlSwitch(adherence_handle_cm->getName(), post_adherence_pressure_cs, move_outside_bin_failure_cm->getName());


        /////////////////////////////////////////////////////////////
        //Create eleventh Mode B
        //stop object recognition
        ha::ControlMode::Ptr stop_obj_rec_after_failure_cm(new ha::ControlMode());
        ha::ControlSwitch::Ptr stop_obj_rec_after_failure_cs(new ha::ControlSwitch());
        this->CreateStopObjRecCMAndCS(stop_obj_rec_after_failure_cm, stop_obj_rec_after_failure_cs,"stop_object_recognition_after_failure");

        //add controlmode and switch to HA
        grasp_ha->addControlMode(stop_obj_rec_after_failure_cm);

        //go to failure mode
        grasp_ha->addControlSwitchAndMode(stop_obj_rec_after_failure_cm->getName(), stop_obj_rec_after_failure_cs, failure_cm);


        /////////////////////////////////////////////////////////////
        //Create extra failure handling Mode: too close to dropping bin
        //move_outside_bin after being too close to dropping bin
        ha::ControlMode::Ptr move_outside_bin_tooclose_cm(new ha::ControlMode());
        ha::ControlSwitch::Ptr move_outside_bin_tooclose_convergence_cs(new ha::ControlSwitch());

        ha::ControlSwitch::Ptr too_close_to_bin_cs(new ha::ControlSwitch());
        too_close_to_bin_cs->setName("too_close_to_dropping_bin");
        ha::JumpCondition::Ptr too_close_to_bin_cond = _createDistanceToDroppingBinCondition();
        too_close_to_bin_cs->add(too_close_to_bin_cond);

        std::string move_outside_bin_tooclose_cm_name = "move_outside_bin_tooclose";
        this->CreateGoToTfCMAndConvergenceCS(move_outside_bin_tooclose_cm, move_outside_bin_tooclose_convergence_cs, move_outside_bin_tooclose_cm_name, move_outside_frame, 0.065, 0.25, false);
        ha::ControlSwitch::Ptr move_outside_bin_tooclose_time_cs = CreateMaxTimeControlSwitch(move_outside_bin_tooclose_cm_name, 20);

        grasp_ha->addControlSwitchAndMode(post_grasp_cm->getName(), too_close_to_bin_cs, move_outside_bin_tooclose_cm);

    ha::ControlMode::Ptr move_to_basket_1_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr move_to_basket_1_convergence_cs(new ha::ControlSwitch());
    ha::ControlMode::Ptr move_to_basket_2_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr move_to_basket_2_convergence_cs(new ha::ControlSwitch());

    ha::ControlSwitch::Ptr move_to_basket_1_time_cs = CreateMaxTimeControlSwitch("Move_to_Basket_1", 20);
    ha::ControlSwitch::Ptr move_to_basket_2_time_cs = CreateMaxTimeControlSwitch("Move_to_Basket_2", 20);
    this->CreateJointSpaceTrajectoryToBinCMandCS(move_to_basket_1_cm, move_to_basket_1_convergence_cs, move_to_basket_2_cm, move_to_basket_2_convergence_cs,"Move_to_Basket");
    grasp_ha->addControlSwitchAndMode(post_grasp_cm->getName(), post_grasp_cs, move_to_basket_1_cm);
    grasp_ha->addControlSwitch(post_grasp_cm->getName(), post_grasp_time_cs, move_to_basket_1_cm->getName());

    //also go to bin after safety stop of base
    grasp_ha->addControlSwitch(move_outside_bin_tooclose_cm->getName(), move_outside_bin_tooclose_convergence_cs, move_to_basket_1_cm->getName());
    grasp_ha->addControlSwitch(move_outside_bin_tooclose_cm->getName(), move_outside_bin_tooclose_time_cs, move_to_basket_1_cm->getName());


    grasp_ha->addControlSwitchAndMode(move_to_basket_1_cm->getName(), move_to_basket_1_convergence_cs, move_to_basket_2_cm);
    grasp_ha->addControlSwitch(move_to_basket_1_cm->getName(), move_to_basket_1_time_cs, move_to_basket_2_cm->getName());


    //Move to drop point of basket
    ha::ControlMode::Ptr move_drop_basket_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr move_drop_basket_convergence_cs(new ha::ControlSwitch());
    std::string move_drop_basket_cm_name = "Move_drop_Basket";
    this->CreateGoToTfCMAndConvergenceCS(move_drop_basket_cm, move_drop_basket_convergence_cs, move_drop_basket_cm_name, basket_drop_frame, 0.1 , 0.25, false);
    ha::ControlSwitch::Ptr move_drop_basket_time_cs = this->CreateMaxTimeControlSwitch(move_drop_basket_cm_name, 20);
    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(move_to_basket_2_cm->getName(), move_to_basket_2_convergence_cs, move_drop_basket_cm);
    grasp_ha->addControlSwitch(move_to_basket_2_cm->getName(), move_to_basket_2_time_cs, move_drop_basket_cm->getName());


    /////////////////////////////////////////////////////////////
    //Create tenth Mode
    //ungrasp
    ha::ControlMode::Ptr ungrasp_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr ungrasp_pressure_cs(new ha::ControlSwitch());
    ha::ControlSwitch::Ptr ungrasp_time_cs(new ha::ControlSwitch());
    this->CreateUngraspCMAndCS(ungrasp_cm, ungrasp_pressure_cs, ungrasp_time_cs, "Ungrasp", tool);

    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(move_drop_basket_cm->getName(), move_drop_basket_convergence_cs, ungrasp_cm);
    grasp_ha->addControlSwitch(move_drop_basket_cm->getName(), move_drop_basket_time_cs, ungrasp_cm->getName());


    /////////////////////////////////////////////////////////////
    //Create eleventh Mode
    //stop object recognition
    ha::ControlMode::Ptr stop_obj_rec_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr stop_obj_rec_cs(new ha::ControlSwitch());
    std::string stop_obj_rec_cm_name = "stop_object_recognition";
    this->CreateStopObjRecAndGoToPrehomeCMAndCS(stop_obj_rec_cm, stop_obj_rec_cs, stop_obj_rec_cm_name);
    ha::ControlSwitch::Ptr stop_obj_rec_time_cs = CreateMaxTimeControlSwitch(stop_obj_rec_cm_name, 10);

    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(ungrasp_cm->getName(), ungrasp_pressure_cs, stop_obj_rec_cm);
    grasp_ha->addControlSwitch(ungrasp_cm->getName(), ungrasp_time_cs, stop_obj_rec_cm->getName());



    /////////////////////////////////////////////////////////////
    //Create last success Mode
    //gravity compensation
    ha::ControlMode::Ptr success_cm(new ha::ControlMode());
    this->CreateGCCM(success_cm, "finished");

    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(stop_obj_rec_cm->getName(), stop_obj_rec_cs, success_cm);
    grasp_ha->addControlSwitchAndMode(stop_obj_rec_cm->getName(), stop_obj_rec_time_cs, success_cm);

    // If we do not want to grasp, but only to watch and collect images, we make that both control switches after object recognition
    // point to the same next control mode, stop object recognition CM. If we want to grasp, we point to goto_home2_cm
    if(tool == 2)
    {
        grasp_ha->addControlSwitch(run_obj_rec_cm->getName(), after_objrec_success_cs,  stop_obj_rec_cm->getName());
    }else{
        grasp_ha->addControlSwitch(run_obj_rec_cm->getName(), after_objrec_success_cs,  goto_home2_cm->getName());
    }
    grasp_ha->addControlSwitch(run_obj_rec_cm->getName(), after_objrec_failure_cs,  stop_obj_rec_after_failure_cm->getName());
    grasp_ha->addControlSwitch(run_obj_rec_cm->getName(), after_objrec_time_cs,  stop_obj_rec_after_failure_cm->getName());




    /////////////////////////////////////////////////////////////////////////////////////
    // Safety mode for failed grasp attempts
    /////////////////////////////////////////////////////////////
    //This mode checks the pressure shortly before dropping the object - if no pressure is sensed, the HA goes
    //to failure mode which will reattempt the grasp
    ha::ControlMode::Ptr failure_ungrasp_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr failure_ungrasp_pressure_cs(new ha::ControlSwitch());

    failure_ungrasp_pressure_cs->setName("Ungrasp_after_failure_pressure_time_cs");
    ha::JumpConditionPtr failure_ungrasp_pressure_jc(new ha::JumpCondition());
    ha::ROSTopicSensor::Ptr failure_ungrasp_pressure_sensor(new ha::ROSTopicSensor());
    failure_ungrasp_pressure_sensor->setTopic("/pneumaticbox/pressure_0", "Float64");
    failure_ungrasp_pressure_jc->setSensor(failure_ungrasp_pressure_sensor);
    failure_ungrasp_pressure_jc->setConstantGoal(18.8);
    failure_ungrasp_pressure_jc->setEpsilon(0);
    failure_ungrasp_pressure_jc->setJumpCriterion(ha::JumpCondition::THRESH_LOWER_BOUND);
    failure_ungrasp_pressure_cs->add(failure_ungrasp_pressure_jc);

    ha::JumpConditionPtr failure_ungrasp_time_jc(new ha::JumpCondition());
    ha::SensorPtr failure_ungrasp_clock_sensor(new ha::ClockSensor());
    failure_ungrasp_time_jc->setSensor(failure_ungrasp_clock_sensor);
    failure_ungrasp_time_jc->setConstantGoal(0.2);
    failure_ungrasp_time_jc->setGoalRelative();
    failure_ungrasp_time_jc->setEpsilon(0);
    failure_ungrasp_time_jc->setJumpCriterion(ha::JumpCondition::THRESH_LOWER_BOUND);
    failure_ungrasp_pressure_cs->add(failure_ungrasp_time_jc);

    ha::ControlSwitch::Ptr failure_ungrasp_convergence_cs(new ha::ControlSwitch());
    this->CreateStopObjRecAndGoToPrehomeCMAndCS(failure_ungrasp_cm, failure_ungrasp_convergence_cs, "Ungrasp_after_failure");


    //add controlmode and switch to HA
    grasp_ha->addControlSwitchAndMode(ungrasp_cm->getName(), failure_ungrasp_pressure_cs, failure_ungrasp_cm);
    grasp_ha->addControlSwitch(failure_ungrasp_cm->getName(), failure_ungrasp_convergence_cs, stop_obj_rec_after_failure_cm->getName());


    //Roberto: new cs for safety operations
    // CS1: Move_inside_Bin_cm -> failure_move_outside_cm
    ha::ControlSwitch::Ptr safety_towards_bin_ft_cs(new ha::ControlSwitch());
    // threshold for FT sensor when going into the bin
    Eigen::MatrixXd safety_towards_bin_ft_thres(6,1);
    safety_towards_bin_ft_thres << 0,0,-20,0,0,0;
    Eigen::MatrixXd safety_towards_bin_ft_weights(6,1);
    safety_towards_bin_ft_weights << 0,0,1,0,0,0;
    safety_towards_bin_ft_cs->setName(std::string("safety_towards_bin_ft_cs"));
    ha::JumpConditionPtr safety_towards_bin_ft_jc(new ha::JumpCondition());
    ha::ForceTorqueSensorPtr safety_towards_bin_ft_sensor(new ha::ForceTorqueSensor());
    safety_towards_bin_ft_jc->setSensor(safety_towards_bin_ft_sensor);
    safety_towards_bin_ft_jc->setGoalRelative();
    safety_towards_bin_ft_jc->setConstantGoal(safety_towards_bin_ft_thres);
    safety_towards_bin_ft_jc->setJumpCriterion(ha::JumpCondition::THRESH_LOWER_BOUND ,safety_towards_bin_ft_weights);
    safety_towards_bin_ft_jc->setEpsilon(0.0);
    safety_towards_bin_ft_cs->add(safety_towards_bin_ft_jc);

     ha::ControlSwitch::Ptr move_outside_bin_failure_time_cs = CreateMaxTimeControlSwitch(move_outside_bin_failure_cm_name, 10);
    //add controlmode and switch to HA
    grasp_ha->addControlSwitch(move_in_bin_cm->getName(), safety_towards_bin_ft_cs, move_outside_bin_failure_cm->getName());
    grasp_ha->addControlSwitch(move_outside_bin_failure_cm->getName(), move_outside_bin_failure_cs, stop_obj_rec_after_failure_cm->getName());
    grasp_ha->addControlSwitch(move_outside_bin_failure_cm->getName(), move_outside_bin_failure_time_cs, stop_obj_rec_after_failure_cm->getName());
    grasp_ha->addControlSwitch(move_in_bin_cm->getName(), move_in_bin_time_cs, move_outside_bin_failure_cm->getName());

    /////////////////////////////////////////////////////////////
    //When retreating, don't collide with the bin!
    ha::ControlMode::Ptr move_outside_bin_failure_tooclose_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr move_outside_bin_failure_tooclose_convergence_cs(new ha::ControlSwitch());

    ha::ControlSwitch::Ptr too_close_to_bin_failure_cs(new ha::ControlSwitch());
    too_close_to_bin_failure_cs->setName("too_close_to_dropping_bin_when retreating");
    ha::JumpCondition::Ptr too_close_to_bin_failure_cond = _createDistanceToDroppingBinCondition();
    too_close_to_bin_failure_cond->setConstantGoal(0.48);
    too_close_to_bin_failure_cs->add(too_close_to_bin_failure_cond);

    std::string move_outside_bin_failure_tooclose_cm_name = "move_outside_bin_failure_arm";
    this->CreateGoToTfCMAndConvergenceCS(move_outside_bin_failure_tooclose_cm, move_outside_bin_failure_tooclose_convergence_cs, move_outside_bin_failure_tooclose_cm_name, move_outside_frame, 0.15, 0.25, false);
    ha::ControlSwitch::Ptr move_outside_bin_failure_tooclose_time_cs = CreateMaxTimeControlSwitch(move_outside_bin_failure_tooclose_cm_name, 10);
    grasp_ha->addControlSwitchAndMode(move_outside_bin_failure_cm->getName(), too_close_to_bin_failure_cs, move_outside_bin_failure_tooclose_cm);
    grasp_ha->addControlSwitch(move_outside_bin_failure_tooclose_cm->getName(), move_outside_bin_failure_tooclose_convergence_cs, stop_obj_rec_after_failure_cm->getName());
    grasp_ha->addControlSwitch(move_outside_bin_failure_tooclose_cm->getName(), move_outside_bin_failure_tooclose_time_cs, stop_obj_rec_after_failure_cm->getName());


    //Roberto: new cs for safety operations
    // CS2: Move_drop_basket_cm -> ungrasp_cm
    ha::ControlSwitch::Ptr safety_against_piling_cs(new ha::ControlSwitch());
    // threshold for FT sensor when going to dropping pose
    Eigen::MatrixXd safety_against_piling_thres(6,1);
    safety_against_piling_thres << -5,0,0,0,0,0;
    Eigen::MatrixXd safety_against_piling_weights(6,1);
    safety_against_piling_weights << 1,0,0,0,0,0;
    safety_against_piling_cs->setName(std::string("safety_against_piling_cs"));
    ha::JumpConditionPtr safety_against_piling_jc(new ha::JumpCondition());
    ha::ForceTorqueSensorPtr safety_against_piling_sensor(new ha::ForceTorqueSensor());
    safety_against_piling_jc->setSensor(safety_against_piling_sensor);
    safety_against_piling_jc->setGoalRelative();
    safety_against_piling_jc->setConstantGoal(safety_against_piling_thres);
    safety_against_piling_jc->setJumpCriterion(ha::JumpCondition::THRESH_LOWER_BOUND ,safety_against_piling_weights);
    safety_against_piling_jc->setEpsilon(0.0);
    safety_against_piling_cs->add(safety_against_piling_jc);

    grasp_ha->addControlSwitch(move_drop_basket_cm->getName(), safety_against_piling_cs, ungrasp_cm->getName());


    ROS_INFO_STREAM_NAMED("createHybridAutomatonFromTask", "createHybridAutomatonFromTask: Created HA to grasp "
            << APCDefinitions::getInstance().getObjectLabelById(task.getGraspObjectID())
            << " out of Bin Nr: " << task.getBinID()+1);


    return grasp_ha;
}


}
