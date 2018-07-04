
@mainpage RBO Lab's Hybrid Automaton Library
 
This library implements a platform agnostic framework for constructing and executing Hybrid Automatons. In most places, the classes have to be derived to implement the system specific portions such as the control loop, controllers and control switches.


# Hybrid Automaton Basics

A [Hybrid Automaton](@ref ha::HybridAutomaton)  is a directed graph where the nodes are continuous behaviours defined by one or more controllers and the edges are discrete transitions between them.

## Control Mode
A [Control Mode](@ref ha::ControlMode) is a node in this graph. Only one [Control Mode](@ref ha::ControlMode) can be active at a time. The active mode completely defines the motion of the robot.

Each [Control Mode](@ref ha::ControlMode) contains one [Control Set](@ref ha::ControlSet). The [Control Set](@ref ha::ControlSet) is an interface for an algorithm that combines the output of one or more [Controllers](@ref ha::Controller) into a control signal for the robot. Each [Control Set](@ref ha::ControlSet) contains one or more [Controllers](@ref ha::Controller). 

## Control Switch 

A [Control Switch](@ref ha::ControlSwitch) is a transition between two Control Modes. A Control Switch is said to be active when the transition should be executed. 

Each ControlSwitch contains one or more [Jump Conditions](@ref ha::JumpCondition). A Control Switch gets active, when all its Jump Conditions are met.

If two or more Control Switches get active simultaneously, the one that was added first to the Hybrid Automaton will be executed. 

## Jump Condition 
A [Jump Condition](@ref ha::JumpCondition) is the smallest unit of a system state. Jump conditions can relate to both internal states (e.g. elapsed time, controller convergence) and external observations (e.g. force threshold).

Typical Jump Conditions are:

* Reaching a certain joint configuration
* Reaching a certain time stamp
* Registering a force threshold

## Sensors
A [Sensor](@ref ha::Sensor) is the description of a type of information source that shapes the jump conditions. Different sensor classes (i.e. encoder, force/torque, etc.) are used to inform and monitor system state. Each sensor class returns its state at a given time and have inherent serialization/deserialization functions to uniquely express them in hybrid automaton descriptions.

# Installation
See our [GitLab WIKI](https://gitlab.tubit.tu-berlin.de/rbo-lab/rswin/wikis/ha_build)

## Dependencies

The library depends on the following external libraries:
* TinyXml (libtinyxml-dev package)
* LibEigen (libeigen3-dev package), [Homepage] (http://eigen.tuxfamily.org/)


 






































 




