
@mainpage RBO Lab's Hybrid Automaton Library
 
This library implements a platform agnostic framework for constructing and executing Hybrid Automatons. In most places, the classes have to be derived to implement the system specific portions such as the control loop, controllers and control switches.


# Hybrid Automaton Basics

A Hybrid Automaton is a directed graph where the nodes are controllers and the edges are transitions between controllers.

## Control Switch 

A Control Switch is a transition between two Controllers. A Control Switch is said to be active when the transition should be executed 

A Control Switch gets active, when all its Jump Conditions are met

If two or more Control Switches get active simultaneously, the one that was added first to the Hybrid Automaton will be executed. 


## Jump Condition 
A Jump Condition is the smallest unit of a system state. Jump conditions can relate to both internal states (e.g. elapsed time, controller convergence) and external observations (e.g. force threshold). In the latter case, they resemble perceptual feature detectors.

Typical Jump Conditions are:

* Reaching a certain joint configuration
* Reaching a certain time stamp
* Registering a force threshold

## Controller 

Every node of a Hybrid Automaton is called Controller. 
In fact, in our implementation every node is a set of controllers to simplify reuse.




A Controller is a set of control algorithms that are executed concurrently until any of the outgoing Control Switches are met. 



# Installation

## Dependencies

The library depends on the following external libraries:
* TinyXml (libtinyxml-dev package)
* LibEigen (libeigen3-dev package), [Homepage] (http://eigen.tuxfamily.org/)


 






































 




