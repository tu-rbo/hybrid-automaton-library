cmake_minimum_required (VERSION 2.6)

# Maps to Visual Studio solution file (Tutorial.sln)
# The solution will have all targets (exe, lib, dll) 
# as Visual Studio projects (.vcproj)
project (hybrid_automaton)

# Turn on the ability to create folders to organize projects (.vcproj)
# It creates "CMakePredefinedTargets" folder by default and adds CMake
# defined projects like INSTALL.vcproj and ZERO_CHECK.vcproj
set_property (GLOBAL PROPERTY USE_FOLDERS ON)

#find_package(Boost REQUIRED COMPONENTS signals thread)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

SET (BOOST_ROOT "C:/Program Files/boost/boost_1_57_0")
SET (EIGEN_DIR "C:/Program Files/eigen/eigen-eigen-1306d75b4a21")

find_package(TinyXML)
if(NOT TinyXML_FOUND)
	message ("TinyXML not found!!")
else()
	message ("TinyXML found")
endif()

find_package(Boost)
if(NOT Boost_FOUND)
	message ("Boost not found!!")
else()
	message ("Boost found")
endif()

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
	message ("Eigen not found!!")
else()
	message ("Eigen found")
endif()

include_directories(${TinyXML_INCLUDE_DIRS})



# Set compiler flags and options. 
# Here it is setting the Visual Studio warning level to 4
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")

# Command to output information to the console
# Useful for displaying errors, warnings, and debugging
#message ("cxx Flags: " ${CMAKE_CXX_FLAGS})

include_directories(${PROJECT_SOURCE_DIR}"/include")
include_directories(${BOOST_ROOT} )
include_directories(${EIGEN3_INCLUDE_DIR} )

# Collect sources into the variable HA_SOURCES 
set (HA_CORE_HEADERS
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/HybridAutomaton.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/HybridAutomatonOStringStream.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/HybridAutomatonRegistration.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/ControlMode.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/ControlSwitch.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/ControlSet.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/Controller.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/JumpCondition.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/System.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/Serializable.h")

set (HA_SENSOR_HEADERS
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/Sensor.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/ClockSensor.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/ForceTorqueSensor.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/FramePoseSensor.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/JointConfigurationSensor.h")

set (HA_DESCRIPTION_HEADERS
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/DescriptionTree.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/DescriptionTreeNode.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/DescriptionTreeXML.h"
	"${PROJECT_SOURCE_DIR}/include/hybrid_automaton/DescriptionTreeNodeXML.h")

subdirs(src)


