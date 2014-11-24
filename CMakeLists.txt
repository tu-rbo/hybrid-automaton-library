cmake_minimum_required (VERSION 2.6)

# Maps to Visual Studio solution file (Tutorial.sln)
# The solution will have all targets (exe, lib, dll) 
# as Visual Studio projects (.vcproj)
project (hybrid_automaton)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

option(UNIT_TESTS "Build all tests." OFF)

# If you want to do special things on linux
if(CMAKE_COMPILER_IS_GNUCXX)
#	add_definitions(-Wall -ansi -Wno-deprecated -pthread)
endif()

# If you want to do special things on windows
if(MSVC)
#    set(MSVC_COMPILER_DEFS "-D_VARIADIC_MAX=10")
endif() 

# Turn on the ability to create folders to organize projects (.vcproj)
# It creates "CMakePredefinedTargets" folder by default and adds CMake
# defined projects like INSTALL.vcproj and ZERO_CHECK.vcproj
set_property (GLOBAL PROPERTY USE_FOLDERS ON)

find_package(TinyXML)
if(NOT TinyXML_FOUND AND MSVC)
	message ("TinyXML not found automatically. Retrying using TinyXML_ROOT_DIR=${PROJECT_SOURCE_DIR}/../thirdparty/tinyxml/")
	set (TinyXML_ROOT_DIR "${PROJECT_SOURCE_DIR}/../thirdparty/tinyxml/")
	find_package(TinyXML)
endif()

if(NOT TinyXML_FOUND)
	message (FATAL_ERROR "TinyXML not found!!")
else()
	message ("TinyXML found with include directories TinyXML_INCLUDE_DIRS=${TinyXML_INCLUDE_DIRS}")
	include_directories(${TinyXML_INCLUDE_DIRS})
endif()

find_package(Boost)
if(NOT Boost_FOUND AND MSVC)
	message ("Boost not found automatically. Retrying using BOOST_ROOT=C:/Program Files/boost/boost_1_57_0")
	set (BOOST_ROOT "C:/Program Files/boost/boost_1_57_0")
	find_package(Boost)
endif()
	
if(NOT Boost_FOUND)
	message (FATAL_ERROR "Boost not found!!")
else()
	message ("Boost found with root directory BOOST_ROOT=${BOOST_ROOT}")
	include_directories(${BOOST_INCLUDE_DIRS})
	include_directories(${BOOST_ROOT} )
endif()

find_package(Eigen3)
if(NOT EIGEN3_FOUND AND MSVC)
	message ("Eigen not found automatically. Retrying using EIGEN_DIR=C:/Program Files/eigen/eigen-eigen-1306d75b4a21")
	set (EIGEN_DIR "C:/Program Files/eigen/eigen-eigen-1306d75b4a21")
	find_package(Eigen3)
endif()

if(NOT EIGEN3_FOUND)
	message (FATAL_ERROR "Eigen not found!!")
else()
	message ("Eigen found with include directories EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")
	include_directories(${EIGEN3_INCLUDE_DIR} )
endif()

# Set compiler flags and options. 
# Here it is setting the Visual Studio warning level to 4
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")

# Command to output information to the console
# Useful for displaying errors, warnings, and debugging
#message ("cxx Flags: " ${CMAKE_CXX_FLAGS})

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

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

if(UNIT_TESTS)
	message ("Compiling with unit tests. Set UNIT_TESTS to false in cmake if you dont want to include them in the build.")
    enable_testing()
	subdirs(tests)
else()
	message ("Compiling without unit tests (default behavior). Set UNIT_TESTS to true in cmake if you want to include them in the build.")
endif()

