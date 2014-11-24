cmake_minimum_required (VERSION 2.6)

# Maps to Visual Studio solution file (Tutorial.sln)
# The solution will have all targets (exe, lib, dll) 
# as Visual Studio projects (.vcproj)
project (hybrid_automaton)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

option(UNIT_TESTS "Build all tests." OFF)

if(NOT UNIT_TESTS)
	message ("Compiling without unit tests (default behavior). Set UNIT_TESTS to true in cmake if you want to have them.")
else()
	message ("Compiling with unit tests. Set UNIT_TESTS to false in cmake if you dont want to have them")
	find_package(GTest)
	if(NOT GTEST_FOUND)
		message ("Gtest not found automatically. Retrying using GTEST_ROOT=${PROJECT_SOURCE_DIR}/../thirdparty/gtest/")
		set (GTEST_ROOT "${PROJECT_SOURCE_DIR}/../thirdparty/gtest/")
		find_package(GTest)
		if(NOT GTEST_FOUND)
			message ("Gtest not found!!")
		else()
		    message ("Gtest found in GTEST_ROOT=${PROJECT_SOURCE_DIR}/../thirdparty/gtest/")
		endif()
	else()
		message ("Gtest found")
	endif()
	
	if(GTEST_FOUND)
		include_directories(${GTEST_INCLUDE_DIRS})
	endif()
	
	
	find_package(GMock)
	if(NOT GMOCK_FOUND)
		message ("GMock not found automatically. Retrying using GMOCK_ROOT=${PROJECT_SOURCE_DIR}/../thirdparty/gmock/")
		set (GMOCK_ROOT "${PROJECT_SOURCE_DIR}/../thirdparty/gmock/")
		find_package(GMock)
		if(NOT GMOCK_FOUND)
			message ("GMock not found!!")
		else()
		    message ("GMock found in GMOCK_ROOT=${PROJECT_SOURCE_DIR}/../thirdparty/gmock/")
		endif()
	else()
		message ("GMock found")
	endif()
	
	if(GMOCK_FOUND)
		include_directories(${GMOCK_INCLUDE_DIRS})
	endif()
	
endif()

# Just in case we want to do fancy things for windows vs linux
if(CMAKE_COMPILER_IS_GNUCXX)
#	add_definitions(-Wall -ansi -Wno-deprecated -pthread)
endif()

if(MSVC)
#vc 2012 fix for vararg templates
#    set(MSVC_COMPILER_DEFS "-D_VARIADIC_MAX=10")
endif() 

# Turn on the ability to create folders to organize projects (.vcproj)
# It creates "CMakePredefinedTargets" folder by default and adds CMake
# defined projects like INSTALL.vcproj and ZERO_CHECK.vcproj
set_property (GLOBAL PROPERTY USE_FOLDERS ON)

#find_package(Boost REQUIRED COMPONENTS signals thread)

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

# Set compiler flags and options. 
# Here it is setting the Visual Studio warning level to 4
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")

# Command to output information to the console
# Useful for displaying errors, warnings, and debugging
#message ("cxx Flags: " ${CMAKE_CXX_FLAGS})

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${BOOST_ROOT} )
include_directories(${EIGEN3_INCLUDE_DIR} )
include_directories(${TinyXML_INCLUDE_DIRS})

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
    enable_testing()
	subdirs(tests)
endif()

