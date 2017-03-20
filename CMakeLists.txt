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
    set(MSVC_COMPILER_DEFS "/MP")
    add_definitions(/MP)
endif() 

# Turn on the ability to create folders to organize projects (.vcproj)
# It creates "CMakePredefinedTargets" folder by default and adds CMake
# defined projects like INSTALL.vcproj and ZERO_CHECK.vcproj
set_property (GLOBAL PROPERTY USE_FOLDERS ON)

find_package(TinyXML)
if(NOT TinyXML_FOUND)
    #message ("TinyXML not found automatically. Retrying using TinyXML_ROOT_DIR=${PROJECT_SOURCE_DIR}/../thirdparty/tinyxml/")
    set (TinyXML_ROOT_DIR "${PROJECT_SOURCE_DIR}/../thirdparty/tinyxml/")
    find_package(TinyXML)
endif()

if(NOT TinyXML_FOUND)
    message (FATAL_ERROR "TinyXML not found!!")
    if (CMAKE_COMPILER_IS_GNUCXX)
        message("Hint: Try to install package libtinyxml-dev or set -DTinyXML_ROOT_DIR=/path/to/custom/tinyxml")	
    endif()
else()
	message ("TinyXML found with include directories TinyXML_INCLUDE_DIRS=${TinyXML_INCLUDE_DIRS}")
	include_directories(${TinyXML_INCLUDE_DIRS})
endif()

find_package(Boost 1.46.0)
if(NOT Boost_FOUND AND WIN32)
    message ("Boost not found automatically. Retrying using BOOST_ROOT=C:/Program Files/boost/boost_1_57_0")
    set (BOOST_ROOT "C:/Program Files/boost/boost_1_57_0")
    find_package(Boost 1.46.0)
endif()
	
if(NOT Boost_FOUND)
    message (FATAL_ERROR "Boost not found!!")
else()
    message ("Boost found with root directory BOOST_ROOT=${BOOST_ROOT}")
    include_directories(${Boost_INCLUDE_DIR})
    include_directories(${BOOST_ROOT} )
endif()

find_package(Eigen3)
if(NOT EIGEN3_FOUND AND WIN32)
    message ("Eigen not found automatically. Retrying using EIGEN_DIR=C:/Program Files/eigen/eigen-eigen-1306d75b4a21")
    set (EIGEN_DIR "C:/Program Files/eigen/eigen-eigen-1306d75b4a21")
    find_package(Eigen3)
endif()

if(NOT EIGEN3_FOUND)
    message (FATAL_ERROR "Eigen not found!!")
    if (UNIX)
        message("Hint: Try to install package libeigen3-dev or set -DEIGEN_DIR=/path/to/custom/libeigen3")	
    endif()
else()
    message ("Eigen found with include directories EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")
    include_directories(${EIGEN3_INCLUDE_DIR} )
endif()




# code for compiling the doxygen documentation
find_package(Doxygen)
if(NOT DOXYGEN_FOUND)
    message ("Doxygen not found. You will not be able to compile the documentation.")
else()
    message ("Doxygen found: ${DOXYGEN_EXECUTABLE}.")
    message ("Hint: Use make doc to compile documentation.")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)  #configure doxygen to build into the build directory
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen_redirect_html.in ${CMAKE_CURRENT_SOURCE_DIR}/documentation.html @ONLY) #create a nice redirection to the build directory so documentation is easy to find
    add_custom_target(doc 
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        COMMENT "Generating API documentation with Doxygen in ${CMAKE_CURRENT_BINARY_DIR}/html and ${CMAKE_CURRENT_BINARY_DIR}/latex" VERBATIM
    )
endif()




# Set compiler flags and options. 
# Here it is setting the Visual Studio warning level to 4
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")

# Command to output information to the console
# Useful for displaying errors, warnings, and debugging
#message ("cxx Flags: " ${CMAKE_CXX_FLAGS})

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(examples)

# Collect sources into the variable HA_SOURCES 
set (HA_CORE_HEADERS
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/HybridAutomaton.h"
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/HybridAutomatonStringStream.h"
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
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/JointConfigurationSensor.h"
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/JointVelocitySensor.h"
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/FrameDisplacementSensor.h"
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/FrameOrientationSensor.h"
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/ROSTopicSensor.h"
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/SubjointConfigurationSensor.h"
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/SubjointVelocitySensor.h")

set (HA_DESCRIPTION_HEADERS
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/DescriptionTree.h"
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/DescriptionTreeNode.h"
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/DescriptionTreeXML.h"
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/DescriptionTreeNodeXML.h")

set (HA_FACTORY_HEADERS
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/HybridAutomatonAbstractFactory.h"
    "${PROJECT_SOURCE_DIR}/include/hybrid_automaton/HybridAutomatonRBOFactory.h")


set (HA_CORE_SOURCES
    "${PROJECT_SOURCE_DIR}/src/HybridAutomaton.cpp"
    "${PROJECT_SOURCE_DIR}/src/HybridAutomatonStringStream.cpp"
    "${PROJECT_SOURCE_DIR}/src/ControlMode.cpp"
    "${PROJECT_SOURCE_DIR}/src/ControlSwitch.cpp"
    "${PROJECT_SOURCE_DIR}/src/ControlSet.cpp"
    "${PROJECT_SOURCE_DIR}/src/Controller.cpp"
    "${PROJECT_SOURCE_DIR}/src/JumpCondition.cpp")

set (HA_DESCRIPTION_SOURCES
    "${PROJECT_SOURCE_DIR}/src/DescriptionTreeNode.cpp"
    "${PROJECT_SOURCE_DIR}/src/DescriptionTreeXML.cpp"
    "${PROJECT_SOURCE_DIR}/src/DescriptionTreeNodeXML.cpp")

set (HA_SENSOR_SOURCES
    "${PROJECT_SOURCE_DIR}/src/Sensor.cpp"
    "${PROJECT_SOURCE_DIR}/src/ClockSensor.cpp"
    "${PROJECT_SOURCE_DIR}/src/ForceTorqueSensor.cpp"
    "${PROJECT_SOURCE_DIR}/src/FramePoseSensor.cpp"
    "${PROJECT_SOURCE_DIR}/src/JointConfigurationSensor.cpp"
    "${PROJECT_SOURCE_DIR}/src/JointVelocitySensor.cpp"
    "${PROJECT_SOURCE_DIR}/src/FrameDisplacementSensor.cpp"
    "${PROJECT_SOURCE_DIR}/src/FrameOrientationSensor.cpp"
    "${PROJECT_SOURCE_DIR}/src/ROSTopicSensor.cpp"
    "${PROJECT_SOURCE_DIR}/src/SubjointConfigurationSensor.cpp"
    "${PROJECT_SOURCE_DIR}/src/SubjointVelocitySensor.cpp")

set (HA_FACTORY_SOURCES
    "${PROJECT_SOURCE_DIR}/src/HybridAutomatonAbstractFactory.cpp"
    "${PROJECT_SOURCE_DIR}/src/HybridAutomatonRBOFactory.cpp")

source_group("Header Files" FILES ${HA_CORE_HEADERS})
source_group("Header Files\\Description" FILES ${HA_DESCRIPTION_HEADERS})
source_group("Header Files\\Sensor" FILES ${HA_SENSOR_HEADERS})
source_group("Source Files" FILES ${HA_CORE_SOURCES})
source_group("Source Files\\Description" FILES ${HA_DESCRIPTION_SOURCES})
source_group("Source Files\\Sensor" FILES ${HA_SENSOR_SOURCES})
source_group("Source Files\\Factory" FILES ${HA_FACTORY_SOURCES})

add_library(hybrid_automaton STATIC
    ${HA_CORE_SOURCES} 
    ${HA_DESCRIPTION_SOURCES} 
    ${HA_SENSOR_SOURCES}
    ${HA_FACTORY_SOURCES} 
    ${HA_CORE_HEADERS}
    ${HA_DESCRIPTION_HEADERS} 
    ${HA_SENSOR_HEADERS}
    ${HA_FACTORY_HEADERS})
target_link_libraries(hybrid_automaton ${TinyXML_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

if (UNIX)
    # Build also the shared version of the library -- not under windows
    add_library(hybrid_automaton_shared SHARED
        ${HA_CORE_SOURCES} 
        ${HA_DESCRIPTION_SOURCES} 
        ${HA_SENSOR_SOURCES}
        ${HA_FACTORY_SOURCES} 
        ${HA_CORE_HEADERS}
        ${HA_DESCRIPTION_HEADERS} 
        ${HA_SENSOR_HEADERS}
        ${HA_FACTORY_HEADERS})
    target_link_libraries(hybrid_automaton_shared ${TinyXML_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})
    set_target_properties(hybrid_automaton_shared PROPERTIES OUTPUT_NAME hybrid_automaton)
endif()

# because of sensor registration hybrid_automaton_visualizer needs to compile
# the sources again and cannot link against the libraries
add_executable(hybrid_automaton_visualizer
    src/hybrid_automaton_visualizer.cpp
    ${HA_CORE_SOURCES}
    ${HA_DESCRIPTION_SOURCES}
    ${HA_SENSOR_SOURCES}
    ${HA_FACTORY_SOURCES} 
    ${HA_CORE_HEADERS}
    ${HA_DESCRIPTION_HEADERS} 
    ${HA_SENSOR_HEADERS}
    ${HA_FACTORY_HEADERS})
target_link_libraries(hybrid_automaton_visualizer ${TinyXML_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

#subdirs(src)

if(UNIT_TESTS)
    message ("Compilation of unit tests is ENABLED.")
    message ("Hint: use -DUNIT_TESTS=false if if you dontwant them built.")
    enable_testing()
    subdirs(tests)
else()
    message ("Compilation of unit tests is disabled. (default behavior).")
    message ("Hint: use -DUNIT_TESTS=true if you want them built.")
endif()

if(WIN32)
    install(TARGETS hybrid_automaton DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/lib")
else()
    install(TARGETS hybrid_automaton DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/lib")
    install(TARGETS hybrid_automaton_shared DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/lib")
    install(TARGETS hybrid_automaton DESTINATION "/usr/local/lib")
    install(TARGETS hybrid_automaton_shared DESTINATION "/usr/local/lib")
    install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/hybrid_automaton" DESTINATION "/usr/local/include")
endif()
