# CMakeLists files in this project can
# refer to the root source directory of the project as ${ROS_BRIDGE_SOURCE_DIR} and
# to the root binary directory of the project as ${ROS_BRIDGE_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)
project (HYBRID_AUTOMATON)

set (CMAKE_BUILD_TYPE RelWithDebInfo)

# setting the output directories
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${HYBRID_AUTOMATON_BINARY_DIR}/bin)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${HYBRID_AUTOMATON_BINARY_DIR}/lib)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${HYBRID_AUTOMATON_BINARY_DIR}/lib)

include_directories( ./include)


# building the library
add_library (hybrid_automaton STATIC 
	src/CSpaceMilestone.cpp
	src/Milestone.cpp
	src/MotionBehaviour.cpp
) 

include_directories(${RLAB_INC_PATH})

#target_link_library(hybrid_automaton graph rxControlSDK_9)