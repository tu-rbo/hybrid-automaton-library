# Finding GTest
# We set GTEST_ROOT manually because we need the specific version that we include in the thirdparty folder
message ("Gtest must be the version included in HA thirdparty. Searching for GTest in GTEST_ROOT=${PROJECT_SOURCE_DIR}/../thirdparty/gmock-1.7.0/gtest/")
set (GTEST_ROOT "${PROJECT_SOURCE_DIR}/../thirdparty/gmock-1.7.0/gtest/")
find_package(GTest)
	
if(NOT GTEST_FOUND)
	message (FATAL_ERROR "Gtest not found!")
else()
    message ("Gtest found with include directories GTEST_INCLUDE_DIRS=${GTEST_INCLUDE_DIRS}")
	include_directories(${GTEST_INCLUDE_DIRS})
endif()
	
# Finding GMock
# We set GMOCK_ROOT manually because we need the specific version that we include in the thirdparty folder
message ("GMock must be the version included in HA thirdparty. Searching for GMock in GMOCK_ROOT=${PROJECT_SOURCE_DIR}/../thirdparty/gmock-1.7.0/")
set (GMOCK_ROOT "${PROJECT_SOURCE_DIR}/../thirdparty/gmock-1.7.0/")
find_package(GMock)

if(NOT GMOCK_FOUND)
	message (FATAL_ERROR "GMock not found!!")
else()
	message ("GMock found with include directoires GMOCK_INCLUDE_DIRS=${GMOCK_INCLUDE_DIRS}")
	include_directories(${GMOCK_INCLUDE_DIRS})
endif()

include_directories(${hybrid_automaton_SOURCE_DIR}/tests)
link_directories(${hybrid_automaton_SOURCE_DIR}/lib)

if(CMAKE_COMPILER_IS_GNUCXX)
	find_package(Threads)
endif()

set (HA_TESTS_SOURCES
	"gtest_mem_main.cpp"
    "controller_test.cpp"
    "controlmode_test.cpp"
    "controlset_test.cpp"
    "controlswitch_test.cpp"
    "jumpcondition_test.cpp"
    "description_tree_test.cpp"
    "description_tree_tinyxml_test.cpp"
    "hybrid_automaton_serialization_test.cpp"
	"hybrid_automaton_stepping_test.cpp"
	"ftsensor_test.cpp"
	)

set (HA_TESTS_HEADERS
    "MockDescriptionTree.h"
    "MockDescriptionTreeNode.h"
	)
	
	
add_executable(hybrid_automaton_tests ${HA_TESTS_SOURCES} ${HA_TESTS_HEADERS} )

if(CMAKE_COMPILER_IS_GNUCXX)
	target_link_libraries(hybrid_automaton_tests 
				${CMAKE_THREAD_LIBS_INIT}
				)
endif()

target_link_libraries(hybrid_automaton_tests 
			${TinyXML_LIBRARIES} 
			${Boost_LIBRARIES} 
			${Eigen3_LIBRARIES} 
			)
			
target_link_libraries(hybrid_automaton_tests 
			debug ${GTEST_LIBRARY_DEBUG} optimized ${GTEST_LIBRARY}
			debug ${GTEST_MAIN_LIBRARY_DEBUG} optimized ${GTEST_MAIN_LIBRARY}
			debug ${GMOCK_LIBRARY_DEBUG} optimized ${GMOCK_LIBRARY}
			debug ${GMOCK_MAIN_LIBRARY_DEBUG} optimized ${GMOCK_MAIN_LIBRARY}
			)
target_link_libraries(hybrid_automaton_tests hybrid_automaton)
	
source_group("Header Files" FILES ${HA_TESTS_HEADERS})

source_group("Source Files" FILES ${HA_TESTS_SOURCES})

# This is so you can do 'make test' to see all your tests run, instead of
# manually running the executable runUnitTests to see those specific tests.
add_test(myhybrid_automaton_tests hybrid_automaton_tests)

						
