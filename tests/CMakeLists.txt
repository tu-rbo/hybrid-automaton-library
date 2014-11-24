# Finding GTest
find_package(GTest)
	
# If we are building on windows visual studio and we did't find GTest, we try setting GTEST_ROOT manually
if(NOT GTEST_FOUND AND MSVC)
	message ("Gtest not found automatically. Retrying using GTEST_ROOT=${PROJECT_SOURCE_DIR}/../thirdparty/gtest/")
	set (GTEST_ROOT "${PROJECT_SOURCE_DIR}/../thirdparty/gtest/")
	find_package(GTest)
endif()
	
if(NOT GTEST_FOUND)
	message (FATAL_ERROR "Gtest not found!")
else()
    message ("Gtest found with include directories GTEST_INCLUDE_DIRS=${GTEST_INCLUDE_DIRS}")
	include_directories(${GTEST_INCLUDE_DIRS})
endif()
	
# Finding GMock
find_package(GMock)

# If we are building on windows visual studio and we didn't find GMock, we try setting GMOCK_ROOT manually
if(NOT GMOCK_FOUND AND MSVC)
	message ("GMock not found automatically. Retrying using GMOCK_ROOT=${PROJECT_SOURCE_DIR}/../thirdparty/gmock/")
	set (GMOCK_ROOT "${PROJECT_SOURCE_DIR}/../thirdparty/gmock/")
	find_package(GMock)
endif()

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
    "controller_test.cpp"
    "controlmode_test.cpp"
    "controlset_test.cpp"
    "controlswitch_test.cpp"
    "description_tree_test.cpp"
    "description_tree_tinyxml_test.cpp"
    "hybrid_automaton_serialization_test.cpp"
	"hybrid_automaton_stepping_test.cpp"
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
			${GMOCK_LIBRARIES} 
			${GMOCK_MAIN_LIBRARIES}
			${GTEST_LIBRARIES} 
			${GTEST_MAIN_LIBRARIES}
			hybrid_automaton)
	
source_group("Header Files" FILES ${HA_TESTS_HEADERS})

source_group("Source Files" FILES ${HA_TESTS_SOURCES})

# This is so you can do 'make test' to see all your tests run, instead of
# manually running the executable runUnitTests to see those specific tests.
add_test(myhybrid_automaton_tests hybrid_automaton_tests)

						
