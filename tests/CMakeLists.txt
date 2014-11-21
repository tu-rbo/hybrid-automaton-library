include_directories(${hybrid_automaton_SOURCE_DIR}/tests)
link_directories(${hybrid_automaton_SOURCE_DIR}/lib)

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

target_link_libraries(hybrid_automaton_tests ${TinyXML_LIBRARIES} 
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

						
