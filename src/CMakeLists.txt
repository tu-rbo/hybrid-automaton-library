include_directories(${hybrid_automaton_SOURCE_DIR}/include)
link_directories(${hybrid_automaton_SOURCE_DIR}/include/hybrid_automaton)

set (HA_CORE_SOURCES
    "HybridAutomaton.cpp"
    "HybridAutomatonOStringStream.cpp"
    "ControlMode.cpp"
    "ControlSwitch.cpp"
    "ControlSet.cpp"
    "Controller.cpp"
    "JumpCondition.cpp")

set (HA_DESCRIPTION_SOURCES
    "DescriptionTreeNode.cpp"
    "DescriptionTreeXML.cpp"
    "DescriptionTreeNodeXML.cpp")

set (HA_SENSOR_SOURCES
    "Sensor.cpp"
    "ClockSensor.cpp"
    "ForceTorqueSensor.cpp"
    "FramePoseSensor.cpp"
    "JointConfigurationSensor.cpp")
	
source_group("Header Files" FILES ${HA_CORE_HEADERS})

source_group("Header Files\\Description" FILES ${HA_DESCRIPTION_HEADERS})

source_group("Header Files\\Sensor" FILES ${HA_SENSOR_HEADERS})

source_group("Source Files" FILES ${HA_CORE_SOURCES})

source_group("Source Files\\Description" FILES ${HA_DESCRIPTION_SOURCES})

source_group("Source Files\\Sensor" FILES ${HA_SENSOR_SOURCES})
	
add_library(hybrid_automaton STATIC ${HA_CORE_SOURCES} 
                        ${HA_DESCRIPTION_SOURCES} 
                        ${HA_SENSOR_SOURCES} 
						${HA_CORE_HEADERS} 
                        ${HA_DESCRIPTION_HEADERS} 
                        ${HA_SENSOR_HEADERS})
						
target_link_libraries(hybrid_automaton ${TinyXML_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})