cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_robot_unitree2)

init_submodule(extern/robot_sdk/unitree_sdk2 WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
set(UNITREE_SDK2_HOME ${STEPIT_HOME_DIRECTORY}/extern/robot_sdk/unitree_sdk2)
stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/common.cpp
      src/g1.cpp
      src/joystick.cpp
      src/quadruped.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/robot/unitree2/common.h
        include/stepit/robot/unitree2/g1.h
        include/stepit/robot/unitree2/joystick.h
        include/stepit/robot/unitree2/quadruped.h
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${UNITREE_SDK2_HOME}/include>
    $<BUILD_INTERFACE:${UNITREE_SDK2_HOME}/thirdparty/include>
    $<BUILD_INTERFACE:${UNITREE_SDK2_HOME}/thirdparty/include/ddscxx>
)
target_link_directories(${PROJECT_NAME} BEFORE PUBLIC
    $<BUILD_INTERFACE:${UNITREE_SDK2_HOME}/lib/${CMAKE_SYSTEM_PROCESSOR}>
    $<BUILD_INTERFACE:${UNITREE_SDK2_HOME}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_joystick_base
    ddsc
    ddscxx
    rt
    unitree_sdk2
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${UNITREE_SDK2_HOME}/lib/${CMAKE_SYSTEM_PROCESSOR};${UNITREE_SDK2_HOME}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

add_executable(unitree2_switch_mode app/switch_mode.cpp)
target_link_libraries(unitree2_switch_mode PRIVATE ${PROJECT_NAME})
install(TARGETS unitree2_switch_mode
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
)

set(ICEORYX_INCLUDE_DIR ${UNITREE_SDK2_HOME}/thirdparty/include/iceoryx/v2.0.2)
if (EXISTS "${ICEORYX_INCLUDE_DIR}")
  target_include_directories(${PROJECT_NAME} PUBLIC ${ICEORYX_INCLUDE_DIR})
endif ()
