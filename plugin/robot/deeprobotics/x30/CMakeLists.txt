cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_robot_deeprobotics_x30)

init_submodule(extern/robot_sdk/deeprobotics_x30_sdk WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
set(X30_SDK_HOME ${STEPIT_HOME_DIRECTORY}/extern/robot_sdk/deeprobotics_x30_sdk)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      ${X30_SDK_HOME}/src/command.cpp
      ${X30_SDK_HOME}/src/command_list.cpp
      ${X30_SDK_HOME}/src/parse_cmd.cpp
      ${X30_SDK_HOME}/src/send_to_robot.cpp
      src/deeprobotics_x30.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/robot/deeprobotics_x30.h
)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${X30_SDK_HOME}/include>)
target_link_libraries(${PROJECT_NAME} PUBLIC stepit_core)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
