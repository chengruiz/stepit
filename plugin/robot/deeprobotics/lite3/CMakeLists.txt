cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_robot_deeprobotics_lite3)

init_submodule(extern/robot_sdk/deeprobotics_lite3_sdk WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
set(LITE3_SDK_HOME ${STEPIT_HOME_DIRECTORY}/extern/robot_sdk/deeprobotics_lite3_sdk)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/deeprobotics_lite3.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/robot/deeprobotics_lite3.h
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${LITE3_SDK_HOME}/include>
    $<BUILD_INTERFACE:${LITE3_SDK_HOME}/include/common>
)
target_link_directories(${PROJECT_NAME} BEFORE PUBLIC $<BUILD_INTERFACE:${LITE3_SDK_HOME}/lib>)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    deeprobotics_legged_sdk_${CMAKE_SYSTEM_PROCESSOR}
)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${LITE3_SDK_HOME}/lib")
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
