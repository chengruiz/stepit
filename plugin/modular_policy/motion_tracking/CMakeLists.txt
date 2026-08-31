cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_modular_policy_motion_tracking)

find_package(pinocchio REQUIRED)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/forward_kinematics.cpp
      src/geometry_operator.cpp
      src/motion_clip.cpp
      src/motion_player.cpp
      src/relative_pose_source.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/modular_policy/forward_kinematics.h
        include/stepit/modular_policy/geometry_operator.h
        include/stepit/modular_policy/motion_clip.h
        include/stepit/modular_policy/motion_player.h
        include/stepit/modular_policy/relative_pose_source.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_modular_policy_base
    stepit_plugin_pyutils
    stepit_plugin_joystick_base
    pinocchio::pinocchio
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
target_compile_definitions(${PROJECT_NAME} PUBLIC COAL_DISABLE_HPP_FCL_WARNINGS)
