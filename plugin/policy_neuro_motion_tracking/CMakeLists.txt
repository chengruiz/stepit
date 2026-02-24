cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_policy_neuro_motion_tracking)

find_package(pinocchio)
if (NOT pinocchio_FOUND)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing pinocchio.")
  return()
endif()

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/motion_trajectory.cpp
      src/forward_kinematics.cpp
      src/relative_pose_source.cpp
    INCLUDES
      include
    LINK_LIBS
      pinocchio::pinocchio
    FLAGS
      -Wall -Wextra -Wpedantic -Wno-unused-parameter
    DEPENDS
      policy_neuro
      pyutils
)
target_compile_definitions(${PROJECT_NAME} PUBLIC COAL_DISABLE_HPP_FCL_WARNINGS)
