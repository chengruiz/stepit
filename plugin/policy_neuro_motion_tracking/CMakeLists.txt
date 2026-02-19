cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_policy_neuro_motion_tracking)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/motion_trajectory.cpp
    INCLUDES
      include
    FLAGS
      -Wall -Wextra -Wpedantic -Wno-unused-parameter
    DEPENDS
      policy_neuro
      pyutils
)
