cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_policy_neuro)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/action_source.cpp
      src/actuator.cpp
      src/cmd_posture_source.cpp
      src/cmd_vel_source.cpp
      src/field.cpp
      src/field_assembler.cpp
      src/field_scaling.cpp
      src/heightmap_source.cpp
      src/joint_reordering.cpp
      src/neuro_module.cpp
      src/neuro_policy.cpp
      src/proprioceptor.cpp
      src/roll_pitch_source.cpp
      src/subscriber_action.cpp
    INCLUDES
      include
    FLAGS
      -Wall -Wextra -Wpedantic -Wno-unused-parameter
    DEPENDS
      joystick_base
      nnrt_base
)
