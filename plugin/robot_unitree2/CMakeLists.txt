cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_robot_unitree2)

if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|aarch64")
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Requires x86_64 or aarch64 architectures, got '${CMAKE_SYSTEM_PROCESSOR}'.")
  return()
endif ()

init_submodule(extern/robot_sdk/unitree_sdk2 WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
set(UNITREE_SDK2_HOME ${STEPIT_HOME_DIRECTORY}/extern/robot_sdk/unitree_sdk2)
stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/aliengo_mujoco.cpp
      src/b2.cpp
      src/common.cpp
      src/g1.cpp
      src/go2.cpp
      src/joystick.cpp
    INCLUDES
      include
      ${UNITREE_SDK2_HOME}/include
      ${UNITREE_SDK2_HOME}/thirdparty/include
      ${UNITREE_SDK2_HOME}/thirdparty/include/ddscxx
    LINK_DIRS
      ${UNITREE_SDK2_HOME}/lib/${CMAKE_SYSTEM_PROCESSOR}
      ${UNITREE_SDK2_HOME}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}
    LINK_LIBS
      ddsc ddscxx rt unitree_sdk2
    DEPENDS
      joystick_base
)

stepit_add_executable(unitree2_switch_mode app/switch_mode.cpp)
target_link_libraries(unitree2_switch_mode PRIVATE ${PROJECT_NAME})

set(ICEORYX_INCLUDE_DIR ${UNITREE_SDK2_HOME}/thirdparty/include/iceoryx/v2.0.2)
if (EXISTS "${ICEORYX_INCLUDE_DIR}")
  target_include_directories(${PROJECT_NAME} PUBLIC ${ICEORYX_INCLUDE_DIR})
endif ()
