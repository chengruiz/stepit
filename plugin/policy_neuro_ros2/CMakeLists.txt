cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_policy_neuro_ros2)
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake QUIET)
if (NOT ament_cmake_FOUND)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing ament_cmake.")
  return()
endif ()

find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/cmd_posture_subscriber2.cpp
      src/cmd_vel_subscriber2.cpp
      src/field_subscriber2.cpp
      src/heightmap_subscriber2.cpp
    INCLUDES
      include
    DEPENDS
      policy_neuro
      ros2_base
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    geometry_msgs
    grid_map_msgs
    grid_map_ros
    nav_msgs
    sensor_msgs
    std_msgs
    tf2_ros
)


if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
