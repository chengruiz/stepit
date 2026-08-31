cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_modular_policy_locomotion_ros2)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/cmd_posture_subscriber2.cpp
      src/cmd_vel_subscriber2.cpp
      src/heightmap_subscriber2.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/modular_policy_locomotion_ros2/cmd_posture_subscriber2.h
        include/stepit/modular_policy_locomotion_ros2/cmd_vel_subscriber2.h
        include/stepit/modular_policy_locomotion_ros2/heightmap_subscriber2.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_modular_policy_locomotion
    stepit_plugin_ros2_base
)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    geometry_msgs
    grid_map_msgs
    grid_map_ros
    nav_msgs
    rclcpp
    sensor_msgs
    std_msgs
    tf2_ros
)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
