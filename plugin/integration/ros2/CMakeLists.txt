cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_ros2_base)
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stepit_ros2_msgs QUIET)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/control.cpp
      src/joystick.cpp
      src/node.cpp
      src/publisher.cpp
      src/spin.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/ros2/control.h
        include/stepit/ros2/joystick.h
        include/stepit/ros2/node.h
        include/stepit/ros2/publisher.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_joystick_base
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
set(entry_target ${PROJECT_NAME}_entry)
stepit_add_entry(${entry_target})
target_sources(${entry_target}
    PRIVATE
      src/entry.cpp
)
target_link_libraries(${entry_target} PRIVATE ${PROJECT_NAME})
install(TARGETS ${entry_target}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    diagnostic_msgs geometry_msgs rclcpp sensor_msgs std_msgs
)

if (stepit_ros2_msgs_FOUND)
  #  The project is built with colcon
  ament_target_dependencies(${PROJECT_NAME} PUBLIC stepit_ros2_msgs)
else ()
  add_subdirectory(${STEPIT_HOME_DIRECTORY}/package/ros2/stepit_ros2_msgs ../stepit_ros2_msgs)
  rosidl_get_typesupport_target(stepit_ros2_msgs_target "stepit_ros2_msgs" "rosidl_typesupport_cpp")
  target_link_libraries(${PROJECT_NAME} PUBLIC ${stepit_ros2_msgs_target})
endif ()

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
