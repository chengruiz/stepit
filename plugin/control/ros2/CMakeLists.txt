cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_control_ros2)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stepit_ros2_msgs QUIET)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/control.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/ros2/control.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_ros2_base
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp
    std_msgs
)

if (stepit_ros2_msgs_FOUND)
  ament_target_dependencies(${PROJECT_NAME} PUBLIC stepit_ros2_msgs)
else ()
  add_subdirectory(${STEPIT_HOME_DIRECTORY}/package/ros2/stepit_ros2_msgs ../stepit_ros2_msgs)
  rosidl_get_typesupport_target(stepit_ros2_msgs_target "stepit_ros2_msgs" "rosidl_typesupport_cpp")
  target_link_libraries(${PROJECT_NAME} PUBLIC ${stepit_ros2_msgs_target})
endif ()

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
