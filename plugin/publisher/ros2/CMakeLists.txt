cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_publisher_ros2)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/publisher.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/ros2/publisher.h
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
    diagnostic_msgs
    geometry_msgs
    rclcpp
    sensor_msgs
    std_msgs
)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
