cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_modular_policy_locomotion_ros)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    grid_map_ros
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2_ros
    topic_tools
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS geometry_msgs grid_map_ros nav_msgs roscpp sensor_msgs std_msgs tf2_ros topic_tools
)
if (NOT STEPIT_BUILDING_WITH_CATKIN)
  get_target_property(CMAKE_RUNTIME_OUTPUT_DIRECTORY stepit_core RUNTIME_OUTPUT_DIRECTORY)
  get_target_property(CMAKE_LIBRARY_OUTPUT_DIRECTORY stepit_core LIBRARY_OUTPUT_DIRECTORY)
  get_target_property(CMAKE_ARCHIVE_OUTPUT_DIRECTORY stepit_core ARCHIVE_OUTPUT_DIRECTORY)
endif ()

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/cmd_posture_subscriber.cpp
      src/cmd_vel_subscriber.cpp
      src/heightmap_subscriber.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/modular_policy_locomotion_ros/cmd_posture_subscriber.h
        include/stepit/modular_policy_locomotion_ros/cmd_vel_subscriber.h
        include/stepit/modular_policy_locomotion_ros/heightmap_subscriber.h
)
foreach (catkin_include_dir ${catkin_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${catkin_include_dir}>")
  if (NOT catkin_include_dir MATCHES "^${CMAKE_BINARY_DIR}(/|$)" AND
      NOT catkin_include_dir MATCHES "^${CATKIN_DEVEL_PREFIX}(/|$)")
    target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:${catkin_include_dir}>")
  endif ()
endforeach ()
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_modular_policy_locomotion
    stepit_plugin_ros_base
    ${catkin_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
