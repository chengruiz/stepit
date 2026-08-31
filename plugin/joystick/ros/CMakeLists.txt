cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_joystick_ros)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp sensor_msgs
)
if (NOT STEPIT_BUILDING_WITH_CATKIN)
  get_target_property(CMAKE_RUNTIME_OUTPUT_DIRECTORY stepit_core RUNTIME_OUTPUT_DIRECTORY)
  get_target_property(CMAKE_LIBRARY_OUTPUT_DIRECTORY stepit_core LIBRARY_OUTPUT_DIRECTORY)
  get_target_property(CMAKE_ARCHIVE_OUTPUT_DIRECTORY stepit_core ARCHIVE_OUTPUT_DIRECTORY)
endif ()

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/joystick.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/ros/joystick.h
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
    stepit_plugin_joystick_base
    stepit_plugin_ros_base
    ${catkin_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
