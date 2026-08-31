cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_control_ros)

if (STEPIT_BUILDING_WITH_CATKIN)
  find_package(catkin REQUIRED COMPONENTS roscpp std_msgs stepit_ros_msgs)
else ()
  find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
  add_subdirectory(${STEPIT_HOME_DIRECTORY}/package/ros/stepit_ros_msgs ../stepit_ros_msgs)
endif ()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs stepit_ros_msgs
)
if (NOT STEPIT_BUILDING_WITH_CATKIN)
  get_target_property(CMAKE_RUNTIME_OUTPUT_DIRECTORY stepit_core RUNTIME_OUTPUT_DIRECTORY)
  get_target_property(CMAKE_LIBRARY_OUTPUT_DIRECTORY stepit_core LIBRARY_OUTPUT_DIRECTORY)
  get_target_property(CMAKE_ARCHIVE_OUTPUT_DIRECTORY stepit_core ARCHIVE_OUTPUT_DIRECTORY)
endif ()

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/control.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/ros/control.h
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
    stepit_plugin_ros_base
    ${catkin_LIBRARIES}
)
if (TARGET stepit_ros_msgs)
  target_link_libraries(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:stepit_ros_msgs>)
endif ()
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
