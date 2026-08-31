stepit_declare_plugin(NAME publisher_ros DEPENDS ros_base)

find_package(catkin QUIET COMPONENTS
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
)
if (NOT catkin_FOUND)
  stepit_plugin_mark_unbuildable("Missing catkin.")
endif ()
