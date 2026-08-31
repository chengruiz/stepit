stepit_declare_plugin(NAME joystick_ros DEPENDS
    joystick_base
    ros_base
)

find_package(catkin QUIET COMPONENTS roscpp sensor_msgs)
if (NOT catkin_FOUND)
  stepit_plugin_mark_unbuildable("Missing catkin.")
endif ()
